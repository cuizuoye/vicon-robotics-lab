#!/usr/bin/env python3
"""Live ROS2 waypoint trajectory integration test, with no physical actuation.

No UDP/serial bridge or Vicon receiver is imported or launched. Domain 97 is
used only for these processes. Published /car/pose is simulated, not Vicon.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/mecanum_vicon_control'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from mecanum_vicon_control.simulated_vicon_car import SimulatedViconCar
from mecanum_vicon_control.car_state_bridge import CarStateBridge
from mecanum_vicon_control.pose_controller import PoseController
from mecanum_vicon_control.safety_watchdog import SafetyWatchdog
from mecanum_vicon_control.trajectory_generator import TrajectoryGenerator
from mecanum_vicon_control.car_visualizer import CarVisualizer
from mecanum_vicon_control.trajectory_metrics import point_segment_distance, tracking_metrics


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--preview-seconds', type=int, default=0)
    p.add_argument('--shape', choices=['rectangle', 'circle', 'figure_eight'], default='rectangle')
    p.add_argument('--loops', type=int, default=1)
    p.add_argument('--waypoints', type=int, default=8)
    p.add_argument('--ros-domain-id', type=int, choices=[97, 98, 99], default=97)
    a = p.parse_args()
    if not 0 <= a.preview_seconds <= 600:
        p.error('preview duration must be 0..600')
    if not 1 <= a.loops <= 5 or not 8 <= a.waypoints <= 128:
        p.error('loops must be 1..5 and waypoints 8..128')
    output = Path(a.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    params = [f'shape:={a.shape}', 'width_m:=1.0', 'height_m:=1.0', 'radius_m:=0.5',
              f'waypoint_count:={a.waypoints}', f'loop_count:={a.loops}', 'heading_mode:=fixed',
              'fixed_yaw_rad:=0.0', 'relative_to_start:=true', 'start_enabled:=false',
              'position_noise_std_m:=0.0']
    args = ['--ros-args']
    for param in params:
        args += ['-p', param]
    rclpy.init(args=args, domain_id=a.ros_domain_id)
    executor = SingleThreadedExecutor()
    simulator, bridge, controller = SimulatedViconCar(), CarStateBridge(), PoseController()
    watchdog, trajectory, visualizer = SafetyWatchdog(), TrajectoryGenerator(), CarVisualizer()
    observer = Node('batteryless_rectangle_observer')
    nodes = [simulator, bridge, controller, watchdog, trajectory, visualizer, observer]
    for node in nodes:
        executor.add_node(node)
    rows = {'poses': [], 'commands': [], 'goals': [], 'paths': [], 'markers': 0,
            'active_segment_errors': [], 'waypoint_indices': []}
    recording = False
    subscriptions = []
    started = time.monotonic()
    def pose_cb(m):
        if recording:
            point = (m.pose.position.x, m.pose.position.y)
            rows['poses'].append([time.monotonic()-started, *point])
            index = trajectory.waypoint_index
            start = trajectory.waypoints[max(0, index-1)][:2]
            end = trajectory.waypoints[index][:2]
            rows['active_segment_errors'].append(point_segment_distance(point, start, end))
            rows['waypoint_indices'].append(index)
    def cmd_cb(m):
        rows['commands'].append([time.monotonic()-started, m.linear.x, m.linear.y, m.angular.z])
    def goal_cb(m):
        point = [m.pose.position.x, m.pose.position.y]
        if not rows['goals'] or point != rows['goals'][-1]:
            rows['goals'].append(point)
    def path_cb(m):
        rows['paths'].append([time.monotonic()-started, len(m.poses)])
    def marker_cb(_m):
        rows['markers'] += 1
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    for typ, topic, cb, qos in [
        (PoseStamped, '/car/pose', pose_cb, 10),
        (Twist, '/car/cmd_vel', cmd_cb, 10),
        (PoseStamped, '/car/goal_pose', goal_cb, latched),
        (RosPath, '/car/path_actual', path_cb, 10),
        (Marker, '/car/goal_marker', marker_cb, latched),
    ]:
        subscriptions.append(observer.create_subscription(typ, topic, cb, qos))
    enable = observer.create_publisher(Bool, '/car/enable', 10)
    trajectory_enable = observer.create_publisher(Bool, '/car/trajectory_enable', 10)
    manual_goal = observer.create_publisher(PoseStamped, '/car/goal_pose', latched)
    display = None
    display_log = None
    result = {'simulated_poses': True, 'real_vicon_used': False, 'hardware_bridge_loaded': False,
              'physical_motor_commands_sent': False, 'ros_domain_id': a.ros_domain_id, 'success': False}
    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.005)
    try:
        spin_for(1.0)
        names = observer.get_node_names()
        assert not any('udp_bridge' in n or 'vicon_client' in n for n in names), names
        assert bridge.last_valid_ns is not None and watchdog.is_fresh(watchdog.get_clock().now().nanoseconds)
        if a.preview_seconds:
            display_log = open(str(output.with_suffix('.rviz.log')), 'w')
            display = subprocess.Popen([
                'ros2', 'launch', 'mecanum_description', 'display.launch.py',
                'wheelbase:=0.122', 'track_width:=0.170', 'wheel_diameter:=0.060'],
                env={**os.environ, 'ROS_DOMAIN_ID': str(a.ros_domain_id)}, start_new_session=True,
                stdout=display_log, stderr=subprocess.STDOUT)
        enable.publish(Bool(data=True))
        spin_for(0.2)
        assert watchdog.enabled
        recording = True
        trajectory_enable.publish(Bool(data=True))
        loop_reports = []
        deadline = time.monotonic() + max(90, a.loops*a.waypoints*6)
        while trajectory.completed_loops < a.loops and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
            if trajectory.completed_loops > len(loop_reports):
                loop_reports.append({'loop': trajectory.completed_loops,
                    'time_s': time.monotonic()-started,
                    'closure_error_m': math.hypot(simulator.x, simulator.y)})
                print('LOOP_COMPLETE:', loop_reports[-1], flush=True)
        assert trajectory.completed_loops == a.loops and not trajectory.active, 'trajectory did not finish'
        spin_for(0.5)
        recording = False
        plan = [p[:2] for p in trajectory.waypoints]
        expected = plan + plan[1:]*(a.loops-1)
        assert len(rows['goals']) == len(expected), rows['goals']
        assert all(math.hypot(x-ex, y-ey) < .005
                   for (x,y),(ex,ey) in zip(rows['goals'], expected))
        closure = math.hypot(simulator.x, simulator.y)
        assert closure < .03
        assert len(rows['paths']) > 100 and rows['paths'][-1][1] > 100 and rows['markers'] >= len(expected)
        dirs = {'forward': any(c[1]>.01 for c in rows['commands']),
                'left': any(c[2]>.01 for c in rows['commands']),
                'backward': any(c[1]<-.01 for c in rows['commands']),
                'right': any(c[2]<-.01 for c in rows['commands'])}
        assert all(dirs.values())
        result['trajectory'] = {'shape': a.shape, 'width_m': 1, 'height_m': 1,
            'radius_m': .5, 'completed_loops': a.loops, 'loop_reports': loop_reports,
            'closure_error_m': closure, 'fixed_yaw_rad': simulator.yaw,
            'goals': list(rows['goals']), 'directions': dirs,
            'duration_s': time.monotonic()-started,
            'actual_path_points': rows['paths'][-1][1], 'goal_marker_messages': rows['markers']}
        if a.shape == 'rectangle':
            result['rectangle'] = result['trajectory']
        result['metrics'], errors = tracking_metrics(rows['poses'], plan, rows['active_segment_errors'])
        assert result['metrics']['active_segment']['max_m'] < .10
        assert all(r['closure_error_m'] < .03 for r in loop_reports)
        with output.with_suffix('.csv').open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time_s','x_m','y_m','nearest_path_error_m','active_segment_error_m','waypoint_index'])
            for pose, error, active, index in zip(rows['poses'], errors,
                    rows['active_segment_errors'], rows['waypoint_indices']):
                writer.writerow([*pose, error, active, index])
        # Move only the simulated goal, then withhold simulated pose messages.
        goal = PoseStamped(); goal.header.frame_id='map'
        goal.header.stamp=observer.get_clock().now().to_msg()
        goal.pose.position.x=simulator.x + .3; goal.pose.position.y=simulator.y
        goal.pose.orientation.w=1.0
        manual_goal.publish(goal)
        spin_for(.3)
        assert abs(watchdog.latest_cmd.linear.x) > .01
        executor.remove_node(simulator)
        spin_for(.5)
        assert not watchdog.enabled and watchdog.last_fault == 'pose_or_command_timeout'
        assert all(abs(v)<1e-9 for v in rows['commands'][-1][1:])
        executor.add_node(simulator)
        spin_for(.5)
        assert not watchdog.enabled and all(abs(v)<1e-9 for v in rows['commands'][-1][1:])
        result['lost_pose_test'] = {'zero_output_latched': True,
                                    'data_restore_does_not_reenable': True}
        result['success'] = True
        result['samples'] = rows
        output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='samples'}), flush=True)
        # Keep an explicitly disabled, simulated preview for a bounded duration.
        if a.preview_seconds:
            print(f'PREVIEW_READY: domain {a.ros_domain_id}, control disabled, {a.preview_seconds}s maximum', flush=True)
            spin_for(a.preview_seconds)
    except Exception as e:
        result['error'] = str(e)
        result['samples'] = rows
        output.write_text(json.dumps(result, indent=2)+'\n')
        raise
    finally:
        enable.publish(Bool(data=False))
        watchdog.enabled = False
        if display is not None and display.poll() is None:
            os.killpg(display.pid, signal.SIGINT)
            try:
                display.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(display.pid, signal.SIGTERM)
                display.wait(timeout=5)
        if display_log is not None:
            display_log.close()
        for node in nodes:
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
