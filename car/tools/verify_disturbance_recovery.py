#!/usr/bin/env python3
"""Deterministic ROS 2 disturbance-recovery test with simulated poses only."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/mecanum_vicon_control'))

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

from mecanum_vicon_control.car_state_bridge import CarStateBridge
from mecanum_vicon_control.path_recovery_controller import (
    PathRecoveryController,
    RECOVERY,
    TRACKING,
    VICON_LOST,
)
from mecanum_vicon_control.safety_watchdog import SafetyWatchdog
from mecanum_vicon_control.simulated_vicon_car import SimulatedViconCar
from mecanum_vicon_control.trajectory_generator import TrajectoryGenerator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--ros-domain-id', type=int, choices=[94, 95, 96], default=96)
    parser.add_argument('--disturbance-y-m', type=float, default=0.20)
    args = parser.parse_args()
    if not 0.12 <= args.disturbance_y_m <= 0.30:
        parser.error('disturbance-y-m must be 0.12..0.30')

    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    params = [
        'shape:=rectangle', 'width_m:=2.0', 'height_m:=1.0',
        'waypoint_count:=8', 'loop_count:=1', 'heading_mode:=fixed',
        'fixed_yaw_rad:=0.0', 'relative_to_start:=true', 'start_enabled:=false',
        'position_noise_std_m:=0.0', 'nominal_speed_mps:=0.10',
        'max_linear_speed_mps:=0.10', 'recovery_enter_m:=0.08',
        'recovery_exit_m:=0.03', 'recovery_settle_s:=0.50',
    ]
    ros_args = ['--ros-args']
    for parameter in params:
        ros_args += ['-p', parameter]
    rclpy.init(args=ros_args, domain_id=args.ros_domain_id)

    executor = SingleThreadedExecutor()
    simulator = SimulatedViconCar()
    bridge = CarStateBridge()
    controller = PathRecoveryController()
    watchdog = SafetyWatchdog()
    trajectory = TrajectoryGenerator()
    observer = Node('disturbance_recovery_observer')
    nodes = [simulator, bridge, controller, watchdog, trajectory, observer]
    for node in nodes:
        executor.add_node(node)

    rows = []
    states = []
    latest = {'pose': None, 'cmd': Twist(), 'cross': None, 'progress': None,
              'state': None, 'safety': None}
    started = time.monotonic()

    def timestamp():
        return time.monotonic() - started

    def pose_cb(msg):
        latest['pose'] = msg

    def cmd_cb(msg):
        latest['cmd'] = msg

    def cross_cb(msg):
        latest['cross'] = float(msg.data)

    def progress_cb(msg):
        latest['progress'] = float(msg.data)

    def state_cb(msg):
        latest['state'] = msg.data
        if not states or states[-1][1] != msg.data:
            states.append([timestamp(), msg.data])

    def safety_cb(msg):
        latest['safety'] = bool(msg.data)

    subscriptions = [
        observer.create_subscription(PoseStamped, '/car/pose', pose_cb, 10),
        observer.create_subscription(Twist, '/car/cmd_vel', cmd_cb, 10),
        observer.create_subscription(Float64, '/car/cross_track_error', cross_cb, 10),
        observer.create_subscription(Float64, '/car/segment_progress', progress_cb, 10),
        observer.create_subscription(String, '/car/recovery_state', state_cb, 10),
        observer.create_subscription(Bool, '/car/safety_enabled', safety_cb, 10),
    ]
    enable_pub = observer.create_publisher(Bool, '/car/enable', 10)
    trajectory_pub = observer.create_publisher(Bool, '/car/trajectory_enable', 10)
    disturbance_pub = observer.create_publisher(Pose2D, '/car/sim_disturbance', 10)

    def record():
        pose = latest['pose']
        cmd = latest['cmd']
        if pose is None:
            return
        rows.append({
            'time_s': timestamp(),
            'x_m': pose.pose.position.x,
            'y_m': pose.pose.position.y,
            'cross_track_m': latest['cross'],
            'segment_progress': latest['progress'],
            'state': latest['state'],
            'cmd_vx_mps': cmd.linear.x,
            'cmd_vy_mps': cmd.linear.y,
            'cmd_wz_radps': cmd.angular.z,
            'safety_enabled': latest['safety'],
        })

    next_record_at = time.monotonic()

    def record_if_due():
        nonlocal next_record_at
        now = time.monotonic()
        if now < next_record_at:
            return
        record()
        next_record_at = max(next_record_at + 0.02, now)

    def spin_for(seconds, record_rows=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
            if record_rows:
                record_if_due()

    result = {
        'success': False,
        'simulated_poses': True,
        'real_vicon_used': False,
        'hardware_bridge_loaded': False,
        'physical_motor_commands_sent': False,
        'ros_domain_id': args.ros_domain_id,
        'disturbance_y_m': args.disturbance_y_m,
    }
    try:
        spin_for(1.0, record_rows=False)
        names = observer.get_node_names()
        assert not any(
            'udp_bridge' in name or 'vicon_client' in name
            or 'rectangle_run_guard' in name for name in names), names
        assert bridge.last_valid_ns is not None

        enable_pub.publish(Bool(data=True))
        spin_for(0.25, record_rows=False)
        assert watchdog.enabled
        trajectory_pub.publish(Bool(data=True))

        start_deadline = time.monotonic() + 8.0
        while time.monotonic() < start_deadline:
            executor.spin_once(timeout_sec=0.005)
            record_if_due()
            if (
                    trajectory.waypoint_index == 1
                    and simulator.x >= 0.35
                    and latest['state'] == TRACKING):
                break
        else:
            raise AssertionError('car did not reach the first disturbance point')

        disturbance_time = timestamp()
        disturbance_pub.publish(Pose2D(x=0.0, y=args.disturbance_y_m, theta=0.0))
        recovery_seen = False
        recovered_time = None
        recovery_deadline = time.monotonic() + 7.0
        while time.monotonic() < recovery_deadline:
            executor.spin_once(timeout_sec=0.005)
            record_if_due()
            recovery_seen |= latest['state'] == RECOVERY
            if (
                    recovery_seen
                    and latest['state'] == TRACKING
                    and latest['cross'] is not None
                    and abs(latest['cross']) <= 0.03):
                recovered_time = timestamp()
                break
        assert recovery_seen, 'RECOVERY state was never observed'
        assert recovered_time is not None, 'cross-track error did not settle'

        # Removing the simulator emulates a complete Vicon pose outage.  The
        # controller and watchdog must latch zero; pose restoration alone must
        # not resume the active trajectory.
        executor.remove_node(simulator)
        spin_for(0.45)
        assert controller.state == VICON_LOST
        assert not watchdog.enabled
        executor.add_node(simulator)
        spin_for(0.35)
        assert controller.state == VICON_LOST
        assert not watchdog.enabled
        assert all(abs(value) < 1e-9 for value in (
            latest['cmd'].linear.x, latest['cmd'].linear.y,
            latest['cmd'].angular.z))

        trajectory_pub.publish(Bool(data=False))
        enable_pub.publish(Bool(data=False))
        spin_for(0.20)

        post_disturbance = [
            row for row in rows
            if row['time_s'] >= disturbance_time and row['cross_track_m'] is not None]
        command_magnitudes = [
            math.hypot(row['cmd_vx_mps'], row['cmd_vy_mps']) for row in rows]
        peak_cross = max(abs(row['cross_track_m']) for row in post_disturbance)
        final_cross = abs(next(
            row['cross_track_m'] for row in reversed(post_disturbance)
            if row['state'] == TRACKING))
        assert peak_cross >= args.disturbance_y_m - 0.03
        assert final_cross <= 0.03
        assert max(command_magnitudes) <= 0.1005

        result.update({
            'success': True,
            'recovery': {
                'disturbance_time_s': disturbance_time,
                'recovered_time_s': recovered_time,
                'recovery_duration_s': recovered_time - disturbance_time,
                'peak_cross_track_m': peak_cross,
                'settled_cross_track_m': final_cross,
                'recovery_state_seen': recovery_seen,
                'state_transitions': states,
            },
            'limits': {
                'max_commanded_translation_mps': max(command_magnitudes),
                'configured_limit_mps': 0.10,
            },
            'lost_pose_test': {
                'controller_state': VICON_LOST,
                'zero_output_latched': True,
                'pose_restore_does_not_reenable': True,
            },
            'sample_count': len(rows),
        })
        with output.with_suffix('.csv').open('w', newline='') as file:
            fieldnames = list(rows[0])
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
    except Exception as error:
        result['error'] = str(error)
        result['state_transitions'] = states
        result['sample_count'] = len(rows)
        output.write_text(json.dumps(result, indent=2) + '\n')
        raise
    finally:
        try:
            trajectory_pub.publish(Bool(data=False))
            enable_pub.publish(Bool(data=False))
            spin_for(0.10, record_rows=False)
        except Exception:
            pass
        for node in nodes:
            try:
                executor.remove_node(node)
                node.destroy_node()
            except Exception:
                pass
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
