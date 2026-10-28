"""Launch and verify the actual ROS simulation in isolated domain 98."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import Pose
from .ros_nodes import pose_message


class Probe(Node):
    def __init__(self):
        super().__init__('wand_sim_probe', namespace='/wand_sim')
        self.pub = self.create_publisher(PoseStamped, 'wand_pose', 10)
        self.impulse = self.create_publisher(Pose2D, 'disturbance', 10)
        self.car = None
        self.status = {}
        self.commands = []
        self.create_subscription(PoseStamped, 'car_pose', self.on_car, qos_profile_sensor_data)
        self.create_subscription(String, 'status', self.on_status, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.service_clients = {name: self.create_client(Trigger, name)
                        for name in ('engage', 'pause', 'stop')}
        self.local = (0.0, 0.0)
        self.hand_origin = (-.4, -.8)
        self.yaw = math.pi/2
        self.publish_hand = True
        self.frozen_stamp = None
        self.last_publish = 0.0

    def on_car(self, msg):
        self.car = msg

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def on_command(self, msg):
        self.commands.append((time.monotonic(), math.hypot(msg.linear.x, msg.linear.y),
                              abs(msg.angular.z)))

    def pump(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            now = time.monotonic()
            if self.publish_hand and now-self.last_publish >= .019:
                f, l = self.local
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                self.pub.publish(pose_message(Pose(
                    self.hand_origin[0]+c*f-s*l, self.hand_origin[1]+s*f+c*l, 1.0),
                    self.frozen_stamp or self.get_clock().now().to_msg()))
                self.last_publish = now
            rclpy.spin_once(self, timeout_sec=.005)

    def wait(self, predicate, timeout=10):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.pump(.025)
            if predicate():
                return
        raise AssertionError(f'ROS wait timed out; status={self.status}')

    def call(self, name):
        self.wait(lambda: self.service_clients[name].service_is_ready())
        future = self.service_clients[name].call_async(Trigger.Request())
        self.wait(future.done, 3)
        result = future.result()
        assert result.success, result.message
        self.pump(.10)
        return result.message

    def ramp(self, target, duration=2):
        old, started = self.local, time.monotonic()
        while time.monotonic()-started < duration:
            f = min(1, (time.monotonic()-started)/duration)
            self.local = tuple(a+f*(b-a) for a, b in zip(old, target))
            self.pump(.02)
        self.local = target
        self.pump(.10)

    def position(self):
        return (self.car.pose.position.x, self.car.pose.position.y)

    def settle(self, target, origin, timeout=13):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        expected = (origin[0]+2*(c*target[0]-s*target[1]),
                    origin[1]+2*(s*target[0]+c*target[1]))
        self.wait(lambda: math.dist(self.position(), expected) < .024, timeout)
        self.pump(.5)
        assert self.status['state'] == 'ACTIVE', self.status
        return math.dist(self.position(), expected)


def verify(probe):
    p = probe
    p.wait(lambda: p.car is not None and p.status.get('state') == 'STANDBY', 15)
    assert all(v == 0 and w == 0 for _, v, w in p.commands)
    p.wait(lambda: p.impulse.get_subscription_count() > 0)
    p.impulse.publish(Pose2D(x=1.0, y=-.5, theta=p.yaw))
    p.pump(.5)
    origin = p.position()
    p.call('engage')
    p.wait(lambda: p.status.get('state') == 'ACTIVE')
    assert abs(p.status['anchor_yaw_rad']-p.yaw) < 1e-6

    p.ramp((.5, 0))
    forward_error = p.settle((.5, 0), origin)
    forward_distance = math.dist(p.position(), origin)
    p.ramp((0, 0))
    return_error = p.settle((0, 0), origin)

    p.ramp((.1, .1), duration=1)
    p.call('pause')
    p.pump(.8)
    parked = p.position()
    mark = len(p.commands)
    p.hand_origin = (-1.0, .4)
    p.local = (0, 0)
    p.pump(.25)
    assert p.status['state'] == 'PAUSED'
    assert all(v == 0 and w == 0 for _, v, w in p.commands[mark:])
    p.call('engage')
    p.pump(.3)
    reengage_drift = math.dist(p.position(), parked)
    assert reengage_drift < .005

    mark = len(p.commands)
    p.publish_hand = False
    lost_started = time.monotonic()
    p.wait(lambda: p.status.get('state') == 'FAULT', .6)
    timeout_latency = time.monotonic()-lost_started
    assert p.commands[-1][1:] == (0, 0)
    p.publish_hand = True
    p.pump(.3)
    assert p.status['state'] == 'FAULT', 'data recovery must not re-enable motion'
    p.call('engage')
    p.frozen_stamp = p.get_clock().now().to_msg()
    p.wait(lambda: p.status.get('state') == 'FAULT', .7)
    p.frozen_stamp = None
    p.pump(.3)
    assert p.status['state'] == 'FAULT'
    p.call('engage')
    bad = pose_message(Pose(*p.hand_origin, 1.0), p.get_clock().now().to_msg())
    bad.pose.orientation.w = 0.0
    p.pub.publish(bad)
    p.wait(lambda: p.status.get('state') == 'FAULT', .5)
    p.pump(.2)
    assert p.status['state'] == 'FAULT'
    p.call('stop')
    p.wait(lambda: p.status.get('state') == 'STANDBY')
    topics = sorted(name for name, _ in p.get_topic_names_and_types())
    assert not any(name.startswith(('/car/', '/vicon/')) for name in topics), topics
    publishers = p.get_publishers_info_by_topic('/wand_sim/cmd_vel')
    assert len(publishers) == 1 and publishers[0].node_name == 'wand_sim_controller'
    max_speed = max(v for _, v, _ in p.commands)
    assert max_speed <= .15+1e-9
    return {
        'passed': True, 'ros_domain_id': 98, 'scale': 2.0,
        'initial_yaw_deg': 90, 'forward_distance_m': forward_distance,
        'forward_settled_error_m': forward_error, 'return_error_m': return_error,
        'pause_reengage_drift_m': reengage_drift, 'wand_loss_latch_latency_s': timeout_latency,
        'max_command_speed_mps': max_speed,
        'fault_checks': ['wand stream absent', 'repeated old source timestamp',
                         'invalid quaternion', 'no automatic resume'],
        'topics': topics, 'final_state': p.status['state'],
        'verification': 'ROS SOFTWARE SIMULATION ONLY; physical car was not connected',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='wand_ros_results')
    args = parser.parse_args()
    destination = Path(args.output).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = '98'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    report = {'passed': False}
    process = probe = None
    with (destination/'launch.log').open('w') as log:
        try:
            rclpy.init(domain_id=98)
            probe = Probe()
            process = subprocess.Popen([
                'ros2', 'launch', 'mecanum_wand_control', 'wand_sim.launch.py',
                'ros_domain_id:=98', 'scripted:=false', 'start_rviz:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            report = verify(probe)
        except Exception as exc:
            report['error'] = repr(exc)
        finally:
            if probe is not None:
                probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process is not None and process.poll() is None:
                # Let launch forward SIGINT once. Signalling the whole group
                # also interrupts children a second time during ROS cleanup.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
    (destination/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
