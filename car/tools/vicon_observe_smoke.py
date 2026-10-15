#!/usr/bin/env python3
"""隔离ROS域的纯观测验收：仅发布合成位姿，不访问真实Vicon、串口或电机。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


FORBIDDEN_TOPICS = ('/car/cmd_vel', '/car/cmd_vel_raw', '/car/enable',
                    '/car/control_enable', '/car/estop', '/car/clear_faults',
                    '/car/safety_enabled', '/car/trajectory_enable')


class Probe(Node):
    def __init__(self):
        super().__init__('vicon_observe_synthetic_probe')
        self.counts = dict(pose=0, odometry=0, path=0)
        self.valid = None
        self.last_pose = None
        self.path_points = 0
        self.parents = {}
        self.source = self.create_publisher(PoseStamped, '/vicon_test/car01/base', 10)
        self.create_subscription(PoseStamped, '/car/pose', self.on_pose, 10)
        self.create_subscription(Odometry, '/car/odometry', lambda msg: self.increment('odometry'), 10)
        self.create_subscription(RosPath, '/car/path_actual', self.on_path, 10)
        self.create_subscription(Bool, '/car/vicon_valid', lambda msg: setattr(self, 'valid', msg.data), 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 10)
        static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf, static_qos)

    def increment(self, key):
        self.counts[key] += 1

    def on_pose(self, msg):
        self.increment('pose')
        self.last_pose = msg

    def on_path(self, msg):
        self.increment('path')
        self.path_points = len(msg.poses)

    def on_tf(self, msg):
        for transform in msg.transforms:
            self.parents.setdefault(transform.child_frame_id, set()).add(transform.header.frame_id)

    def idle(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.01)

    def stream(self, kind, seconds):
        started = time.monotonic()
        index = 0
        while time.monotonic() - started < seconds:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = 0.002 * index if kind == 'valid' else 999.0
            msg.pose.position.y = 0.01 * math.sin(index * 0.05)
            msg.pose.orientation.z = math.sin(0.15)
            msg.pose.orientation.w = math.cos(0.15)
            if kind == 'wrong_frame':
                msg.header.frame_id = 'vicon'
            elif kind == 'zero_quaternion':
                msg.pose.orientation.z = msg.pose.orientation.w = 0.0
            elif kind == 'nonfinite':
                msg.pose.position.x = math.nan
            elif kind == 'stale':
                msg.header.stamp.sec -= 1
            self.source.publish(msg)
            index += 1
            self.idle(0.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if os.getenv('ROS_DOMAIN_ID') != '72' or os.getenv('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('requires ROS_DOMAIN_ID=72 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    output = Path(args.output)
    if output.exists():
        parser.error('refusing to overwrite existing evidence')
    record = dict(source='synthetic_ros_stream_not_real_vicon', domain_id=72,
                  motor_commands_sent=False, serial_opened=False, tests={})
    rclpy.init()
    node = Probe()
    process = None
    try:
        node.idle(1.0)
        if any(node.count_publishers(topic) for topic in FORBIDDEN_TOPICS):
            raise RuntimeError('isolated domain contains command/enable publishers')
        process = subprocess.Popen([
            'ros2', 'launch', 'mecanum_vicon_control', 'vicon_observe.launch.py',
            'vicon_pose_topic:=/vicon_test/car01/base', 'start_rviz:=false',
        ], start_new_session=True)
        deadline = time.monotonic() + 15.0
        while node.source.get_subscription_count() == 0 and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('observation launch exited during startup')
            node.idle(0.05)
        if node.source.get_subscription_count() == 0:
            raise RuntimeError('no observation input subscriber discovered')
        node.idle(1.0)
        node.stream('valid', 2.0)
        node.idle(0.08)
        record['tests']['valid_stream'] = (node.counts['pose'] >= 10 and
                                          node.counts['odometry'] >= 10 and node.path_points >= 5)
        for kind in ('wrong_frame', 'zero_quaternion', 'nonfinite', 'stale'):
            before = node.counts['pose']
            node.stream(kind, 0.35)
            node.idle(0.08)
            record['tests'][kind + '_rejected'] = node.counts['pose'] == before and node.valid is False
        node.stream('valid', 0.8)
        node.idle(0.4)
        record['tests']['outage_invalid'] = node.valid is False
        before = node.counts['pose']
        node.stream('valid', 0.8)
        node.idle(0.08)
        record['tests']['fresh_stream_recovered'] = node.counts['pose'] > before and node.valid is True
        record['forbidden_publishers'] = {topic: node.count_publishers(topic) for topic in FORBIDDEN_TOPICS}
        record['tests']['no_control_publishers'] = not any(record['forbidden_publishers'].values())
        record['tf_parents'] = {child: sorted(parents) for child, parents in node.parents.items()}
        record['tests']['single_parent_tf'] = (node.parents.get('base_footprint') == {'map'} and
                                              node.parents.get('base_link') == {'base_footprint'})
        record['frames'] = node.counts
        record['actual_path_points'] = node.path_points
        record['passed'] = all(record['tests'].values())
    except Exception as error:
        record['passed'] = False
        record['error'] = str(error)
    finally:
        # 先只通知本脚本创建的launch父进程，由它向子节点传递一次退出信号。
        # 直接向整组发送SIGINT会让launch再次转发，打断子节点的资源清理。
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.0)
        if process is not None:
            record['launch_returncode'] = process.returncode
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            json.dump(record, stream, ensure_ascii=False, indent=2)
        print(json.dumps(record, ensure_ascii=False, indent=2))
    return 0 if record['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
