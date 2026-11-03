#!/usr/bin/env python3
"""只订阅隔离域中的矩形软件演示；不发布速度、使能或网络指令。"""
import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from mecanum_vicon_control.math_utils import world_error_to_body, wrap_angle, yaw_from_quaternion


def xy_yaw(msg):
    q = msg.pose.orientation
    return [msg.pose.position.x, msg.pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w)]


class Observer(Node):
    def __init__(self, args):
        super().__init__('rectangle_sim_observer')
        self.args = args
        self.started = time.monotonic()
        self.points = None
        self.pose = None
        self.last_index = None
        self.last_sample = 0.0
        self.error = None
        self.record = dict(mode='software_in_loop_only', domain_id=os.getenv('ROS_DOMAIN_ID'),
                           completed_loops=0, pose_frames=0, command_frames=0,
                           safety_enabled_seen=False, max_abs_wz=0.0,
                           direction_frames=dict(forward=0, left=0, backward=0, right=0),
                           goals=[], poses=[], closures=[])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(RosPath, '/car/trajectory_path', self.on_path, qos)
        self.create_subscription(PoseStamped, '/car/goal_pose', self.on_goal, qos)
        self.create_subscription(PoseStamped, '/car/pose', self.on_pose, 10)
        self.create_subscription(Twist, '/car/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, '/car/safety_enabled', self.on_safety, 10)

    def on_path(self, msg):
        if not msg.poses:
            return
        if msg.header.frame_id != 'map' or len(msg.poses) != 9:
            self.error = 'expected a closed 8-waypoint map rectangle'
            return
        self.points = [xy_yaw(p) for p in msg.poses]
        x, y, yaw = self.points[0]
        expected = [(0, 0), (self.args.forward_m, 0),
                    (self.args.forward_m, self.args.left_m), (0, self.args.left_m), (0, 0)]
        for actual, target in zip(self.points[::2], expected):
            local = world_error_to_body(actual[0] - x, actual[1] - y, yaw)
            if math.dist(local, target) > 1e-6:
                self.error = 'planned corners do not match forward/left/back/right dimensions'
        if any(abs(wrap_angle(p[2] - yaw)) > 1e-6 for p in self.points):
            self.error = 'planned car heading is not constant'
        self.record['path'] = self.points

    def on_goal(self, msg):
        if self.points is None:
            self.error = 'goal received without an anchored path'
            return
        goal = xy_yaw(msg)
        distances = [math.dist(goal[:2], p[:2]) for p in self.points]
        index = min(range(len(distances)), key=distances.__getitem__)
        if distances[index] > 0.02:
            self.error = 'goal lies outside planned waypoints'
            return
        # 路径末点与起点相同，按推进顺序将其识别为第8点。
        if self.last_index == 7 and index == 0:
            index = 8
        if index == self.last_index:
            return
        expected = 0 if self.last_index is None else (1 if self.last_index == 8 else self.last_index + 1)
        if index != expected:
            self.error = f'waypoint sequence skipped: expected {expected}, got {index}'
            return
        if self.last_index == 8 and index == 1:
            if self.pose is None:
                self.error = 'loop completion has no actual simulated pose'
                return
            closure_error = math.dist(self.pose[:2], self.points[0][:2])
            if closure_error > 0.035:
                self.error = f'loop closure error too large: {closure_error}'
                return
            self.record['completed_loops'] += 1
            self.record['closures'].append(dict(time_s=time.monotonic() - self.started,
                                               error_m=closure_error, pose=self.pose))
        self.last_index = index
        self.record['goals'].append(dict(time_s=time.monotonic() - self.started, index=index, pose=goal))
        print(f'goal {index}: x={goal[0]:.3f}, y={goal[1]:.3f}, yaw={goal[2]:.3f}', flush=True)

    def on_pose(self, msg):
        self.pose = xy_yaw(msg)
        self.record['pose_frames'] += 1
        elapsed = time.monotonic() - self.started
        if elapsed - self.last_sample >= 0.05:
            self.last_sample = elapsed
            self.record['poses'].append(dict(time_s=elapsed, pose=self.pose))

    def on_cmd(self, msg):
        self.record['command_frames'] += 1
        self.record['max_abs_wz'] = max(self.record['max_abs_wz'], abs(msg.angular.z))
        x, y = msg.linear.x, msg.linear.y
        if abs(x) > 0.04 and abs(x) > 5 * abs(y):
            self.record['direction_frames']['forward' if x > 0 else 'backward'] += 1
        if abs(y) > 0.04 and abs(y) > 5 * abs(x):
            self.record['direction_frames']['left' if y > 0 else 'right'] += 1

    def on_safety(self, msg):
        if self.record['safety_enabled_seen'] and not msg.data:
            self.error = 'simulation watchdog latched; do not auto-resume'
        self.record['safety_enabled_seen'] |= msg.data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--forward-m', type=float, default=6.0)
    parser.add_argument('--left-m', type=float, default=5.0)
    parser.add_argument('--timeout-s', type=float, default=240.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if os.getenv('ROS_DOMAIN_ID') != '72':
        parser.error('run this simulation observer in isolated ROS_DOMAIN_ID=72')
    rclpy.init()
    node = Observer(args)
    last_graph_check = 0.0
    print('simulation observer ready; no command publishers', flush=True)
    try:
        while time.monotonic() - node.started < args.timeout_s:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last_graph_check >= 1.0:
                last_graph_check = time.monotonic()
                if any('cmd_vel_udp_bridge' in name for name, _namespace in node.get_node_names_and_namespaces()):
                    raise RuntimeError('hardware UDP bridge found in simulation domain')
            if node.error:
                raise RuntimeError(node.error)
            if node.record['completed_loops'] >= 1:
                if not node.record['safety_enabled_seen'] or not all(node.record['direction_frames'].values()):
                    raise RuntimeError('missing safety enable or one of the four movement directions')
                if node.record['max_abs_wz'] > 0.01:
                    raise RuntimeError('car turned instead of keeping constant heading')
                node.record['success'] = True
                break
        else:
            raise RuntimeError('rectangle simulation did not complete one loop in time')
    except BaseException as error:
        node.record['success'] = False
        node.record['error'] = str(error)
        raise
    finally:
        node.record['elapsed_s'] = time.monotonic() - node.started
        Path(args.output).write_text(json.dumps(node.record, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({k: v for k, v in node.record.items() if k not in ('poses', 'goals', 'path')},
                         ensure_ascii=False), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
