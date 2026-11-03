#!/usr/bin/env python3
"""Subscribe-only evidence recorder for real-hand and physical-occlusion tests."""

import argparse
import csv
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self, folder):
        super().__init__('sign_live_sim_readonly_monitor', namespace='/wand_sim')
        self.folder = folder
        folder.mkdir(parents=True, exist_ok=False)
        self.status, self.source, self.car, self.goal, self.hand = {}, {}, None, None, None
        self.speed, self.yaw_rate = 0.0, 0.0
        self.started = time.monotonic()
        self.max_speed = 0.0
        self.events, self.source_events = [], []
        self.last_state, self.last_source_state = None, None
        self.samples = 0
        self.create_subscription(String, 'status', self.on_status, 10)
        self.create_subscription(String, 'source_status', self.on_source, 10)
        self.create_subscription(PoseStamped, 'car_pose', self.on_car, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)
        self.create_subscription(PoseStamped, 'wand_pose', self.on_hand, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.stream = (folder/'samples.csv').open('x', buffering=65536)
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['elapsed_s', 'state', 'reason', 'real_wand_fresh', 'synthetic_occlusion',
            'hand_x', 'hand_y', 'hand_z', 'virtual_x', 'virtual_y', 'goal_x', 'goal_y',
            'goal_error_m', 'speed_mps', 'yaw_rate_radps'])
        self.create_timer(.02, self.record)
        self.create_timer(5, self.report)

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def on_source(self, msg):
        self.source = json.loads(msg.data)

    def on_car(self, msg):
        self.car = msg

    def on_goal(self, msg):
        self.goal = msg

    def on_hand(self, msg):
        self.hand = msg

    def on_command(self, msg):
        self.speed = math.hypot(msg.linear.x, msg.linear.y)
        self.yaw_rate = abs(msg.angular.z)
        self.max_speed = max(self.max_speed, self.speed)

    def record(self):
        elapsed = time.monotonic()-self.started
        state = self.status.get('state', 'WAITING')
        if state != self.last_state:
            self.events.append({'elapsed_s': elapsed, 'state': state,
                                'reason': self.status.get('reason', '')})
            self.last_state = state
        source_state = (self.source.get('real_wand_fresh'), self.source.get('synthetic_occlusion'))
        if source_state != self.last_source_state:
            self.source_events.append({'elapsed_s': elapsed, 'real_wand_fresh': source_state[0],
                                       'synthetic_occlusion': source_state[1]})
            self.last_source_state = source_state
        c = self.car.pose.position if self.car else None
        g = self.goal.pose.position if self.goal else None
        h = self.hand.pose.position if self.hand else None
        error = math.hypot(c.x-g.x, c.y-g.y) if c and g else ''
        self.writer.writerow([elapsed, state, self.status.get('reason', ''),
            self.source.get('real_wand_fresh', ''), self.source.get('synthetic_occlusion', ''),
            h.x if h else '', h.y if h else '', h.z if h else '',
            c.x if c else '', c.y if c else '', g.x if g else '', g.y if g else '',
            error, self.speed, self.yaw_rate])
        self.samples += 1

    def report(self):
        self.stream.flush()
        report = {'verification': 'SUBSCRIBE ONLY; REAL SIGN / VIRTUAL CAR',
                  'elapsed_s': time.monotonic()-self.started, 'samples': self.samples,
                  'max_speed_mps': self.max_speed, 'controller_events': self.events,
                  'source_events': self.source_events, 'last_status': self.status,
                  'last_source': self.source}
        temporary = self.folder/'summary.json.tmp'
        temporary.write_text(json.dumps(report, indent=2)+'\n')
        temporary.replace(self.folder/'summary.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=1200)
    args = parser.parse_args()
    rclpy.init(domain_id=99, signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = Monitor(args.output.resolve())
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        deadline = time.monotonic()+args.duration
        while not stopping and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
    finally:
        node.report()
        node.stream.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
