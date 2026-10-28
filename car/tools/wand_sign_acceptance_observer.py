#!/usr/bin/env python3
"""Subscribe-only recorder for Sign acceptance in domain94; explicit mode label.

No publishers, service clients, transport, key generation, or synthetic poses.
Metrics are observations, not an automatic claim of hardware acceptance.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

from mecanum_wand_control.safety_nodes import LATEST, SENSOR, decode_pose
from mecanum_wand_control.safety_ros_smoke import audit_domain_zero

NS = '/wand_acceptance'


class Observer(Node):
    def __init__(self, output, mode='shadow'):
        super().__init__('sign_acceptance_readonly_observer', namespace=NS)
        self.output = output
        self.mode = mode
        self.log = (output/'events.jsonl').open('x')
        self.values, self.counts, self.states = {}, Counter(), Counter()
        self.started = time.monotonic()
        self.max_speed = self.max_feedback_speed = 0.
        self.max_abs_yaw_rate = 0.
        self.last_transition = None
        self.transitions = []
        self.running = True
        for name in ('supervisor_status', 'gate_status', 'source_status', 'plant_feedback',
                     'operator_lease', 'candidate'):
            self.create_subscription(String, name,
                lambda msg, name=name: self.receive(name, json.loads(msg.data)), LATEST)
        for name in ('wand_pose', 'car_pose'):
            self.create_subscription(PoseStamped, name,
                lambda msg, name=name: self.pose(name, msg), SENSOR)
        self.create_subscription(Twist, 'cmd_vel', self.command, LATEST)
        if mode == 'hardware':
            for name in ('transport_status', '/car/mc_link_diagnostics'):
                self.create_subscription(String, name,
                    lambda msg, name=name: self.receive(name, json.loads(msg.data)), LATEST)
        self.create_timer(1., self.snapshot)

    def receive(self, name, value):
        now = time.monotonic()
        self.counts[name] += 1
        self.values[name] = dict(received_steady=now, value=value)
        self.log.write(json.dumps(dict(topic=name, received_steady=now, value=value),
                                  allow_nan=False)+'\n')
        if name == 'supervisor_status':
            self.states[value['state']] += 1
            transition = (value['state'], value['reason'], value['epoch'])
            if transition != self.last_transition:
                row = dict(steady=now, state=value['state'], reason=value['reason'],
                           epoch=value['epoch'], stop_confirmed=value['stop_confirmed'],
                           hard_fault=value['hard_fault'])
                self.transitions.append(row)
                self.last_transition = transition
                print(json.dumps(row), flush=True)
            # Mapping lag and speed-governed reference lag are distinct.
            anchors = value.get('anchors')
            goal = value.get('mapped_goal') or value.get('goal')
            wand = self.values.get('wand_pose', {}).get('value')
            if value['state'] == 'ACTIVE' and anchors and goal and wand:
                origin = anchors['wand_origin']
                expected = (anchors['car_origin']['x']+2*(wand['x']-origin['x']),
                            anchors['car_origin']['y']+2*(wand['y']-origin['y']))
                error = math.dist(expected, (goal['x'], goal['y']))
                self.values['mapping_observation'] = dict(received_steady=now,
                    value=dict(raw_expected_xy=expected, filtered_goal_xy=[goal['x'], goal['y']],
                               raw_to_goal_difference_m=error,
                               tracking_reference=value.get('goal'),
                               goal_reference_policy=value.get('goal_reference_policy', 'direct'),
                               goal_limited=bool(value.get('goal_limited'))))
        if name == 'plant_feedback':
            self.max_feedback_speed = max(self.max_feedback_speed, value['speed'])

    def pose(self, name, msg):
        try:
            pose, stamp, roll, pitch = decode_pose(msg, 'map')
            self.receive(name, dict(**asdict(pose), stamp=stamp, roll=roll, pitch=pitch))
        except (ValueError, TypeError) as error:
            self.receive(name+'_invalid', dict(error=str(error)))

    def command(self, msg):
        speed = math.hypot(msg.linear.x, msg.linear.y)
        self.max_speed = max(self.max_speed, speed)
        self.max_abs_yaw_rate = max(self.max_abs_yaw_rate, abs(msg.angular.z))
        self.receive('cmd_vel', dict(vx=msg.linear.x, vy=msg.linear.y, wz=msg.angular.z,
                                     speed_mps=speed))

    def snapshot(self):
        now = time.monotonic()
        graph = dict(nodes=[dict(name=n, namespace=ns)
                           for n, ns in self.get_node_names_and_namespaces()],
                     publishers={name: [dict(name=p.node_name, namespace=p.node_namespace)
                         for p in self.get_publishers_info_by_topic(NS+'/'+name)]
                         for name in ('wand_pose', 'car_pose', 'operator_lease', 'cmd_vel')})
        hardware_present = any(n['name'] == 'wand_hardware_bridge' for n in graph['nodes'])
        report = dict(verification=('REAL Sign / REAL MCU / REAL CAR OBSERVATION' if self.mode == 'hardware'
                                   else 'REAL Sign / SOFTWARE CAR ONLY')+'; SUBSCRIBE-ONLY OBSERVER',
            mode=self.mode,
            elapsed_s=now-self.started, counts=dict(self.counts), state_samples=dict(self.states),
            max_command_speed_mps=self.max_speed, max_feedback_speed_mps=self.max_feedback_speed,
            max_command_abs_yaw_rate_radps=self.max_abs_yaw_rate,
            command_speed_cap_observed=self.max_speed <= .05+1e-9,
            hardware_bridge_present=hardware_present, graph=graph,
            latest=self.values, transitions=self.transitions)
        # Atomic replacement prevents readers seeing partial live JSON.
        temporary = self.output/'latest.json.tmp'
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        temporary.replace(self.output/'latest.json')
        self.log.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('shadow', 'hardware'), default='shadow')
    parser.add_argument('--seconds', type=float, default=900.,
                        help='bounded observation duration; never controls the GUI/car')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 1800: raise ValueError('observer duration must be 1..1800s')
    args.output.mkdir(parents=True, exist_ok=False)
    before = audit_domain_zero()
    (args.output/'domain0_before.json').write_text(json.dumps(before, indent=2)+'\n')
    os.environ['ROS_DOMAIN_ID'] = '94'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    rclpy.init(domain_id=94)
    node = Observer(args.output, args.mode)
    def stop(*_): node.running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while node.running and rclpy.ok() and time.monotonic()-node.started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.snapshot()
        node.log.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    after = audit_domain_zero()
    (args.output/'domain0_after.json').write_text(json.dumps(after, indent=2)+'\n')


if __name__ == '__main__': main()
