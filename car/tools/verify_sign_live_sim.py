#!/usr/bin/env python3
"""Verify live Sign simulation using virtual disturbances and relay-only loss.

Only domain99 simulation services/publishers are used. Domain0 is audited by
subscription/discovery only. Deliberate physical occlusion remains a separate
human test; the relay gate here never changes the receiver or the real preview.
"""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class Probe(Node):
    def __init__(self, context):
        super().__init__('live_sign_sim_acceptance', namespace='/wand_sim', context=context)
        self.executor_for_probe = SingleThreadedExecutor(context=context)
        self.executor_for_probe.add_node(self)
        self.status, self.source, self.car, self.goal = {}, {}, None, None
        self.commands = []
        self.events = []
        self.create_subscription(String, 'status', self.on_status, 10)
        self.create_subscription(String, 'source_status', self.on_source, 10)
        self.create_subscription(PoseStamped, 'car_pose', self.on_car, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.impulse_pub = self.create_publisher(Pose2D, 'disturbance', 10)
        self.service_clients = {name: self.create_client(Trigger, name)
                                for name in ('engage', 'pause')}
        self.occlusion_client = self.create_client(SetBool, 'simulate_occlusion')

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        if not self.events or self.events[-1]['state'] != self.status['state']:
            self.events.append({'time_s': time.monotonic(), 'state': self.status['state'],
                                'reason': self.status['reason']})

    def on_source(self, msg):
        self.source = json.loads(msg.data)

    def on_car(self, msg):
        self.car = msg

    def on_goal(self, msg):
        self.goal = msg

    def on_command(self, msg):
        self.commands.append((time.monotonic(), math.hypot(msg.linear.x, msg.linear.y),
                              abs(msg.angular.z)))

    def pump(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.executor_for_probe.spin_once(timeout_sec=.005)

    def wait(self, predicate, seconds=10):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.pump(.015)
            if predicate():
                return
        raise AssertionError(f'timeout: status={self.status}, source={self.source}')

    def call(self, name, blocked=None):
        client = self.occlusion_client if name == 'simulate_occlusion' else self.service_clients[name]
        self.wait(client.service_is_ready)
        request = SetBool.Request(data=blocked) if blocked is not None else Trigger.Request()
        future = client.call_async(request)
        self.wait(future.done, 3)
        result = future.result()
        assert result.success, result.message
        self.pump(.06)

    def position(self):
        return (self.car.pose.position.x, self.car.pose.position.y)

    def goal_error(self):
        if self.car is None or self.goal is None:
            return math.inf
        return math.dist(self.position(), (self.goal.pose.position.x, self.goal.pose.position.y))

    def disturb(self, x, y):
        self.impulse_pub.publish(Pose2D(x=float(x), y=float(y), theta=0.0))
        self.pump(.1)


def audit_real_domain():
    context = Context()
    rclpy.init(context=context, domain_id=0)
    node = Node('live_sign_sim_real_domain_audit', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.monotonic()+2
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.02)
    topics = ['/car/cmd_vel', '/car/cmd_vel_raw', '/car/enable', '/car/control_enable',
              '/car/run_lease', '/car/clear_faults', '/car/trajectory_enable']
    counts = {topic: node.count_publishers(topic) for topic in topics}
    source_publishers = {topic: [info.node_name for info in node.get_publishers_info_by_topic(topic)
                                if info.node_name == 'sign_sim_source']
                         for topic, _ in node.get_topic_names_and_types() if topic.startswith('/car/')}
    executor.shutdown()
    node.destroy_node()
    context.shutdown()
    assert not any(counts.values()), counts
    assert not any(source_publishers.values()), source_publishers
    return counts


def verify(probe):
    p = probe
    p.wait(lambda: p.status.get('state') in ('ACTIVE', 'PAUSED', 'FAULT') and p.car is not None
           and p.source.get('seeded') and p.source.get('real_wand_fresh'), 15)
    if p.status['state'] != 'ACTIVE':
        p.call('engage')
        p.wait(lambda: p.status.get('state') == 'ACTIVE')
    assert not p.source.get('synthetic_occlusion')
    start_position = p.position()
    p.disturb(.35, .25)
    p.wait(lambda: p.commands[-1][1] > .12)
    p.wait(lambda: p.goal_error() < .025, 10)
    p.pump(.5)
    settled_error = p.goal_error()

    p.disturb(.30, -.20)
    p.wait(lambda: p.commands[-1][1] > .05)
    p.call('pause')
    p.wait(lambda: p.status.get('state') == 'PAUSED')
    p.pump(.8)
    paused_position, mark = p.position(), len(p.commands)
    p.pump(.4)
    assert all(v == 0 and w == 0 for _, v, w in p.commands[mark:])
    p.call('engage')
    p.pump(.3)
    pause_resume_drift = math.dist(p.position(), paused_position)
    assert pause_resume_drift < .005

    p.disturb(.25, .20)
    p.wait(lambda: p.commands[-1][1] > .06)
    lost_started = time.monotonic()
    p.call('simulate_occlusion', True)
    p.wait(lambda: p.status.get('state') == 'FAULT', .5)
    loss_latency = time.monotonic()-lost_started
    assert p.commands[-1][1:] == (0.0, 0.0)
    assert p.source.get('real_wand_fresh'), 'expected a relay-only loss, not a real source outage'
    assert loss_latency < .30
    p.call('simulate_occlusion', False)
    p.pump(.6)
    assert p.status['state'] == 'FAULT'
    parked = p.position()
    p.call('engage')
    p.pump(.3)
    loss_resume_drift = math.dist(p.position(), parked)
    assert loss_resume_drift < .005

    # Return the virtual plant to the unpowered physical car's initial reference.
    p.call('pause')
    p.pump(.5)
    current = p.position()
    p.disturb(start_position[0]-current[0], start_position[1]-current[1])
    p.pump(.3)
    p.call('engage')
    p.wait(lambda: p.status.get('state') == 'ACTIVE')
    max_speed = max(v for _, v, _ in p.commands)
    assert max_speed <= .15+1e-9
    assert abs(p.status['scale']-2) < 1e-9
    publishers = p.get_publishers_info_by_topic('/wand_sim/cmd_vel')
    assert len(publishers) == 1 and publishers[0].node_name == 'wand_sim_controller'
    return {
        'passed': True, 'verification': 'REAL SIGN TO SOFTWARE CAR; no physical car motion',
        'input_source': '/vicon/Sign/Sign in domain0', 'simulation_domain': 99,
        'scale': p.status['scale'], 'max_command_speed_mps': max_speed,
        'virtual_disturbance_recovery_error_m': settled_error,
        'pause_resume_position_drift_m': pause_resume_drift,
        'relay_input_loss_stop_latency_s': loss_latency,
        'relay_recovery_does_not_auto_resume': True,
        'explicit_reengage_drift_m': loss_resume_drift,
        'source_forwarded_frames': p.source.get('forwarded_frames'),
        'controller_states': p.events, 'final_state': p.status['state'],
        'physical_occlusion_test': 'not performed by this tool; requires covering the real wand',
        'virtual_car_reset_to_reference': True,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    context = Context()
    rclpy.init(context=context, domain_id=99)
    probe = Probe(context)
    report = {'passed': False}
    try:
        real_counts = audit_real_domain()
        report = verify(probe)
        report['hardware_publishers_in_real_domain'] = real_counts
        assert not any(audit_real_domain().values())
    except Exception as exc:
        report['error'] = repr(exc)
        if probe.service_clients['pause'].service_is_ready():
            probe.call('pause')
    finally:
        probe.executor_for_probe.shutdown()
        probe.destroy_node()
        context.shutdown()
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
