"""Actual domain118 node/process tests, including SIGSTOP fault injection.

Creates only simulated operator messages. Domain0 inspection is discovery-only.
No hardware launch, sockets, serial ports, firmware tools or real car services.
"""

import argparse
import csv
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import Pose
from .safety_core import Lease
from .safety_nodes import DOMAIN, NAMESPACE, LATEST, SENSOR, pose_message


def audit_domain_zero():
    context = Context()
    rclpy.init(context=context, domain_id=0)
    node = Node('safe_wand_readonly_graph_audit', namespace='/safe_audit', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        end = time.monotonic()+1.5
        while time.monotonic() < end: executor.spin_once(timeout_sec=.02)
        topics = ('/car/cmd_vel', '/car/cmd_vel_raw', '/car/enable', '/car/control_enable',
                  '/car/run_lease', '/car/clear_faults', '/car/trajectory_enable')
        counts = {topic: node.count_publishers(topic) for topic in topics}
        assert not any(counts.values()), counts
        return counts
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


class Probe(Node):
    def __init__(self):
        super().__init__('safe_sim_operator', namespace=NAMESPACE)
        self.wand_pub = self.create_publisher(PoseStamped, 'wand_pose', LATEST)
        self.lease_pub = self.create_publisher(String, 'operator_lease', LATEST)
        self.inject_pub = self.create_publisher(String, 'inject', LATEST)
        self.supervisor, self.gate, self.car, self.feedback = {}, {}, None, {}
        self.commands, self.rows = [], []
        self.events = []
        self.last_state = None
        self.held, self.wand_on, self.lease_on = False, True, True
        self.hand = Pose(-.4, -.4, 1.)
        self.sequence, self.last_publish = 0, -math.inf
        self.source = uuid.uuid4().hex
        self.suspended_pid = None
        self.started = time.monotonic()
        self.create_subscription(String, 'supervisor_status', self.on_supervisor, LATEST)
        self.create_subscription(String, 'gate_status', lambda m: setattr(self, 'gate', json.loads(m.data)), LATEST)
        self.create_subscription(String, 'plant_feedback', lambda m: setattr(self, 'feedback', json.loads(m.data)), LATEST)
        self.create_subscription(PoseStamped, 'car_pose', lambda m: setattr(self, 'car', m), SENSOR)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, LATEST)
        self.service_clients = {k: self.create_client(Trigger, k)
                         for k in ('start', 'pause', 'stop', 'acknowledge', 'estop')}

    def on_supervisor(self, msg):
        self.supervisor = json.loads(msg.data)
        if self.supervisor['state'] != self.last_state:
            self.last_state = self.supervisor['state']
            self.events.append(dict(elapsed_s=time.monotonic()-self.started,
                                    state=self.last_state, reason=self.supervisor['reason']))

    def on_command(self, msg):
        speed = math.hypot(msg.linear.x, msg.linear.y)
        self.commands.append((time.monotonic(), speed, abs(msg.angular.z)))
        p = self.car.pose.position if self.car else None
        self.rows.append(dict(elapsed_s=time.monotonic()-self.started,
            state=self.supervisor.get('state'), reason=self.supervisor.get('reason'),
            gate_fault=self.gate.get('fault'), speed_mps=speed,
            car_x=p.x if p else '', car_y=p.y if p else '', held=self.held,
            stop_confirmed=self.supervisor.get('stop_confirmed')))

    def pump(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now-self.last_publish >= .019:
                stamp = self.get_clock().now()
                if self.wand_on: self.wand_pub.publish(pose_message(self.hand, stamp.to_msg()))
                if self.lease_on:
                    self.sequence += 1
                    lease = Lease(self.source, self.sequence, now, stamp.nanoseconds*1e-9, self.held)
                    self.lease_pub.publish(String(data=json.dumps(asdict(lease))))
                self.last_publish = now
            rclpy.spin_once(self, timeout_sec=.002)

    def wait(self, predicate, timeout=10.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.pump(.02)
            if predicate(): return
        raise AssertionError(f'wait failed: supervisor={self.supervisor}, gate={self.gate}')

    def call(self, name, expected=True):
        client = self.service_clients[name]
        self.wait(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        self.wait(future.done, 3.)
        result = future.result()
        assert result.success is expected, result.message
        return result.message

    def position(self):
        return (self.car.pose.position.x, self.car.pose.position.y)

    def arm(self):
        self.held = False
        self.wait(lambda: self.supervisor.get('ready_to_start'), 8.)
        self.call('start')
        self.wait(lambda: self.supervisor.get('state') == 'ARMED'
                  and self.supervisor.get('gate_ready'))
        self.held = True
        self.wait(lambda: self.supervisor.get('state') == 'ACTIVE')

    def move(self):
        old = self.hand
        for i in range(1, 51):
            self.hand = Pose(old.x+.002*i, old.y, old.z)
            self.pump(.02)
        self.wait(lambda: self.commands[-1][1] > .03, 2.)

    def recover(self):
        self.held = False
        self.wait(lambda: self.supervisor.get('state') == 'FAULT'
                  and self.supervisor.get('stop_confirmed'), 5.)
        self.call('acknowledge')
        self.arm()

    def suspend(self, role):
        pid = self.supervisor['pid'] if role == 'supervisor' else self.gate['pid']
        # PID comes from the exact discovered simulation node, never a broad kill.
        self.suspended_pid = int(pid)
        cmdline = Path(f'/proc/{self.suspended_pid}/cmdline').read_bytes().decode().replace('\0', ' ')
        parent = next(line.split()[1] for line in Path(f'/proc/{self.suspended_pid}/status').read_text().splitlines()
                      if line.startswith('PPid:'))
        assert f'wand_safe_{"supervisor" if role == "supervisor" else "gate"}' in cmdline
        assert int(parent) == self.backend_pid, 'refuse to signal a process outside this test launch'
        os.kill(self.suspended_pid, signal.SIGSTOP)
        return time.monotonic()

    def resume_suspended(self):
        if self.suspended_pid is not None:
            os.kill(self.suspended_pid, signal.SIGCONT)
            self.suspended_pid = None


def functional(probe):
    p = probe
    p.wait(lambda: p.supervisor.get('ready_to_start') and p.car is not None, 15.)
    assert all(v == 0 and w == 0 for _, v, w in p.commands)
    p.held = True
    p.pump(.15)
    p.call('start', expected=False)
    assert p.supervisor['state'] == 'STANDBY'
    p.arm()
    print('ROS functional: explicitly armed', flush=True)
    origin = p.position()
    p.move()
    p.wait(lambda: math.dist(p.position(), (origin[0]+.2, origin[1])) < .024, 8.)
    settled = math.dist(p.position(), (origin[0]+.2, origin[1]))
    assert p.supervisor['state'] == 'ACTIVE'
    print('ROS functional: 2x settled target', flush=True)
    goal = p.supervisor['goal']
    assert math.dist((goal['x'], goal['y']), (origin[0]+.2, origin[1])) < 1e-6
    p.call('pause')
    p.wait(lambda: p.supervisor.get('state') == 'PAUSED' and p.supervisor['stop_confirmed'])
    p.held = False
    p.hand = Pose(.3, .2, 1.)
    parked = p.position()
    p.arm()
    p.pump(.3)
    drift = math.dist(parked, p.position())
    assert drift < .001
    print('ROS functional: pause and rebase', flush=True)
    p.move()
    start = time.monotonic()
    p.wand_on = False
    p.wait(lambda: p.commands[-1][1] == 0 and p.supervisor.get('state') in ('STOPPING', 'FAULT'), .5)
    loss_latency = time.monotonic()-start
    assert loss_latency < .30
    p.wand_on = True
    p.pump(.8)
    assert p.supervisor['state'] == 'FAULT'
    assert p.commands[-1][1:] == (0., 0.)
    p.recover()
    print('ROS functional: occlusion recovery latch', flush=True)
    p.move()
    start = time.monotonic()
    p.lease_on = False
    p.wait(lambda: p.commands[-1][1] == 0, .4)
    lease_latency = time.monotonic()-start
    assert lease_latency < .25
    p.lease_on = True
    p.pump(.3)
    assert p.commands[-1][1:] == (0., 0.)
    p.recover()
    print('ROS functional: operator lease loss', flush=True)
    p.move()
    p.inject_pub.publish(String(data=json.dumps({'feedback_on': False})))
    p.wait(lambda: p.supervisor.get('state') == 'STOP_UNCONFIRMED', 3.)
    assert not p.supervisor['stop_confirmed'] and p.commands[-1][1:] == (0., 0.)
    p.inject_pub.publish(String(data=json.dumps({'feedback_on': True})))
    p.recover()
    print('ROS functional: missing stop feedback remains unconfirmed', flush=True)
    p.move()
    p.inject_pub.publish(String(data=json.dumps({'dz': .3})))
    p.wait(lambda: p.commands[-1][1] == 0, .4)
    p.pump(.4)
    assert p.supervisor['state'] in ('STOPPING', 'FAULT')
    p.inject_pub.publish(String(data=json.dumps({'dz': -.3})))
    p.recover()
    p.move()
    rogue = p.create_publisher(Twist, 'cmd_vel', LATEST)
    try:
        p.pump(.15)
        bad = Twist()
        bad.linear.x = .04
        rogue.publish(bad)
        p.wait(lambda: p.feedback.get('output_zero'), .4)
        p.pump(.3)
        assert p.feedback.get('fault_bits', 0) == 0
    finally:
        p.destroy_publisher(rogue)
    p.pump(.3)
    assert p.feedback['output_zero'] and p.commands[-1][1:] == (0., 0.)
    p.recover()
    p.move()
    forged = p.create_publisher(String, 'actuator_command', LATEST)
    try:
        p.pump(.10)
        forged.publish(String(data='{"payload":{},"signature":"invalid-test-signature"}'))
        p.wait(lambda: p.feedback.get('fault_bits', 0) != 0, .6)
        p.wait(lambda: p.feedback.get('output_zero'), .4)
        p.pump(.3)
        assert p.supervisor['hard_fault']
    finally:
        p.destroy_publisher(forged)
    return dict(case='functional', passed=True, scale=2., settled_error_m=settled,
                rebase_drift_m=drift, wand_loss_zero_latency_s=loss_latency,
                operator_loss_zero_latency_s=lease_latency,
                missing_feedback_stays_unconfirmed=True, lifted_car_stops=True,
                ordinary_twist_cannot_drive_plant=True,
                forged_actuator_envelope_rejected=True)


def process_loss(probe, role):
    p = probe
    p.arm()
    p.move()
    start = p.suspend(role)
    try:
        if role == 'supervisor':
            p.wait(lambda: p.commands[-1][1] == 0, .5)
        else:
            p.wait(lambda: p.feedback.get('output_zero'), .5)
        latency = time.monotonic()-start
        p.pump(.3)
    finally:
        p.resume_suspended()
    p.pump(.4)
    assert p.commands[-1][1:] == (0., 0.)
    assert p.supervisor['state'] != 'ACTIVE'
    return dict(case=role+'_SIGSTOP', passed=True, zero_latency_s=latency,
                no_auto_resume_after_SIGCONT=True,
                stop_measured_at='final gate output' if role == 'supervisor' else 'simulated actuator watchdog')


def run_one(folder, case):
    process = probe = None
    with (folder/(case+'.log')).open('w') as log:
        try:
            rclpy.init(domain_id=DOMAIN)
            probe = Probe()
            process = subprocess.Popen(['ros2', 'launch', 'mecanum_wand_4wsa_control',
                'wand_safe_sim.launch.py'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            probe.backend_pid = process.pid
            result = functional(probe) if case == 'functional' else process_loss(probe, case)
            # Includes the intentionally injected 0.04 m/s foreign frame;
            # plant identity validation must reject it independently.
            result['maximum_speed_mps'] = max(v for _, v, _ in probe.commands)
            assert result['maximum_speed_mps'] <= .05+1e-9
            topics = [t for t, _ in probe.get_topic_names_and_types()]
            assert not any(t.startswith(('/car/', '/vicon/', '/wand_sim/')) for t in topics)
            result['only_gate_publishes_final_output'] = [i.node_name for i in
                probe.get_publishers_info_by_topic(NAMESPACE+'/cmd_vel')] == ['safe_wand_gate']
            assert result['only_gate_publishes_final_output']
            result['events'] = probe.events
            return result
        finally:
            if probe is not None:
                probe.resume_suspended()
                if probe.rows:
                    with (folder/(case+'.csv')).open('w') as stream:
                        writer = csv.DictWriter(stream, fieldnames=list(probe.rows[0]))
                        writer.writeheader()
                        writer.writerows(probe.rows)
                probe.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(DOMAIN)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    report = dict(passed=False, verification='ACTUAL ROS SOFTWARE NODES ONLY; NO PHYSICAL CAR',
                  ros_domain=DOMAIN, cases=[])
    try:
        report['domain0_before'] = audit_domain_zero()
        for case in ('functional', 'supervisor', 'gate'):
            result = run_one(args.output, case)
            report['cases'].append(result)
            print(f'PASS ROS {case}', flush=True)
        report['domain0_after'] = audit_domain_zero()
        report['passed'] = True
    except Exception as exc:
        report['error'] = repr(exc)
    (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
