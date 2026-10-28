"""Actual domain93/94 ROS nodes + loopback MC-Link emulator. NO REAL CAR.

Only 127.0.0.1:54600/54601 are used; real Vicon/domain0 are read-only audited.
The emulator checks framing, session, enable, healthy rearm and wheel telemetry.
It does not validate real firmware timing, RF, battery, friction or braking.
"""

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.context import Context
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from mecanum_vicon_control.bench_link_status import STATUS
from mecanum_vicon_control.mecanum_kinematics import body_to_wheel_angular
from mecanum_vicon_control.transport_protocol import (MessageType as MT, FrameFlags as FF,
    decode_frame, encode_frame, unpack_cmd_vel, unpack_control_enable, WheelTelemetry, pack_wheel_telemetry)
from mecanum_wand_4ws8_control.core import Pose, Velocity
from mecanum_wand_4ws8_control.offline_suite import Plant
from mecanum_wand_4ws8_control.safety_core import Lease, ZERO
from mecanum_wand_4ws8_control.safety_nodes import LATEST, SENSOR, pose_message
from mecanum_wand_4ws8_control.safety_ros_smoke import audit_domain_zero
from .commissioning import DOMAIN, NAMESPACE


class McuEmulator:
    def __init__(self, feedback_period=.04):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 54600))
        self.sock.setblocking(False)
        self.session, self.sequence, self.peer = 0, -1, None
        self.permit, self.zero_seen, self.fault = False, False, 0
        self.command, self.started_ms = ZERO, 0
        self.plant = Plant(Pose(0., 0., .10))
        self.last_command, self.last_status, self.last_tick = -math.inf, -math.inf, time.monotonic()
        self.still_since = time.monotonic()
        self.feedback_on = True
        self.freeze_samples, self.sample_tick = False, 0
        self.received, self.enables, self.rearms, self.estops, self.errors = [], 0, 0, 0, []
        self.watchdog_stops = []
        self.feedback_period = feedback_period
        self.hellos, self.controls = [], []

    def send(self, kind, payload=b''):
        if self.peer:
            tick = self.sample_tick if kind in (MT.LINK_STATUS, MT.WHEEL_TELEMETRY) else int(time.monotonic()*1000) & 0xffffffff
            self.sock.sendto(encode_frame(kind, max(0, self.sequence), self.session,
                tick, payload, FF.RESPONSE), self.peer)

    def receive(self, data, sender):
        assert sender == ('127.0.0.1', 54601), 'emulator refuses non-test endpoint'
        f = decode_frame(data)
        now = time.monotonic()
        if f.message_type == MT.HELLO:
            self.hellos.append((now, f.session_id))
            if self.permit:
                self.errors.append('HELLO while MCU permit still active')
                self.permit, self.command, self.fault = False, ZERO, 1
                return
            self.session, self.sequence, self.peer, self.zero_seen = f.session_id, f.sequence, sender, False
            self.send(MT.HELLO_ACK)
            return
        if f.session_id != self.session or f.sequence <= self.sequence: return
        self.sequence = f.sequence
        if f.message_type == MT.CMD_VEL:
            c = unpack_cmd_vel(f.payload)
            assert c.timeout_ms == 600
            enabled = bool(f.flags & FF.ENABLED)
            command = Velocity(c.vx_mps, c.vy_mps, c.wz_radps)
            assert math.hypot(command.vx, command.vy) <= .051 and abs(command.wz) <= .101
            self.received.append((now, enabled, command.vx, command.vy, command.wz))
            if not enabled:
                assert command == ZERO
                self.permit, self.command, self.zero_seen = False, ZERO, True
            elif not self.permit or self.fault:
                self.errors.append('ENABLED command without MCU permit')
                self.command, self.fault = ZERO, 1
            else:
                self.command = command
                if command != ZERO and not self.started_ms: self.started_ms = max(1, int(now*1000) & 0xffffffff)
            self.last_command = now
        elif f.message_type == MT.CONTROL_ENABLE:
            c = unpack_control_enable(f.payload)
            self.controls.append((now, f.session_id, c.enabled, c.clear_latched_faults))
            if c.clear_latched_faults:
                if (self.permit or self.fault or self.command != ZERO or now-self.still_since < .5
                        or not self.zero_seen or now-self.last_command >= .10):
                    self.errors.append('unsafe rearm request')
                else:
                    self.started_ms = 0
                    self.rearms += 1
            elif c.enabled:
                if self.fault or (not self.permit and (not self.zero_seen or self.started_ms
                        or now-self.last_command >= .10 or now-self.still_since < .5)):
                    self.errors.append('enable without healthy fresh zero/rearm baseline')
                else:
                    self.permit = True
                    self.enables += 1
            else: self.permit, self.command = False, ZERO
        elif f.message_type == MT.ESTOP:
            self.estops += 1
            self.permit, self.command, self.fault = False, ZERO, 1

    def tick(self):
        now = time.monotonic()
        while True:
            try: data, sender = self.sock.recvfrom(1024)
            except BlockingIOError: break
            self.receive(data, sender)
        if self.permit and now-self.last_command > .6:
            self.permit, self.command = False, ZERO
            self.watchdog_stops.append(now)
        self.plant.step(self.command, min(.05, max(0., now-self.last_tick)))
        self.last_tick = now
        if self.command != ZERO or math.hypot(self.plant.velocity.vx, self.plant.velocity.vy) > .003:
            self.still_since = now
        if not self.peer or not self.feedback_on or now-self.last_status < self.feedback_period: return
        self.last_status = now
        if not self.freeze_samples: self.sample_tick = int(now*1000) & 0xffffffff
        w = body_to_wheel_angular(self.command.vx, self.command.vy, self.command.wz, .03, .061, .085).as_tuple()
        targets = [round(v*120) for v in w]
        duty = [(100 if t > 0 else -100 if t < 0 else 0)*s for t, s in zip(targets, (1, -1, 1, -1))]
        cc = [v for d in duty for v in ([100, 100-d//10] if d > 0 else [100-abs(d)//10, 100] if d < 0 else [0, 0])]
        body = [round(v*1000) for v in (self.command.vx, self.command.vy, self.command.wz)]
        status = STATUS.pack(0x42475734, 100, 100, 10, 10, 20, self.started_ms,
            min(500, int((now-self.still_since)*1000)), *cc, *duty, *targets, *targets, *body,
            0, 0, 0, int(self.permit))
        self.send(MT.LINK_STATUS, status)
        flags = (1 << 1 if self.permit else 0) | (1 << 3 if self.command == ZERO else 0)
        self.send(MT.WHEEL_TELEMETRY, pack_wheel_telemetry(WheelTelemetry(
            self.sample_tick, tuple(v*60/(2*math.pi) for v in w), (0, 0, 0, 0),
            12300, flags, self.fault, max(0, self.sequence))))


class Probe(Node):
    def __init__(self, emulator):
        super().__init__('wand_real_operator', namespace=NAMESPACE)
        self.mcu = emulator
        self.raw_context = Context()
        rclpy.init(context=self.raw_context, domain_id=93)
        self.raw = Node('vicon_emulator', context=self.raw_context, use_global_arguments=False)
        self.raw_pubs = {s: self.raw.create_publisher(PoseStamped, t, SENSOR) for s, t in
                         (('wand', '/vicon/Sign/Sign'), ('car', '/vicon/car01/car01'))}
        self.wand, self.wand_on, self.car_z = Pose(-.4, -.4, 1.), True, .10
        self.lease_pub = self.create_publisher(String, 'operator_lease', LATEST)
        self.held, self.lease_on, self.sequence, self.owner = False, True, 0, uuid.uuid4().hex
        self.last_publish = -math.inf
        self.status, self.gate, self.transport, self.source = {}, {}, {}, {}
        self.commands, self.events = [], []
        for topic, attr in (('supervisor_status', 'status'), ('gate_status', 'gate'),
                            ('transport_status', 'transport'), ('source_status', 'source')):
            self.create_subscription(String, topic, lambda m, attr=attr: self.receive(attr, m), LATEST)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self.commands.append(
            (time.monotonic(), m.linear.x, m.linear.y, m.angular.z)), LATEST)
        self.service_clients = {n: self.create_client(Trigger, n) for n in ('start', 'pause', 'stop', 'acknowledge', 'estop')}

    def receive(self, attr, msg):
        value = json.loads(msg.data)
        if attr == 'status' and value.get('state') != self.status.get('state'):
            self.events.append(dict(time=time.monotonic(), state=value.get('state'), reason=value.get('reason')))
        setattr(self, attr, value)

    def pump(self, duration):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            self.mcu.tick()
            now = time.monotonic()
            if now-self.last_publish >= .019:
                stamp = self.raw.get_clock().now().to_msg()
                if self.wand_on: self.raw_pubs['wand'].publish(pose_message(self.wand, stamp, world='map'))
                p = self.mcu.plant.pose
                self.raw_pubs['car'].publish(pose_message(Pose(p.x, p.y, self.car_z,
                    p.yaw-2.356194490192345), stamp, world='map'))
                if self.lease_on:
                    self.sequence += 1
                    self.lease_pub.publish(String(data=json.dumps(asdict(Lease(self.owner, self.sequence,
                        now, self.get_clock().now().nanoseconds*1e-9, self.held)))))
                self.last_publish = now
            rclpy.spin_once(self, timeout_sec=.002)

    def wait(self, fn, timeout=10.):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            self.pump(.02)
            if fn(): return
        raise AssertionError(f'acceptance wait: supervisor={self.status}; transport={self.transport}; gate={self.gate}; '
                             f'emu_errors={self.mcu.errors}')

    def call(self, name):
        client = self.service_clients[name]
        self.wait(client.service_is_ready)
        f = client.call_async(Trigger.Request())
        self.wait(f.done, 3.)
        assert f.result().success, f.result().message

    def arm(self):
        self.held = False
        self.wait(lambda: self.status.get('ready_to_start') and self.status.get('stop_confirmed'), 12.)
        self.call('start')
        self.wait(lambda: self.status.get('state') == 'ARMED' and self.status.get('gate_ready'), 6.)
        self.held = True
        self.wait(lambda: self.status.get('state') == 'ACTIVE')

    def move(self):
        self.wand = Pose(self.wand.x+.10, self.wand.y, 1.)
        self.wait(lambda: self.mcu.command.vx > .025, 5.)

    def recover(self):
        self.held = False
        self.wait(lambda: self.status.get('state') == 'FAULT' and self.status.get('stop_confirmed'))
        self.call('acknowledge')
        self.wait(lambda: self.status.get('state') == 'PAUSED')

    def destroy_node(self):
        self.raw.destroy_node()
        self.raw_context.shutdown()
        return super().destroy_node()


def run_checks(p, process, log_path):
    m, checks = p.mcu, []
    p.wait(lambda: p.status.get('ready_to_start'), 12.)
    # All legacy raw motion topics are inert, even though real bridge code is running.
    bypass = [(Twist, '/car/cmd_vel'), (Bool, '/car/safety_enabled'), (Bool, '/car/control_enable'),
              (Bool, '/car/clear_faults')]
    pubs = [(p.create_publisher(t, name, 10), t) for t, name in bypass]
    for _ in range(10):
        for pub, kind in pubs:
            msg = Twist() if kind is Twist else Bool(data=True)
            if kind is Twist: msg.linear.x = .05
            pub.publish(msg)
        p.pump(.03)
    for pub, _ in pubs: p.destroy_publisher(pub)
    assert m.command == ZERO and m.enables == 0
    checks.append('default disabled; raw Twist/Bool cannot arm or move emulator')
    p.arm()
    p.move()
    p.wait(lambda: m.plant.pose.x > .025, 3.)
    p.wait(lambda: abs(p.status.get('goal', {}).get('x', 0.)-.2) < .002, 2.)
    assert m.enables > 0 and not m.errors
    checks.append('raw Sign/domain93 -> domain94 safety -> signed gate -> real MC-Link UDP -> emulator; 2x target')
    at = time.monotonic()
    p.held = False
    p.wait(lambda: m.command == ZERO and not m.permit, .6)
    release_latency = time.monotonic()-at
    p.wait(lambda: p.status.get('state') == 'PAUSED' and p.status.get('stop_confirmed'))
    p.held = True
    p.pump(.3)
    assert not m.permit and m.command == ZERO
    checks.append('release revokes MCU permit; re-hold alone cannot resume')
    p.arm()
    p.move()
    assert m.rearms >= 1
    checks.append('second run resets only a healthy disabled stationary 4WGB run budget')
    p.wand_on = False
    p.wait(lambda: p.status.get('state') == 'FAULT' and p.status.get('stop_confirmed'), 4.)
    p.wand_on = True
    p.pump(.4)
    assert not m.permit
    p.recover()
    checks.append('Sign loss stops MCU; restoration does not resume')
    p.arm()
    p.move()
    p.car_z = .35
    p.pump(.07)
    p.car_z = .10
    p.wait(lambda: p.status.get('state') == 'FAULT' and p.status.get('stop_confirmed'), 4.)
    p.recover()
    checks.append('raw car height is retained; lift then fresh valid pose cannot mask stop')
    p.arm()
    p.move()
    p.lease_on = False
    p.wait(lambda: p.status.get('state') == 'FAULT' and p.status.get('stop_confirmed'), 4.)
    p.held, p.lease_on = False, True
    p.pump(.3)
    p.recover()
    checks.append('operator lease loss stops real protocol output; restoring leases cannot resume')
    p.arm()
    p.move()
    m.freeze_samples = True
    p.wait(lambda: p.status.get('state') == 'STOP_UNCONFIRMED', 4.)
    assert not p.status.get('stop_confirmed') and m.command == ZERO
    m.freeze_samples = False
    p.recover()
    checks.append('advancing UDP packets with frozen MCU samples cannot renew feedback; stop stays unconfirmed')
    p.arm()
    p.move()
    conflict = p.raw.create_publisher(Bool, '/car/run_lease', 10)
    p.wait(lambda: p.status.get('state') == 'FAULT' and p.status.get('stop_confirmed'), 4.)
    p.raw.destroy_publisher(conflict)
    p.pump(.3)
    p.recover()
    checks.append('legacy mode publisher in source domain forces stop; actual domain0 untouched')
    p.arm()
    p.move()
    forged = p.create_publisher(String, 'actuator_command', LATEST)
    for _ in range(5): forged.publish(String(data='{}')); p.pump(.025)
    p.wait(lambda: m.estops > 0 and m.command == ZERO, 1.)
    p.destroy_publisher(forged)
    checks.append('forged signed authority hard-locks and sends MCU ESTOP; raw source conflict also detected')
    assert not m.errors, m.errors
    max_speed = max(math.hypot(r[2], r[3]) for r in m.received)
    assert max_speed <= .051
    return dict(passed=True, checks=checks, release_to_disabled_wire_s=release_latency,
        maximum_wire_speed_mps=max_speed, mcu_enable_frames=m.enables, healthy_rearms=m.rearms,
        mcu_estops=m.estops, protocol_errors=m.errors, events=p.events)


def run_freeze_check(p, process, log_path):
    # SIGSTOP can also delay DDS users and legitimately trip their 100ms hard
    # timer protection. Isolate this destructive episode; never require a hard
    # fault to acknowledge/rearm just so another test can continue.
    m = p.mcu
    p.wait(lambda: p.status.get('ready_to_start'), 12.)
    p.arm()
    p.move()
    text = log_path.read_text()
    match = re.search(r'\[wand_acceptance_transport-\d+\]: process started with pid \[(\d+)\]', text)
    assert match, text
    pid = int(match.group(1))
    stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    assert int(stat[1]) == process.pid, 'refusing to signal a process not owned by this test launch'
    last_command = m.last_command
    os.kill(pid, signal.SIGSTOP)
    try:
        p.wait(lambda: m.command == ZERO and not m.permit, .9)
        watchdog_latency = time.monotonic()-last_command
    finally: os.kill(pid, signal.SIGCONT)
    p.wait(lambda: p.status.get('state') == 'FAULT' and p.status.get('stop_confirmed'), 5.)
    p.pump(.4)
    assert not m.permit
    assert m.watchdog_stops, 'expected the independent emulator watchdog, not a host command'
    hard = p.status.get('hard_fault')
    if hard:
        f = p.service_clients['acknowledge'].call_async(Trigger.Request())
        p.wait(f.done, 3.)
        assert not f.result().success and 'hard fault' in f.result().message
    assert not m.errors, m.errors
    return dict(passed=True, checks=['transport SIGSTOP: independent 600ms MCU watchdog; SIGCONT cannot resume'],
        transport_freeze_mcu_watchdog_s=watchdog_latency, additional_timer_hard_fault=hard,
        hard_fault_acknowledgement_rejected=bool(hard), maximum_wire_speed_mps=max(
            math.hypot(r[2], r[3]) for r in m.received), protocol_errors=m.errors, events=p.events)


def run_one(folder, case):
    report = dict(passed=False, case=case)
    process = probe = emulator = None
    try:
        rclpy.init(domain_id=DOMAIN)
        emulator = McuEmulator()
        probe = Probe(emulator)
        log_path = folder/(case+'_launch.log')
        with log_path.open('x') as stream:
            process = subprocess.Popen(['ros2', 'launch', 'mecanum_wand_4ws8_hardware',
                'wand_sign_acceptance.launch.py', 'mode:=emulator', 'gui:=false'],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            report.update((run_checks if case == 'functional' else run_freeze_check)(probe, process, log_path))
    except Exception as error:
        import traceback
        report.update(error=repr(error), traceback=traceback.format_exc())
        if probe: report.update(supervisor=probe.status, gate=probe.gate, transport=probe.transport,
                                source=probe.source, events=probe.events)
        if emulator: report.update(protocol_errors=emulator.errors, wire_frames=len(emulator.received))
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if probe: probe.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if emulator: emulator.sock.close()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    folder = args.output.resolve()
    os.environ['ROS_DOMAIN_ID'], os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = str(DOMAIN), 'LOCALHOST'
    report = dict(passed=False, verification='ROS + LOOPBACK MC-LINK EMULATOR ONLY; NO HARDWARE', cases=[])
    try:
        report['domain0_before'] = audit_domain_zero()
        for case in ('functional', 'transport_freeze'):
            result = run_one(folder, case)
            report['cases'].append(result)
            if not result['passed']: raise AssertionError(case+' failed; see case diagnostics')
            print('PASS acceptance '+case, flush=True)
        functional, freeze = report['cases']
        report.update(passed=True, checks=functional['checks']+freeze['checks'],
            release_to_disabled_wire_s=functional['release_to_disabled_wire_s'],
            transport_freeze_mcu_watchdog_s=freeze['transport_freeze_mcu_watchdog_s'],
            maximum_wire_speed_mps=max(c['maximum_wire_speed_mps'] for c in report['cases']),
            protocol_errors=[e for c in report['cases'] for e in c['protocol_errors']])
        report['domain0_after'] = audit_domain_zero()
    except Exception as error:
        report.update(passed=False, error=repr(error))
    (folder/'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
