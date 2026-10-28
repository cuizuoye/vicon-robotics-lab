"""Opt-in commissioning adapters. Shadow default; hardware requires review."""

from dataclasses import asdict
import copy
import fcntl
import json
import math
import os
import socket
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from .continuous_bridge import ContinuousCmdVelUdpBridge as CmdVelUdpBridge
from mecanum_vicon_control.math_utils import wrap_angle
from .continuous_status import disabled_motion_status_ready
from mecanum_vicon_control.transport_protocol import (decode_frame, unpack_wheel_telemetry,
    MessageType, ProtocolError)
from .commissioning import (DOMAIN, NAMESPACE, WORLD, AuthoritySigner, TransportPolicy,
                            McuSampleClock, runtime_from_environment)
from mecanum_wand_control.safety_core import ZERO, fresh_envelope
from mecanum_wand_control.safety_nodes import (SupervisorNode, GateNode, PlantNode, SENSOR, LATEST,
                           decode_pose, pose_message)
from mecanum_wand_control.ros_nodes import run_node

LEGACY_MOTION = ('/car/cmd_vel', '/car/cmd_vel_raw', '/car/enable', '/car/control_enable',
                 '/car/run_lease', '/car/clear_faults', '/car/trajectory_enable')
LEGACY_NODES = ('cmd_vel_udp_bridge', 'continuous_loop_supervisor', 'rectangle_run_guard')


class ProgressSocket:
    def __init__(self, sock, ip, session_id, trace_path=''):
        self.socket, self.ip, self.session_id, self.clock = sock, ip, session_id, McuSampleClock()
        self.trace = (os.fdopen(os.open(trace_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w')
                      if trace_path else None)

    def __getattr__(self, key): return getattr(self.socket, key)

    def close(self):
        try: return self.socket.close()
        finally:
            if self.trace: self.trace.close()

    def recvfrom(self, size):
        while True:
            data, sender = self.socket.recvfrom(size)
            if sender[0] != self.ip: continue
            try:
                f = decode_frame(data)
                if f.session_id != self.session_id(): continue
                if f.message_type in (MessageType.WHEEL_TELEMETRY, MessageType.LINK_STATUS):
                    if (f.message_type == MessageType.WHEEL_TELEMETRY
                            and unpack_wheel_telemetry(f.payload).sample_time_ms != f.sender_time_ms):
                        continue
                    now = time.monotonic()
                    before = dict(latest_tick=self.clock.latest_tick, local_baseline=self.clock.local_baseline,
                                  elapsed_ms=self.clock.elapsed_ms, previous_kind_tick=self.clock.by_kind.get(f.message_type))
                    accepted = self.clock.observe(f.message_type, f.sender_time_ms, now)
                    if self.trace:
                        self.trace.write(json.dumps(dict(received_steady=now, kind=MessageType(f.message_type).name,
                            tick=f.sender_time_ms, sequence=f.sequence, session=f.session_id,
                            accepted_sample_steady=accepted, clock_before=before, payload_hex=f.payload.hex()))+'\n')
                        self.trace.flush()
                    if accepted is None:
                        continue
            except ProtocolError:
                return data, sender  # Parent retains its existing malformed-frame diagnostics.
            return data, sender


class RawSource(Node):
    """Read-only domain0 subscriber/graph audit, spun in the same relay loop."""
    def __init__(self, context):
        super().__init__('wand_acceptance_readonly', context=context, use_global_arguments=False)
        self.samples, self.last_stamps, self.rejected = {}, {}, {}
        for source, topic in (('wand', '/vicon/Sign/Sign'), ('car', '/vicon/car01/car01')):
            self.create_subscription(PoseStamped, topic,
                lambda msg, source=source: self.receive(source, msg), SENSOR)

    def receive(self, source, msg):
        try:
            # No projected 2D state: retain raw z/roll/pitch and original header.
            _, stamp, _, _ = decode_pose(msg, WORLD)
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if not -.005 <= age <= .20: raise ValueError('raw source age')
            if stamp <= self.last_stamps.get(source, -math.inf): return
            self.last_stamps[source] = stamp
            self.samples[source] = copy.deepcopy(msg)
        except (ValueError, TypeError, AttributeError):
            self.samples.pop(source, None)
            self.rejected[source] = self.rejected.get(source, 0)+1

    def audit(self):
        conflicts = [t for t in LEGACY_MOTION if self.count_publishers(t)]
        conflicts += [n for n, _ in self.get_node_names_and_namespaces() if n in LEGACY_NODES]
        for topic in ('/vicon/Sign/Sign', '/vicon/car01/car01'):
            if self.count_publishers(topic) > 1: conflicts.append('duplicate source '+topic)
        return conflicts


class SignSource(Node):
    def __init__(self):
        self.runtime = runtime_from_environment()
        super().__init__('wand_sign_source', namespace=NAMESPACE)
        self.runtime.guard(self)
        self.source_context = Context()
        rclpy.init(context=self.source_context, domain_id=self.runtime.source_domain,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.raw = RawSource(self.source_context)
        self.source_executor = SingleThreadedExecutor(context=self.source_context)
        self.source_executor.add_node(self.raw)
        self.pubs = {s: self.create_publisher(PoseStamped, s+'_pose', SENSOR)
                     for s in (('wand',) if self.runtime.mode == 'shadow' else ('wand', 'car'))}
        self.mode_pub = self.create_publisher(String, 'mode_claim', LATEST)
        self.info = self.create_publisher(String, 'source_status', LATEST)
        self.forwarded, self.conflicts, self.audit_at = {}, ['discovery pending'], -math.inf
        self.discovery_until = time.monotonic()+2.
        self.create_timer(.01, self.tick)

    def tick(self):
        for _ in range(8): self.source_executor.spin_once(timeout_sec=0.)
        now = time.monotonic()
        if now-self.audit_at >= .10:
            self.conflicts, self.audit_at = self.raw.audit(), now
            if now < self.discovery_until: self.conflicts.append('discovery pending')
        self.mode_pub.publish(String(data='conflict' if self.conflicts else 'wand'))
        for source, pub in self.pubs.items():
            msg = self.raw.samples.get(source)
            if msg is None: continue
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if self.forwarded.get(source) == stamp: continue
            out = msg
            if source == 'car':
                pose, _, roll, pitch = decode_pose(msg, WORLD)
                pose = type(pose)(pose.x, pose.y, pose.z, wrap_angle(pose.yaw+self.runtime.yaw_offset))
                out = pose_message(pose, msg.header.stamp, roll, pitch, WORLD)
            pub.publish(out)
            self.forwarded[source] = stamp
        self.info.publish(String(data=json.dumps(dict(mode=self.runtime.mode,
            source_domain=self.runtime.source_domain, raw_stamps=self.raw.last_stamps,
            rejected=self.raw.rejected, conflicts=self.conflicts,
            issued_steady=now, issued_ros=self.get_clock().now().nanoseconds*1e-9))))

    def destroy_node(self):
        self.source_executor.shutdown()
        self.raw.destroy_node()
        self.source_context.shutdown()
        return super().destroy_node()


class AcceptanceGate(GateNode):
    def __init__(self):
        super().__init__(runtime_from_environment())
        if self.runtime.mode != 'shadow':
            self.authority_signer = AuthoritySigner(os.environ['WAND_SAFE_CHANNEL_KEY'])

    def publish_output(self, output):
        if self.runtime.mode == 'shadow': return super().publish_output(output)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = output.vx, output.vy, output.wz
        self.commands.publish(msg)  # Observation only; hardware adapter ignores raw Twist.
        f = self.core.frame
        permit = bool(f and f.epoch and f.epoch == self.core.primed_epoch
                      and not self.core.fault and not self.core.hard_fault
                      and f.state in ('ARMED', 'ACTIVE'))
        self.actuator.publish(String(data=self.authority_signer.encode(output, *self.clocks(),
            epoch=f.epoch if f else '', state=f.state if f else 'STANDBY', permit=permit,
            estop=self.core.hard_fault)))


class AcceptanceSupervisor(SupervisorNode):
    def __init__(self): super().__init__(runtime_from_environment())

    def operator_gate_ready(self, steady, ros):
        if self.runtime.mode == 'shadow': return True
        t = self.transport_report
        return bool(t and fresh_envelope(t['issued_steady'], t['issued_ros'], steady, ros, .20)
                    and t.get('ready') and t.get('armed_epoch') == self.core.epoch)


class HardwareBridge(CmdVelUdpBridge):
    """Use the proven MC-Link handshake but close ALL unsigned motion inputs.

    This class is the sole socket owner. It never publishes hardware commands
    into domain0. The emulator path is forced to loopback and different ports.
    """
    def __init__(self):
        self.runtime = runtime_from_environment()
        if self.runtime.mode == 'shadow': raise RuntimeError('shadow must not construct hardware transport')
        # Fail before constructing the existing bridge (and therefore before
        # it can open a socket) if another transport already owns the port.
        self.port_lock = open('/tmp/wand_mc_link_'+str(self.runtime.telemetry_port)+'.lock', 'a')
        fcntl.flock(self.port_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try: probe.bind(('0.0.0.0', self.runtime.telemetry_port))
        finally: probe.close()
        super().__init__()
        self.runtime.guard(self)
        if (self.esp_ip != self.runtime.ip or self.network_interface != self.runtime.interface
                or self.command_port != self.runtime.command_port or self.telemetry_port != self.runtime.telemetry_port
                or self.required_motion_tag != self.runtime.motion_tag or self.wire_timeout_ms != 600
                or not self.bounded_motion_mode or self.invert_body_xy or self.readonly_mode
                or self.max_vx_mps != .05 or self.max_vy_mps != .05 or self.max_wz_radps != .10):
            self.sock.close()
            self.sock = None
            raise RuntimeError('hardware bridge parameters differ from reviewed acceptance profile')
        self.policy = TransportPolicy(os.environ['WAND_SAFE_CHANNEL_KEY'])
        self.sock = ProgressSocket(self.sock, self.esp_ip, lambda: self.session.session_id,
                                   os.environ.get('WAND_RAW_RECEIVE_TRACE', ''))
        self.feedback = self.create_publisher(String, 'plant_feedback', LATEST)
        self.transport_info = self.create_publisher(String, 'transport_status', LATEST)
        self.create_subscription(String, 'actuator_command', self.on_signed, LATEST)
        self.feedback_sequence, self.last_feedback_identity = 0, None
        self.last_armed_epoch, self.enabled_epoch, self.rearm_epoch = '', '', ''
        self.permit_observed_epoch = ''
        self.rearm_started = None
        self.baseline_since = self.baseline_session = None
        self.baseline_epoch = self.prepared_epoch = ''
        self.prepared_session = self.enable_requested_at = None
        self.create_timer(.01, self.safety_tick)

    # Override every unsigned legacy entry point. Only on_signed/safety_tick
    # can call the parent implementations after authenticated authority.
    def on_cmd(self, msg): pass
    def on_enable(self, msg): pass
    def on_control_enable(self, msg): pass
    def on_clear_faults(self, msg): pass
    def on_estop(self, msg): pass

    def on_signed(self, msg):
        self.policy.receive(msg.data, time.monotonic(), self.get_clock().now().nanoseconds*1e-9)
        self.safety_tick()

    def drain_receive(self):
        super().drain_receive()
        samples = self.sock.clock.accepted_at
        if MessageType.WHEEL_TELEMETRY in samples:
            self.last_wheel_telemetry_monotonic = samples[MessageType.WHEEL_TELEMETRY]
        if MessageType.LINK_STATUS in samples:
            self.last_motion_status_monotonic = samples[MessageType.LINK_STATUS]

    def latch_local_stop(self, reason):
        # Do not re-HELLO into a permit still held by an unreachable MCU. Try
        # a disabled zero on the old session, then wait out its 600ms lease.
        if self.sock is not None and self.session.connected:
            try:
                self.sock.sendto(self.session.cmd_vel(0., 0., 0., self.wire_timeout_ms,
                    False, self.monotonic_ms()), self.destination)
            except (OSError, ProtocolError): pass
        self.reconnect_after = time.monotonic()+.65
        if hasattr(self, 'policy') and self.policy.epoch:
            self.policy.revoke('MC_LINK_STOP_LATCHED')
        return super().latch_local_stop(reason)

    def send_hello_if_due(self):
        if time.monotonic() < getattr(self, 'reconnect_after', -math.inf): return
        return super().send_hello_if_due()

    def reset_preparation(self):
        self.baseline_since = self.baseline_session = None
        self.baseline_epoch = self.prepared_epoch = ''
        self.prepared_session = self.enable_requested_at = None

    def feedback_is_fresh(self, now):
        return bool(self.session.connected and self.last_motion_status is not None and all(
            v is not None and 0 <= now-v <= .20 for v in
            (self.last_motion_status_monotonic, self.last_wheel_telemetry_monotonic)))

    def baseline_is_current(self, now):
        """A genuine disabled-zero lease, never a renewed cached diagnostic."""
        return bool(self.feedback_is_fresh(now) and self.last_mcu_fault_bits == 0
            and disabled_motion_status_ready(self.last_motion_status)
            and self.pending_control_action is None and not self.motion_gate_requested
            and not self.safety_enabled and not self.policy.fault and not self.policy.hard_fault
            and (not self.local_stop_latched or self.control_enable_reset_seen)
            and self.last_command_progress_monotonic is not None
            and 0 <= now-self.last_command_progress_monotonic <= .20)

    def baseline_is_prepared(self, now):
        return bool(self.baseline_is_current(now) and self.baseline_since is not None
            and now-self.baseline_since >= .60 and self.baseline_epoch == self.policy.epoch
            and self.baseline_session == self.session.session_id)

    def queue_control_action(self, action, repeats=5):
        if action not in ('enable', 'clear_faults'):
            self.reset_preparation()
            # Retain real, originally stamped evidence on ordinary disable.
            # A cached permitted=1 is NOT a stop acknowledgement, and can
            # neither prime nor enable: host intent is already revoked.
            session = self.session.session_id
            snapshot = (self.last_motion_status, self.last_motion_status_monotonic)
            result = super().queue_control_action(action, repeats)
            if action == 'disable' and self.session.connected and self.session.session_id == session:
                self.last_motion_status, self.last_motion_status_monotonic = snapshot
            return result
        now = time.monotonic()
        p, epoch = self.policy.envelope or {}, self.policy.epoch
        prepared = self.baseline_is_prepared(now)
        if action == 'enable':
            prepared = (prepared and self.policy.active(now) and self.prepared_epoch == epoch
                and self.prepared_session == self.session.session_id
                and self.last_motion_status['started_ms'] == 0)
        else:
            # This is only the healthy, disabled 4WGB/4WGC run-budget reset. It
            # cannot clear a true MCU fault or reset a running/ACTIVE session.
            prepared = prepared and p.get('state') == 'ARMED' and bool(self.last_motion_status['started_ms'])
        if not prepared:
            self.policy.revoke('MCU_NOT_PREPARED')
            self.force_local_disabled()
            self.motion_gate_requested = False
            self.get_logger().error('拒绝未准备好的同会话控制请求；必须重新准备')
            return
        # The 600ms disabled primer ran BEFORE gate_ready. Reuse its live
        # session; do not HELLO or erase genuine diagnostic/progress caches
        # on the human held-key edge. The inherited sender still requires a
        # fresh MCU permit before ANY nonzero velocity can reach the wire.
        self.pending_control_action, self.pending_control_repeats = action, repeats
        self.motion_gate_requested = action == 'enable'
        self.enable_prime_started, self.enable_prime_complete = self.baseline_since, True
        self.enable_command_complete, self.last_tx_mode = False, None
        if action == 'enable': self.enable_requested_at = now
        else: self.reset_preparation()

    def stop_wire(self, hard=False):
        self.reset_preparation()
        if self.motion_gate_requested or self.pending_control_action == 'enable' or self.safety_enabled:
            CmdVelUdpBridge.on_control_enable(self, Bool(data=False))
        self.force_local_disabled()
        self.enabled_epoch = ''
        if hard and self.pending_control_action != 'estop' and not getattr(self, 'estop_sent', False):
            self.estop_sent = True
            CmdVelUdpBridge.on_estop(self, Bool(data=True))

    def safety_tick(self):
        now = time.monotonic()
        active = self.policy.active(now)
        p = self.policy.envelope or {}
        status = self.last_motion_status
        fresh = self.feedback_is_fresh(now)
        if active and not fresh:
            # Same 200ms sample deadline as the independent supervisor/gate.
            # Data returning cannot revive this arm epoch.
            self.policy.revoke('MCU_FEEDBACK_TIMEOUT')
            self.stop_wire()
            active = False
        if self.policy.hard_fault:
            self.stop_wire(hard=True)
        elif not self.policy.epoch or self.policy.fault:
            self.stop_wire()
        else:
            epoch = self.policy.epoch
            if p.get('state') == 'ARMED' and epoch != self.last_armed_epoch:
                self.stop_wire()
                self.last_armed_epoch, self.rearm_started = epoch, None
                self.permit_observed_epoch = ''
            # A reconnect/rearm can invalidate feedback on the first ARMED
            # tick. Retry the explicit disabled-reset step once fresh, rather
            # than remembering that missed first tick as a completed reset.
            if (p.get('state') == 'ARMED' and self.local_stop_latched and fresh
                    and self.last_mcu_fault_bits == 0 and disabled_motion_status_ready(status)
                    and not self.control_enable_reset_seen):
                CmdVelUdpBridge.on_control_enable(self, Bool(data=False))
            if p.get('state') == 'ARMED':
                if self.baseline_is_current(now):
                    if (self.baseline_since is None or self.baseline_epoch != epoch
                            or self.baseline_session != self.session.session_id):
                        self.baseline_since, self.baseline_epoch = now, epoch
                        self.baseline_session = self.session.session_id
                    if self.baseline_is_prepared(now) and not status['started_ms']:
                        self.prepared_epoch, self.prepared_session = epoch, self.session.session_id
                else:
                    self.reset_preparation()
            elif active and (self.prepared_epoch != epoch
                    or self.prepared_session != self.session.session_id or not self.session.connected):
                self.policy.revoke('MCU_PREPARED_SESSION_LOST')
                self.stop_wire()
                active = False
            if active and fresh and status and status['permitted']:
                self.permit_observed_epoch = epoch
            elif active and fresh and self.permit_observed_epoch == epoch:
                self.policy.revoke('MCU_PERMIT_LOST')
                self.stop_wire()
                active = False
            if fresh and self.last_mcu_fault_bits == 0 and disabled_motion_status_ready(status):
                if (p.get('state') == 'ARMED' and status['started_ms'] and self.rearm_epoch != epoch
                        and self.baseline_is_prepared(now)):
                    # Both reviewed identities reset HEALTHY stationary run
                    # budget, not clear true faults. Once only, no UDP retries.
                    self.rearm_epoch, self.rearm_started = epoch, now
                    CmdVelUdpBridge.on_clear_faults(self, Bool(data=True))
                elif not status['started_ms'] and active and self.enabled_epoch != epoch:
                    CmdVelUdpBridge.on_control_enable(self, Bool(data=True))
                    if self.motion_gate_requested and not self.local_stop_latched:
                        self.enabled_epoch = epoch
            if self.rearm_started is not None:
                if (fresh and status and status['started_ms'] == 0
                        and self.last_motion_status_monotonic > self.rearm_started):
                    self.rearm_started = None
                elif now-self.rearm_started > 3.:
                    self.policy.revoke('MCU_REARM_TIMEOUT')
                    self.stop_wire()
            if (active and self.enable_requested_at is not None
                    and now-self.enable_requested_at > 1. and self.permit_observed_epoch != epoch):
                self.policy.revoke('MCU_ENABLE_TIMEOUT')
                self.stop_wire()
                active = False
            if active and self.local_stop_latched:
                self.policy.revoke('MC_LINK_STOP_LATCHED')
                self.stop_wire()
            elif active and self.enabled_epoch == epoch:
                command = self.policy.command
                twist = Twist()
                twist.linear.x, twist.linear.y, twist.angular.z = command.vx, command.vy, command.wz
                CmdVelUdpBridge.on_cmd(self, twist)
                CmdVelUdpBridge.on_enable(self, Bool(data=True))
            elif not active:
                self.force_local_disabled()
        # Evidence is stamped only when BOTH independently received MCU
        # telemetry and diagnostic snapshots advance. Never renew cached data.
        status = self.last_motion_status
        identity = (self.last_wheel_telemetry_monotonic, self.last_motion_status_monotonic)
        fresh = self.feedback_is_fresh(now)
        if fresh and identity != self.last_feedback_identity:
            self.last_feedback_identity = identity
            self.feedback_sequence += 1
            zero = not any(status[k] for k in ('permitted',)) and not any(
                status['compares']+status['raw_duty']+status['target_cps']+status['body_milli'])
            # Wheel speed is an independent conservative bound, not a claim
            # that commanded body velocity is an actual odometry measurement.
            speed = max(abs(v) for v in self.last_wheel_rpm)*2*math.pi*.03/60.
            issued = min(identity)
            ros = self.get_clock().now().nanoseconds*1e-9-(now-issued)
            self.feedback.publish(String(data=json.dumps(dict(sequence=self.feedback_sequence,
                issued_steady=issued, issued_ros=ros, output_zero=bool(zero),
                enabled=bool(status['permitted']), speed=speed, yaw_rate=speed/.146,
                fault_bits=int(self.last_mcu_fault_bits or 0) | ((1 << 30) if self.policy.hard_fault else 0)))))
        self.transport_info.publish(String(data=json.dumps(dict(issued_steady=now,
            issued_ros=self.get_clock().now().nanoseconds*1e-9, mode=self.runtime.mode,
            fresh=fresh, local_stop=self.local_stop_latched, fault=self.policy.fault,
            feedback_ages_s=dict(
                diagnostic=now-self.last_motion_status_monotonic if self.last_motion_status_monotonic is not None else None,
                wheel=now-self.last_wheel_telemetry_monotonic if self.last_wheel_telemetry_monotonic is not None else None),
            preparation_stage=('WAIT_FEEDBACK' if not fresh else
                'RESET_HEALTHY_BUDGET' if self.rearm_started is not None else
                'WAIT_DISABLED_ZERO' if not disabled_motion_status_ready(status) else
                'WAIT_BASELINE' if not self.baseline_is_prepared(now) else 'BASELINE_READY'),
            armed_epoch=self.last_armed_epoch,
            prepared_epoch=self.prepared_epoch, policy_state=p.get('state'),
            baseline_elapsed_s=now-self.baseline_since if self.baseline_since is not None else None,
            baseline_current=self.baseline_is_current(now),
            command_progress_age_s=(now-self.last_command_progress_monotonic
                if self.last_command_progress_monotonic is not None else None),
            pending_control_action=self.pending_control_action,
            safety_enabled=self.safety_enabled, motion_gate_requested=self.motion_gate_requested,
            ready=bool(fresh and self.last_mcu_fault_bits == 0 and disabled_motion_status_ready(status)
                and status['started_ms'] == 0 and self.rearm_started is None
                and self.pending_control_action is None and not self.policy.fault
                and not self.policy.hard_fault
                and (p.get('state') != 'ARMED' or (self.baseline_is_prepared(now)
                     and self.prepared_epoch == self.policy.epoch
                     and self.prepared_session == self.session.session_id))
                and (not self.local_stop_latched or self.control_enable_reset_seen)),
            enabled_epoch=self.enabled_epoch, rearm_pending=self.rearm_started is not None))))

    def tick(self):
        # A resumed transport callback cannot run the parent's send loop with
        # a cached enabled command before checking the independent authority.
        if hasattr(self, 'policy'): self.safety_tick()
        return super().tick()

    def destroy_node(self):
        self.stop_wire()
        result = super().destroy_node()
        self.port_lock.close()
        return result


def source_main(): run_node(SignSource)
def supervisor_main(): run_node(AcceptanceSupervisor)
def gate_main(): run_node(AcceptanceGate)
def shadow_main():
    def create():
        runtime = runtime_from_environment()
        if runtime.mode != 'shadow': raise RuntimeError('software plant forbidden in hardware mode')
        return PlantNode(runtime)
    run_node(create)
def hardware_main(): run_node(HardwareBridge)
