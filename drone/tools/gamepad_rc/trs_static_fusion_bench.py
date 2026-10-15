#!/usr/bin/env python3
"""断动力、拆桨、锁定输出：仅静止TRS VPE + 临时65ms延迟/XYZ航向融合。"""
import argparse
from collections import Counter
import datetime
import hashlib
import json
import math
from pathlib import Path
import secrets
import subprocess
import sys
import time
import os

import serial
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.mavlink_vision import PoseGate, checked_quaternion, finite_vector, transform_rigid_pose
from mavlink_readonly_probe import Evidence, decode_parameter
from estop_only_policy import parse_actuator_snapshot
from usb_vision_policy import parse_visual_snapshot, shell_message
from persistent_shell import reply_complete
from trs_uart_readonly_check import UID2, USB
from trs_vicon_receive_bench import TRS
import compact_vision as compact
import usb_combined_fusion_policy as combined
import control_readonly_policy as control
import trs_static_fusion_policy as legacy_policy
import trs_range_static_policy as range_policy
import trs_vision_reference_policy as vision_policy
import trs_vision_reference_permit
from usb_fusion_policy import aid_topic_for_estimator
from disarmed_offboard_policy import DisarmedHoldGate, GroundObservation, restorable_mode, restore_mode_allowed
from gamepad_estop_monitor import find_pad, EVENT, HOME
from usb_fusion_policy import parse_topic
from incident_guard import require_active_bench_clear
from static_fusion_permit import consume as consume_static_permit
from combined_input_loss import CombinedInputLoss
from rigid_body_revision import require_current_calibration, inspect_current_calibration
import rigid_revision_static_permit


def load_backup(path, *, range_reference_static=False, vision_reference_static=False):
    policy = vision_policy if vision_reference_static else range_policy if range_reference_static else legacy_policy
    p = path / 'parameters_verified.json'
    raw = p.read_bytes()
    record = json.loads(raw)
    manifest = json.loads((path / 'parameters_manifest.json').read_text())
    if (not record.get('complete') or record.get('received_count') != 926
            or record.get('expected_count') != 926 or record.get('missing_indices')
            or record.get('firmware', {}).get('version') != '1.14.3'
            or record.get('system_id') != 1 or record.get('component_id') != 1
            or hashlib.sha256(raw).hexdigest() != manifest['sha256']['parameters_verified.json']
            or not 0 <= time.time() - p.stat().st_mtime < 3600):
        raise ValueError('Fresh verified complete 926 parameter backup required')
    rows = record['records']
    values = {r['name']: r['value'] for r in rows}
    if (len(values) != 926 or {r['index'] for r in rows} != set(range(926))
            or not all(n in values for n in policy.READ_PARAMS)
            or values['EKF2_EV_CTRL'] != (2 if vision_reference_static else 0) or values['EKF2_EV_DELAY'] != 0
            or not policy.scope_ok(values, values)):
        raise ValueError('Unsupported baseline, incomplete indices or nonzero EV parameters')
    return values


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--backup', type=Path, required=True)
    ap.add_argument('--heading-candidate', type=Path, required=True)
    ap.add_argument('--board-center-candidate', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--physical-confirmed', action='store_true')
    ap.add_argument('--static-permit', type=Path,
                    help='One-use evidence-bound battery-off static exception; no general guard clear')
    ap.add_argument('--vision-reference-permit', type=Path,
                    help='Independent props-installed, battery-off Vision-reference TRS exception')
    ap.add_argument('--usb-only-props-installed-vision-reference-confirmed', action='store_true')
    ap.add_argument('--disarmed-offboard-ground-check',action='store_true',
                    help='Separate permit: locked/unarmed frozen-position targets and one Offboard request, never arm')
    ap.add_argument('--rigid-revision-static-validation', action='store_true',
                    help='Distinct one-use permit: validate new provisional body frame, static fusion ONLY')
    ap.add_argument('--source-loss-test', action='store_true',
                    help='After verified static fusion, mask only this subscriber and verify latched stop/rollback')
    args = ap.parse_args()
    vision_mode = bool(args.vision_reference_permit)
    ground_check = args.disarmed_offboard_ground_check
    revision_validation = args.rigid_revision_static_validation
    if revision_validation and (not vision_mode or ground_check or args.source_loss_test
                                or args.static_permit or args.physical_confirmed):
        ap.error('Revision validation requires its distinct Vision permit and forbids ground/mode/source-loss/legacy profiles')
    if ground_check and not vision_mode:
        ap.error('Disarmed Offboard check requires separate Vision-reference ground permit')
    if vision_mode and (args.static_permit or args.physical_confirmed or args.source_loss_test
                       or not args.usb_only_props_installed_vision_reference_confirmed):
        ap.error('Vision-reference exception requires its own USB-only/props-installed confirmation, no other modes')
    if args.usb_only_props_installed_vision_reference_confirmed and not vision_mode:
        ap.error('Props-installed confirmation requires Vision-reference permit')
    if args.source_loss_test and not args.static_permit:
        ap.error('Source-loss test requires its own scoped static permit')
    if not args.static_permit and not vision_mode:
        require_active_bench_clear(ap)
    if not (args.physical_confirmed or vision_mode) or args.output.exists():
        ap.error('Unpowered/props-off/static confirmation and new output required')
    try:
        (inspect_current_calibration if revision_validation else require_current_calibration)(
            args.heading_candidate, args.board_center_candidate)
    except (OSError, ValueError, KeyError) as exc:
        ap.error(str(exc))
    policy = vision_policy if vision_mode else range_policy if args.static_permit else legacy_policy
    baseline = load_backup(args.backup, range_reference_static=bool(args.static_permit), vision_reference_static=vision_mode)
    for path in (USB, TRS):
        if subprocess.run(['fuser', str(Path(path).resolve())], capture_output=True).returncode == 0:
            ap.error('Port occupied; no process will be killed: ' + path)
    heading = json.loads(args.heading_candidate.read_text())
    board = json.loads(args.board_center_candidate.read_text())
    if (heading.get('manual_forward_horizontal_confirmed') is not True
            or (not revision_validation and (heading.get('warnings') or board.get('marker_name') != 'MircoAir4'))
            or board.get('imu_offset_verified') is not False
            or (args.board_center_candidate.parent / board.get('source_heading', '')).resolve()
                != args.heading_candidate.resolve()):
        ap.error('Labelled approximate body references required')
    mount = checked_quaternion(heading['body_orientation_in_rigid_xyzw_candidate'])
    offset = finite_vector(board['board_center_in_rigid_m_candidate'], 3)
    permit = None
    if vision_mode:
        try:
            if revision_validation:
                permit = rigid_revision_static_permit.consume(args.vision_reference_permit, args.backup,
                    args.output, args.heading_candidate, args.board_center_candidate)
            else:
                permit = trs_vision_reference_permit.consume(args.vision_reference_permit,args.backup,args.output,
                                                           args.heading_candidate,args.board_center_candidate,ground_check=ground_check)
        except (OSError,ValueError,KeyError) as exc:
            ap.error(str(exc))
    if args.static_permit:
        try:
            permit = consume_static_permit(args.static_permit, args.backup, args.output,
                                           physical_confirmed=args.physical_confirmed,
                                           source_loss=args.source_loss_test)
            if (Path(permit['heading_path']).resolve() != args.heading_candidate.resolve()
                    or Path(permit['board_center_path']).resolve() != args.board_center_candidate.resolve()):
                raise ValueError('Permit mounting-reference paths differ from this invocation')
        except (OSError, ValueError, KeyError) as exc:
            ap.error(str(exc))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped

    class Source(Node):
        def __init__(self):
            super().__init__('trs_static_fusion_bench')
            self.gate = PoseGate(max_age_ms=150)
            self.mask_input = False
            self.suppressed = 0
            self.sub = self.create_subscription(PoseStamped, '/vicon/MircoAir/MircoAir', self.receive,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))

        def receive(self, msg):
            if self.mask_input:
                self.suppressed += 1
                return
            p, q = msg.pose.position, msg.pose.orientation
            self.gate.ingest(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                self.get_clock().now().nanoseconds, time.monotonic_ns(), msg.header.frame_id,
                [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    rclpy.init(args=[])
    node = Source()
    ev = {r: Evidence(source_component=198 if r == 'trs' else 191, expected_parameters=policy.READ_PARAMS)
          for r in ('usb', 'trs')}
    enc = {r: m.MAVLink(None, srcSystem=250, srcComponent=ev[r].source_component) for r in ev}
    dec = {r: m.MAVLink(None) for r in ev}
    for parser in dec.values():
        parser.robust_parsing = True
    ports, tx = {}, {r: Counter() for r in ev}
    start = time.monotonic()
    parameters, param_times = {}, {}
    next_hb, next_version = {r: 0 for r in ev}, {r: 0 for r in ev}
    next_params = 0
    buffer = ''
    active = restoring = receive_time_verified = False
    stream_start = None
    next_emit = 0
    arm = arm_before = arm_after = None
    arm_at = -100
    attitude = None
    alignment = origin_p = origin_q = None
    primary = None
    writes, sent, listeners, fusion_rows, receive_groups, status_texts = [], [], [], [], [], []
    possibly_changed = set()
    restored = {}
    error, restore_errors = None, []
    failed_gate_observation = None
    phase = 'fusion_off_warmup'
    rate_was_verified = False
    fusion_started = fusion_finished = None
    aid_mappings = {}
    aid_slots = {}
    ground_gate = None
    local_live = None
    local_at = -100
    pad_fd = None
    pad_pending = b''
    home_seen = False
    mode_acks = []
    initial_mode = None
    initial_submode = None
    mode_request_at = None
    mode_restore_at = None
    mode_restoring = False
    mode_restored = False
    offboard_mode_verified = False
    hold_sends = []
    next_hold = 0

    def ground_observation():
        now=elapsed()
        sample=node.gate.last
        age=(node.get_clock().now().nanoseconds-sample.stamp_ns)/1e9 if sample else 999
        return GroundObservation(now,UID2,'1.14.3',tuple(local_live[k] for k in ('x','y','z')),
            local_live['heading'],tuple(local_live[k] for k in ('xy_reset_counter','z_reset_counter','heading_reset_counter')),
            primary,armed=not bool(arm and not arm['armed']),lockdown=bool(arm and arm['lockdown']),
            pose_age_s=age,status_age_s=max(now-local_at,now-arm_at),
            estimator_ready=control.assess(fusion_rows,primary)['estimator_prerequisites_observed'],
            gamepad_available=pad_fd is not None,home_latched=home_seen)
    loss = CombinedInputLoss() if args.source_loss_test else None
    loss_result = None

    class FusionWindowEnded(Exception):
        pass

    def elapsed():
        return time.monotonic() - start

    def identity(route):
        e = ev[route]
        return bool(e.identity and e.identity['uid2'] == UID2 and e.identity['firmware'] == '1.14.3'
                    and not e.fatal and e.heartbeats and not e.heartbeats[-1]['armed']
                    and elapsed() - e.heartbeats[-1]['elapsed_s'] < 2)

    def restore_safe():
        return identity('usb')

    def safe():
        return bool(all(identity(r) for r in ev) and policy.scope_ok(parameters, baseline)
                    and all(elapsed() - param_times.get(n, -100) < 4 for n in policy.READ_PARAMS))

    def locked_static():
        if not (arm and not arm['armed'] and arm['lockdown'] and elapsed() - arm_at < 4
                and origin_p is not None and node.gate.fresh(node.get_clock().now().nanoseconds, time.monotonic_ns())):
            return False
        p, q = transform_rigid_pose(node.gate.last.position, node.gate.last.quaternion, offset, mount)
        return policy.stationary(p, q, origin_p, origin_q)

    def send(route, msg):
        control_packet = ground_check and (msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED'
            or (msg.get_type()=='COMMAND_LONG' and msg.command==m.MAV_CMD_DO_SET_MODE))
        if control_packet:
            restoring_mode_ok = bool(mode_restoring and route=='trs'
                and restore_mode_allowed(msg,initial_mode,initial_submode,safe=restore_safe(),
                    locked=bool(arm and not arm['armed'] and arm['lockdown'] and elapsed()-arm_at<4)))
            permitted = restoring_mode_ok or bool(route=='trs' and safe() and locked_static()
                and ground_gate and ground_gate.permits(msg,ground_observation()))
        else:
            permitted = policy.allowed(msg, route=route, safe=safe(), authorized=args.physical_confirmed or vision_mode,
                              backup_verified=True, locked_static=locked_static(),
                              receive_time_verified=receive_time_verified and rate_was_verified,
                              restore_safe=restore_safe())
        if not permitted:
            raise ValueError('Static bench policy rejected ' + route + '/' + msg.get_type())
        if control_packet and msg.get_type()=='COMMAND_LONG' and not mode_restoring:
            ground_gate.record_actual_mode_attempt()  # Consume before any transport ambiguity.
        packet = msg.pack(enc[route])
        if ports[route].write(packet) != len(packet):
            raise OSError('Short serial write')
        enc[route].seq = (enc[route].seq + 1) % 256
        tx[route][msg.get_type()] += 1

    def emit():
        nonlocal next_emit, failed_gate_observation, active, next_hold
        now = time.monotonic()
        if not active:
            return
        if loss and loss.masked_at_s is not None:
            fresh = node.gate.fresh(node.get_clock().now().nanoseconds, time.monotonic_ns())
            if not loss.permits(fresh, elapsed(), len(sent)):
                active = False  # Latched: fresh source recovery cannot restart transmission.
                return
        safe_now, locked_now = safe(), locked_static()
        if not safe_now or not locked_now:
            # Observe each failing condition; never relax a gate to diagnose it.
            sample = node.gate.last
            ros_ns, mono_ns = node.get_clock().now().nanoseconds, time.monotonic_ns()
            delta_mm = angle_deg = None
            if sample and origin_p is not None:
                p_now, q_now = transform_rigid_pose(sample.position, sample.quaternion, offset, mount)
                delta_mm = 1000 * math.sqrt(sum((a-b)**2 for a,b in zip(p_now, origin_p)))
                angle_deg = math.degrees(2 * math.acos(min(1., max(0.,
                    abs(sum(a*b for a,b in zip(q_now, origin_q)))))))
            failed_gate_observation = {
                'elapsed_s': elapsed(), 'identity_parameters_fresh': safe_now,
                'locked_source_static': locked_now,
                'identities_fresh': {r: identity(r) for r in ev},
                'parameter_scope_ok': policy.scope_ok(parameters, baseline),
                'parameter_ages_s': {n: elapsed()-param_times.get(n, -100) for n in policy.READ_PARAMS},
                'actuator': arm, 'actuator_observation_age_s': elapsed()-arm_at,
                'source_reason': node.gate.reason,
                'source_fresh': node.gate.fresh(ros_ns, mono_ns),
                'source_ros_age_ms': (ros_ns-sample.stamp_ns)/1e6 if sample else None,
                'source_arrival_age_ms': (mono_ns-sample.received_monotonic_ns)/1e6 if sample else None,
                'position_delta_mm': delta_mm, 'attitude_delta_deg': angle_deg,
                'static_limits_mm_deg': [10, 2],
            }
            raise RuntimeError('Identity/parameters/lock/source/static gate lost; stop and restore')
        if now - stream_start > (25 if ground_check else 15) or len(sent) >= (800 if ground_check else 480):
            raise RuntimeError('Static pose stream hard time/packet limit reached')
        if now < next_emit:
            return
        sample = node.gate.take_new(node.get_clock().now().nanoseconds, time.monotonic_ns())
        if sample is None:
            return
        mono = time.monotonic_ns()
        age = node.get_clock().now().nanoseconds - sample.stamp_ns
        if not 0 <= age <= node.gate.max_age_ns:
            raise RuntimeError('Source age exceeded')
        p, q = transform_rigid_pose(sample.position, sample.quaternion, offset, mount)
        if not policy.stationary(p, q, origin_p, origin_q):
            raise RuntimeError('Body moved beyond stationary-only boundary')
        stamp = (mono - age) // 1000
        msg, p, q = compact.message(enc['trs'], stamp, p, q, alignment)
        send('trs', msg)
        sent.append({'elapsed_s': elapsed(), 'source_stamp_ns': sample.stamp_ns,
                     'wire_time_usec': stamp, 'position': p, 'q': q, 'phase': phase})
        if ground_gate and not mode_restoring and now>=next_hold:
            o=ground_observation()
            send('trs',ground_gate.hold_message(enc['trs'],attitude['time_boot_ms'],o))
            ground_gate.record_actual_hold_send(elapsed())
            hold_sends.append({'elapsed_s':elapsed(),'position':ground_gate.origin.position,'yaw':ground_gate.origin.yaw})
            next_hold=now+.05
        next_emit += 1 / 32
        if next_emit < now:
            next_emit = now + 1 / 32

    def pump():
        nonlocal next_params, buffer, attitude, pad_pending, home_seen
        now = elapsed()
        if vision_mode and not Path(USB).exists():
            raise RuntimeError('Known FC USB disappeared; no reconnect or flight test')
        if (active and not restoring and fusion_started is not None
                and time.monotonic() - fusion_started >= (20 if ground_check else 10)):
            raise FusionWindowEnded()
        if now > 60 and not restoring:
            raise RuntimeError('Static bench global time limit')
        rclpy.spin_once(node, timeout_sec=0)
        if pad_fd is not None:
            try:
                chunk=os.read(pad_fd,EVENT.size*64)
                if not chunk:raise RuntimeError('Gamepad device disconnected during unarmed ground check')
                pad_pending+=chunk
            except BlockingIOError:
                pass
            while len(pad_pending)>=EVENT.size:
                _,_,kind,code,value=EVENT.unpack(pad_pending[:EVENT.size]);pad_pending=pad_pending[EVENT.size:]
                if kind==0 and code==3:raise RuntimeError('Gamepad input overflow')
                if kind==1 and code==HOME and value in (1,2):home_seen=True
        for route, port in ports.items():
            if restoring and route != 'usb':
                continue  # TRS故障不阻止经同机USB的独立参数回退。
            for msg in dec[route].parse_buffer(port.read(min(port.in_waiting, 65536))) or []:
                ev[route].ingest(msg, now)
                if ev[route].fatal:
                    raise RuntimeError(ev[route].fatal)
                if ev[route].identity and ev[route].identity['uid2'] != UID2:
                    raise RuntimeError('Wrong aircraft identity')
                if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                    continue
                if route=='trs' and msg.get_type()=='COMMAND_ACK' and msg.command==176:
                    mode_acks.append({'elapsed_s':elapsed(),'result':msg.result,
                        'target_system':msg.target_system,'target_component':msg.target_component})
                if route == 'usb' and msg.get_type() == 'PARAM_VALUE' and str(msg.param_id) in policy.READ_PARAMS:
                    name = str(msg.param_id)
                    if name in ev['usb'].requests:
                        parameters[name] = decode_parameter(msg)
                        param_times[name] = now
                        if not restoring and name not in ('EKF2_EV_CTRL', 'EKF2_EV_DELAY') and parameters[name] != baseline[name]:
                            raise RuntimeError('Monitored parameter diverged from backup: ' + name)
                        if not restoring and name in ('EKF2_EV_CTRL', 'EKF2_EV_DELAY'):
                            candidate = 11 if name == 'EKF2_EV_CTRL' else policy.DELAY_MS
                            allowed_values = (baseline[name], candidate) if name in possibly_changed else (baseline[name],)
                            if parameters[name] not in allowed_values:
                                raise RuntimeError('Unrequested or out-of-scope parameter change: ' + name)
                if route == 'usb' and msg.get_type() == 'SERIAL_CONTROL':
                    buffer += bytes(msg.data[:msg.count]).decode('utf-8', 'replace')
                    if len(buffer) > 24000:
                        raise RuntimeError('USB listener overflow')
                if route == 'usb' and msg.get_type() == 'ATTITUDE_QUATERNION':
                    if attitude and msg.time_boot_ms < attitude['time_boot_ms']:
                        raise RuntimeError('Flight-controller boot clock regressed; stop')
                    if attitude is None or msg.time_boot_ms > attitude['time_boot_ms']:
                        attitude = {'q': checked_quaternion([msg.q1,msg.q2,msg.q3,msg.q4]).tolist(),
                                    'elapsed_s': now, 'time_boot_ms': msg.time_boot_ms}
                if route == 'usb' and msg.get_type() == 'STATUSTEXT':
                    status_texts.append({'elapsed_s': now, 'text': str(msg.text), 'severity': msg.severity})
            if now >= next_hb[route]:
                send(route, enc[route].heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
                next_hb[route] = now + 1
            if ev[route].heartbeats and ev[route].identity is None and now >= next_version[route]:
                ev[route].version_requested_at = now
                send(route, enc[route].command_long_encode(1,1,512,0,148,0,0,0,0,0,0))
                next_version[route] = now + 2
        if identity('usb') and now >= next_params:
            for name in policy.READ_PARAMS:
                ev['usb'].requests[name] = now
                send('usb', enc['usb'].param_request_read_encode(1,1,name.encode(),-1))
            next_params = now + 2
        emit()
        time.sleep(.001)

    def listener(topic):
        nonlocal buffer, arm, arm_at, local_live, local_at
        parsers = {'actuator_armed': parse_actuator_snapshot, 'vehicle_visual_odometry': parse_visual_snapshot,
                   'visual_series': compact.visual_series, 'selector': combined.parse_selector,
                   'local_position': combined.parse_local_position, 'failsafe': combined.parse_failsafe}
        parse = (combined.parse_flags if topic.startswith('flags_') else
                 combined.parse_status if topic.startswith('status_') else
                 combined.parse_height_aid if topic.startswith('height_aid_') else
                 combined.parse_yaw_aid if topic.startswith('yaw_aid_') else
                 combined.parse_aid if topic.startswith('aid_') else parsers[topic])
        buffer = ''
        payload = policy.COMMANDS[topic]
        requested, next_poll = time.monotonic(), time.monotonic() + .25
        send('usb', enc['usb'].serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0')))
        while time.monotonic() - requested < 3:
            pump()
            if not safe():
                raise RuntimeError('Baseline freshness lost during internal observation')
            result = parse(buffer)
            if result and reply_complete(buffer, payload):
                listeners.append({'topic': topic, 'elapsed_s': elapsed(), 'text': buffer, 'parsed': result})
                if topic == 'actuator_armed':
                    arm, arm_at = result, elapsed()
                    if arm['armed'] or not arm['lockdown']:
                        raise RuntimeError('Disarmed output lock lost')
                if ground_check and topic=='local_position':
                    counters=parse_topic(buffer,'vehicle_local_position',('xy_reset_counter','z_reset_counter','heading_reset_counter'))
                    if not counters:raise RuntimeError('Missing live reset counters; no frozen target')
                    local_live={**result,**counters};local_at=elapsed()
                return result
            if time.monotonic() >= next_poll:
                send('usb', shell_message(enc['usb']))
                next_poll = time.monotonic() + .25
        listeners.append({'topic': topic, 'elapsed_s': elapsed(), 'text': buffer, 'incomplete': True})
        raise RuntimeError('Incomplete internal listener: ' + topic)

    def write_parameter(name, value):
        # Full parameter backups preserve integer values as JSON floats.
        # Restore integer parameters with their declared wire type, not Python float.
        value = int(value) if name == 'EKF2_EV_CTRL' else float(value)
        if value != baseline[name]:
            possibly_changed.add(name)  # 在发送前记录，ACK丢失也必须回退。
        when = elapsed()
        send('usb', policy.parameter_message(enc['usb'], name, value))
        writes.append({'elapsed_s': when, 'name': name, 'value': value})
        ev['usb'].requests[name] = when
        send('usb', enc['usb'].param_request_read_encode(1,1,name.encode(),-1))
        until = time.monotonic() + 3
        while time.monotonic() < until:
            pump()
            if param_times.get(name, -100) >= when and parameters.get(name) == value:
                if value == baseline[name]:
                    restored[name] = True
                return
        raise RuntimeError('Parameter write/read timeout: ' + name)

    try:
        if ground_check:
            pad=find_pad();pad_fd=os.open(pad['path'],os.O_RDONLY|os.O_NONBLOCK)
        for route, path in (('usb', USB), ('trs', TRS)):
            ports[route] = serial.Serial(path,115200,timeout=0,write_timeout=.5,exclusive=True)
            if vision_mode:
                ports[route].reset_input_buffer()
        while not safe() or node.gate.accepted < 50 or not node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns()):
            pump()
            if elapsed() > 12:
                raise RuntimeError('Identity/backup/source preflight timeout; no writes')
        if any(parameters[n] != baseline[n] for n in policy.READ_PARAMS):
            raise RuntimeError('Live baseline does not match full backup')
        if ground_check:
            initial_mode=(ev['trs'].heartbeats[-1]['custom_mode']>>16)&255
            initial_submode=(ev['trs'].heartbeats[-1]['custom_mode']>>24)&255
            if not restorable_mode(initial_mode,initial_submode):
                raise RuntimeError('Ground check requires exactly restorable manual/altitude/position/loiter/land baseline')
        arm_before = listener('actuator_armed')
        selected = listener('selector')
        primary = selected['primary_instance']
        if not 0 <= primary <= 8 or selected['gyro_fault_detected'] or selected['accel_fault_detected']:
            raise RuntimeError('Unhealthy estimator selector')
        flags = listener(f'flags_{primary}')
        local = listener('local_position')
        if (not flags['cs_tilt_align'] or not flags['cs_baro_hgt'] or not local['z_valid']
                or not combined.inactive_other_heading_sources(flags)
                or any(flags[n] for n in ('cs_in_air','cs_ev_pos','cs_ev_hgt','cs_ev_yaw','cs_ev_vel',
                                          'fs_bad_pos_d','fs_bad_hdg'))):
            raise RuntimeError('Static tilt/baro/heading prerequisites not satisfied')
        attitude_deadline = time.monotonic() + 2
        while not attitude or elapsed() - attitude['elapsed_s'] > .25:
            pump()
            if not safe() or time.monotonic() > attitude_deadline:
                raise RuntimeError('Fresh FC attitude unavailable; no freshness threshold relaxed')
        origin_p, origin_q = transform_rigid_pose(node.gate.last.position,node.gate.last.quaternion,offset,mount)
        alignment = compact.yaw_alignment(origin_q, attitude['q'])
        listener('actuator_armed')
        usec, seq = time.monotonic_ns()//1000, secrets.randbits(31)
        ev['trs'].pings[(usec,seq)] = elapsed()
        send('trs', enc['trs'].ping_encode(usec,seq,0,0))
        active = True
        stream_start = next_emit = time.monotonic()
        until = time.monotonic() + .75
        while time.monotonic() < until:
            pump()
        group = listener('visual_series')
        timing = compact.series_timing(group)
        receive_groups.append({'phase': phase, 'timing': timing, 'snapshots': group})
        receive_time_verified = all(policy.receive_time_mode(s) for s in group)
        rate_was_verified = timing['rate_hz'] >= 30 and timing['maximum_publication_gap_ms'] <= 50
        if not (receive_time_verified and rate_was_verified and compact.match(group[-1], sent) and ev['trs'].ping_rtt_ms):
            raise RuntimeError('TRS real reception/rate/receive-time prerequisite failed; no parameters written')
        listener('actuator_armed')
        write_parameter('EKF2_EV_DELAY', policy.DELAY_MS)
        write_parameter('EKF2_EV_CTRL', 11)
        phase = 'temporary_xyz_yaw_fusion'
        fusion_started = time.monotonic()
        print('STATIC FUSION: TRS pose32Hz, provisional EV_DELAY65ms, EV_CTRL11; no arming or motion', flush=True)
        for target in (2,5,8):
            while time.monotonic() - fusion_started < target:
                pump()
            listener('actuator_armed')
            selected = listener('selector')
            if selected['primary_instance'] != primary:
                raise RuntimeError('Primary estimator changed; restore')
            if vision_mode and not aid_slots:
                count = selected['instances_available']
                if not 1 <= count <= 9:
                    raise RuntimeError('Invalid EKF count for aid discovery')
                for prefix in ('aid','height_aid','yaw_aid'):
                    aid_mappings[prefix] = {slot:listener(f'{prefix}_{slot}') for slot in range(count)}
                    aid_slots[prefix] = aid_topic_for_estimator(aid_mappings[prefix],primary)
            aids = {prefix:listener(f'{prefix}_{aid_slots.get(prefix, primary)}')
                    for prefix in ('aid','height_aid','yaw_aid')}
            if vision_mode and any(a['estimator_instance'] != primary for a in aids.values()):
                raise RuntimeError('Aid topic no longer belongs to primary EKF')
            row = {'selector': selected, 'flags': listener(f'flags_{primary}'),
                   **aids, 'local': listener('local_position'),
                   'status': listener(f'status_{primary}'), 'failsafe': listener('failsafe')}
            if not attitude:
                raise RuntimeError('FC attitude missing')
            row['yaw_comparison'] = combined.compare_heading(sent[-1]['q'],attitude['q'],
                attitude_age_ms=(elapsed()-attitude['elapsed_s'])*1000,
                sent_age_ms=(elapsed()-sent[-1]['elapsed_s'])*1000)
            row['elapsed_s'] = elapsed()
            snap = listener('vehicle_visual_odometry')
            if not policy.receive_time_mode(snap) or not compact.match(snap, sent):
                raise RuntimeError('Receive-time interpretation or received pose changed; restore')
            row['visual'] = snap
            fusion_rows.append(row)
            if row['flags']['cs_in_air']:
                raise RuntimeError('In-air flag appeared in unpowered bench')
            print('FUSION SNAPSHOT: ' + str({k:row['flags'][k] for k in ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw')}),flush=True)
        if loss:
            if not combined.fusion_evidence_ok(fusion_rows, primary):
                raise RuntimeError('Current static fusion not verified; no planned source loss')
            loss.mask(elapsed(), fusion_verified=True)
            node.mask_input = True
            print('SOURCE LOSS: masking only this subscriber; shared Vicon receiver stays running', flush=True)
            while not loss.latch.blocked:
                pump()
                if elapsed() - loss.masked_at_s > .4:
                    raise RuntimeError('Source loss not latched within bounded observation')
        else:
            if ground_check:
                if not combined.fusion_evidence_ok(fusion_rows,primary) or not control.assess(fusion_rows,primary)['estimator_prerequisites_observed']:
                    raise RuntimeError('Actual XYZ/yaw/control readiness missing; zero hold targets')
                listener('actuator_armed');listener('local_position')
                ground_gate=DisarmedHoldGate(ground_observation(),operator_authorized=True)
                deadline=time.monotonic()+7
                next_observe=0
                while time.monotonic()<deadline:
                    pump()
                    if time.monotonic()>=next_observe:
                        listener('actuator_armed');listener('local_position')
                        next_observe=time.monotonic()+.2
                    if mode_request_at is None and ground_gate.sent_count>=20 and elapsed()-ground_gate.first_sent>=2:
                        msg=ground_gate.mode_message(enc['trs'],ground_observation())
                        mode_request_at=elapsed()
                        send('trs',msg)
                    replies=[a for a in mode_acks if mode_request_at is not None and a['elapsed_s']>=mode_request_at
                             and (a['target_system'],a['target_component'])==(250,198)]
                    if replies and replies[-1]['result']!=m.MAV_RESULT_ACCEPTED:
                        raise RuntimeError('Disarmed Offboard mode request rejected: '+str(replies[-1]['result']))
                    hb=ev['trs'].heartbeats[-1]
                    if replies and hb['elapsed_s']>=mode_request_at and ((hb['custom_mode']>>16)&255)==6:
                        offboard_mode_verified=True;break
                if not offboard_mode_verified:raise RuntimeError('No matching ACK and observed Offboard mode within ground-check deadline')
            else:
                while time.monotonic() - fusion_started < 10:
                    pump()
        fusion_finished = time.monotonic()
    except FusionWindowEnded:
        fusion_finished = time.monotonic()
        if len(fusion_rows) != 3:
            error = 'Fixed fusion deadline reached before three complete snapshots'
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc)
    finally:
        active = False
        if ground_check and mode_request_at is not None:
            try:
                mode_restoring=True;mode_restore_at=elapsed()
                send('trs',enc['trs'].command_long_encode(1,1,176,0,1,initial_mode,initial_submode,0,0,0,0))
                until=time.monotonic()+3
                while time.monotonic()<until:
                    pump()
                    hb=ev['trs'].heartbeats[-1]
                    if (hb['elapsed_s']>=mode_restore_at and ((hb['custom_mode']>>16)&255)==initial_mode
                            and ((hb['custom_mode']>>24)&255)==initial_submode):
                        mode_restored=True;break
                if not mode_restored:raise RuntimeError('Original unarmed mode restore not observed')
            except Exception as exc:restore_errors.append('Ground mode restore: '+str(exc))
            mode_restoring=False
        restoring = True
        if fusion_started is not None and fusion_finished is None:
            fusion_finished = time.monotonic()
        # 先停发，再关闭融合，最后恢复延迟；每项独立尝试，源/时钟丢失不阻止回退。
        for name in ('EKF2_EV_CTRL','EKF2_EV_DELAY'):
            if name not in possibly_changed:
                continue
            try:
                write_parameter(name, baseline[name])
            except Exception as exc:
                restore_errors.append(name + ': ' + str(exc))
        if loss and loss.masked_at_s is not None:
            try:
                # Only restore this subscriber after rollback, with transmission
                # already disabled. Observe that incoming data cannot restart it.
                node.mask_input = False
                recover_until = time.monotonic() + .4
                while time.monotonic() < recover_until:
                    pump()
                    if not restore_safe():
                        raise RuntimeError('USB identity lost during source recovery observation')
                loss_result = loss.result(last_tx_s=sent[-1]['elapsed_s'],
                    tx_count_after_recovery=len(sent), suppressed=node.suppressed,
                    fresh_restored=node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns()),
                    rollback_verified=all(restored.get(n) for n in ('EKF2_EV_CTRL','EKF2_EV_DELAY')))
                if not loss_result['verified']:
                    restore_errors.append('Planned source-loss verification incomplete; no retry')
            except Exception as exc:
                restore_errors.append('Source recovery observer: ' + str(exc))
        restoring = False
        if 'usb' in ports:
            try:
                if safe():
                    arm_after = listener('actuator_armed')
            except Exception as exc:
                restore_errors.append('Final observer: ' + str(exc))
        for port in ports.values():
            port.close()
        if pad_fd is not None:os.close(pad_fd)
        node.destroy_node()
        rclpy.shutdown()
    fusion_ok = bool(not error and not restore_errors and combined.fusion_evidence_ok(fusion_rows, primary))
    result = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope': 'Unpowered stationary wireless fusion with provisional arrival delay; NOT flight validation',
              'identities': {r:ev[r].identity for r in ev}, 'tx_messages': {r:dict(tx[r]) for r in tx},
              'received_fc_messages': {r:dict(ev[r].fc_counts) for r in ev},
              'last_fc_attitude': attitude,
              'parameter_writes': writes, 'parameters_after': parameters, 'restored': restored,
              'backup_path': str(args.backup), 'actuator_before': arm_before, 'actuator_after': arm_after,
              'source_accepted': node.gate.accepted, 'source_rejected': node.gate.rejected,
              'source_last_reason': node.gate.reason,
              'heading_candidate_path': str(args.heading_candidate),
              'board_center_candidate_path': str(args.board_center_candidate),
              'internal_listeners': listeners, 'sent_poses': sent, 'receive_groups': receive_groups,
              'fusion_rows': fusion_rows, 'status_texts': status_texts,
              'arrival_time_fallback_verified': receive_time_verified,
              'pre_enable_receiver_rate_verified': rate_was_verified,
              'fusion_duration_s': fusion_finished-fusion_started if fusion_finished else 0,
              'ekf_stationary_xyz_yaw_fusion_verified': fusion_ok,
              'control_readiness_during_fusion': control.assess(fusion_rows,primary),
              'alignment_yaw_candidate_rad': alignment,
              'frame_scope': 'FC-yaw-aligned local NED candidate; not calibrated North or flight control mapping',
              'delay_compensation_ms_provisional': policy.DELAY_MS,
              'trs_sampling_clock_conversion_verified': False, 'camera_exposure_latency_verified': False,
              'imu_reference_verified': False, 'heading_state_rollback_attempted': False,
              'arming_mode_motion_or_unlock_commands_sent':bool(hold_sends or mode_request_at is not None),
              'arming_or_output_unlock_commands_sent':False,
              'usb_used_as_readonly_observer_and_parameter_maintenance': True,
              'standalone_wireless_fusion_verified': False, 'flight_ready': False,
              'static_permit_path': str(args.static_permit) if args.static_permit else None,
              'vision_reference_permit_path': str(args.vision_reference_permit) if vision_mode else None,
              'vision_reference_trs_static_mode': vision_mode,
              'rigid_revision_static_validation': revision_validation,
              'rigid_body_revision_id': heading.get('rigid_body_revision_id'),
              'revision_registry_modified': False,
              'aid_topic_slots': aid_slots, 'aid_topic_mappings': aid_mappings,
              'physical_scope': 'battery-off/props-installed/USB-only/locked' if vision_mode else 'legacy props-off confirmation',
              'disarmed_offboard_ground_check_requested':ground_check,
              'disarmed_offboard_mode_verified':offboard_mode_verified,
              'original_unarmed_mode_restored':mode_restored,
              'original_mode':initial_mode,'original_submode':initial_submode,
              'mode_request_at_s':mode_request_at,'mode_acks':mode_acks,
              'hold_target_sends':hold_sends,
              'general_incident_guard_retained': bool(permit),
              'height_reference_preserved': baseline['EKF2_HGT_REF'],
              'error': error, 'restore_errors': restore_errors, 'serial_ports_released': True}
    result['source_loss_test_requested'] = args.source_loss_test
    result['local_source_loss'] = loss_result
    result['source_loss_verified'] = bool(loss and loss_result and loss_result['verified'] and fusion_ok)
    result['failed_gate_observation'] = failed_gate_observation
    result['static_origin_position_frd_m'] = origin_p
    result['static_origin_quaternion_wxyz'] = origin_q
    with args.output.open('x') as f:
        json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('internal_listeners','sent_poses','fusion_rows','status_texts')},ensure_ascii=False,indent=2))
    return 0 if fusion_ok and (not loss or result['source_loss_verified']) and (not ground_check or (offboard_mode_verified and mode_restored)) else 2


if __name__ == '__main__':
    raise SystemExit(main())
