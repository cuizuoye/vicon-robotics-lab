#!/usr/bin/env python3
"""Vicon→USB台架：默认只预检；显式短时水平/XYZ辅助，始终禁止飞行。"""

import argparse
from collections import Counter
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import serial
from persistent_shell import reply_complete
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.mavlink_vision import PoseGate, finite_vector, checked_quaternion, transform_rigid_pose, odometry_message
from mavlink_readonly_probe import Evidence, decode_parameter
from estop_only_policy import parse_actuator_snapshot
from usb_vision_policy import (PARAMS, fusion_disabled, send_allowed, shell_message,
                              parse_visual_snapshot, match_snapshot, parse_timesync_snapshot,
                              check_clock_probe, SourceLossLatch, ClockReadiness, assess_clock_series)
import usb_fusion_policy as horizontal_fusion
import usb_height_fusion_policy as height_fusion
import usb_yaw_fusion_policy as yaw_fusion
import usb_combined_fusion_policy as combined_fusion
from combined_input_loss import CombinedInputLoss
from manual_translation import endpoint as translation_endpoint, assess_translation, validate_control
from manual_vertical import endpoint as vertical_endpoint, assess_vertical
from incident_guard import require_active_bench_clear
import receive_only_permit
import horizontal_fusion_permit
import vision_reference_xyz_policy
import vision_reference_fusion_permit
from mavlink_readonly_probe import FC_USB
from rigid_body_revision import require_current_calibration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--uid2', required=True)
    parser.add_argument('--heading-candidate', type=Path, required=True)
    parser.add_argument('--board-center-candidate', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--bench-send', action='store_true')
    permit_group = parser.add_mutually_exclusive_group()
    permit_group.add_argument('--receive-only-permit', type=Path,
                        help='One-use battery-off USB static receive exception; forbids fusion and all parameter writes')
    permit_group.add_argument('--clock-sample-permit', type=Path,
                        help='Separate one-use fusion-off exception for exactly one held clock sample, no stream')
    permit_group.add_argument('--horizontal-fusion-permit', type=Path,
                        help='Separate one-use static horizontal-only exception; props installed, battery off, backup required')
    permit_group.add_argument('--vision-reference-permit', type=Path,
                        help='Separate one-use static XYZ exception; Vision reference and EV_CTRL 2->3->2')
    parser.add_argument('--clock-loss-check', action='store_true',
                        help='Fusion-off clock warmup, held real sample, local input-loss test')
    parser.add_argument('--clock-status-only', action='store_true', help='Only fixed internal clock listener, no position')
    parser.add_argument('--clock-characterize', action='store_true', help='60 second synchronization-only drift/lag capture')
    parser.add_argument('--clock-probe-only', action='store_true', help='Stability-gated held real pose only; no stream or repeated loss test')
    parser.add_argument('--clock-settle-seconds', type=float, default=30,
                        help='Bounded continuous stability wait, 15..300s; does not authorize motion')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fusion-preflight', action='store_true', help='Zero-write EKF/selector prerequisite checks')
    parser.add_argument('--horizontal-fusion-bench', action='store_true', help='Temporary EV_CTRL 0->1->0, no flight')
    parser.add_argument('--height-fusion-bench', action='store_true', help='Temporary EV_CTRL 0->3->0, baro reference unchanged, no flight')
    parser.add_argument('--yaw-fusion-bench', action='store_true', help='Temporary EV_CTRL 0->8->0; may reset heading state, no flight')
    parser.add_argument('--combined-fusion-bench', action='store_true', help='Temporary LOCAL_FRD EV_CTRL 0->11->0; XYZ/yaw, no velocity or flight')
    parser.add_argument('--combined-source-loss-bench', action='store_true', help='Only with combined bench: mask this subscriber after verified fusion, latch stop and restore0; no shared receiver/network change')
    parser.add_argument('--restore-horizontal-only', action='store_true', help='Restore backed-up EV_CTRL=0 only, no positions')
    parser.add_argument('--restore-height-only', action='store_true', help='Restore backed-up EV_CTRL 0/3 to 0 only, no positions')
    parser.add_argument('--restore-yaw-only', action='store_true', help='Restore backed-up EV_CTRL 0/8 to 0; NOT heading state undo')
    parser.add_argument('--restore-combined-only', action='store_true', help='Restore backed-up EV_CTRL 0/11 to 0, no positions or state undo')
    parser.add_argument('--fusion-physical-confirmed', action='store_true', help='Battery disconnected, props removed, USB only, stable body')
    parser.add_argument('--fusion-backup', type=Path, help='Fresh complete parameter backup directory with SHA256 manifest')
    parser.add_argument('--manual-translation-bench', action='store_true', help='Bounded unpowered manual translation during horizontal bench')
    parser.add_argument('--manual-vertical-bench', action='store_true', help='Bounded unpowered manual lift during XYZ aiding, no flight')
    parser.add_argument('--manual-window-seconds', type=float, default=180, help='Manual observation window, 60..180s')
    parser.add_argument('--manual-control-file', type=Path, help='New finish/cancel observation JSON in evidence directory')
    args = parser.parse_args()
    if args.output.exists() or not 5 <= args.seconds <= 10:
        parser.error('New output and 5..10 seconds required')
    if not 15 <= args.clock_settle_seconds <= 300:
        parser.error('Clock stability wait must be 15..300 seconds')
    if args.clock_loss_check and not args.bench_send:
        parser.error('--clock-loss-check requires explicit --bench-send')
    if args.clock_status_only and args.bench_send:
        parser.error('--clock-status-only forbids position transmission')
    if args.clock_characterize and (args.bench_send or args.clock_status_only):
        parser.error('--clock-characterize is a standalone zero-position diagnostic')
    if args.clock_probe_only and (not args.bench_send or args.clock_loss_check):
        parser.error('--clock-probe-only requires --bench-send and excludes --clock-loss-check')
    fusion = (vision_reference_xyz_policy if args.vision_reference_permit else
              combined_fusion if args.combined_fusion_bench or args.restore_combined_only else
              yaw_fusion if args.yaw_fusion_bench or args.restore_yaw_only else
              height_fusion if args.height_fusion_bench or args.restore_height_only else horizontal_fusion)
    active_fusion = args.horizontal_fusion_bench or args.height_fusion_bench or args.yaw_fusion_bench or args.combined_fusion_bench
    restore_fusion = args.restore_horizontal_only or args.restore_height_only or args.restore_yaw_only or args.restore_combined_only
    use_yaw = args.yaw_fusion_bench or args.combined_fusion_bench
    use_height = args.height_fusion_bench or args.combined_fusion_bench
    enable_value = (11 if args.combined_fusion_bench or args.restore_combined_only else
                    8 if args.yaw_fusion_bench or args.restore_yaw_only else
                    3 if args.height_fusion_bench or args.restore_height_only else 1)
    restore_value = 2 if args.vision_reference_permit else 0
    fusion_mode = args.fusion_preflight or active_fusion or restore_fusion
    if sum((args.fusion_preflight, args.horizontal_fusion_bench, args.height_fusion_bench,
            args.yaw_fusion_bench, args.combined_fusion_bench, args.restore_horizontal_only,
            args.restore_height_only, args.restore_yaw_only, args.restore_combined_only)) > 1:
        parser.error('Select one fusion mode')
    if fusion_mode and (args.clock_loss_check or args.clock_probe_only or args.clock_characterize or args.clock_status_only):
        parser.error('Fusion modes cannot be mixed with separate clock modes')
    if active_fusion and not (args.bench_send and (args.fusion_physical_confirmed or args.horizontal_fusion_permit or args.vision_reference_permit)):
        parser.error('Active fusion requires --bench-send and explicit physical bench confirmation')
    if args.combined_source_loss_bench and not args.combined_fusion_bench:
        parser.error('Planned combined source-loss requires explicit combined fusion bench')
    if (args.fusion_preflight or restore_fusion) and args.bench_send:
        parser.error('Fusion preflight/restore cannot send positions')
    manual_mode = args.manual_translation_bench or args.manual_vertical_bench
    if args.manual_translation_bench and args.manual_vertical_bench:
        parser.error('Select one manual measurement mode')
    if manual_mode:
        if ((args.manual_translation_bench and not args.horizontal_fusion_bench)
                or (args.manual_vertical_bench and not args.height_fusion_bench)
                or not 60 <= args.manual_window_seconds <= 180
                or not args.manual_control_file or args.manual_control_file.exists()
                or args.manual_control_file.resolve().parent != args.output.resolve().parent
                or any((args.output.parent / n).exists() for n in ('manual_baseline.json','manual_endpoint.json'))):
            parser.error('Manual bench requires matching explicit fusion mode, bounded window and fresh local evidence files')
    if args.bench_send:
        try:
            require_current_calibration(args.heading_candidate, args.board_center_candidate)
        except (OSError, ValueError, KeyError) as exc:
            parser.error(str(exc))
    receive_permit = None
    horizontal_permit = None
    vision_permit = None
    restricted_permit_path = args.receive_only_permit or args.clock_sample_permit
    if args.vision_reference_permit:
        try:
            vision_reference_fusion_permit.validate_request(args)
            if not FC_USB.exists() or Path(args.port).resolve() != FC_USB.resolve():
                raise ValueError('Known FC USB must remain present')
            vision_permit = vision_reference_fusion_permit.consume(args.vision_reference_permit,
                args.output, args.heading_candidate, args.board_center_candidate, args.fusion_backup)
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
    elif args.horizontal_fusion_permit:
        try:
            horizontal_fusion_permit.validate_request(args)
            if not FC_USB.exists() or Path(args.port).resolve() != FC_USB.resolve():
                raise ValueError('Known FC USB must remain present')
            horizontal_permit = horizontal_fusion_permit.consume(args.horizontal_fusion_permit,
                args.output, args.heading_candidate, args.board_center_candidate, args.fusion_backup)
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
    elif restricted_permit_path:
        try:
            receive_only_permit.validate_request(args, clock_sample=bool(args.clock_sample_permit))
            if not FC_USB.exists() or Path(args.port).resolve() != FC_USB.resolve():
                raise ValueError('Known FC USB must remain present')
            receive_permit = receive_only_permit.consume(restricted_permit_path, args.output,
                                                         args.heading_candidate, args.board_center_candidate,
                                                         clock_sample=bool(args.clock_sample_permit))
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
    elif (args.bench_send or args.fusion_preflight or active_fusion):
        require_active_bench_clear(parser)
    restricted_static = bool(receive_permit or horizontal_permit or vision_permit)
    measurement_endpoint = vertical_endpoint if args.manual_vertical_bench else translation_endpoint
    active_seconds = args.manual_window_seconds if manual_mode else args.seconds
    max_packets = 1 if args.clock_sample_permit else 301 if horizontal_permit or vision_permit else int(active_seconds*30)+10 if manual_mode else 300
    clock_bench = args.clock_loss_check or args.clock_probe_only or active_fusion
    read_params = fusion.READ_PARAMS if fusion_mode else PARAMS
    backup_values = {}
    if fusion_mode:
        if not args.fusion_backup:
            parser.error('Fusion mode requires --fusion-backup')
        backup_path = args.fusion_backup / 'parameters_verified.json'
        manifest = json.loads((args.fusion_backup / 'parameters_manifest.json').read_text())
        backup = json.loads(backup_path.read_text())
        if (not backup.get('complete') or backup.get('received_count') != backup.get('expected_count')
                or backup.get('firmware', {}).get('version') != '1.14.3'
                or hashlib.sha256(backup_path.read_bytes()).hexdigest() != manifest['sha256']['parameters_verified.json']
                or not 0 <= time.time() - backup_path.stat().st_mtime < 3600):
            parser.error('Incomplete, mismatched, corrupted or old backup; no serial writes')
        backup_values = {row['name']: row['value'] for row in backup['records']}
        if backup_values.get('EKF2_EV_CTRL') != restore_value or any(name not in backup_values for name in read_params):
            parser.error('Required baseline backup must contain selected restore value and all monitored parameters')
        if (use_height or use_yaw or args.restore_height_only or args.restore_yaw_only or args.restore_combined_only) and not fusion.fusion_scope_ok(backup_values):
            parser.error('Height aiding trial requires existing baro reference/aid and zero EV offsets; no automatic reference change')
    if subprocess.run(['fuser', str(Path(args.port).resolve())], capture_output=True).returncode == 0:
        parser.error('Serial port occupied; no process will be killed')
    heading = json.loads(args.heading_candidate.read_text())
    board = json.loads(args.board_center_candidate.read_text())
    if heading.get('manual_forward_horizontal_confirmed') is not True or board.get('marker_name') != 'MircoAir4':
        parser.error('Explicit physical reference candidates required')
    mount = checked_quaternion(heading['body_orientation_in_rigid_xyzw_candidate'])
    offset = finite_vector(board['board_center_in_rigid_m_candidate'], 3)
    if board.get('imu_offset_verified') is not False:
        parser.error('This tool uses a labelled approximate BOARD center, not flight-calibrated IMU')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped

    class Source(Node):
        def __init__(self):
            super().__init__('usb_vicon_bench_source')
            self.gate = PoseGate(max_age_ms=150)
            self.accept_input = True
            self.observed = self.suppressed = 0
            self.kept = self.create_subscription(
                PoseStamped, '/vicon/MircoAir/MircoAir', self.receive,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE))

        def receive(self, msg):
            self.observed += 1
            if not self.accept_input:
                self.suppressed += 1
                return
            p, q = msg.pose.position, msg.pose.orientation
            self.gate.ingest(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                             self.get_clock().now().nanoseconds, time.monotonic_ns(), msg.header.frame_id,
                             [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    rclpy.init()
    node = Source()
    evidence = Evidence(expected_parameters=())
    encoder = m.MAVLink(None, srcSystem=250, srcComponent=190)
    decoder = m.MAVLink(None)
    decoder.robust_parsing = True
    tx = Counter()
    parameters, parameter_times = {}, {}
    snapshots, sent, shell_records = [], [], []
    arm_before = arm_after = visual_before = None
    next_hb = next_version = next_params = 0
    shell_buffer = ''
    start = time.monotonic()
    stream_started = None
    last_sync_request = None
    error = None
    clock_status = clock_probe = loss_result = None
    loss_latch = SourceLossLatch()
    combined_loss = CombinedInputLoss() if args.combined_source_loss_bench else None
    clock_rows, sync_requests = [], []
    clock_ready = ClockReadiness()
    backup_verified = clock_verified = False
    enable_may_have_applied = restoring = False
    rollback_verified = False
    rollback_error = None
    parameter_writes, fusion_rows = [], []
    fc_status_texts = []
    selector_at_abort = None
    selector_before = flags_before = selector_after = flags_after = local_before = local_after = None
    primary = None
    horizontal_aid_topic_instance = None
    horizontal_aid_mapping = None
    height_aid_topic_instance = None
    height_aid_mapping = None
    latest_fc_attitude = None
    attitude_before = attitude_after = None
    manual_pairs = []
    manual_start = manual_end = None
    finish_requested_at = None
    last_control_sequence = 0
    receive_arm = None
    receive_arm_at = None
    static_reference = None
    horizontal_window = horizontal_fusion_permit.FusionWindow() if horizontal_permit or vision_permit else None
    horizontal_window_expired = False

    def restore_safe():
        now = time.monotonic() - start
        return bool(identified() and evidence.heartbeats and now - evidence.heartbeats[-1]['elapsed_s'] < 2
                    and not evidence.heartbeats[-1]['armed'])

    def identified():
        return evidence.identity is not None and evidence.identity['uid2'] == args.uid2 and evidence.identity['firmware'] == '1.14.3'

    def safe():
        now = time.monotonic() - start
        return (identified() and evidence.heartbeats and now - evidence.heartbeats[-1]['elapsed_s'] < 2
                and not evidence.heartbeats[-1]['armed']
                and (fusion.fusion_scope_ok(parameters) if fusion_mode else fusion_disabled(parameters))
                and all(now - parameter_times.get(name, -100) < 4 for name in read_params))

    def send(msg):
        if restricted_static and msg.get_type() == 'ODOMETRY':
            if (receive_arm_at is None or time.monotonic()-start-receive_arm_at > 2.5
                    or not receive_arm or not receive_arm['lockdown'] or receive_arm['armed']):
                raise RuntimeError('Fresh locked output state required before each position send')
        permitted = (fusion.allowed(msg, authorized=active_fusion or restore_fusion,
                                     safe=bool(safe()), restore_safe=restore_safe(), backup_verified=backup_verified,
                                     clock_verified=clock_verified, position_authorized=active_fusion)
                     if fusion_mode else send_allowed(msg, bench_authorized=args.bench_send, safe=bool(safe())))
        if not permitted:
            raise ValueError('USB bench policy rejected ' + msg.get_type())
        packet = msg.pack(encoder)
        if port.write(packet) != len(packet):
            raise OSError('Short serial write')
        encoder.seq = (encoder.seq + 1) % 256
        tx[msg.get_type()] += 1

    def pump():
        nonlocal next_hb, next_version, next_params, shell_buffer, last_sync_request, latest_fc_attitude
        rclpy.spin_once(node, timeout_sec=.001)
        now = time.monotonic() - start
        if restricted_static and not FC_USB.exists():
            raise RuntimeError('FC USB disappeared; no reconnect or retry')
        if horizontal_window:
            horizontal_window.check(time.monotonic(), restoring=restoring)
        global_limit = 95 + args.clock_settle_seconds if clock_bench else (120 if args.clock_characterize else 35)
        if manual_mode:
            global_limit += active_seconds
        if now > global_limit and not restoring:
            raise RuntimeError('Bench global timeout')
        for msg in decoder.parse_buffer(port.read(min(port.in_waiting, 65536))) or []:
            evidence.ingest(msg, now)
            if evidence.fatal:
                raise RuntimeError(evidence.fatal)
            if evidence.identity and not identified():
                raise RuntimeError('Wrong UID or unsupported firmware')
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                continue
            if msg.get_type() == 'STATUSTEXT':
                # 只保留被动诊断，不新增发送、选择实例或解除锁定操作；分片按原样保留。
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode('utf-8', 'replace')
                fc_status_texts.append({'elapsed_s': now, 'severity': int(msg.severity),
                                       'text': str(text).rstrip('\0'),
                                       'id': getattr(msg, 'id', None),
                                       'chunk_seq': getattr(msg, 'chunk_seq', None)})
                if len(fc_status_texts) > 100:
                    del fc_status_texts[0]
            if msg.get_type() == 'ATTITUDE_QUATERNION':
                try:
                    q = checked_quaternion([msg.q1,msg.q2,msg.q3,msg.q4]).tolist()
                    latest_fc_attitude = {'q': q, 'elapsed_s': now, 'time_boot_ms': msg.time_boot_ms}
                except ValueError:
                    latest_fc_attitude = None
            if msg.get_type() == 'PARAM_VALUE' and msg.param_id in read_params and msg.param_id in evidence.requests:
                parameters[msg.param_id] = decode_parameter(msg)
                parameter_times[msg.param_id] = now
                if not fusion_mode and parameters[msg.param_id] != 0:
                    raise RuntimeError('Bench requires fusion disabled and zero FC offset: ' + msg.param_id)
                if fusion_mode and not restoring:
                    expected = backup_values[msg.param_id]
                    if msg.param_id == 'EKF2_EV_CTRL':
                        if parameters[msg.param_id] not in (restore_value, enable_value):
                            raise RuntimeError('Fusion scope exceeded: EV_CTRL outside selected profile')
                    elif parameters[msg.param_id] != expected:
                        raise RuntimeError('Monitored parameter changed from backup: ' + msg.param_id)
            if msg.get_type() == 'SERIAL_CONTROL':
                shell_buffer += bytes(msg.data[:msg.count]).decode('utf-8', 'replace')
                if len(shell_buffer) > 16384:
                    raise RuntimeError('Listener response overflow')
            if msg.get_type() == 'TIMESYNC' and msg.tc1 == 0 and msg.ts1 > 0 and safe():
                if msg.ts1 != last_sync_request:
                    last_sync_request = msg.ts1
                    pc_reply_ns = time.monotonic_ns()
                    sync_requests.append({'elapsed_s': now, 'fc_originate_ns': msg.ts1,
                                          'pc_receive_ns': pc_reply_ns,
                                          'pc_raw_ns': time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)})
                    send(encoder.timesync_encode(pc_reply_ns, msg.ts1))
        if now >= next_hb:
            send(encoder.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            next_hb = now + 1
        hb_fresh = evidence.heartbeats and now - evidence.heartbeats[-1]['elapsed_s'] < 2
        if hb_fresh and not evidence.identity and now >= next_version:
            evidence.version_requested_at = now
            send(encoder.command_long_encode(1, 1, m.MAV_CMD_REQUEST_MESSAGE, 0,
                                            m.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0))
            next_version = now + 2
        if identified() and hb_fresh and now >= next_params:
            for name in read_params:
                evidence.requests[name] = now
                send(encoder.param_request_read_encode(1, 1, name.encode(), -1))
            next_params = now + 2

    def listener(topic, emit=None):
        nonlocal shell_buffer, receive_arm, receive_arm_at
        shell_buffer = ''
        requested = time.monotonic() - start
        request = (fusion.fusion_shell_message(encoder, topic) if topic in fusion.COMMANDS and fusion_mode else shell_message(encoder, topic))
        send(request)
        poll_at = 0
        while time.monotonic() - start - requested < 4:
            pump()
            if not safe():
                raise RuntimeError('Safety/parameter freshness lost during listener')
            if emit:
                emit()
            parsers = {'actuator_armed': parse_actuator_snapshot,
                      'vehicle_visual_odometry': parse_visual_snapshot,
                      'timesync_status': parse_timesync_snapshot,
                      'selector': fusion.parse_selector, 'local_position': fusion.parse_local_position}
            parse = (fusion.parse_status if topic.startswith('status_') else
                     fusion.parse_failsafe if topic == 'failsafe' else
                     fusion.parse_yaw_aid if topic.startswith('yaw_aid_') else
                     fusion.parse_height_aid if topic.startswith('height_aid_') else
                     fusion.parse_flags if topic.startswith('flags_') else
                     fusion.parse_aid if topic.startswith('aid_') else parsers[topic])
            parsed = parse(shell_buffer)
            absent = (topic == 'vehicle_visual_odometry' or topic.startswith(('aid_', 'height_aid_'))) and 'never published' in shell_buffer
            if (parsed or absent) and reply_complete(shell_buffer, bytes(request.data[:request.count])):
                record = {'topic': topic, 'requested_at_s': requested,
                          'reply_at_s': time.monotonic()-start, 'text': shell_buffer,
                          'parsed': parsed, 'never_published': absent,
                          'latest_fc_attitude_clock': latest_fc_attitude}
                shell_records.append(record)
                if restricted_static and topic == 'actuator_armed':
                    boot_ms = latest_fc_attitude['time_boot_ms'] if latest_fc_attitude else None
                    record['actuator_minus_attitude_clock_us'] = (
                        parsed['timestamp_us']-boot_ms*1000 if parsed and boot_ms is not None else None)
                    if not receive_only_permit.locked_state(parsed, boot_ms):
                        raise RuntimeError('Receive-only test requires fresh disarmed output lockdown')
                    receive_arm, receive_arm_at = parsed, time.monotonic()-start
                return parsed
            if time.monotonic() >= poll_at:
                send(shell_message(encoder))
                poll_at = time.monotonic() + .25
        shell_records.append({'topic': topic, 'requested_at_s': requested,
                              'reply_at_s': time.monotonic() - start,
                              'text': shell_buffer, 'parsed': None, 'incomplete': True})
        raise RuntimeError('Incomplete listener reply: ' + topic)

    next_emit = 0

    def send_sample(sample):
        nonlocal static_reference
        if not safe():
            raise RuntimeError('Safety gate lost before position encoding')
        if clock_bench and clock_ready.count < clock_ready.required:
            raise RuntimeError('Clock startup gate not ready; no position sent')
        now_ns = time.monotonic_ns()
        ros_now = node.get_clock().now().nanoseconds
        if not (0 <= ros_now - sample.stamp_ns <= node.gate.max_age_ns
                and 0 <= now_ns - sample.received_monotonic_ns <= node.gate.max_age_ns):
            raise RuntimeError('Held sample expired; no position sent')
        if len(sent) >= max_packets:
            raise RuntimeError('Position packet hard limit reached')
        stamp_us = (now_ns - (ros_now - sample.stamp_ns)) // 1000
        position, q = transform_rigid_pose(sample.position, sample.quaternion, offset, mount)
        if restricted_static:
            import math
            if static_reference is None:
                static_reference = (position, q)
            distance = math.dist(position, static_reference[0])
            dot = abs(sum(a*b for a,b in zip(q, static_reference[1])))
            angle = math.degrees(2*math.acos(min(1., dot)))
            if distance > .01 or angle > 2.:
                raise RuntimeError('Static reception boundary exceeded; no further poses')
        send(odometry_message(encoder, stamp_us, position, q))
        sent.append({'elapsed_s': time.monotonic() - start, 'source_stamp_ns': sample.stamp_ns,
                     'wire_time_usec': stamp_us, 'position': position, 'q': q})

    def emit():
        nonlocal next_emit
        now_ns = time.monotonic_ns()
        now = now_ns / 1e9
        if stream_started is None or now - stream_started >= active_seconds:
            return
        fresh = node.gate.fresh(node.get_clock().now().nanoseconds, now_ns)
        if combined_loss and combined_loss.masked_at_s is not None:
            if not combined_loss.permits(fresh,time.monotonic()-start,len(sent)):
                return
        if args.clock_loss_check and not loss_latch.permits(fresh):
            return
        if not safe() or not fresh:
            raise RuntimeError('Live source/heartbeat/fusion gate lost; transmission stopped')
        if now < next_emit:
            return
        sample = node.gate.take_new(node.get_clock().now().nanoseconds, now_ns)
        if sample is None:
            return
        # ROS发布stamp换到PC monotonic时域；不是Vicon曝光同步，更不设置FC实时钟。
        send_sample(sample)
        next_emit = now + 1 / 30

    def write_control(value):
        nonlocal enable_may_have_applied, rollback_verified
        if value != restore_value and (not clock_verified or not node.gate.fresh(node.get_clock().now().nanoseconds, time.monotonic_ns())):
            raise RuntimeError('No clock/source permission for temporary fusion enable')
        if value != restore_value:
            # 保守记录“可能写入”，短写/异常也进入finally回退，不能等ACK才标记。
            enable_may_have_applied = True
            if horizontal_window:
                horizontal_window.start(time.monotonic())
        requested = time.monotonic() - start
        send(fusion.control_message(encoder, value))
        parameter_writes.append({'elapsed_s': requested, 'name': 'EKF2_EV_CTRL', 'value': value})
        evidence.requests['EKF2_EV_CTRL'] = requested
        send(encoder.param_request_read_encode(1, 1, b'EKF2_EV_CTRL', -1))
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            pump()
            if (parameter_times.get('EKF2_EV_CTRL', -100) > requested
                    and parameters.get('EKF2_EV_CTRL') == value):
                if value == restore_value:
                    rollback_verified = True
                    if horizontal_window:
                        horizontal_window.restored()
                return
        raise RuntimeError('EV_CTRL write/read verification timeout: ' + str(value))

    try:
        with serial.Serial(args.port, 115200, timeout=0, write_timeout=.5, exclusive=True) as port:
            # Host backlog is not current FC time. Clear only host receive data,
            # then collect live telemetry without positions or state changes.
            port.reset_input_buffer()
            settle_at = time.monotonic()
            while time.monotonic()-settle_at < 1.:
                pump()
            while not safe():
                pump()
                if time.monotonic() - start > 8:
                    raise RuntimeError('No verified disarmed/fusion-off preflight')
            arm_before = listener('actuator_armed')
            if arm_before['armed']:
                raise RuntimeError('Internal actuator state armed')
            visual_before = listener('vehicle_visual_odometry')
            if fusion_mode:
                if (any(parameters.get(name) != backup_values[name] for name in read_params
                        if name != 'EKF2_EV_CTRL') or parameters.get('EKF2_EV_CTRL') not in
                        ((restore_value, enable_value) if restore_fusion else (restore_value,))):
                    raise RuntimeError('Live fusion baseline does not match complete backup')
                backup_verified = True
                if not arm_before['lockdown']:
                    raise RuntimeError('Existing output lockdown not retained; no fusion trial')
                if restore_fusion:
                    restoring = True
                    write_control(restore_value)
                    restoring = False
                else:
                    selector_before = listener('selector')
                    primary = selector_before['primary_instance']
                    if not 0 <= primary <= 8 or selector_before['gyro_fault_detected'] or selector_before['accel_fault_detected']:
                        raise RuntimeError('No healthy primary estimator selection')
                    flags_before = listener(f'flags_{primary}')
                    local_before = listener('local_position')
                    if not flags_before['cs_tilt_align'] or flags_before['cs_in_air'] or flags_before['cs_ev_pos']:
                        raise RuntimeError('Tilt/in-air/prior-fusion prerequisites not satisfied')
                    if use_height and (flags_before['cs_ev_hgt'] or not flags_before['cs_baro_hgt']
                                                   or not local_before['z_valid'] or flags_before['fs_bad_pos_d']):
                        raise RuntimeError('Healthy baro height baseline required; no height enable')
                    if use_yaw and (not fusion.inactive_other_heading_sources(flags_before)
                                                or flags_before['cs_ev_yaw'] or flags_before['cs_ev_vel']
                                                or flags_before['cs_ev_hgt'] or flags_before['fs_bad_hdg']):
                        raise RuntimeError('Conflicting navigation source or heading fault; no yaw enable')
                    attitude_before = latest_fc_attitude
            if args.clock_status_only:
                clock_status = listener('timesync_status')
            if args.clock_characterize:
                observed_start = time.monotonic()
                next_status = observed_start
                while time.monotonic() - observed_start < 60:
                    pump()
                    if not safe():
                        raise RuntimeError('Safety gate lost during zero-position clock characterization')
                    if time.monotonic() >= next_status:
                        clock_status = listener('timesync_status')
                        pc_now_us = time.monotonic_ns() // 1000
                        ready = clock_ready.update(clock_status, pc_now_us)
                        clock_rows.append({'elapsed_s': time.monotonic() - start, 'pc_read_us': pc_now_us,
                                           'status': clock_status, 'ready': ready, 'consecutive_good': clock_ready.count})
                        print(f'CLOCK READ: {time.monotonic()-observed_start:.1f}s, '
                              f'residual={clock_status["observed_offset"]-clock_status["estimated_offset"]}us, '
                              f'rtt={clock_status["round_trip_time"]}us, consecutive={clock_ready.count}', flush=True)
                        next_status = time.monotonic() + 5
            if args.bench_send:
                source_wait_started = time.monotonic()
                while node.gate.accepted < 50 or not node.gate.fresh(node.get_clock().now().nanoseconds, time.monotonic_ns()):
                    pump()
                    if not safe() or time.monotonic() - source_wait_started > 5:
                        raise RuntimeError('No fresh live Vicon source; zero position packets sent')
                if clock_bench:
                    warm_start = time.monotonic()
                    sync_base = tx['TIMESYNC']
                    next_progress = 0
                    print('CLOCK: warming up with TIMESYNC only; zero new position packets', flush=True)
                    while time.monotonic() - warm_start < 60 or tx['TIMESYNC'] - sync_base < 550:
                        pump()
                        if not safe():
                            raise RuntimeError('Safety gate lost during clock warmup')
                        if time.monotonic() - warm_start > 75:
                            raise RuntimeError('Insufficient synchronization replies within 75 seconds')
                        if time.monotonic() >= next_progress:
                            print(f'CLOCK: {time.monotonic()-warm_start:.1f}s, replies={tx["TIMESYNC"]-sync_base}', flush=True)
                            next_progress = time.monotonic() + 10
                    stable_started = time.monotonic()
                    while True:
                        clock_status = listener('timesync_status')
                        pc_now_us = time.monotonic_ns() // 1000
                        ready = clock_ready.update(clock_status, pc_now_us)
                        clock_rows.append({'elapsed_s': time.monotonic() - start, 'pc_read_us': pc_now_us,
                                           'status': clock_status, 'ready': ready, 'consecutive_good': clock_ready.count})
                        print(f'CLOCK GATE: residual={clock_status["observed_offset"]-clock_status["estimated_offset"]}us, '
                              f'consecutive={clock_ready.count}/3', flush=True)
                        if ready:
                            break
                        if time.monotonic() - stable_started > args.clock_settle_seconds:
                            raise RuntimeError('Clock stability gate timed out; zero positions sent')
                        wait_until = time.monotonic() + 2
                        while time.monotonic() < wait_until:
                            pump()
                            if not safe():
                                raise RuntimeError('Safety gate lost while waiting for clock stability')
                    if restricted_static:
                        listener('actuator_armed')
                    sample = node.gate.take_new(node.get_clock().now().nanoseconds, time.monotonic_ns())
                    if sample is None:
                        raise RuntimeError('No real sample for clock probe')
                    held_at = time.monotonic()
                    while time.monotonic() - held_at < .05:
                        pump()
                    send_sample(sample)
                    probe_snapshot = listener('vehicle_visual_odometry')
                    clock_probe = check_clock_probe(clock_status, probe_snapshot, sent[-1], time.monotonic_ns() // 1000)
                    clock_probe['snapshot'] = probe_snapshot
                    if not clock_probe['verified'] or not clock_probe['clock_accuracy_candidate_pass']:
                        raise RuntimeError('FC sampling-time conversion not verified; no loss test')
                    print('CLOCK: stability gate and internal timestamp probe verified', flush=True)
                    clock_verified = True
                if active_fusion:
                    try:
                        current_arm = listener('actuator_armed')
                        if current_arm['armed'] or not current_arm['lockdown']:
                            raise RuntimeError('Internal disarmed/lockdown state changed before fusion')
                        current_selector = listener('selector')
                        current_flags = listener(f'flags_{primary}')
                        if (current_selector['primary_instance'] != primary or not current_flags['cs_tilt_align']
                                or current_flags['cs_ev_pos'] or current_flags['cs_in_air']):
                            raise RuntimeError('Estimator prerequisites changed before enable; no write')
                        if use_yaw and not fusion.inactive_other_heading_sources(current_flags):
                            raise RuntimeError('Heading source changed before yaw enable; no write')
                        write_control(enable_value)
                        stream_started = time.monotonic()
                        next_check = stream_started + 2
                        next_pair = stream_started
                        next_control = stream_started
                        next_locked_check = stream_started + .5
                        print(f'FUSION: temporary EV_CTRL={enable_value}; height reference unchanged, bounded stream, no flight', flush=True)
                        while time.monotonic() - stream_started < active_seconds:
                            pump()
                            emit()
                            if (horizontal_permit or vision_permit) and time.monotonic() >= next_locked_check:
                                listener('actuator_armed', emit)
                                next_locked_check = time.monotonic() + 1
                            if combined_loss and combined_loss.latch.blocked:
                                print('LOSS: stale input latched; zero further position writes; restoring fusion off',flush=True)
                                break
                            if time.monotonic() >= next_check:
                                selected = listener('selector', emit)
                                if selected['primary_instance'] != primary:
                                    selector_at_abort = selected
                                    raise RuntimeError('Primary EKF instance changed; stop and restore')
                                flags = listener(f'flags_{primary}', emit)
                                if horizontal_permit or vision_permit:
                                    if horizontal_aid_topic_instance is None:
                                        count = selected['instances_available']
                                        if not 1 <= count <= 9:
                                            raise RuntimeError('Invalid EKF count for aid-topic discovery')
                                        horizontal_aid_mapping = {
                                            slot: listener(f'aid_{slot}', emit) for slot in range(count)}
                                        horizontal_aid_topic_instance = horizontal_fusion.aid_topic_for_estimator(
                                            horizontal_aid_mapping, primary)
                                    aid = listener(f'aid_{horizontal_aid_topic_instance}', emit)
                                    if not aid or aid['estimator_instance'] != primary:
                                        raise RuntimeError('Aid topic no longer belongs to primary EKF; stop and restore')
                                else:
                                    aid = listener(f'aid_{primary}', emit) if not args.yaw_fusion_bench else None
                                if vision_permit:
                                    if height_aid_topic_instance is None:
                                        height_aid_mapping = {slot: listener(f'height_aid_{slot}', emit)
                                            for slot in range(selected['instances_available'])}
                                        height_aid_topic_instance = horizontal_fusion.aid_topic_for_estimator(height_aid_mapping, primary)
                                    height_aid = listener(f'height_aid_{height_aid_topic_instance}', emit)
                                    if not height_aid or height_aid['estimator_instance'] != primary:
                                        raise RuntimeError('Height aid no longer belongs to primary EKF')
                                else:
                                    height_aid = listener(f'height_aid_{primary}', emit) if use_height else None
                                yaw_aid = listener(f'yaw_aid_{primary}', emit) if use_yaw else None
                                local = listener('local_position', emit)
                                current_clock = listener('timesync_status', emit)
                                if not clock_ready.update(current_clock, time.monotonic_ns() // 1000):
                                    raise RuntimeError('Clock readiness lost during fusion; stop and restore')
                                pair_at = time.monotonic()-start
                                yaw_comparison = (fusion.compare_heading(sent[-1]['q'], latest_fc_attitude['q'],
                                                                        attitude_age_ms=(pair_at-latest_fc_attitude['elapsed_s'])*1000,
                                                                        sent_age_ms=(pair_at-sent[-1]['elapsed_s'])*1000)
                                                  if use_yaw and sent and latest_fc_attitude else None)
                                status = listener(f'status_{primary}', emit) if args.combined_fusion_bench else None
                                failsafe = listener('failsafe', emit) if args.combined_fusion_bench else None
                                fusion_rows.append({'elapsed_s': pair_at, 'selector': selected,
                                                    'aid_topic_instance': horizontal_aid_topic_instance,
                                                    'height_aid_topic_instance': height_aid_topic_instance,
                                                    'flags': flags, 'aid': aid, 'height_aid': height_aid,
                                                    'yaw_aid': yaw_aid, 'yaw_comparison': yaw_comparison,
                                                    'local': local, 'clock': current_clock,
                                                    'status': status, 'failsafe': failsafe})
                                if use_yaw and not fusion.inactive_other_heading_sources(flags):
                                    raise RuntimeError('Unexpected active GPS/magnetic heading source; stop and restore')
                                if (combined_loss and combined_loss.masked_at_s is None
                                        and fusion.fusion_evidence_ok(fusion_rows,primary)):
                                    combined_loss.mask(time.monotonic()-start,fusion_verified=True)
                                    node.accept_input=False
                                    print('LOSS: verified fusion; suppressing only this local subscriber input',flush=True)
                                next_check = time.monotonic() + 2
                            if manual_mode and time.monotonic() >= next_pair:
                                local = listener('local_position', emit)
                                pair_at = time.monotonic()-start
                                if (sent and latest_fc_attitude and pair_at-latest_fc_attitude['elapsed_s'] < .3
                                        and (not manual_pairs or local['timestamp_sample'] > manual_pairs[-1]['local']['timestamp_sample'])
                                        and (not manual_pairs or sent[-1]['source_stamp_ns'] > manual_pairs[-1]['sent']['source_stamp_ns'])):
                                    manual_pairs.append({'elapsed_s': pair_at, 'local': local, 'sent': sent[-1],
                                                         'fc_q': latest_fc_attitude['q'],
                                                         'attitude_age_ms': (pair_at-latest_fc_attitude['elapsed_s'])*1000})
                                next_pair = time.monotonic() + .2
                                if manual_start is None and fusion.fusion_evidence_ok(fusion_rows[-2:], primary):
                                    manual_start = measurement_endpoint(manual_pairs[-5:])
                                    if manual_start:
                                        with (args.output.parent / 'manual_baseline.json').open('x') as f:
                                            json.dump(manual_start, f, indent=2, allow_nan=False)
                                        print('MANUAL READY: stationary start captured; wait for operator move instruction', flush=True)
                                if finish_requested_at is not None:
                                    end_pairs = [r for r in manual_pairs if r['elapsed_s'] > finish_requested_at]
                                    manual_end = measurement_endpoint(end_pairs[-5:])
                                    if manual_end:
                                        with (args.output.parent / 'manual_endpoint.json').open('x') as f:
                                            json.dump(manual_end, f, indent=2, allow_nan=False)
                                        print('MANUAL END: stationary endpoint captured; restoring fusion off', flush=True)
                                        break
                                    if time.monotonic()-start-finish_requested_at > 5:
                                        raise RuntimeError('No settled endpoint within 5 seconds; stop and restore')
                            if manual_mode and time.monotonic() >= next_control:
                                if args.manual_control_file.exists():
                                    control = validate_control(json.loads(args.manual_control_file.read_text()))
                                    if control['sequence'] > last_control_sequence:
                                        last_control_sequence = control['sequence']
                                        if control['command'] == 'cancel':
                                            raise RuntimeError('Manual observation cancelled; stop and restore')
                                        if manual_start is None:
                                            raise RuntimeError('Finish requested before valid start; no displacement result')
                                        if finish_requested_at is None:
                                            finish_requested_at = time.monotonic()-start
                                next_control = time.monotonic() + .2
                    except horizontal_fusion_permit.WindowExpired:
                        horizontal_window_expired = True
                        print(f'STATIC: enable window ended; restoring EV_CTRL={restore_value}', flush=True)
                    finally:
                        stream_started = None
                        if enable_may_have_applied:
                            restoring = True
                            try:
                                write_control(restore_value)
                            except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
                                rollback_error = str(exc) or 'Restore interrupted'
                            restoring = False
                    if rollback_error:
                        raise RuntimeError('CRITICAL: EV_CTRL restore not verified: ' + rollback_error)
                    if combined_loss:
                        node.accept_input=True
                        restored_at=time.monotonic()
                        while time.monotonic()-restored_at<.3:
                            pump()
                        loss_result=combined_loss.result(last_tx_s=sent[-1]['elapsed_s'],
                            tx_count_after_recovery=len(sent),suppressed=node.suppressed,
                            fresh_restored=node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns()),
                            rollback_verified=rollback_verified)
                        if not loss_result['verified']:
                            raise RuntimeError('Combined local source-loss validation incomplete; no repeat or restart')
                    # 给EKF停止EV融合留出时间；只读，不再发定位。
                    stopped_at = time.monotonic()
                    while time.monotonic() - stopped_at < 1:
                        pump()
                    selector_after = listener('selector')
                    flags_after = listener(f'flags_{primary}')
                    local_after = listener('local_position')
                stream_started = None if args.clock_probe_only or active_fusion else time.monotonic()
                next_snapshot = (stream_started or time.monotonic()) + (.5 if receive_permit else 2)
                if not args.clock_probe_only and not active_fusion:
                    print('USB BENCH: identity/disarmed/fusion-off checked; starting bounded 30Hz Vicon transmission', flush=True)
                while not args.clock_probe_only and not active_fusion and time.monotonic() - stream_started < (2 if args.clock_loss_check else args.seconds):
                    pump()
                    emit()
                    if time.monotonic() >= next_snapshot:
                        if receive_permit:
                            listener('actuator_armed', emit)
                        snapshot = listener('vehicle_visual_odometry', emit)
                        if snapshot is None:
                            raise RuntimeError('Visual topic still never published after send')
                        snapshot['matching_sent_sample'] = match_snapshot(snapshot, sent[-60:])
                        snapshots.append(snapshot)
                        next_snapshot = time.monotonic() + (1 if receive_permit else 2)
                if args.clock_loss_check:
                    masked_at = time.monotonic() - start
                    observed_before = node.observed
                    node.accept_input = False
                    loss_started = time.monotonic()
                    while time.monotonic() - loss_started < .4:
                        pump()
                        emit()
                        if not safe():
                            raise RuntimeError('Safety gate lost during local loss test')
                    count_at_stale = len(sent)
                    stale_first = listener('vehicle_visual_odometry')
                    stopped_at = time.monotonic()
                    while time.monotonic() - stopped_at < 1:
                        pump()
                        emit()
                        if not safe():
                            raise RuntimeError('Safety gate lost during inhibited observation')
                    stale_second = listener('vehicle_visual_odometry')
                    # 仅恢复本地订阅输入，锁存仍禁止发送；不触碰真实接收器。
                    node.accept_input = True
                    restored_at = time.monotonic()
                    while time.monotonic() - restored_at < .3:
                        pump()
                        emit()
                    loss_result = {'kind': 'local subscriber input suppression, NOT optical occlusion',
                                   'masked_at_s': masked_at, 'inhibited': loss_latch.blocked,
                                   'tx_count_when_stale': count_at_stale, 'tx_count_after_recovery': len(sent),
                                   'last_tx_relative_to_mask_ms': (sent[-1]['elapsed_s'] - masked_at)*1000,
                                   'source_messages_observed_during_check': node.observed - observed_before,
                                   'locally_suppressed_messages': node.suppressed,
                                   'snapshot_before_wait': stale_first, 'snapshot_after_wait': stale_second,
                                   'fresh_source_restored': node.gate.fresh(node.get_clock().now().nanoseconds, time.monotonic_ns())}
                    loss_result['verified'] = bool(loss_latch.blocked and len(sent) == count_at_stale
                                                   and node.suppressed > 50 and loss_result['fresh_source_restored']
                                                   and stale_first and stale_first == stale_second
                                                   and loss_result['last_tx_relative_to_mask_ms'] <= 150)
            arm_after = listener('actuator_armed')
            attitude_after = latest_fc_attitude
            if arm_after['armed']:
                raise RuntimeError('Unexpected internal armed state')
            # 再读一遍关闭融合的参数，不能用发送前快照代替发送后验证。
            request_end = time.monotonic() - start
            next_params = 0
            while not all(parameter_times.get(name, -100) > request_end for name in read_params):
                pump()
                if time.monotonic() - start - request_end > 4:
                    raise RuntimeError('Postflight parameter read timeout')
    except (OSError, ValueError, RuntimeError, serial.SerialException, KeyboardInterrupt) as exc:
        error = str(exc) or 'Interrupted'
    finally:
        node.destroy_node()
        rclpy.shutdown()

    baseline_stamp = visual_before['timestamp'] if visual_before else 0
    received = (args.bench_send and not error and len(sent) >= 100 and len(snapshots) >= 2
                and all(s['matching_sent_sample'] for s in snapshots)
                and snapshots[0]['timestamp'] > baseline_stamp
                and all(b['timestamp'] > a['timestamp'] and b['timestamp_sample'] > a['timestamp_sample']
                        for a, b in zip(snapshots, snapshots[1:]))
                and fusion_disabled(parameters) and arm_before and arm_after
                and not arm_before['armed'] and not arm_after['armed'])
    if receive_permit:
        received = bool(received and arm_before['lockdown'] and arm_after['lockdown'] and len(sent) <= 300)
    elapsed = [r['elapsed_s'] for r in sent]
    result = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'receive_only_permit': receive_permit,
              'horizontal_fusion_permit': horizontal_permit,
              'vision_reference_permit': vision_permit,
              'vision_reference_static_xyz_requested': bool(vision_permit),
              'maintenance_restore_value': restore_value,
              'height_aid_topic_instance': height_aid_topic_instance,
              'height_aid_mapping': height_aid_mapping,
              'horizontal_enable_window_expired': horizontal_window_expired,
              'horizontal_aid_topic_instance': horizontal_aid_topic_instance,
              'horizontal_aid_mapping': horizontal_aid_mapping,
              'receive_only_exception_used': bool(receive_permit),
              'general_guards_modified': False,
              'single_clock_sample_requested': bool(args.clock_sample_permit),
              'single_clock_sample_verified': bool(args.clock_sample_permit and not error
                  and len(sent) == 1 and clock_verified and fusion_disabled(parameters)
                  and arm_before and arm_after and arm_before['lockdown'] and arm_after['lockdown']),
              'scope': ('Static XYZ fusion with Vision height reference; restore EV_CTRL=2 and stop stream; NOT flight validation'
                        if vision_permit else 'Temporary LOCAL_FRD combined XYZ/yaw aiding; no velocity or flight validation'
                        if args.combined_fusion_bench else 'Temporary LOCAL_FRD yaw-only aiding; heading state may change; NOT flight validation'
                        if args.yaw_fusion_bench else 'Bounded unpowered manual vertical XYZ aiding; unchanged baro reference; NOT flight validation'
                        if args.manual_vertical_bench else 'Temporary XYZ aiding bench, unchanged baro height reference; NOT flight validation'
                        if args.height_fusion_bench else 'Temporary horizontal-only fusion bench; NOT flight validation'
                        if args.horizontal_fusion_bench else 'Disarmed USB reception/preflight bench; NOT flight validation'),
              'reference': 'approximate flight-controller BOARD center, NOT calibrated IMU',
              'identity': evidence.identity, 'parameters_after': parameters,
              'source_heading': str(args.heading_candidate.resolve()),
              'source_board_center': str(args.board_center_candidate.resolve()),
              'authorized_bench_send': args.bench_send,
              'actual_sent_to_fc': len(sent), 'tx_messages': dict(tx),
              'transmit_rate_hz': (len(sent) - 1) / (elapsed[-1] - elapsed[0]) if len(sent) > 1 else None,
              'maximum_transmit_gap_ms': max((b-a)*1000 for a,b in zip(elapsed,elapsed[1:])) if len(sent) > 1 else None,
              'source_gate': {'accepted': node.gate.accepted, 'rejected': node.gate.rejected, 'expired': node.gate.expired},
              'actuator_before': arm_before, 'actuator_after': arm_after,
              'visual_before': visual_before, 'visual_snapshots': snapshots,
              'listener_records': shell_records, 'sent': sent, 'heartbeats': evidence.heartbeats,
              'fc_visual_reception_verified': bool(received), 'preflight_passed': bool(not error and arm_before and fusion_disabled(parameters)),
              'fusion_disabled_confirmed': fusion_disabled(parameters),
              'fusion_enable_command_sent': False, 'imu_origin_verified': False,
              'camera_time_sync_verified': False, 'flight_ready': False,
              'clock_loss_check_requested': args.clock_loss_check,
              'clock_status': clock_status, 'clock_probe': clock_probe, 'local_source_loss': loss_result,
              'clock_loss_check_passed': bool(args.clock_loss_check and not error and clock_probe
                                              and clock_probe['verified'] and loss_result and loss_result['verified']),
              'clock_accuracy_candidate_pass': bool(clock_probe and clock_probe.get('clock_accuracy_candidate_pass')),
              'clock_probe_only_requested': args.clock_probe_only,
              'clock_stability_wait_limit_s': args.clock_settle_seconds,
              'clock_characterization': assess_clock_series(clock_rows, sync_requests) if clock_rows else None,
              'clock_readiness': {'ready': clock_ready.count >= clock_ready.required,
                                  'consecutive_good': clock_ready.count, 'required': clock_ready.required},
              'clock_status_series': clock_rows, 'sync_requests': sync_requests,
              'error': error, 'serial_port_released': True}
    result.update({'rx_message_counts': dict(evidence.counts), 'fc_status_texts': fc_status_texts,
                   'selector_at_abort': selector_at_abort,
                   'status_text_collection_is_passive_and_bounded': True})
    fusion_passed = bool(active_fusion and not error and rollback_verified
                         and flags_after and not flags_after['cs_ev_pos']
                         and (not use_height or bool(vision_permit) or not flags_after['cs_ev_hgt'])
                         and (not use_yaw or not flags_after['cs_ev_yaw'])
                         and selector_after['primary_instance'] == primary
                         and arm_before and arm_after and arm_before['lockdown'] and arm_after['lockdown']
                         and fusion.fusion_evidence_ok(fusion_rows, primary))
    result.update({'fusion_preflight_requested': args.fusion_preflight,
                   'horizontal_fusion_bench_requested': args.horizontal_fusion_bench,
                   'height_fusion_bench_requested': args.height_fusion_bench,
                   'yaw_fusion_bench_requested': args.yaw_fusion_bench,
                   'combined_fusion_bench_requested': args.combined_fusion_bench,
                   'combined_source_loss_bench_requested': args.combined_source_loss_bench,
                   'combined_source_loss_verified': bool(args.combined_source_loss_bench and not error
                        and loss_result and loss_result['verified'] and fusion_passed),
                   'fusion_enable_command_sent': any(row['value'] != restore_value for row in parameter_writes),
                   'vision_reference_static_xyz_verified': bool(vision_permit and fusion_passed),
                   'reference_selected_for_bench': 'approximate board center; IMU/CG lever arms unverified',
                   'fusion_backup_verified': backup_verified, 'parameter_writes': parameter_writes,
                   'rollback_verified': rollback_verified, 'rollback_error': rollback_error,
                   'enable_attempted_or_may_have_applied': enable_may_have_applied,
                   'primary_estimator': primary, 'selector_before': selector_before, 'flags_before': flags_before,
                   'local_before': local_before, 'selector_after': selector_after, 'flags_after': flags_after,
                   'local_after': local_after, 'fusion_snapshots': fusion_rows,
                   'horizontal_fusion_verified': bool(args.horizontal_fusion_bench and fusion_passed),
                   'xyz_aiding_verified': bool(args.height_fusion_bench and fusion_passed),
                   'yaw_aiding_verified': bool(args.yaw_fusion_bench and fusion_passed),
                   'combined_aiding_verified': bool(args.combined_fusion_bench and fusion_passed),
                   'combined_control_prerequisites': combined_fusion.control.assess(fusion_rows, primary)
                        if args.combined_fusion_bench else None,
                   'fc_attitude_before': attitude_before, 'fc_attitude_after': attitude_after,
                   'heading_state_undo_not_attempted': bool(use_yaw or args.restore_yaw_only or args.restore_combined_only)})
    result.update({'manual_translation_requested': args.manual_translation_bench,
                   'manual_vertical_requested': args.manual_vertical_bench,
                   'manual_window_limit_s': active_seconds if manual_mode else None,
                   'position_packet_limit': max_packets,
                   'manual_start': manual_start, 'manual_end': manual_end,
                   'manual_finish_requested_at_s': finish_requested_at,
                   'manual_observation_window_expired_without_finish': bool(manual_mode
                       and finish_requested_at is None and not error and rollback_verified),
                   'manual_pairs': manual_pairs,
                   'manual_translation': assess_translation(manual_start, manual_end) if args.manual_translation_bench else None,
                   'manual_vertical': assess_vertical(manual_start, manual_end) if args.manual_vertical_bench else None})
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    with args.output.open('x') as stream:
        stream.write(encoded + '\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('sent','heartbeats','listener_records','sync_requests','clock_status_series','manual_pairs','fusion_snapshots')}, indent=2, ensure_ascii=False))
    if args.clock_loss_check:
        return 0 if not error and clock_probe and clock_probe['verified'] and loss_result and loss_result['verified'] else 2
    if args.clock_probe_only:
        return 0 if not error and clock_probe and clock_probe['verified'] and clock_probe['clock_accuracy_candidate_pass'] else 2
    if active_fusion:
        measurement = result['manual_vertical'] if args.manual_vertical_bench else result['manual_translation']
        return 0 if fusion_passed and (not manual_mode or measurement['verified']) else 2
    return 0 if received or (not args.bench_send and not error) else 2


if __name__ == '__main__':
    sys.exit(main())
