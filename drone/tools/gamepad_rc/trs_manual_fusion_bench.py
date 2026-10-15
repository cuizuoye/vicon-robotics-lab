#!/usr/bin/env python3
"""断动力、拆桨、锁定输出的有限手动平移/抬升；不控制飞机、不起飞。"""
import argparse
import os
import fcntl
from collections import Counter
import datetime
import hashlib
import json
from pathlib import Path
import secrets
import subprocess
import sys
import time

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
import trs_manual_fusion_policy as policy
from trs_static_fusion_bench import load_backup
from gamepad_estop_monitor import EVENT, HOME, find_pad
from manual_translation import endpoint as horizontal_endpoint, assess_translation, validate_control
from manual_vertical import endpoint as vertical_endpoint, assess_vertical
from incident_guard import require_active_bench_clear
from rigid_body_revision import require_current_calibration



def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--backup', type=Path, required=True)
    ap.add_argument('--heading-candidate', type=Path, required=True)
    ap.add_argument('--board-center-candidate', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--physical-confirmed', action='store_true')
    ap.add_argument('--axis', choices=('translation','vertical'), required=True)
    ap.add_argument('--control-file', type=Path, required=True)
    ap.add_argument('--window-seconds', type=float, default=120)
    args = ap.parse_args()
    require_active_bench_clear(ap)
    if not args.physical_confirmed or args.output.exists():
        ap.error('Unpowered/props-off/static confirmation and new output required')
    if (args.control_file.exists() or not 60 <= args.window_seconds <= 180
            or args.control_file.resolve().parent != args.output.resolve().parent
            or any((args.output.parent/n).exists() for n in ('manual_baseline.json','manual_endpoint.json'))):
        ap.error('New local finish/cancel file and bounded 60..180s manual window required')
    try:
        require_current_calibration(args.heading_candidate, args.board_center_candidate)
    except (OSError, ValueError, KeyError) as exc:
        ap.error(str(exc))
    baseline = load_backup(args.backup)
    for path in (USB, TRS):
        if subprocess.run(['fuser', str(Path(path).resolve())], capture_output=True).returncode == 0:
            ap.error('Port occupied; no process will be killed: ' + path)
    heading = json.loads(args.heading_candidate.read_text())
    board = json.loads(args.board_center_candidate.read_text())
    if (heading.get('manual_forward_horizontal_confirmed') is not True
            or heading.get('warnings') or board.get('marker_name') != 'MircoAir4'
            or board.get('imu_offset_verified') is not False
            or (args.board_center_candidate.parent / board.get('source_heading', '')).resolve()
                != args.heading_candidate.resolve()):
        ap.error('Labelled approximate body references required')
    mount = checked_quaternion(heading['body_orientation_in_rigid_xyzw_candidate'])
    offset = finite_vector(board['board_center_in_rigid_m_candidate'], 3)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped

    class Source(Node):
        def __init__(self):
            super().__init__('trs_static_fusion_bench')
            self.gate = PoseGate(max_age_ms=150)
            self.sub = self.create_subscription(PoseStamped, '/vicon/MircoAir/MircoAir', self.receive,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))

        def receive(self, msg):
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
    emergency_restore = None
    phase = 'fusion_off_warmup'
    rate_was_verified = False
    manual_pairs, manual_start, manual_end = [], None, None
    finish_at = None
    pad_fd, pad_pending, pad_error = None, b'', None
    endpoint = horizontal_endpoint if args.axis == 'translation' else vertical_endpoint
    fusion_started = fusion_finished = None

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

    def locked_manual():
        if not (arm and not arm['armed'] and arm['lockdown'] and elapsed() - arm_at < 4
                and origin_p is not None and node.gate.fresh(node.get_clock().now().nanoseconds, time.monotonic_ns())):
            return False
        p, q = transform_rigid_pose(node.gate.last.position, node.gate.last.quaternion, offset, mount)
        return policy.within_envelope(p, q, origin_p, origin_q, args.axis)

    def send(route, msg):
        if not policy.allowed(msg, route=route, safe=safe(), authorized=args.physical_confirmed,
                              backup_verified=True, locked_manual=locked_manual(),
                              receive_time_verified=receive_time_verified and rate_was_verified,
                              restore_safe=restore_safe()):
            raise ValueError('Static bench policy rejected ' + route + '/' + msg.get_type())
        packet = msg.pack(enc[route])
        if ports[route].write(packet) != len(packet):
            raise OSError('Short serial write')
        enc[route].seq = (enc[route].seq + 1) % 256
        tx[route][msg.get_type()] += 1

    def emit():
        nonlocal next_emit
        now = time.monotonic()
        if not active:
            return
        if not safe() or not locked_manual():
            raise RuntimeError('Identity/parameters/lock/source/static gate lost; stop and restore')
        if now - stream_start > args.window_seconds + 3 or len(sent) >= int((args.window_seconds+3)*32)+16:
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
        if not policy.within_envelope(p, q, origin_p, origin_q, args.axis):
            raise RuntimeError('Body moved beyond bounded manual envelope')
        stamp = (mono - age) // 1000
        msg, p, q = compact.message(enc['trs'], stamp, p, q, alignment)
        send('trs', msg)
        sent.append({'elapsed_s': elapsed(), 'source_stamp_ns': sample.stamp_ns,
                     'wire_time_usec': stamp, 'position': p, 'q': q, 'phase': phase})
        next_emit += 1 / 32
        if next_emit < now:
            next_emit = now + 1 / 32

    def pump():
        nonlocal next_params, buffer, attitude, pad_pending, pad_error
        now = elapsed()
        if now > args.window_seconds + 60 and not restoring:
            raise RuntimeError('Static bench global time limit')
        if pad_fd is not None and not restoring:
            try:
                chunk = os.read(pad_fd, EVENT.size*64)
                if not chunk:
                    raise OSError('Gamepad disconnected')
                pad_pending += chunk
                while len(pad_pending) >= EVENT.size:
                    _,_,kind,code,value = EVENT.unpack(pad_pending[:EVENT.size])
                    pad_pending = pad_pending[EVENT.size:]
                    if (kind==0 and code==3) or (kind==1 and code==HOME and value in (1,2)):
                        raise OSError('HOME or dropped input: manual fusion cancelled')
            except BlockingIOError:
                pass
            except OSError as exc:
                pad_error = str(exc)
                raise RuntimeError(pad_error)
        rclpy.spin_once(node, timeout_sec=0)
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
                if route == 'usb' and msg.get_type() == 'PARAM_VALUE' and str(msg.param_id) in policy.READ_PARAMS:
                    name = str(msg.param_id)
                    if name in ev['usb'].requests:
                        parameters[name] = decode_parameter(msg)
                        param_times[name] = now
                        if not restoring and name not in ('EKF2_EV_CTRL', 'EKF2_EV_DELAY') and parameters[name] != baseline[name]:
                            raise RuntimeError('Monitored parameter diverged from backup: ' + name)
                        if not restoring and name in ('EKF2_EV_CTRL', 'EKF2_EV_DELAY'):
                            candidate = 11 if name == 'EKF2_EV_CTRL' else policy.DELAY_MS
                            allowed_values = (0, candidate) if name in possibly_changed else (0,)
                            if parameters[name] not in allowed_values:
                                raise RuntimeError('Unrequested or out-of-scope parameter change: ' + name)
                if route == 'usb' and msg.get_type() == 'SERIAL_CONTROL':
                    buffer += bytes(msg.data[:msg.count]).decode('utf-8', 'replace')
                    if len(buffer) > 65536:  # 48帧固定只读窗口，独立于静止工具的12帧上限。
                        raise RuntimeError('USB listener overflow')
                if route == 'usb' and msg.get_type() == 'ATTITUDE_QUATERNION':
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
        nonlocal buffer, arm, arm_at
        parsers = {'actuator_armed': parse_actuator_snapshot, 'vehicle_visual_odometry': parse_visual_snapshot,
                   'visual_series': compact.visual_series, 'manual_visual_series': policy.visual_series,
                   'selector': combined.parse_selector,
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
                return result
            if time.monotonic() >= next_poll:
                send('usb', shell_message(enc['usb']))
                next_poll = time.monotonic() + .25
        listeners.append({'topic': topic, 'elapsed_s': elapsed(), 'text': buffer, 'incomplete': True})
        raise RuntimeError('Incomplete internal listener: ' + topic)

    def write_parameter(name, value):
        if value:
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
                if value == 0:
                    restored[name] = True
                return
        raise RuntimeError('Parameter write/read timeout: ' + name)

    try:
        pad = find_pad()
        pad_fd = os.open(pad['path'], os.O_RDONLY|os.O_NONBLOCK)
        bitmap = fcntl.ioctl(pad_fd,0x80000000|(96<<16)|(ord('E')<<8)|0x18,bytes(96))
        if bitmap[HOME//8] & (1<<(HOME%8)):
            raise RuntimeError('Release HOME before manual bench')
        for route, path in (('usb', USB), ('trs', TRS)):
            ports[route] = serial.Serial(path,115200,timeout=0,write_timeout=.5,exclusive=True)
        while not safe() or node.gate.accepted < 50 or not node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns()):
            pump()
            if elapsed() > 12:
                raise RuntimeError('Identity/backup/source preflight timeout; no writes')
        if any(parameters[n] != baseline[n] for n in policy.READ_PARAMS):
            raise RuntimeError('Live baseline does not match full backup')
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
        group = listener('manual_visual_series')
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
        print('MANUAL PREFLIGHT: TRS pose32Hz, provisional EV_DELAY65ms, EV_CTRL11; no arming or motion', flush=True)
        for target in (2,5,8):
            while time.monotonic() - fusion_started < target:
                pump()
            listener('actuator_armed')
            selected = listener('selector')
            if selected['primary_instance'] != primary:
                raise RuntimeError('Primary estimator changed; restore')
            row = {'selector': selected, 'flags': listener(f'flags_{primary}'),
                   'aid': listener(f'aid_{primary}'), 'height_aid': listener(f'height_aid_{primary}'),
                   'yaw_aid': listener(f'yaw_aid_{primary}'), 'local': listener('local_position'),
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
        if not combined.fusion_evidence_ok(fusion_rows,primary):
            raise RuntimeError('Real XYZ/yaw fusion not verified before manual operation')
        phase = 'manual_' + args.axis
        next_pair = next_arm = time.monotonic()
        while time.monotonic() - fusion_started < args.window_seconds:
            pump()
            now = time.monotonic()
            if now >= next_arm:
                listener('actuator_armed')
                selected = listener('selector')
                flags = listener(f'flags_{primary}')
                if (selected['primary_instance'] != primary or flags['cs_in_air']
                        or not combined.inactive_other_heading_sources(flags)
                        or not all(flags[k] for k in ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw'))):
                    raise RuntimeError('Manual fusion/selector prerequisites lost')
                visual = listener('vehicle_visual_odometry')
                if not policy.receive_time_mode(visual) or not compact.match(visual,sent):
                    raise RuntimeError('Manual receive-time or real visual pose gate lost')
                next_arm = time.monotonic()+1
            if now >= next_pair:
                local = listener('local_position')
                pair_time = elapsed()
                if (sent and attitude and pair_time-attitude['elapsed_s'] <= .3
                        and (not manual_pairs or local['timestamp_sample']>manual_pairs[-1]['local']['timestamp_sample'])
                        and (not manual_pairs or sent[-1]['source_stamp_ns']>manual_pairs[-1]['sent']['source_stamp_ns'])):
                    manual_pairs.append({'elapsed_s':pair_time,'local':local,'sent':sent[-1],
                                         'fc_q':attitude['q'],'attitude_age_ms':(pair_time-attitude['elapsed_s'])*1000})
                next_pair = time.monotonic()+.2
                if manual_start is None:
                    manual_start = endpoint(manual_pairs[-6:])
                    if manual_start:
                        with (args.output.parent/'manual_baseline.json').open('x') as f:
                            json.dump(manual_start,f,indent=2,allow_nan=False)
                        print('MANUAL READY: settled baseline captured; waiting for operator move instruction',flush=True)
                if finish_at is not None:
                    end_rows = [r for r in manual_pairs if r['elapsed_s']>finish_at]
                    manual_end = endpoint(end_rows[-6:])
                    if manual_end:
                        with (args.output.parent/'manual_endpoint.json').open('x') as f:
                            json.dump(manual_end,f,indent=2,allow_nan=False)
                        print('MANUAL END: settled endpoint captured; stopping poses and restoring parameters',flush=True)
                        break
                    if elapsed()-finish_at>8:
                        raise RuntimeError('No settled endpoint within8s after finish')
            if args.control_file.exists():
                command = validate_control(json.loads(args.control_file.read_text()))
                if command['command']=='cancel':
                    raise RuntimeError('Operator cancelled manual observation')
                if manual_start is None:
                    raise RuntimeError('Finish received before settled baseline')
                if finish_at is None:
                    finish_at = elapsed()
        if manual_start is None or manual_end is None:
            raise RuntimeError('Manual window expired without both stationary endpoints')
        fusion_finished = time.monotonic()
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc)
    finally:
        active = False
        restoring = True
        if fusion_started is not None and fusion_finished is None:
            fusion_finished = time.monotonic()
        # 先停发，再关闭融合，最后恢复延迟；每项独立尝试，源/时钟丢失不阻止回退。
        for name in ('EKF2_EV_CTRL','EKF2_EV_DELAY'):
            if name not in possibly_changed:
                continue
            try:
                write_parameter(name, 0)
            except Exception as exc:
                restore_errors.append(name + ': ' + str(exc))
        restoring = False
        if 'usb' in ports:
            try:
                if safe():
                    arm_after = listener('actuator_armed')
            except Exception as exc:
                restore_errors.append('Final observer: ' + str(exc))
        for port in ports.values():
            port.close()
        # USB断连时重新打开TRS仅执行零值回退；不自动重连定位，不解除输出锁定。
        if restore_errors and possibly_changed:
            rescue_path = args.output.with_name(args.output.stem + '_emergency_restore.json')
            try:
                rescue = subprocess.run([
                    sys.executable, str(Path(__file__).with_name('trs_fusion_restore_only.py')),
                    '--port', TRS, '--backup', str(args.backup), '--output', str(rescue_path)],
                    capture_output=True, text=True, timeout=40)
                emergency_restore = {'exit_code':rescue.returncode,'capture':str(rescue_path),
                                     'stderr':rescue.stderr[-2000:]}
                if rescue_path.exists():
                    receipt = json.loads(rescue_path.read_text())
                    emergency_restore['restore_verified'] = receipt.get('restore_verified') is True
                    if emergency_restore['restore_verified']:
                        parameters.update(receipt['after'])
                        restored.update(EKF2_EV_CTRL=True,EKF2_EV_DELAY=True)
            except Exception as exc:
                emergency_restore = {'restore_verified':False,'error':str(exc)}
        if pad_fd is not None:
            os.close(pad_fd)
        node.destroy_node()
        rclpy.shutdown()
    fusion_ok = bool(not error and not restore_errors and combined.fusion_evidence_ok(fusion_rows, primary))
    measurement = (assess_translation if args.axis=='translation' else assess_vertical)(manual_start,manual_end)
    result = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope': 'Unpowered bounded manual wireless endpoint consistency; NOT flight or latency validation',
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
              'manual_axis':args.axis,'manual_measurement':measurement,
              'manual_start':manual_start,'manual_end':manual_end,'manual_pairs':manual_pairs,
              'gamepad_error':pad_error,
              'fusion_rows': fusion_rows, 'status_texts': status_texts,
              'arrival_time_fallback_verified': receive_time_verified,
              'pre_enable_receiver_rate_verified': rate_was_verified,
              'fusion_duration_s': fusion_finished-fusion_started if fusion_finished else 0,
              'pre_manual_xyz_yaw_fusion_verified': fusion_ok,
              'control_readiness_during_fusion': control.assess(fusion_rows,primary),
              'alignment_yaw_candidate_rad': alignment,
              'frame_scope': 'FC-yaw-aligned local NED candidate; not calibrated North or flight control mapping',
              'delay_compensation_ms_provisional': policy.DELAY_MS,
              'trs_sampling_clock_conversion_verified': False, 'camera_exposure_latency_verified': False,
              'imu_reference_verified': False, 'heading_state_rollback_attempted': False,
              'arming_mode_motion_or_unlock_commands_sent': False,
              'usb_used_as_readonly_observer_and_parameter_maintenance': True,
              'standalone_wireless_fusion_verified': False, 'flight_ready': False,
              'error': error, 'restore_errors': restore_errors, 'serial_ports_released': True}
    result['emergency_restore'] = emergency_restore
    with args.output.open('x') as f:
        json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('internal_listeners','sent_poses','fusion_rows','status_texts','manual_pairs','receive_groups')},ensure_ascii=False,indent=2))
    return 0 if fusion_ok and measurement['verified'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

