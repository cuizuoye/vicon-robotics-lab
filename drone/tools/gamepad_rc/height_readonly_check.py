#!/usr/bin/env python3
"""指定PX4 1.14.3的高度源只读诊断：零定位、零写参、零控制。"""

import argparse
from collections import Counter
import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import serial
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from mavlink_readonly_probe import Evidence
from estop_only_policy import parse_actuator_snapshot
from usb_fusion_policy import parse_selector, parse_local_position, parse_topic
from usb_vision_policy import shell_message as empty_shell
from persistent_shell import reply_complete
from readonly_parameter_poll import ParameterPoll
import height_readonly_policy as height_policy
import ekf_compare_readonly_policy as comparison_policy
import control_readonly_policy as control_policy


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--uid2', required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--compare-ekfs', action='store_true', help='Read both available EKF0/1 and full selector scores; zero writes')
    ap.add_argument('--control-readiness', action='store_true', help='Read local velocity/heading, estimator innovations and commander failsafe flags; no targets/writes')
    args = ap.parse_args()
    if args.compare_ekfs and args.control_readiness:
        ap.error('Select one read-only audit profile')
    policy = control_policy if args.control_readiness else comparison_policy if args.compare_ekfs else height_policy
    selector_parser = policy.parse_selector if args.compare_ekfs else parse_selector
    if args.output.exists() or not args.output.parent.is_dir():
        ap.error('New output in existing directory required')
    if subprocess.run(['fuser', str(Path(args.port).resolve())], capture_output=True).returncode == 0:
        ap.error('Serial port occupied; no process will be killed')
    evidence = Evidence(expected_parameters=policy.PARAMS)
    encoder, decoder = m.MAVLink(None, srcSystem=250, srcComponent=190), m.MAVLink(None)
    decoder.robust_parsing = True
    start = time.monotonic()
    tx, records, rows, comparisons = Counter(), [], [], []
    parameter_times = {}
    sensor_telemetry = {'DISTANCE_SENSOR': [], 'OPTICAL_FLOW_RAD': []}
    next_hb = next_info = 0
    parameter_poll = ParameterPoll(policy.PARAMS)
    shell_pending = False
    buffer = ''
    actuator_before = actuator_after = None
    primary = None
    error = None
    parameters_unchanged = None
    port = None

    def fresh_identity():
        now = time.monotonic() - start
        return bool(evidence.identity and evidence.identity['uid2'] == args.uid2
                    and evidence.identity['firmware'] == '1.14.3' and evidence.heartbeats
                    and now - evidence.heartbeats[-1]['elapsed_s'] < 2 and not evidence.heartbeats[-1]['armed'])

    def send(msg):
        if not policy.allowed(msg, safe=fresh_identity()):
            raise ValueError('Read-only height policy rejected ' + msg.get_type())
        packet = msg.pack(encoder)
        if port.write(packet) != len(packet):
            raise OSError('Short serial write')
        encoder.seq = (encoder.seq + 1) % 256
        tx[msg.get_type()] += 1

    def pump():
        nonlocal next_hb, next_info, buffer
        now = time.monotonic() - start
        if now > 60:
            raise RuntimeError('Bounded 60-second read-only session expired')
        for msg in decoder.parse_buffer(port.read(min(port.in_waiting, 65536))) or []:
            evidence.ingest(msg, now)
            if evidence.fatal:
                raise RuntimeError(evidence.fatal)
            if evidence.identity and (evidence.identity['uid2'] != args.uid2
                                      or evidence.identity['firmware'] != '1.14.3'):
                raise RuntimeError('Wrong aircraft identity or firmware')
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                continue
            if msg.get_type() in sensor_telemetry:
                # Passive capture only: do not request new streams or change rates.
                fields = ('time_boot_ms', 'min_distance', 'max_distance', 'current_distance',
                          'type', 'id', 'orientation', 'covariance', 'signal_quality') \
                    if msg.get_type() == 'DISTANCE_SENSOR' else \
                    ('time_usec', 'sensor_id', 'integration_time_us', 'integrated_x',
                     'integrated_y', 'quality', 'distance', 'time_delta_distance_us')
                sample = {'received_at_s': now}
                for field in fields:
                    value = getattr(msg, field, None)
                    if isinstance(value, (int, float)):
                        sample[field] = value if math.isfinite(value) else None
                if len(sensor_telemetry[msg.get_type()]) < 1000:
                    sensor_telemetry[msg.get_type()].append(sample)
            if msg.get_type() == 'PARAM_VALUE' and msg.param_id in evidence.parameters:
                parameter_times[msg.param_id] = now
            elif msg.get_type() == 'SERIAL_CONTROL':
                buffer += bytes(msg.data[:msg.count]).decode('utf-8', 'replace')
                if len(buffer) > 16384:
                    raise RuntimeError('Listener overflow')
        if now >= next_hb:
            send(encoder.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            next_hb = now + 1
        if evidence.heartbeats and now - evidence.heartbeats[-1]['elapsed_s'] < 2 and now >= next_info:
            if not evidence.identity:
                evidence.version_requested_at = now
                send(encoder.command_long_encode(1, 1, m.MAV_CMD_REQUEST_MESSAGE, 0,
                                                m.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0))
            next_info = now + 2
        name = parameter_poll.due(now, identity_ready=fresh_identity(), shell_pending=shell_pending)
        if name is not None:
            evidence.requests[name] = now
            send(encoder.param_request_read_encode(1, 1, name.encode(), -1))
        time.sleep(.005)

    def listener(key, parse, optional=False):
        nonlocal buffer, shell_pending
        buffer = ''
        requested = time.monotonic() - start
        request = policy.shell_message(encoder, key)
        shell_pending = True
        send(request)
        poll = 0
        while time.monotonic() - start - requested < 4:
            pump()
            if not fresh_identity():
                raise RuntimeError('Fresh disarmed identity lost')
            parsed = parse(buffer)
            absent = policy.absent_reply(buffer) if optional else None
            if (parsed or absent) and reply_complete(buffer, bytes(request.data[:request.count])):
                records.append({'key': key, 'requested_at_s': requested,
                                'reply_at_s': time.monotonic() - start, 'text': buffer,
                                'parsed': parsed, 'absence_reason': absent})
                shell_pending = False
                return parsed
            if time.monotonic() >= poll:
                send(empty_shell(encoder))
                poll = time.monotonic() + .25
        records.append({'key': key, 'text': buffer, 'incomplete': True})
        raise RuntimeError('Incomplete listener reply: ' + key)

    try:
        port = serial.Serial(args.port, 115200, timeout=.02, write_timeout=1, exclusive=True)
        while not fresh_identity() or set(evidence.parameters) != set(policy.PARAMS):
            pump()
            if time.monotonic() - start > 12:
                raise RuntimeError('Identity/parameter preflight timeout')
        baseline = dict(evidence.parameters)
        if baseline['EKF2_EV_CTRL'] != 0:
            raise RuntimeError('Expected restored EV_CTRL=0; refusing to change it')
        actuator_before = listener('actuator', parse_actuator_snapshot)
        if actuator_before['armed'] or not actuator_before['lockdown']:
            raise RuntimeError('Expected disarmed, output-locked maintenance state')
        for _ in range(3):
            selector = listener('selector', selector_parser)
            if (not selector or not 0 <= selector['primary_instance'] <= 8
                    or selector['gyro_fault_detected'] or selector['accel_fault_detected']):
                raise RuntimeError('Invalid selector/IMU fault')
            if primary is not None and selector['primary_instance'] != primary:
                raise RuntimeError('Primary estimator changed')
            primary = selector['primary_instance']
            if args.compare_ekfs:
                if selector['instances_available'] != 2:
                    raise RuntimeError('This aircraft comparison expects exactly two available EKF instances')
                instances = {}
                for instance in (0, 1):
                    flags = listener(f'flags_{instance}', policy.parse_flags)
                    if flags['cs_in_air'] or flags['fs_bad_pos_d']:
                        raise RuntimeError('Unexpected airborne state or vertical numerical fault')
                    instances[str(instance)] = {
                        'flags': flags,
                        'baro': listener(f'baro_{instance}', lambda t: policy.parse_aid(t, 'baro'), True),
                        'ev': listener(f'ev_{instance}', lambda t: policy.parse_aid(t, 'ev'), True),
                        'xy': listener(f'xy_{instance}', policy.parse_xy, True)}
                local = listener('local', parse_local_position)
                comparisons.append({'selector': selector, 'instances': instances, 'local': local})
                until = time.monotonic() + 1
                while time.monotonic() < until:
                    pump()
                continue
            flags = listener(f'flags_{primary}', policy.parse_flags)
            if flags['cs_in_air'] or flags['fs_bad_pos_d']:
                raise RuntimeError('Unexpected airborne state or vertical numerical fault')
            local = listener('local', policy.parse_local_position if args.control_readiness else parse_local_position)
            if args.control_readiness:
                status = listener(f'status_{primary}', policy.parse_status)
                failsafe = listener('failsafe', policy.parse_failsafe)
                rows.append({'selector': selector, 'flags': flags, 'local': local,
                             'status': status, 'failsafe': failsafe})
                until = time.monotonic() + 1
                while time.monotonic() < until:
                    pump()
                continue
            aids = {s: listener(f'{s}_{primary}', lambda text, s=s: policy.parse_aid(text, s), True)
                    for s in policy.SOURCES}
            for key, topic in (('distance', 'distance_sensor'), ('flow', 'sensor_optical_flow')):
                listener(key, lambda text, topic=topic: parse_topic(text, topic, ('timestamp',)), True)
            rows.append({'selector': selector, 'flags': flags, 'local': local, 'aids': aids})
            until = time.monotonic() + 1
            while time.monotonic() < until:
                pump()
        actuator_after = listener('actuator', parse_actuator_snapshot)
        final_selector = listener('selector', selector_parser)
        if final_selector['primary_instance'] != primary:
            raise RuntimeError('Primary estimator changed at endpoint')
        refresh_started = time.monotonic() - start
        until = time.monotonic() + 4
        while time.monotonic() < until:
            pump()
            if all(parameter_times.get(n, -1) >= refresh_started for n in policy.PARAMS):
                break
        parameters_unchanged = evidence.parameters == baseline
        if not parameters_unchanged or any(parameter_times.get(n, -1) < refresh_started
                                          for n in policy.PARAMS):
            raise RuntimeError('Monitored parameter drift/staleness')
        if actuator_after['armed'] or not actuator_after['lockdown']:
            raise RuntimeError('Final disarmed/lockdown state lost')
    except Exception as exc:
        error = str(exc)
    finally:
        if port is not None:
            port.close()
    result = {'scope': 'Control-prerequisite read-only audit; no targets, positions, writes, modes, arming or reboot'
                        if args.control_readiness else 'Two-EKF read-only comparison; no selection requests, positions, writes, control or reboot'
                        if args.compare_ekfs else 'Height-source read-only maintenance; no positions, writes, control or reboot',
              'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'identity': evidence.identity, 'parameters': evidence.parameters,
              'passive_sensor_telemetry': sensor_telemetry,
              'passive_capture_note': 'Initial buffered messages may predate this connection; use fresh uORB timestamps for current-state conclusions',
              'primary_instance': primary, 'rows': rows, 'listener_records': records,
              'observed_sources': None if args.compare_ekfs or args.control_readiness else policy.assess_sources(rows, primary),
              'actuator_before': actuator_before, 'actuator_after': actuator_after,
              'heartbeats': evidence.heartbeats, 'tx_counts': dict(tx),
              'monitored_parameters_unchanged': parameters_unchanged,
              'ekf_comparison_requested': args.compare_ekfs, 'comparisons': comparisons,
              'ekf_comparison': policy.assess_comparisons(comparisons) if args.compare_ekfs else None,
              'control_readiness_requested': args.control_readiness,
              'control_prerequisites': policy.assess(rows, primary) if args.control_readiness else None,
              'actual_sent_to_fc': 0, 'parameter_writes': 0, 'flight_ready': False,
              'passed': error is None, 'error': error, 'duration_s': time.monotonic() - start}
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'comparisons', 'listener_records', 'heartbeats', 'passive_sensor_telemetry')},
                     ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
