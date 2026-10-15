#!/usr/bin/env python3
"""串口MAVLink双向诊断：只发心跳、PING、版本请求和参数读取，不控制飞机。"""

import argparse
from collections import Counter
import datetime
import json
import math
from pathlib import Path
import secrets
import struct
import subprocess
import time

import serial
from pymavlink.dialects.v20 import common as m

PARAMETERS = ('SYS_AUTOSTART', 'RC_MAP_KILL_SW', 'COM_RC_IN_MODE')
SAFETY_PARAMETERS = ('COM_OBL_RC_ACT', 'COM_OF_LOSS_T', 'COM_RCL_EXCEPT',
                     'COM_RC_LOSS_T', 'NAV_RCL_ACT', 'NAV_DLL_ACT',
                     'COM_DL_LOSS_T', 'COM_KILL_DISARM')
READABLE_PARAMETERS = PARAMETERS + SAFETY_PARAMETERS
FC_USB = Path('/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00')


class PassivePowerEvidence:
    """Only consume existing FC telemetry; never request streams or send controls."""

    def __init__(self):
        self.batteries, self.system_power, self.boot_samples = [], [], []
        self.reboot_observed = False

    def ingest(self, message, now):
        if (message.get_srcSystem(), message.get_srcComponent()) != (1, 1):
            return
        kind = message.get_type()
        if kind == 'BATTERY_STATUS':
            base = [int(v) for v in message.voltages if 0 < v < 65535]
            ext = [int(v) for v in getattr(message, 'voltages_ext', []) if 0 < v < 65535]
            self.batteries.append({
                'elapsed_s': round(now, 3), 'id': message.id,
                'reported_voltage_v': sum(base + ext) / 1000 if base or ext else None,
                'raw_voltages_mv': list(message.voltages),
                'current_a': message.current_battery / 100 if message.current_battery >= 0 else None,
                'remaining_percent': message.battery_remaining if message.battery_remaining >= 0 else None,
                'charge_state': getattr(message, 'charge_state', None),
                'fault_bitmask': getattr(message, 'fault_bitmask', None)})
        elif kind == 'SYS_STATUS':
            self.system_power.append({
                'elapsed_s': round(now, 3),
                'voltage_v': message.voltage_battery / 1000 if 0 < message.voltage_battery < 65535 else None,
                'current_a': message.current_battery / 100 if message.current_battery >= 0 else None,
                'remaining_percent': message.battery_remaining if message.battery_remaining >= 0 else None})
        elif kind == 'ATTITUDE_QUATERNION':
            stamp = int(message.time_boot_ms)
            if self.boot_samples and stamp < self.boot_samples[-1]['time_boot_ms']:
                self.reboot_observed = True
            self.boot_samples.append({'elapsed_s': round(now, 3), 'time_boot_ms': stamp})

    def result(self):
        return {'battery_samples': self.batteries, 'system_power_samples': self.system_power,
                'boot_sample_count': len(self.boot_samples),
                'first_last_boot_ms': [self.boot_samples[0]['time_boot_ms'], self.boot_samples[-1]['time_boot_ms']]
                    if self.boot_samples else None,
                'reboot_observed': self.reboot_observed,
                'scope': 'Passive reported voltage and boot-clock observations, not voltage calibration, load capacity or flight clearance'}


def battery_observation_ok(result, expected_uid, duration):
    power = result['passive_power']
    heartbeats = result['heartbeats']
    volts = [r['reported_voltage_v'] for r in power['battery_samples']] + [r['voltage_v'] for r in power['system_power_samples']]
    props_off = result.get('battery_only_physical_state_confirmed') is True
    guarded_props_on = result.get('guarded_props_installed_readonly_confirmed') is True
    physical_mode_ok = (props_off != guarded_props_on) and (
        props_off or (result.get('propeller_state') == 'installed'
                      and result.get('communication_only_no_motion_authorized') is True))
    return bool(physical_mode_ok and result['fc_usb_absence_monitored']
                and not result['fc_usb_present_at_end'] and not result['error']
                and result['identity'] and result['identity']['uid2'] == expected_uid
                and result['round_trip_verified'] and result['all_parameters_received']
                and result['all_fc_heartbeats_disarmed'] and len(heartbeats) >= max(3, int(duration)-2)
                and all(b['elapsed_s']-a['elapsed_s'] <= 2 for a,b in zip(heartbeats,heartbeats[1:]))
                and power['boot_sample_count'] >= 2 and not power['reboot_observed']
                and 0 <= (power['first_last_boot_ms'][1]-power['first_last_boot_ms'][0])/1000 <= duration+2
                and any(v is not None and v > 0 for v in volts))


def readonly_message(message):
    """发送口白名单，误接控制报文也不能发出去。"""
    kind = message.get_type()
    if kind == 'HEARTBEAT':
        return (message.type == m.MAV_TYPE_GCS and message.base_mode == 0
                and message.autopilot == m.MAV_AUTOPILOT_INVALID)
    if kind == 'PING':
        return message.target_system == 0 and message.target_component == 0
    if kind == 'PARAM_REQUEST_READ':
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\x00')
        return (name in READABLE_PARAMETERS and message.param_index == -1
                and (message.target_system, message.target_component) == (1, 1))
    if kind == 'COMMAND_LONG':
        return ((message.target_system, message.target_component) == (1, 1)
                and message.command == m.MAV_CMD_REQUEST_MESSAGE
                and message.param1 == m.MAVLINK_MSG_ID_AUTOPILOT_VERSION
                and all(getattr(message, f'param{i}') == 0 for i in range(2, 8)))
    return False


def send_readonly(encoder, port, message):
    if not readonly_message(message):
        raise ValueError(f'Refused non-diagnostic message: {message.get_type()}')
    packet = message.pack(encoder)
    written = port.write(packet)
    if written != len(packet):
        raise IOError('Short serial write')
    encoder.seq = (encoder.seq + 1) % 256
    return len(packet)


def decode_parameter(message):
    # PX4传统PARAM协议：整型数值使用float字段中的字节，不是float数值转换。
    raw = struct.pack('<f', message.param_value)
    formats = {1: '<B', 2: '<b', 3: '<H', 4: '<h', 5: '<I', 6: '<i'}
    fmt = formats.get(message.param_type)
    value = (struct.unpack(fmt, raw[:struct.calcsize(fmt)])[0]
             if fmt else float(message.param_value))
    if not math.isfinite(value):
        raise ValueError('Non-finite parameter')
    return value


class Evidence:
    """只有本轮随机PING匹配，或请求后指定回复，才计入双向证据。"""

    def __init__(self, source_system=250, source_component=190, expected_parameters=PARAMETERS):
        self.source_system = source_system
        self.source_component = source_component
        self.fc = (1, 1)
        self.counts = Counter()
        self.fc_counts = Counter()
        self.wire_versions = Counter()
        self.heartbeats = []
        self.identity = None
        self.parameters = {}
        self.requests = {}
        self.pings = {}
        self.ping_rtt_ms = []
        self.ping_matches = []
        self.fatal = None
        self.version_requested_at = None
        self.expected_parameters = tuple(expected_parameters)

    def ingest(self, message, now):
        kind = message.get_type()
        self.counts[kind] += 1
        if kind == 'BAD_DATA' or kind.startswith('UNKNOWN_'):
            return
        packet = message.get_msgbuf()
        if packet:
            self.wire_versions['v2' if packet[0] == 0xfd else 'v1'] += 1
        if (message.get_srcSystem(), message.get_srcComponent()) != self.fc:
            return
        self.fc_counts[kind] += 1
        if kind == 'HEARTBEAT':
            if message.autopilot != m.MAV_AUTOPILOT_PX4:
                self.fatal = 'Expected PX4 but received another autopilot'
                return
            armed = bool(message.base_mode & m.MAV_MODE_FLAG_SAFETY_ARMED)
            self.heartbeats.append({'elapsed_s': round(now, 3), 'armed': armed,
                                    'custom_mode': message.custom_mode})
            if armed:
                self.fatal = 'Armed vehicle: maintenance probe stopped'
        elif kind == 'AUTOPILOT_VERSION' and self.version_requested_at is not None:
            v = message.flight_sw_version
            self.identity = {
                'firmware': f'{v >> 24}.{(v >> 16) & 255}.{(v >> 8) & 255}',
                'board_version': message.board_version,
                'uid': message.uid,
                'uid2': bytes(message.uid2).hex(),
            }
        elif kind == 'PARAM_VALUE':
            name = str(message.param_id).rstrip('\x00')
            requested = self.requests.get(name)
            if requested is not None and now >= requested:
                self.parameters[name] = decode_parameter(message)
        elif kind == 'PING':
            key = (message.time_usec, message.seq)
            if (message.target_system == self.source_system
                    and message.target_component == self.source_component
                    and key in self.pings):
                sent = self.pings.pop(key)
                self.ping_rtt_ms.append(round((now - sent) * 1000, 3))
                self.ping_matches.append({'time_usec': message.time_usec,
                                          'sequence': message.seq,
                                          'sent_elapsed_s': sent, 'reply_elapsed_s': now,
                                          'source': list(self.fc),
                                          'target': [message.target_system, message.target_component]})

    def result(self):
        return {
            'fc_heartbeat_count': len(self.heartbeats),
            'all_fc_heartbeats_disarmed': bool(self.heartbeats)
                and all(not h['armed'] for h in self.heartbeats),
            'identity': self.identity, 'requested_parameters': self.parameters,
            'correlated_ping_replies': len(self.ping_rtt_ms),
            'ping_rtt_ms': self.ping_rtt_ms, 'heartbeats': self.heartbeats,
            'ping_matches': self.ping_matches,
            'messages': dict(self.counts), 'fc_messages': dict(self.fc_counts),
            'wire_versions': dict(self.wire_versions), 'error': self.fatal,
            'downlink_verified': bool(self.heartbeats) and self.fatal is None,
            'round_trip_verified': bool(self.ping_rtt_ms) and self.fatal is None,
            'information_request_reply_verified': bool(self.identity)
                and bool(self.parameters) and self.fatal is None,
            'expected_parameters': list(self.expected_parameters),
            'missing_parameters': sorted(set(self.expected_parameters) - set(self.parameters)),
            'all_parameters_received': bool(self.expected_parameters)
                and set(self.expected_parameters).issubset(self.parameters) and self.fatal is None,
        }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, choices=(57600, 115200), default=115200)
    ap.add_argument('--seconds', type=float, default=12)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--profile', choices=('link', 'safety'), default='link',
                    help='safety adds only failsafe parameter reads; never writes parameters')
    ap.add_argument('--expect-uid2', help='Reject a different FC identity')
    power_mode = ap.add_mutually_exclusive_group()
    power_mode.add_argument('--battery-only-confirmed', action='store_true',
                    help='Operator confirms props removed, FC USB unplugged, aircraft battery-only; no motor commands')
    power_mode.add_argument('--guarded-props-installed-readonly-confirmed', action='store_true',
                    help='Operator confirms props installed, protective frame intact, aircraft stationary in clear area, battery-only and FC USB absent; communication-only, no motion authorization')
    args = ap.parse_args()
    battery_only = args.battery_only_confirmed or args.guarded_props_installed_readonly_confirmed
    if not 5 <= args.seconds <= 40 or args.output.exists():
        ap.error('Use 5..40 seconds and a new output file')
    if battery_only and (not args.expect_uid2 or FC_USB.exists()):
        ap.error('Battery-only check requires expected UID and absent FC USB device')
    owners = subprocess.run(['fuser', str(Path(args.port).resolve())],
                            capture_output=True, text=True)
    if owners.returncode == 0:
        ap.error(f'Port is occupied by PID(s) {owners.stdout.strip()}')
    parameters = READABLE_PARAMETERS if args.profile == 'safety' else PARAMETERS
    evidence = Evidence(expected_parameters=parameters)
    passive_power = PassivePowerEvidence()
    encoder = m.MAVLink(None, srcSystem=250, srcComponent=190)
    decoder = m.MAVLink(None)
    decoder.robust_parsing = True
    tx_counts = Counter()
    rx_bytes = tx_bytes = 0
    start = time.monotonic()
    next_heartbeat = next_request = 0.0
    sequence = secrets.randbits(31)
    error = None
    try:
        # exclusive禁止第二个遵循串口锁的进程同时接入；无自动重连。
        with serial.Serial(args.port, args.baud, timeout=0.02,
                           write_timeout=1, exclusive=True) as port:
            # Drop only host receive backlog, not FC state/configuration, before correlating this round.
            port.reset_input_buffer()
            while time.monotonic() - start < args.seconds:
                now = time.monotonic() - start
                if battery_only and FC_USB.exists():
                    raise RuntimeError('FC USB reappeared during battery-only observation')
                chunk = port.read(4096)
                rx_bytes += len(chunk)
                for message in decoder.parse_buffer(chunk) or []:
                    evidence.ingest(message, now)
                    passive_power.ingest(message, now)
                if evidence.fatal:
                    raise RuntimeError(evidence.fatal)
                if args.expect_uid2 and evidence.identity and evidence.identity['uid2'] != args.expect_uid2:
                    raise RuntimeError('Received a different flight controller identity')
                if battery_only:
                    if passive_power.reboot_observed:
                        raise RuntimeError('FC boot clock regressed during battery-only observation')
                    if evidence.heartbeats and now - evidence.heartbeats[-1]['elapsed_s'] > 2:
                        raise RuntimeError('FC heartbeat lost during battery-only observation')
                to_send = []
                if now >= next_heartbeat:
                    to_send.append(encoder.heartbeat_encode(
                        m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
                    next_heartbeat = now + 1
                # 先确认本轮PX4未解锁；信息请求只发给system=1/component=1。
                fresh = bool(evidence.heartbeats) and now - evidence.heartbeats[-1]['elapsed_s'] < 2
                if fresh and now >= next_request:
                    usec = time.monotonic_ns() // 1000
                    sequence = (sequence + 1) % (2**32)
                    evidence.pings[(usec, sequence)] = now
                    to_send.append(encoder.ping_encode(usec, sequence, 0, 0))
                    evidence.version_requested_at = now
                    to_send.append(encoder.command_long_encode(
                        1, 1, m.MAV_CMD_REQUEST_MESSAGE, 0,
                        m.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0))
                    for name in parameters:
                        evidence.requests[name] = now
                        to_send.append(encoder.param_request_read_encode(1, 1, name.encode(), -1))
                    next_request = now + 3
                for message in to_send:
                    tx_bytes += send_readonly(encoder, port, message)
                    tx_counts[message.get_type()] += 1
    except (OSError, ValueError, RuntimeError, serial.SerialException) as exc:
        error = str(exc)
    result = evidence.result()
    if error:
        result['error'] = error
        for flag in ('downlink_verified', 'round_trip_verified',
                     'information_request_reply_verified', 'all_parameters_received'):
            result[flag] = False
    result.update({'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                   'port': args.port, 'baud': args.baud,
                   'duration_s': round(time.monotonic() - start, 3),
                   'rx_bytes': rx_bytes, 'tx_bytes': tx_bytes,
                   'tx_messages': dict(tx_counts),
                   'profile': args.profile,
                   'passive_power': passive_power.result(),
                   'expected_uid2': args.expect_uid2,
                   'battery_only_physical_state_confirmed': args.battery_only_confirmed,
                   'guarded_props_installed_readonly_confirmed': args.guarded_props_installed_readonly_confirmed,
                   'propeller_state': ('installed' if args.guarded_props_installed_readonly_confirmed
                                       else 'removed' if args.battery_only_confirmed else 'not_confirmed_this_probe'),
                   'physical_conditions_source': 'operator report; not independently physically verified',
                   'communication_only_no_motion_authorized': True,
                   'fc_usb_absence_monitored': battery_only,
                   'fc_usb_present_at_end': FC_USB.exists(),
                   'scope': 'No RC/control/setpoint/parameter-write/calibration/reboot messages'})
    result['battery_only_communication_verified'] = battery_observation_ok(result, args.expect_uid2, args.seconds)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    passed = result['battery_only_communication_verified'] if battery_only else result['round_trip_verified'] and not result['error']
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
