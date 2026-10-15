#!/usr/bin/env python3
"""Bounded TRS-only diagnostic capture: no parameter writes or actuator commands."""
import argparse
from collections import Counter
from contextlib import ExitStack
import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

import serial
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
import control_readonly_policy as control
from estop_only_policy import parse_actuator_snapshot
from mavlink_readonly_probe import Evidence, FC_USB, PassivePowerEvidence, readonly_message
from persistent_shell import reply_complete
from trs_uart_readonly_check import UID2
from trs_only_estop_bench import state_valid

COMMANDS = {
    'actuator': b'listener actuator_armed -n 1\n',
    'status': b'mavlink status\n',
    'flags_0': b'listener estimator_status_flags -i 0 -n 1\n',
    'flags_1': b'listener estimator_status_flags -i 1 -n 1\n',
}


def allowed(msg, safe=False):
    if msg.get_type() in ('HEARTBEAT', 'COMMAND_LONG'):
        return readonly_message(msg)
    return bool(safe and msg.get_type() == 'SERIAL_CONTROL'
                and (msg.device, msg.flags, msg.timeout, msg.baudrate) == (10, 2, 0, 0)
                and any(msg.count == len(p) and bytes(msg.data) == p.ljust(70, b'\0')
                        for p in (b'', *COMMANDS.values())))


def sequence_observation(previous, current):
    if previous is None:
        return {'kind': 'first', 'unseen_sequence_slots': 0}
    delta = (current - previous) % 256
    return {'kind': 'contiguous' if delta == 1 else 'duplicate' if delta == 0
            else 'gap_or_reordering', 'unseen_sequence_slots': delta - 1 if delta > 1 else 0}


def shell_fragment(message, elapsed):
    """Older SERIAL_CONTROL dialects lack target extensions; never invent IDs.

    These optional IDs are diagnostic metadata only. Identity/arming gates remain
    based on the source header, AUTOPILOT_VERSION and fresh FC heartbeat.
    """
    return {'t': elapsed, 'seq': message.get_seq(), 'count': message.count,
            'target': [getattr(message, 'target_system', None),
                       getattr(message, 'target_component', None)],
            'target_fields_available': all(hasattr(message, field)
                                           for field in ('target_system', 'target_component')),
            'text': bytes(message.data[:message.count]).decode('utf8', 'replace')}


def run(args):
    if not args.guarded_props_installed_readonly_confirmed:
        raise ValueError('Guarded stationary, USB-absent read-only physical scope required')
    if args.output.exists() or not args.output.parent.is_dir():
        raise ValueError('New output in existing directory required')
    if FC_USB.exists():
        raise ValueError('FC USB must remain absent')
    if subprocess.run(['fuser', str(Path(args.port).resolve())], capture_output=True).returncode == 0:
        raise ValueError('TRS is occupied; no process killed')
    enc, dec = m.MAVLink(None, srcSystem=250, srcComponent=198), m.MAVLink(None)
    dec.robust_parsing = True
    ev, power = Evidence(250, 198, expected_parameters=()), PassivePowerEvidence()
    started = time.monotonic()
    next_hb = next_version = 0
    previous_seq = None
    packets, fragments, replies, bad_data = [], [], [], []
    tx = Counter()
    buffer = ''
    baseline = endpoint = None
    error = None

    def now():
        return time.monotonic() - started

    def safe():
        return bool(ev.identity and ev.identity['uid2'] == UID2
                    and ev.identity['firmware'] == '1.14.3' and ev.heartbeats
                    and not ev.heartbeats[-1]['armed']
                    and now() - ev.heartbeats[-1]['elapsed_s'] < 2
                    and not ev.fatal and not power.reboot_observed and not FC_USB.exists())

    try:
        with ExitStack() as stack:
            port = stack.enter_context(serial.Serial(args.port, 115200, timeout=.01,
                                                      write_timeout=.5, exclusive=True))
            port.reset_input_buffer()

            def send(msg):
                if not allowed(msg, safe=safe()):
                    raise RuntimeError('Diagnostic whitelist rejected message')
                packet = msg.pack(enc)
                if port.write(packet) != len(packet):
                    raise RuntimeError('Short write')
                enc.seq = (enc.seq + 1) % 256
                tx[msg.get_type()] += 1

            def shell(payload):
                return enc.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0'))

            def pump():
                nonlocal next_hb, next_version, previous_seq, buffer
                t = now()
                if t > 40 or FC_USB.exists():
                    raise RuntimeError('Session deadline or FC USB appeared')
                for msg in dec.parse_buffer(port.read(8192)) or []:
                    ev.ingest(msg, t)
                    power.ingest(msg, t)
                    if msg.get_type() == 'BAD_DATA':
                        bad_data.append({'t': t, 'reason': msg.reason, 'hex': bytes(msg.data).hex()})
                        continue
                    if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                        continue
                    seq = msg.get_seq()
                    packets.append({'t': t, 'seq': seq,
                                    **sequence_observation(previous_seq, seq),
                                    'wire_hex': bytes(msg.get_msgbuf()).hex()})
                    # Keep the actual message kind separate from the sequence classification.
                    packets[-1]['message_type'] = msg.get_type()
                    previous_seq = seq
                    if msg.get_type() == 'SERIAL_CONTROL':
                        fragment = shell_fragment(msg, t)
                        fragments.append(fragment)
                        buffer += fragment['text']
                if ev.fatal or power.reboot_observed:
                    raise RuntimeError(ev.fatal or 'FC reboot observed')
                if ev.identity and not safe():
                    raise RuntimeError('Wrong identity, firmware or stale/disarmed state lost')
                if len(buffer) > 32768 or len(packets) > 10000 or len(bad_data) > 200:
                    raise RuntimeError('Capture bound exceeded')
                if t >= next_hb:
                    send(enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
                    next_hb = t + 1
                if ev.heartbeats and ev.identity is None and t >= next_version:
                    ev.version_requested_at = t
                    send(enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                    next_version = t + 2
                time.sleep(.005)

            def observe(key):
                nonlocal buffer
                buffer = ''
                begin = now()
                send(shell(COMMANDS[key]))
                poll_at = begin + .5
                complete = False
                while now() - begin < 5:
                    pump()
                    if reply_complete(buffer, COMMANDS[key]):
                        complete = True
                        break
                    if now() >= poll_at:
                        send(shell(b''))
                        poll_at = now() + .5
                parsed = parse_actuator_snapshot(buffer) if key == 'actuator' else (
                    control.parse_flags(buffer) if key.startswith('flags_') else None)
                row = {'key': key, 'begin_s': begin, 'end_s': now(), 'text': buffer,
                       'complete_prompt': complete, 'parsed': parsed}
                replies.append(row)
                print(json.dumps({'key': key, 'complete_prompt': complete,
                                  'parsed': parsed is not None, 'chars': len(buffer)}), flush=True)
                if not complete:
                    raise RuntimeError('No terminal prompt; will not overlap another command')
                return row

            while not safe():
                pump()
                if now() > 10:
                    raise RuntimeError('Fresh identity deadline')
            baseline = observe('actuator')['parsed']
            boot_ms = power.boot_samples[-1]['time_boot_ms'] if power.boot_samples else None
            if not state_valid(baseline, boot_ms) or not baseline['lockdown']:
                raise RuntimeError('Expected disarmed output-locked state')
            observe('status')
            # Two EKF instances provide bounded independent long snapshots, not motor tests.
            observe('flags_0')
            observe('flags_1')
            observe('status')
            endpoint = observe('actuator')['parsed']
            boot_ms = power.boot_samples[-1]['time_boot_ms'] if power.boot_samples else None
            if not state_valid(endpoint, boot_ms) or not endpoint['lockdown']:
                raise RuntimeError('Endpoint disarmed/lockdown check failed')
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or 'Interrupted'
    result = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope': 'Guarded battery-only disarmed read-only TRS Shell capture; no actuation or fusion',
              'propeller_state': 'installed', 'physical_conditions_source': 'operator report',
              'identity': ev.identity, 'heartbeats': ev.heartbeats, 'power': power.result(),
              'baseline': baseline, 'endpoint': endpoint, 'replies': replies, 'packets': packets,
              'shell_fragments': fragments, 'bad_data': bad_data, 'tx_counts': dict(tx),
              'sequence_caveat': 'Unseen sequence slots may reflect sender drops, transport loss, or reordering; do not locate the fault by gaps alone',
              'duration_s': now(), 'error': error, 'capture_completed': error is None,
              'parameter_writes': 0, 'motion_commands': 0, 'resident_home_listener': False,
              'fc_usb_present_at_end': FC_USB.exists(), 'flight_ready': False}
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'error': error, 'packets': len(packets), 'bad_data': len(bad_data),
                      'sequence_gaps': sum(p['kind'] == 'gap_or_reordering' for p in packets)}), flush=True)
    return 0 if error is None else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--guarded-props-installed-readonly-confirmed', action='store_true')
    args = ap.parse_args()
    try:
        return run(args)
    except (OSError, ValueError) as exc:
        ap.error(str(exc))


if __name__ == '__main__':
    raise SystemExit(main())
