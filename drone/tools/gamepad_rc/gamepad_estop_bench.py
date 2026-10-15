#!/usr/bin/env python3
"""HOME经TRS锁定未解锁PX4 1.14.3输出；USB只读核对，不解除锁定。"""

import argparse
from collections import Counter
from contextlib import ExitStack
import datetime
import fcntl
import json
import os
from pathlib import Path
import secrets
import subprocess
import time

import serial
from pymavlink.dialects.v20 import common as m

from estop_only_policy import (DurableStopLatch, FIRMWARE, LISTENER, LOCK_COMMAND,
                               lock_message, parse_actuator_snapshot,
                               send_gated, shell_message)
from persistent_shell import reply_complete
from gamepad_estop_monitor import EVENT, HOME, find_pad
from mavlink_readonly_probe import Evidence, FC_USB, PassivePowerEvidence


def physical_mode(args):
    guarded = bool(getattr(args, 'usb_only_props_installed_stop_only_confirmed', False))
    if guarded and args.bench_lockdown:
        raise ValueError('Physical confirmation modes are mutually exclusive')
    if guarded and (not FC_USB.exists() or Path(args.usb_port).resolve() != FC_USB.resolve()):
        raise ValueError('USB-only stop test requires the known FC USB device')
    return {'propeller_state': 'installed' if guarded else 'removed' if args.bench_lockdown else 'not_confirmed',
            'propulsion_battery_disconnected_operator_confirmed': guarded or args.bench_lockdown,
            'usb_only_props_installed_stop_only_confirmed': guarded,
            'physical_conditions_source': 'operator report; not independently physically verified'}


def run(args):
    physical = physical_mode(args)
    lockdown_authorized = bool(args.bench_lockdown or physical['usb_only_props_installed_stop_only_confirmed'])
    for path in (args.output, args.latch_file):
        if path.exists():
            raise ValueError(f'Retained evidence/latch exists; refusing overwrite: {path}')
    if Path(args.trs_port).resolve() == Path(args.usb_port).resolve():
        raise ValueError('TRS and USB must be different devices')
    for path in (args.trs_port, args.usb_port):
        if subprocess.run(['fuser', str(Path(path).resolve())], capture_output=True).returncode == 0:
            raise ValueError(f'Serial device is occupied: {path}')
    device = find_pad()
    latch = DurableStopLatch(args.latch_file, args.expect_uid2)
    evidence = {'trs': Evidence(250, 190), 'usb': Evidence(251, 191)}
    power = PassivePowerEvidence()
    encoders = {key: m.MAVLink(None, srcSystem=system, srcComponent=component)
                for key, system, component in (('trs', 250, 190), ('usb', 251, 191))}
    decoders = {key: m.MAVLink(None) for key in evidence}
    for decoder in decoders.values():
        decoder.robust_parsing = True
    tx, home_events, snapshots, acks, texts = [], [], [], [], []
    baseline = None
    shell_buffer = ''
    snapshot_requested = None
    ready, request_at, release_at = False, None, None
    attempts, next_retry, next_snapshot = 0, 0.0, 0.0
    next_hb = dict.fromkeys(evidence, 0.0)
    next_version = dict.fromkeys(evidence, 0.0)
    next_poll, next_status = 0.0, 0.0
    diagnostic_request = None
    error = None
    started = time.monotonic()
    try:
        with ExitStack() as stack:
            fd = os.open(device['path'], os.O_RDONLY | os.O_NONBLOCK)
            stack.callback(os.close, fd)
            bitmap = fcntl.ioctl(fd, 0x80000000 | (96 << 16) | (ord('E') << 8) | 0x18, bytes(96))
            if bitmap[HOME // 8] & (1 << (HOME % 8)):
                raise RuntimeError('Release HOME before starting')
            ports = {
                'trs': stack.enter_context(serial.Serial(args.trs_port, args.baud, timeout=0.01,
                                                         write_timeout=1, exclusive=True)),
                'usb': stack.enter_context(serial.Serial(args.usb_port, 115200, timeout=0.01,
                                                         write_timeout=1, exclusive=True)),
            }
            pending = b''
            while time.monotonic() - started < args.seconds:
                now = time.monotonic() - started
                if physical['usb_only_props_installed_stop_only_confirmed'] and not FC_USB.exists():
                    raise RuntimeError('FC USB disappeared; stop-only bench ended')
                outgoing = []
                for route, port in ports.items():
                    for message in decoders[route].parse_buffer(port.read(4096)) or []:
                        evidence[route].ingest(message, now)
                        if route == 'usb':
                            power.ingest(message, now)
                            if power.reboot_observed:
                                raise RuntimeError('FC boot clock regressed; output lock cannot be assumed retained')
                        if (message.get_srcSystem(), message.get_srcComponent()) != (1, 1):
                            continue
                        kind = message.get_type()
                        if route == 'trs' and kind == 'COMMAND_ACK' and message.command == LOCK_COMMAND:
                            if (request_at is not None and now >= request_at
                                    and (message.target_system, message.target_component) == (250, 190)):
                                acks.append({'elapsed_s': now, 'result': message.result,
                                             'source': [1, 1], 'target': [250, 190]})
                                if message.result != m.MAV_RESULT_ACCEPTED:
                                    raise RuntimeError(f'Lockdown command rejected: {message.result}')
                        if kind == 'STATUSTEXT':
                            texts.append({'route': route, 'elapsed_s': now, 'text': message.text})
                        if route == 'usb' and kind == 'SERIAL_CONTROL' and snapshot_requested is not None:
                            shell_buffer += bytes(message.data[:message.count]).decode('utf-8', 'replace')
                            if len(shell_buffer) > 16384:
                                raise RuntimeError('USB listener reply overflow')
                            snapshot = parse_actuator_snapshot(shell_buffer)
                            if snapshot and reply_complete(shell_buffer, LISTENER):
                                snapshot.update({'elapsed_s': now, 'requested_at_s': snapshot_requested})
                                snapshots.append(snapshot)
                                snapshot_requested = None
                                shell_buffer = ''
                                if baseline is None:
                                    baseline = snapshot
                                    if any(baseline[name] for name in ('armed', 'lockdown', 'manual_lockdown', 'force_failsafe')):
                                        raise RuntimeError('Baseline is not disarmed/unlocked; no stop test sent')
                                    ready = True
                                    print('READY: 两条链路身份相同、未解锁、原始lockdown=False。现在HOME短按约0.2秒并松开。', flush=True)
                                elif request_at is not None and snapshot['armed']:
                                    raise RuntimeError('Unexpected armed state')
                    if evidence[route].fatal:
                        raise RuntimeError(evidence[route].fatal)
                    identity = evidence[route].identity
                    if identity and (identity['uid2'] != args.expect_uid2 or identity['firmware'] != FIRMWARE):
                        raise RuntimeError(f'{route}: wrong vehicle or unsupported firmware')
                    fresh = bool(evidence[route].heartbeats) and now - evidence[route].heartbeats[-1]['elapsed_s'] < 2
                    if ready and not fresh:
                        raise RuntimeError(f'{route}: stale heartbeat')
                    if now >= next_hb[route]:
                        outgoing.append((route, encoders[route].heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0)))
                        next_hb[route] = now + 1
                    if fresh and identity is None and now >= next_version[route]:
                        evidence[route].version_requested_at = now
                        outgoing.append((route, encoders[route].command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)))
                        next_version[route] = now + 3
                both_identified = all(e.identity is not None for e in evidence.values())
                if not both_identified and now > 12:
                    raise RuntimeError('No two-port verified identity')
                if both_identified and baseline is None and snapshot_requested is None:
                    outgoing.append(('usb', shell_message(encoders['usb'])))
                    snapshot_requested, shell_buffer = now, ''
                if snapshot_requested is not None and now - snapshot_requested > 4:
                    raise RuntimeError('USB listener reply incomplete/expired')
                if snapshot_requested is not None and now >= next_poll:
                    outgoing.append(('usb', shell_message(encoders['usb'], poll=True)))
                    next_poll = now + 1
                try:
                    chunk = os.read(fd, EVENT.size * 64)
                    if not chunk:
                        raise RuntimeError('Gamepad disconnected')
                    pending += chunk
                except BlockingIOError:
                    pass
                while len(pending) >= EVENT.size:
                    _, _, kind, code, value = EVENT.unpack(pending[:EVENT.size])
                    pending = pending[EVENT.size:]
                    if kind == 0 and code == 3:
                        raise RuntimeError('Gamepad input queue overflow')
                    if kind == 1 and code == HOME:
                        if value in (1, 2) and not ready:
                            raise RuntimeError('HOME pressed before baseline ready; no command sent')
                        latch.event(kind, code, value)
                        home_events.append({'elapsed_s': now, 'value': value})
                        if value == 1 and request_at is None:
                            request_at, next_retry = now, now
                            if not lockdown_authorized:
                                usec, seq = time.monotonic_ns() // 1000, secrets.randbits(32)
                                evidence['trs'].pings[(usec, seq)] = now
                                diagnostic_request = {'time_usec': usec, 'sequence': seq}
                                outgoing.append(('trs', encoders['trs'].ping_encode(usec, seq, 0, 0)))
                            print(f'HOME停止已锁存；实际lockdown发送={lockdown_authorized}', flush=True)
                        elif value == 0 and request_at is not None:
                            release_at = now
                            print('HOME松开；不发送解除锁定', flush=True)
                if lockdown_authorized and request_at is not None and not acks and now >= next_retry and attempts < 3:
                    command = lock_message(encoders['trs'], evidence['trs'].identity,
                                           args.expect_uid2, latch, True,
                                           evidence['trs'].heartbeats[-1]['armed'] is False,
                                           confirmation=attempts)
                    outgoing.append(('trs', command))
                    attempts += 1
                    next_retry = now + 0.7
                if request_at is not None and now - request_at > 0.3 and snapshot_requested is None and now >= next_snapshot:
                    outgoing.append(('usb', shell_message(encoders['usb'])))
                    snapshot_requested, shell_buffer = now, ''
                    next_snapshot = now + 1
                for route, message in outgoing:
                    send_gated(encoders[route], ports[route], message, route=route,
                               latch=latch, authorized=lockdown_authorized)
                    tx.append({'route': route, 'elapsed_s': now, 'message': message.get_type(),
                               'command': getattr(message, 'command', None),
                               'param1': getattr(message, 'param1', None)})
                if request_at is not None and now - request_at > 8 and lockdown_authorized:
                    raise RuntimeError('Lockdown ACK/state verification timed out')
                if release_at is not None and now - release_at >= 2:
                    if lockdown_authorized:
                        post_release = [s for s in snapshots if s['requested_at_s'] >= release_at + 1.5]
                        if acks and post_release and post_release[-1]['lockdown'] and not post_release[-1]['armed']:
                            print('PASS: TRS ACK accepted，USB内部lockdown=True；松开后仍锁定、未解锁。', flush=True)
                            break
                    elif evidence['trs'].ping_matches:
                        print('PASS: 诊断PING和本地停止锁存；未发送lockdown。', flush=True)
                        break
                if now >= next_status:
                    print(f'STATUS {now:.1f}s ready={ready} latch={latch.latched} ack={len(acks)} snapshots={len(snapshots)}', flush=True)
                    next_status = now + 10
            else:
                raise RuntimeError('No completed HOME test within the waiting window')
    except (OSError, ValueError, RuntimeError, serial.SerialException, KeyboardInterrupt) as exc:
        error = str(exc) or 'Interrupted'
        if request_at is not None:
            try:
                latch.fail('bench_error')
            except OSError:
                pass
    post = [s for s in snapshots if release_at is not None and s['requested_at_s'] >= release_at + 1.5]
    verified = bool(lockdown_authorized and not error and acks and post and post[-1]['lockdown']
                    and not post[-1]['armed'] and latch.latched and baseline and not baseline['lockdown']
                    and post[-1]['timestamp_us'] > baseline['timestamp_us'])
    result = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope': 'Disarmed software output-lock test; no physical motor-stop test, no RC/arm/mode/clear command',
              'device': device, 'actual_lockdown_authorized': lockdown_authorized,
              'physical_state': physical, 'passive_power': power.result(),
              'flight_authorized': False, 'persistent_monitor_running_after_exit': False,
              'evidence': {route: item.result() for route, item in evidence.items()},
              'home_events': home_events, 'baseline': baseline, 'actuator_snapshots': snapshots,
              'command_acks': acks, 'tx_messages': tx, 'statustext': texts,
              'latch_file': str(args.latch_file), 'software_stop_latched': latch.latched,
              'lockdown_after_release_verified': verified,
              'physical_motor_stop_verified': False,
              'diagnostic_request': diagnostic_request, 'error': error,
              'serial_ports_released': True}
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: result[key] for key in ('baseline', 'actuator_snapshots', 'command_acks',
                     'software_stop_latched', 'lockdown_after_release_verified', 'physical_motor_stop_verified', 'error')},
                    ensure_ascii=False, indent=2), flush=True)
    return 0 if verified or (not lockdown_authorized and not error) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trs-port', required=True)
    parser.add_argument('--usb-port', required=True)
    parser.add_argument('--baud', type=int, choices=(57600, 115200), default=57600)
    parser.add_argument('--expect-uid2', required=True)
    physical = parser.add_mutually_exclusive_group()
    physical.add_argument('--bench-lockdown', action='store_true', help='Only after confirmation: battery disconnected and props removed')
    physical.add_argument('--usb-only-props-installed-stop-only-confirmed', action='store_true',
                          help='Operator confirms propulsion battery disconnected, FC USB-only, props installed; HOME output-lock only, never clear/arm/move')
    parser.add_argument('--seconds', type=float, default=180)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--latch-file', type=Path, required=True)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 300:
        parser.error('Use 10..300 seconds')
    try:
        return run(args)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(2, f'{exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
