#!/usr/bin/env python3
"""One HOME output-lock test, operator-confirmed battery-powered aircraft, TRS only.

USB must remain absent. Fixed actuator listener and param1=2 lockdown only.
No arm, RC, mode, setpoint, real termination, parameter write or clear entry.
This is a one-shot bench, not a persistent emergency-control service.
"""
import argparse
from contextlib import ExitStack
import datetime
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time

import serial
from pymavlink.dialects.v20 import common as m
from estop_only_policy import (DurableStopLatch, FIRMWARE, LISTENER, LOCK_COMMAND,
                              lock_message, parse_actuator_snapshot, shell_message,
                              shell_message_allowed, stop_message_allowed)
from gamepad_estop_monitor import EVENT, HOME, find_pad
from mavlink_readonly_probe import Evidence, FC_USB, PassivePowerEvidence, readonly_message
from persistent_shell import reply_complete
from trs_uart_readonly_check import UID2


def allowed(msg, *, safe=False, authorized=False, latched=False):
    # This new battery bench does not relax the old dual-link/unpowered policy.
    if msg.get_type() in ('HEARTBEAT', 'COMMAND_LONG') and readonly_message(msg):
        return True
    return bool(safe and authorized and (shell_message_allowed(msg) or stop_message_allowed(msg, latched, authorized)))


def state_valid(state, boot_ms):
    return bool(state and boot_ms is not None and not state['armed'] and not state['force_failsafe']
                and not state['manual_lockdown'] and abs(boot_ms * 1000 - state['timestamp_us']) < 2_000_000)


def run(args):
    authorized = bool(args.props_off_battery_confirmed) != bool(args.guarded_props_installed_stop_only_confirmed)
    if not authorized:
        raise ValueError('Select exactly one explicitly confirmed physical mode')
    for path in (args.output, args.latch_file):
        if path.exists():
            raise ValueError('Refusing overwrite of retained evidence/latch')
    if FC_USB.exists():
        raise ValueError('FC USB must be unplugged')
    if subprocess.run(['fuser', str(Path(args.trs_port).resolve())], capture_output=True).returncode == 0:
        raise ValueError('TRS port occupied; no process killed')
    device = find_pad()
    latch = DurableStopLatch(args.latch_file, UID2)
    ev, power = Evidence(250, 199, expected_parameters=()), PassivePowerEvidence()
    enc, dec = m.MAVLink(None, srcSystem=250, srcComponent=199), m.MAVLink(None)
    dec.robust_parsing = True
    start = time.monotonic()
    home_events, snapshots, acks, texts, tx, shell_fragments = [], [], [], [], [], []
    baseline = None
    request_at = release_at = pending_at = None
    buffer = ''
    attempts = 0
    ready = False
    next_hb = next_version = next_poll = next_snapshot = next_retry = next_status = 0
    error = None

    def now():
        return time.monotonic() - start

    def safe():
        return bool(ev.identity and ev.identity['uid2'] == UID2 and ev.identity['firmware'] == FIRMWARE
                    and not ev.fatal and ev.heartbeats and not ev.heartbeats[-1]['armed']
                    and now() - ev.heartbeats[-1]['elapsed_s'] < 2 and not FC_USB.exists()
                    and not power.reboot_observed)

    try:
        with ExitStack() as stack:
            port = stack.enter_context(serial.Serial(args.trs_port, args.baud, timeout=.01, write_timeout=.5, exclusive=True))
            port.reset_input_buffer()
            fd = os.open(device['path'], os.O_RDONLY | os.O_NONBLOCK)
            stack.callback(os.close, fd)
            bitmap = fcntl.ioctl(fd, 0x80000000 | (96 << 16) | (ord('E') << 8) | 0x18, bytes(96))
            if bitmap[HOME // 8] & (1 << (HOME % 8)):
                raise RuntimeError('Release HOME before starting')
            pending_input = b''
            while now() < args.seconds:
                t = now()
                if FC_USB.exists():
                    raise RuntimeError('FC USB appeared; bench stopped')
                outgoing = []
                for msg in dec.parse_buffer(port.read(4096)) or []:
                    ev.ingest(msg, t)
                    power.ingest(msg, t)
                    if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                        continue
                    if msg.get_type() == 'COMMAND_ACK' and msg.command == LOCK_COMMAND and request_at is not None and (
                            msg.target_system, msg.target_component) == (250, 199):
                        acks.append({'elapsed_s': t, 'result': msg.result, 'source': [1, 1], 'target': [250, 199]})
                        if msg.result != m.MAV_RESULT_ACCEPTED:
                            raise RuntimeError('Output-lock command rejected')
                    elif msg.get_type() == 'STATUSTEXT':
                        texts.append({'elapsed_s': t, 'text': msg.text})
                    elif msg.get_type() == 'SERIAL_CONTROL' and pending_at is not None:
                        chunk_text = bytes(msg.data[:msg.count]).decode('utf8', 'replace')
                        shell_fragments.append({'elapsed_s': t, 'count': msg.count, 'text': chunk_text,
                                                'packet_seq': msg.get_seq()})
                        buffer += chunk_text
                        state = parse_actuator_snapshot(buffer)
                        if state and reply_complete(buffer, LISTENER):
                            boot_ms = power.boot_samples[-1]['time_boot_ms'] if power.boot_samples else None
                            if not state_valid(state, boot_ms):
                                raise RuntimeError('Actuator snapshot is unsafe or stale')
                            state.update({'elapsed_s': t, 'requested_at_s': pending_at})
                            snapshots.append(state)
                            buffer, pending_at = '', None
                            if baseline is None:
                                baseline = state
                                ready = True
                                print(f'READY: TRS同机、未解锁、USB未接，原始lockdown={state["lockdown"]}。HOME短按约0.2秒并松开。仅锁定输出。', flush=True)
                if ev.fatal:
                    raise RuntimeError(ev.fatal)
                if ev.identity and (ev.identity['uid2'] != UID2 or ev.identity['firmware'] != FIRMWARE):
                    raise RuntimeError('Wrong vehicle/unsupported firmware')
                if power.reboot_observed:
                    raise RuntimeError('FC reboot observed; lockdown cannot be assumed retained')
                if ready and not safe():
                    raise RuntimeError('Fresh disarmed TRS identity lost')
                if t >= next_hb:
                    outgoing.append(enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
                    next_hb = t + 1
                if ev.heartbeats and ev.identity is None and t >= next_version:
                    ev.version_requested_at = t
                    outgoing.append(enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                    next_version = t + 3
                if not ready and t > 15:
                    raise RuntimeError('Identity/baseline readiness timeout')
                if safe() and baseline is None and pending_at is None:
                    outgoing.append(shell_message(enc))
                    pending_at, buffer = t, ''
                if len(buffer) > 16384 or (pending_at is not None and t-pending_at > 6):
                    raise RuntimeError('TRS actuator listener incomplete/expired')
                if pending_at is not None and t >= next_poll:
                    outgoing.append(shell_message(enc, poll=True))
                    next_poll = t + .5
                try:
                    chunk = os.read(fd, EVENT.size * 64)
                    if not chunk:
                        raise RuntimeError('Gamepad disconnected')
                    pending_input += chunk
                except BlockingIOError:
                    pass
                while len(pending_input) >= EVENT.size:
                    _, _, kind, code, value = EVENT.unpack(pending_input[:EVENT.size])
                    pending_input = pending_input[EVENT.size:]
                    if kind == 0 and code == 3:
                        raise RuntimeError('Gamepad event overflow')
                    if kind == 1 and code == HOME:
                        if value in (1, 2) and not ready:
                            raise RuntimeError('HOME before ready; no stop request sent')
                        latch.event(kind, code, value)
                        home_events.append({'elapsed_s': t, 'value': value})
                        if value == 1 and request_at is None:
                            request_at, next_retry = t, t
                            print('HOME已锁存；仅发送输出锁定，不发解除锁定。', flush=True)
                        elif value == 0 and request_at is not None:
                            release_at = t
                # Prioritize stop ahead of queued informational traffic.
                if request_at is not None and not acks and t >= next_retry and attempts < 3:
                    outgoing.insert(0, lock_message(enc, ev.identity, UID2, latch, True, safe(), confirmation=attempts))
                    attempts += 1
                    next_retry = t + .8
                if request_at is not None and t-request_at > .4 and pending_at is None and t >= next_snapshot:
                    outgoing.append(shell_message(enc))
                    pending_at, buffer, next_snapshot = t, '', t+1
                for msg in outgoing:
                    if not allowed(msg, safe=safe(), authorized=authorized, latched=latch.latched):
                        raise RuntimeError('Single-TRS stop-only send policy rejected message')
                    packet = msg.pack(enc)
                    if port.write(packet) != len(packet):
                        raise OSError('Short serial write')
                    enc.seq = (enc.seq + 1) % 256
                    tx.append({'elapsed_s': t, 'kind': msg.get_type(), 'command': getattr(msg, 'command', None),
                               'param1': getattr(msg, 'param1', None)})
                post = [s for s in snapshots if release_at is not None and s['requested_at_s'] >= release_at+1.5]
                if acks and post and post[-1]['lockdown'] and t-release_at >= 2:
                    print('PASS: HOME经TRS ACK接受；同一路无线内部状态确认lockdown=True，松开后仍锁定，未解锁。', flush=True)
                    break
                if request_at is not None and t-request_at > 10:
                    raise RuntimeError('Lock ACK/post-release state timeout')
                if t >= next_status:
                    print(f'STATUS {t:.1f}s ready={ready} latched={latch.latched} ACK={len(acks)} snapshots={len(snapshots)}', flush=True)
                    next_status = t + 10
            else:
                raise RuntimeError('No completed HOME test in bounded window')
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or 'Interrupted'
        if request_at is not None:
            latch.fail('bench_error')
    post = [s for s in snapshots if release_at is not None and s['requested_at_s'] >= release_at+1.5]
    verified = bool(not error and latch.latched and acks and post and post[-1]['lockdown']
                    and not post[-1]['armed'] and baseline and post[-1]['timestamp_us'] > baseline['timestamp_us'])
    result = {'scope': 'Battery-powered disarmed single-TRS HOME software output-lock bench; no motor/flight test',
              'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'props_off_battery_confirmed': args.props_off_battery_confirmed,
              'guarded_props_installed_stop_only_confirmed': args.guarded_props_installed_stop_only_confirmed,
              'propeller_state': 'installed' if args.guarded_props_installed_stop_only_confirmed else 'removed',
              'physical_conditions_source': 'operator report, not independent physical verification',
              'authorization_scope': 'HOME-triggered output-lock only; no motor/arming/unlock/fusion authorization',
              'battery_remained_connected': True, 'fc_usb_present_at_end': FC_USB.exists(),
              'device': device, 'evidence': ev.result(), 'passive_power': power.result(),
              'baseline': baseline, 'snapshots': snapshots, 'home_events': home_events, 'acks': acks,
              'tx_messages': tx, 'statustext': texts, 'latch_file': str(args.latch_file),
              'shell_fragments': shell_fragments, 'incomplete_shell_text': buffer,
              'error': error, 'lockdown_after_release_verified': verified,
              'lock_transition_false_to_true_observed': bool(verified and not baseline['lockdown']),
              'physical_motor_stop_verified': False, 'independent_of_pc_emergency_verified': False,
              'one_shot_listener_exited': True, 'serial_port_released': True, 'flight_ready': False}
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ('baseline', 'snapshots', 'acks', 'error', 'lockdown_after_release_verified')},
                     ensure_ascii=False, indent=2), flush=True)
    return 0 if verified else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trs-port', required=True)
    ap.add_argument('--baud', type=int, choices=(57600, 115200), default=115200)
    physical = ap.add_mutually_exclusive_group(required=True)
    physical.add_argument('--props-off-battery-confirmed', action='store_true')
    physical.add_argument('--guarded-props-installed-stop-only-confirmed', action='store_true',
                          help='Operator confirms intact protective frame, stationary aircraft, USB absent and battery-only; HOME output-lock only, no actuation')
    ap.add_argument('--seconds', type=float, default=180)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--latch-file', type=Path, required=True)
    args = ap.parse_args()
    if not 10 <= args.seconds <= 300:
        ap.error('10..300 seconds required')
    try:
        raise SystemExit(run(args))
    except (OSError, ValueError, RuntimeError) as exc:
        ap.exit(2, str(exc)+'\n')


if __name__ == '__main__':
    main()
