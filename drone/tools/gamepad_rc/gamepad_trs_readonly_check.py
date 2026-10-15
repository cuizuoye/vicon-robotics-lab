#!/usr/bin/env python3
"""HOME→只读PING→TRS→PX4回应：验证端到端通信，不发送急停/RC/运动指令。"""

import argparse
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

from gamepad_estop_monitor import EVENT, HOME, StopLatch, find_pad
from mavlink_readonly_probe import Evidence, send_readonly


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, choices=(57600, 115200), required=True)
    ap.add_argument('--expect-uid2', required=True)
    ap.add_argument('--seconds', type=float, default=120)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists() or not 10 <= args.seconds <= 180:
        ap.error('Use a new output file and 10..180 seconds')
    if subprocess.run(['fuser', str(Path(args.port).resolve())], capture_output=True).returncode == 0:
        ap.error('Serial port is occupied')
    device = find_pad()
    fd = os.open(device['path'], os.O_RDONLY | os.O_NONBLOCK)
    latch = StopLatch()
    evidence = Evidence()
    enc = m.MAVLink(None, srcSystem=250, srcComponent=190)
    decoder = m.MAVLink(None)
    decoder.robust_parsing = True
    events = []
    request = None
    released_at = None
    error = None
    tx = []
    ready_announced = False
    start = time.monotonic()
    next_heartbeat = next_version = next_status = 0.0
    try:
        bitmap = fcntl.ioctl(fd, 0x80000000 | (96 << 16) | (ord('E') << 8) | 0x18, bytes(96))
        if bitmap[HOME // 8] & (1 << (HOME % 8)):
            raise RuntimeError('Release HOME before starting this check')
        with serial.Serial(args.port, args.baud, timeout=0.02,
                           write_timeout=1, exclusive=True) as port:
            pending = b''
            while time.monotonic() - start < args.seconds:
                now = time.monotonic() - start
                for message in decoder.parse_buffer(port.read(4096)) or []:
                    evidence.ingest(message, now)
                if evidence.fatal:
                    raise RuntimeError(evidence.fatal)
                to_send = []
                if now >= next_heartbeat:
                    to_send.append(enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
                    next_heartbeat = now + 1
                fresh = bool(evidence.heartbeats) and now - evidence.heartbeats[-1]['elapsed_s'] < 2
                if fresh and evidence.identity is None and now >= next_version:
                    evidence.version_requested_at = now
                    to_send.append(enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                    next_version = now + 3
                if evidence.identity:
                    if evidence.identity['uid2'] != args.expect_uid2:
                        raise RuntimeError('TRS vehicle identity differs from the USB baseline')
                    if not ready_announced and fresh:
                        ready_announced = True
                        print('READY: 已识别同一台未解锁飞控。现在HOME按下约1秒并松开；只发送PING，不执行急停。', flush=True)
                elif now > 10:
                    raise RuntimeError('No verified flight-controller identity')
                if ready_announced and not fresh:
                    raise RuntimeError('Flight-controller heartbeat expired')
                try:
                    chunk = os.read(fd, EVENT.size * 64)
                    if not chunk:
                        raise RuntimeError('Gamepad input disconnected')
                    pending += chunk
                except BlockingIOError:
                    pass
                while len(pending) >= EVENT.size:
                    _, _, kind, code, value = EVENT.unpack(pending[:EVENT.size])
                    pending = pending[EVENT.size:]
                    latch.event(kind, code, value)
                    if kind == 0 and code == 3:
                        raise RuntimeError('Input queue overflow')
                    if kind == 1 and code == HOME:
                        events.append({'elapsed_s': now, 'code': code, 'value': value})
                        if value == 1 and ready_announced and request is None:
                            usec, seq = time.monotonic_ns() // 1000, secrets.randbits(32)
                            request = {'time_usec': usec, 'sequence': seq,
                                       'button_elapsed_s': now}
                            evidence.pings[(usec, seq)] = now
                            to_send.append(enc.ping_encode(usec, seq, 0, 0))
                            print('HOME → 发送本轮随机PING', flush=True)
                        elif value == 0 and request is not None:
                            released_at = now
                            print(f'HOME松开；软件锁存={latch.latched}', flush=True)
                for message in to_send:
                    send_readonly(enc, port, message)
                    tx.append({'elapsed_s': now, 'message': message.get_type()})
                if evidence.ping_rtt_ms and released_at is not None and now - released_at > 1:
                    print(f'匹配的PX4 PING回应：RTT={evidence.ping_rtt_ms[-1]}ms', flush=True)
                    break
                if now >= next_status:
                    print(f'STATUS {now:.1f}s fc_hb={len(evidence.heartbeats)} home_ping_reply={len(evidence.ping_rtt_ms)}', flush=True)
                    next_status = now + 10
            if not (evidence.ping_matches and released_at is not None and latch.latched):
                raise RuntimeError('Timed out before a complete HOME press/release and matched FC PING reply')
    except (OSError, ValueError, RuntimeError, serial.SerialException) as exc:
        error = str(exc)
        latch.fail('diagnostic_failed')
    except KeyboardInterrupt:
        error = 'Interrupted'
        latch.fail('diagnostic_interrupted')
    finally:
        os.close(fd)
    result = evidence.result()
    result.update({'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                   'port': args.port, 'baud': args.baud, 'device': device,
                   'home_events': events, 'home_ping_request': request,
                   'released': released_at is not None, 'software_stop_latched': latch.latched,
                   'tx_messages': tx, 'error': error or evidence.fatal,
                   'identity_matches_usb': bool(evidence.identity)
                       and evidence.identity['uid2'] == args.expect_uid2,
                   'home_to_trs_to_fc_verified': bool(evidence.ping_matches)
                       and released_at is not None and latch.latched and error is None,
                   'scope': 'Only GCS heartbeat, AUTOPILOT_VERSION request and HOME-triggered PING; no actual kill/RC/control'})
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: result[key] for key in (
        'identity', 'fc_heartbeat_count', 'home_events', 'home_ping_request',
        'ping_matches', 'ping_rtt_ms', 'released', 'software_stop_latched',
        'identity_matches_usb', 'home_to_trs_to_fc_verified', 'error', 'scope',
    )}, ensure_ascii=False, indent=2), flush=True)
    return 0 if result['home_to_trs_to_fc_verified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
