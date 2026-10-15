#!/usr/bin/env python3
"""TRS波特率排查：仅USB身份/固定串口参数/固定mavlink status，不改配置。"""

import argparse
from collections import Counter
import datetime
import json
from pathlib import Path
import subprocess
import time

import serial
from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import Evidence, decode_parameter, readonly_message

UID2 = '000000000000000000000000000000000000'
USB = '/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00'
PARAMS = tuple(f'MAV_{i}_{suffix}' for i in range(3)
               for suffix in ('CONFIG', 'MODE', 'RATE')) + (
    'SER_TEL1_BAUD', 'SER_TEL2_BAUD', 'SER_TEL3_BAUD', 'SER_TEL4_BAUD')
STATUS = b'mavlink status\n'


def allowed(msg, *, safe=False):
    kind = msg.get_type()
    if kind in ('HEARTBEAT', 'COMMAND_LONG', 'PING'):
        return readonly_message(msg)
    if kind == 'PARAM_REQUEST_READ':
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\0')
        return (safe and name in PARAMS and msg.param_index == -1
                and (msg.target_system, msg.target_component) == (1, 1))
    if kind == 'SERIAL_CONTROL':
        data = bytes(msg.data)
        if (msg.device, msg.timeout, msg.baudrate) != (10, 0, 0):
            return False
        if msg.flags == 0:
            return False
        return (safe and msg.flags == 2 and any(
            msg.count == len(p) and data == p.ljust(70, b'\0') for p in (b'', STATUS)))
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        ap.error('Output must be new')
    owners = subprocess.run(['fuser', str(Path(USB).resolve())], capture_output=True, text=True)
    if owners.returncode == 0:
        ap.error(f'USB occupied: {owners.stdout.strip()}')
    enc = m.MAVLink(None, srcSystem=250, srcComponent=190)
    dec = m.MAVLink(None)
    dec.robust_parsing = True
    ev = Evidence(expected_parameters=PARAMS)
    counts = Counter()
    start = time.monotonic()
    shell = ''
    params = {}
    requests = {}
    status_at = None
    last_hb = last_version = last_params = last_poll = -100.0
    rx = tx = 0
    error = None
    safe = False

    def send(port, msg, gate=False):
        nonlocal tx
        if not allowed(msg, safe=gate):
            raise ValueError('Non-read-only message refused')
        packet = msg.pack(enc)
        if port.write(packet) != len(packet):
            raise OSError('Short write')
        enc.seq = (enc.seq + 1) % 256
        tx += len(packet)
        counts[msg.get_type()] += 1

    def serial_msg(payload=b'', close=False):
        return enc.serial_control_encode(10, 0 if close else 2, 0, 0,
                                         len(payload), payload.ljust(70, b'\0'))

    try:
        with serial.Serial(USB, 115200, timeout=.02, write_timeout=1, exclusive=True) as port:
            try:
                while time.monotonic() - start < 15:
                    now = time.monotonic() - start
                    chunk = port.read(4096)
                    rx += len(chunk)
                    for msg in dec.parse_buffer(chunk) or []:
                        ev.ingest(msg, now)
                        if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                            continue
                        if msg.get_type() == 'PARAM_VALUE':
                            name = str(msg.param_id).rstrip('\0')
                            if name in requests:
                                params[name] = {'value': decode_parameter(msg), 'type': msg.param_type}
                        if msg.get_type() == 'SERIAL_CONTROL' and status_at is not None:
                            shell += bytes(msg.data[:msg.count]).decode('utf-8', 'replace')
                            if len(shell) > 32768:
                                raise RuntimeError('Shell output exceeded bound')
                    if ev.fatal:
                        raise RuntimeError(ev.fatal)
                    fresh = bool(ev.heartbeats) and now - ev.heartbeats[-1]['elapsed_s'] < 2
                    if now - last_hb >= 1:
                        send(port, enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
                        last_hb = now
                    if fresh and ev.identity is None and now - last_version >= 2:
                        ev.version_requested_at = now
                        send(port, enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                        last_version = now
                    safe = bool(fresh and ev.identity and ev.identity['uid2'] == UID2
                                and ev.identity['firmware'] == '1.14.3')
                    if ev.identity and not safe:
                        raise RuntimeError('Identity or fresh disarmed heartbeat gate failed')
                    if safe and now - last_params >= 3:
                        for name in PARAMS:
                            if name not in params:
                                requests[name] = now
                                ev.requests[name] = now
                                send(port, enc.param_request_read_encode(1, 1, name.encode(), -1), True)
                        last_params = now
                    if safe and status_at is None:
                        send(port, serial_msg(STATUS), True)
                        status_at = last_poll = now
                    if safe and status_at is not None and now - last_poll >= .5:
                        send(port, serial_msg(), True)
                        last_poll = now
                    if status_at is not None and now - status_at >= 5 and set(PARAMS) <= set(params):
                        break
            finally:
                pass  # 串口句柄照常释放，不发送会触发旧版Shell销毁的flags=0。
    except (OSError, ValueError, RuntimeError, serial.SerialException) as exc:
        error = str(exc)
    result = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'identity': ev.identity, 'heartbeats': ev.heartbeats, 'parameters': params,
              'missing_parameters': sorted(set(PARAMS) - set(params)),
              'mavlink_status': shell, 'status_requested_at_s': status_at,
              'rx_bytes': rx, 'tx_bytes': tx, 'tx_messages': dict(counts), 'error': error,
              'duration_s': time.monotonic() - start,
              'scope': 'Fixed USB diagnostic reads only; no parameter-write, reboot, RC, mode, arm or setpoint'}
    with args.output.open('x') as f:
        json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if error is None and params and 'instance' in shell else 1


if __name__ == '__main__':
    raise SystemExit(main())
