#!/usr/bin/env python3
"""已断动力拆桨台架：仅SER_TEL1_BAUD 57600↔115200，读回后一次正常重启。"""

import argparse
from collections import Counter
import datetime
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import time

import serial
from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import Evidence, decode_parameter, readonly_message
from trs_uart_readonly_check import UID2, USB

NAME = 'SER_TEL1_BAUD'


def write_message(enc, value):
    if type(value) is not int or value not in (57600, 115200):
        raise ValueError('Only the original baud and trial baud are permitted')
    wire = struct.unpack('<f', struct.pack('<i', value))[0]
    return enc.param_set_encode(1, 1, NAME.encode(), wire, m.MAV_PARAM_TYPE_INT32)


def allowed(msg, *, safe=False, authorized=False, backup_verified=False, readback_verified=False):
    kind = msg.get_type()
    if kind == 'COMMAND_LONG' and msg.command == m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        return bool(safe and authorized and backup_verified and readback_verified
                    and (msg.target_system, msg.target_component, msg.confirmation) == (1, 1, 0)
                    and msg.param1 == 1 and all(getattr(msg, f'param{i}') == 0 for i in range(2, 8)))
    if kind == 'PARAM_SET':
        name = msg.param_id.decode('ascii').rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        value = struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
        return bool(safe and authorized and backup_verified and name == NAME
                    and msg.param_type == m.MAV_PARAM_TYPE_INT32 and value in (57600, 115200)
                    and (msg.target_system, msg.target_component) == (1, 1))
    if kind == 'PARAM_REQUEST_READ':
        name = msg.param_id.decode('ascii').rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        return bool(safe and name in (NAME, 'MAV_0_CONFIG') and msg.param_index == -1
                    and (msg.target_system, msg.target_component) == (1, 1))
    if kind in ('HEARTBEAT', 'PING', 'COMMAND_LONG'):
        return readonly_message(msg)
    return False


def verify_backup(directory):
    manifest = json.loads((directory / 'parameters_manifest.json').read_text())
    capture = json.loads((directory / 'parameters_verified.json').read_text())
    if not (manifest['complete'] and capture['complete'] and capture['expected_count'] == 926
            and capture['received_count'] == 926 and not capture['missing_indices']):
        raise ValueError('Complete 926-parameter backup required')
    for filename in ('parameters_verified.json', 'parameters_verified.txt'):
        if hashlib.sha256((directory / filename).read_bytes()).hexdigest() != manifest['sha256'][filename]:
            raise ValueError('Backup hash mismatch')
    records = {r['name']: r for r in capture['records']}
    if records[NAME]['value'] != 57600 or records[NAME]['type'] != m.MAV_PARAM_TYPE_INT32:
        raise ValueError('Expected original TELEM1=57600 INT32')
    if records['MAV_0_CONFIG']['value'] != 101:
        raise ValueError('Expected MAV_0 mapped to TELEM1')
    return manifest['sha256']


class Session:
    def __init__(self, port):
        self.port = port
        self.enc = m.MAVLink(None, srcSystem=250, srcComponent=190)
        self.dec = m.MAVLink(None)
        self.dec.robust_parsing = True
        self.ev = Evidence()
        self.start = time.monotonic()
        self.next_hb = 0
        self.parameters = {}
        self.sent = Counter()
        self.readback_verified = False
        self.reboot_ack = None

    def safe(self):
        now = time.monotonic() - self.start
        return bool(not self.ev.fatal and self.ev.identity
                    and self.ev.identity['uid2'] == UID2 and self.ev.identity['firmware'] == '1.14.3'
                    and self.ev.heartbeats and not self.ev.heartbeats[-1]['armed']
                    and now - self.ev.heartbeats[-1]['elapsed_s'] < 2)

    def send(self, msg):
        if not allowed(msg, safe=self.safe(), authorized=True, backup_verified=True,
                       readback_verified=self.readback_verified):
            raise ValueError('Maintenance send gate refused message')
        packet = msg.pack(self.enc)
        if self.port.write(packet) != len(packet):
            raise OSError('Short serial write')
        self.enc.seq = (self.enc.seq + 1) % 256
        self.sent[msg.get_type()] += 1

    def pump(self):
        now = time.monotonic() - self.start
        for msg in self.dec.parse_buffer(self.port.read(4096)) or []:
            self.ev.ingest(msg, now)
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                continue
            if msg.get_type() == 'PARAM_VALUE':
                self.parameters[str(msg.param_id).rstrip('\0')] = {'value': decode_parameter(msg), 'type': msg.param_type}
            if msg.get_type() == 'COMMAND_ACK' and msg.command == m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                self.reboot_ack = msg.to_dict()
        if self.ev.fatal:
            raise RuntimeError(self.ev.fatal)
        if now >= self.next_hb:
            self.send(self.enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            self.next_hb = now + 1

    def identity(self):
        deadline = time.monotonic() + 8
        next_request = 0
        while time.monotonic() < deadline:
            self.pump()
            now = time.monotonic() - self.start
            if self.safe():
                return
            if self.ev.identity:
                raise RuntimeError('Wrong identity or stale heartbeat')
            if self.ev.heartbeats and now >= next_request:
                self.ev.version_requested_at = now
                self.send(self.enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                next_request = now + 2
        raise RuntimeError('No verified disarmed expected aircraft')

    def read(self, name):
        self.parameters.pop(name, None)
        deadline = time.monotonic() + 5
        # 明确发出本轮查询，不把写参后排队的PARAM_VALUE回显当作唯一读回证据。
        self.send(self.enc.param_request_read_encode(1, 1, name.encode(), -1))
        next_request = time.monotonic() + 1
        while time.monotonic() < deadline:
            self.pump()
            if name in self.parameters:
                return self.parameters[name]
            if time.monotonic() >= next_request:
                self.send(self.enc.param_request_read_encode(1, 1, name.encode(), -1))
                next_request = time.monotonic() + 1
        raise RuntimeError('No current parameter reply: ' + name)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--action', required=True, choices=('trial', 'restore'))
    ap.add_argument('--backup-dir', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--bench-confirmed', action='store_true')
    args = ap.parse_args()
    if not args.bench_confirmed or args.output.exists():
        ap.error('Fresh physical bench confirmation and new output required')
    hashes = verify_backup(args.backup_dir)
    owners = subprocess.run(['fuser', str(Path(USB).resolve())], capture_output=True, text=True)
    if owners.returncode == 0:
        ap.error('USB is occupied: ' + owners.stdout.strip())
    expected, target = (57600, 115200) if args.action == 'trial' else (115200, 57600)
    record = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'action': args.action, 'expected': expected, 'target': target, 'backup_hashes': hashes,
              'write_sent': False, 'readback_verified': False, 'reboot_sent': False, 'error': None,
              'scope': 'Only SER_TEL1_BAUD and one normal reboot; no arm/RC/mode/setpoints'}
    session = None
    try:
        with serial.Serial(USB, 115200, timeout=.02, write_timeout=1, exclusive=True) as port:
            session = Session(port)
            session.identity()
            before = session.read(NAME)
            config = session.read('MAV_0_CONFIG')
            record.update({'identity': session.ev.identity, 'before': before, 'mav_0_config': config})
            if before != {'value': expected, 'type': 6} or config['value'] != 101:
                raise RuntimeError('Unexpected live baseline; no write')
            session.send(write_message(session.enc, target))
            record['write_sent'] = True
            after = session.read(NAME)
            record['after'] = after
            if after != {'value': target, 'type': 6}:
                raise RuntimeError('Write readback not verified; no reboot')
            session.readback_verified = record['readback_verified'] = True
            session.send(session.enc.command_long_encode(1, 1, m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                                        0, 1, 0, 0, 0, 0, 0, 0))
            record['reboot_sent'] = True
            # 正常重启会断开USB；此处不重试、不解锁、不自动重连或扩展改参。
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                session.pump()
                if session.reboot_ack is not None:
                    break
    except (OSError, ValueError, RuntimeError, serial.SerialException) as exc:
        if record['reboot_sent'] and isinstance(exc, (OSError, serial.SerialException)):
            record['expected_usb_disconnect'] = str(exc)
        else:
            record['error'] = str(exc)
    if session:
        record.update({'tx_messages': dict(session.sent), 'heartbeats': session.ev.heartbeats,
                       'reboot_ack': session.reboot_ack})
    with args.output.open('x') as f:
        json.dump(record, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(record, ensure_ascii=False, indent=2))
    return 0 if record['reboot_sent'] and record['error'] is None else 1


if __name__ == '__main__':
    raise SystemExit(main())
