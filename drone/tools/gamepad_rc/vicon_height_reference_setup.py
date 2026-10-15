#!/usr/bin/env python3
"""USB-only maintenance: select vision height reference, send NO vision data.

Only HGT_REF 2->3 and RNG_CTRL 1->0 are writable. No reboot/pose/control.
Propellers are treated as installed; battery must be disconnected.
"""
import argparse
from collections import Counter
import datetime
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import time

import serial
from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import Evidence, FC_USB, decode_parameter, readonly_message
from estop_only_policy import parse_actuator_snapshot
from persistent_shell import reply_complete
from usb_vision_policy import shell_message, shell_allowed

UID = '000000000000000000000000000000000000'
BASELINE = {'EKF2_HGT_REF': 2, 'EKF2_RNG_CTRL': 1, 'EKF2_EV_CTRL': 0,
            'EKF2_BARO_CTRL': 1, 'EKF2_OF_CTRL': 0, 'EKF2_AID_MASK': 0,
            'COM_RC_IN_MODE': 4, 'COM_OBL_RC_ACT': 4}
TARGET = {'EKF2_HGT_REF': 3, 'EKF2_RNG_CTRL': 0}
# Exact PX4 1.14.3 EKF2::parameters_update() sets VPOS when HGT_REF=EV.
# Do NOT repeatedly write EV_CTRL=0 against that automatic dependency.
ENDPOINT = {**BASELINE, **TARGET, 'EKF2_EV_CTRL': 2}


def write_message(enc, name):
    value = TARGET[name]
    wire = struct.unpack('<f', struct.pack('<i', value))[0]
    return enc.param_set_encode(1, 1, name.encode(), wire, m.MAV_PARAM_TYPE_INT32)


def allowed(msg, *, identity_safe=False, locked=False, backup_verified=False, write_authorized=False):
    kind = msg.get_type()
    if kind in ('HEARTBEAT', 'COMMAND_LONG'):
        return readonly_message(msg)
    if kind == 'PARAM_REQUEST_READ':
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        return bool(identity_safe and name in BASELINE and msg.param_index == -1
                    and (msg.target_system, msg.target_component) == (1, 1))
    if kind == 'SERIAL_CONTROL':
        return bool(identity_safe and shell_allowed(msg)
                    and (msg.count == 0 or bytes(msg.data[:msg.count]) == b'listener actuator_armed -n 1\n'))
    if kind == 'PARAM_SET':
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        value = struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
        return bool(write_authorized and identity_safe and locked and backup_verified and name in TARGET
                    and value == TARGET[name] and msg.param_type == m.MAV_PARAM_TYPE_INT32
                    and (msg.target_system, msg.target_component) == (1, 1))
    return False


def verify_backup(directory):
    capture = json.loads((directory / 'parameters_verified.json').read_text())
    manifest = json.loads((directory / 'parameters_manifest.json').read_text())
    if (not capture.get('complete') or capture.get('expected_count') != 926
            or capture.get('received_count') != 926 or capture.get('missing_indices')
            or capture.get('firmware', {}).get('version') != '1.14.3'
            or not 0 <= time.time() - (directory / 'parameters_verified.json').stat().st_mtime < 3600):
        raise ValueError('Fresh complete PX4 1.14.3 backup required')
    for name in ('parameters_verified.json', 'parameters_verified.txt'):
        if hashlib.sha256((directory / name).read_bytes()).hexdigest() != manifest['sha256'][name]:
            raise ValueError('Backup hash mismatch')
    records = {r['name']: r for r in capture['records']}
    if (len(records) != 926 or any(records.get(k, {}).get('value') != v
                                 or records[k]['type'] != 6 for k, v in BASELINE.items())):
        raise ValueError('Unexpected baseline; no write permitted')
    return manifest['sha256']


class Session:
    def __init__(self, port, writes_enabled=False):
        self.port = port
        self.writes_enabled = writes_enabled
        self.enc = m.MAVLink(None, srcSystem=250, srcComponent=190)
        self.dec = m.MAVLink(None)
        self.dec.robust_parsing = True
        self.ev = Evidence(expected_parameters=BASELINE)
        self.start = time.monotonic()
        self.next_hb = 0
        self.parameters, self.sent = {}, Counter()
        self.buffer = ''
        self.locked_at = float('-inf')
        self.lock_snapshots = []

    def safe(self):
        now = time.monotonic() - self.start
        return bool(not self.ev.fatal and self.ev.identity and self.ev.identity['uid2'] == UID
                    and self.ev.identity['firmware'] == '1.14.3' and self.ev.heartbeats
                    and not self.ev.heartbeats[-1]['armed']
                    and now - self.ev.heartbeats[-1]['elapsed_s'] < 2)

    def send(self, msg):
        if not allowed(msg, identity_safe=self.safe(), locked=time.monotonic()-self.locked_at < 1,
                       backup_verified=True, write_authorized=self.writes_enabled):
            raise RuntimeError('Fixed maintenance message policy rejected ' + msg.get_type())
        packet = msg.pack(self.enc)
        if self.port.write(packet) != len(packet):
            raise OSError('Short serial write')
        self.enc.seq = (self.enc.seq + 1) % 256
        self.sent[msg.get_type()] += 1

    def pump(self):
        now = time.monotonic() - self.start
        if now > 45:
            raise RuntimeError('Bounded maintenance session expired')
        for msg in self.dec.parse_buffer(self.port.read(min(self.port.in_waiting, 65536))) or []:
            self.ev.ingest(msg, now)
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                continue
            if msg.get_type() == 'PARAM_VALUE':
                self.parameters[msg.param_id] = (decode_parameter(msg), msg.param_type)
            elif msg.get_type() == 'SERIAL_CONTROL':
                self.buffer += bytes(msg.data[:msg.count]).decode('utf-8', 'replace')
                if len(self.buffer) > 8192:
                    raise RuntimeError('Shell reply overflow')
        if self.ev.fatal:
            raise RuntimeError(self.ev.fatal)
        if self.ev.identity and not self.safe():
            raise RuntimeError('Expected fresh disarmed identity lost')
        if now >= self.next_hb:
            self.send(self.enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            self.next_hb = now + 1
        time.sleep(.005)

    def identity(self):
        deadline, next_request = time.monotonic() + 8, 0
        while time.monotonic() < deadline:
            self.pump()
            if self.safe():
                return
            if self.ev.heartbeats and time.monotonic() >= next_request:
                self.ev.version_requested_at = time.monotonic()-self.start
                self.send(self.enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                next_request = time.monotonic()+1
        raise RuntimeError('Identity timeout')

    def read(self, name):
        self.parameters.pop(name, None)
        self.send(self.enc.param_request_read_encode(1, 1, name.encode(), -1))
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            self.pump()
            if name in self.parameters:
                return self.parameters[name]
        raise RuntimeError('Readback timeout: '+name)

    def check_lock(self):
        self.buffer = ''
        request = shell_message(self.enc, 'actuator_armed')
        self.send(request)
        deadline, next_poll = time.monotonic()+3, 0
        while time.monotonic() < deadline:
            self.pump()
            snap = parse_actuator_snapshot(self.buffer)
            if snap and reply_complete(self.buffer, bytes(request.data[:request.count])):
                age = re.search(r'timestamp:\s*\d+\s*\(([\d.]+) seconds ago\)', self.buffer)
                if (not age or not 0 <= float(age.group(1)) < 2
                        or snap['armed'] or not snap['lockdown'] or snap.get('force_failsafe')
                        or (self.lock_snapshots and snap['timestamp_us'] < self.lock_snapshots[-1]['snapshot']['timestamp_us'])):
                    raise RuntimeError('Expected output lock state lost')
                self.lock_snapshots.append({'snapshot': snap, 'reply': self.buffer})
                self.locked_at = time.monotonic()
                return
            if time.monotonic() >= next_poll:
                self.send(shell_message(self.enc))
                next_poll = time.monotonic()+.25
        raise RuntimeError('Output lock verification timeout')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--backup-dir', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--battery-off-usb-only-confirmed', action='store_true')
    ap.add_argument('--verify-only', action='store_true', help='Read configured endpoint; rejects ALL parameter writes')
    args = ap.parse_args()
    if not args.battery_off_usb_only_confirmed or args.output.exists() or not args.output.parent.is_dir():
        ap.error('Battery-off USB-only confirmation and new output required')
    hashes = verify_backup(args.backup_dir)
    if subprocess.run(['fuser', str(FC_USB.resolve())], capture_output=True).returncode == 0:
        ap.error('Known FC USB is occupied; no process killed')
    result = {'scope': __doc__, 'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'backup_hashes': hashes, 'writes_attempted': [], 'before': {}, 'after': {},
              'error': None, 'reboot_sent': False, 'pose_packets_sent': 0, 'flight_ready': False,
              'reference_effective_verified': False, 'reboot_required': True,
              'verify_only': args.verify_only,
              'expected_firmware_dependency': 'HGT_REF=3 automatically enables EV_CTRL VPOS bit (2)'}
    session = None
    try:
        with serial.Serial(str(FC_USB), 115200, timeout=.02, write_timeout=1, exclusive=True) as port:
            session = Session(port, writes_enabled=not args.verify_only)
            session.identity()
            session.check_lock()
            for name, value in (ENDPOINT if args.verify_only else BASELINE).items():
                result['before'][name] = session.read(name)
                if result['before'][name] != (value, 6):
                    raise RuntimeError('Live baseline drift: '+name)
            for name in (() if args.verify_only else TARGET):
                session.check_lock()
                result['writes_attempted'].append(name)
                session.send(write_message(session.enc, name))
                if session.read(name) != (TARGET[name], 6):
                    raise RuntimeError('Target readback mismatch: '+name)
            for name, value in ENDPOINT.items():
                result['after'][name] = session.read(name)
                if result['after'][name] != (value, 6):
                    raise RuntimeError('Endpoint drift: '+name)
            session.check_lock()
    except Exception as exc:
        result['error'] = str(exc)
    if session:
        result.update(identity=session.ev.identity, tx_messages=dict(session.sent),
                      lock_snapshots=session.lock_snapshots, heartbeats=session.ev.heartbeats)
    result['parameters_stored_verified'] = result['error'] is None and len(result['after']) == len(BASELINE)
    result['requires_manual_review'] = bool(result['error'] and result['writes_attempted'])
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('lock_snapshots', 'heartbeats')}, indent=2))
    return 0 if result['parameters_stored_verified'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
