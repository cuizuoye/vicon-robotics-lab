#!/usr/bin/env python3
"""Explicitly authorized, USB-only archival of two exact retained fault logs.

No file deletion, writes/truncation, parameter writes, arming, RC or targets.
Byte-exact backups are verified against FC CRC before a recoverable rename.
Reboot is a separate invocation and cannot run without verified archival evidence.
"""
import argparse
from collections import Counter
from contextlib import contextmanager
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import struct
import subprocess
import time
import zlib

import serial
from pymavlink.dialects.v20 import common as m
from estop_only_policy import LISTENER, parse_actuator_snapshot
from mavlink_readonly_probe import Evidence, readonly_message
from persistent_shell import reply_complete
from trs_uart_readonly_check import UID2, USB

ROOT = '/fs/microsd'
ARCHIVE = ROOT + '/svc_20261014'
NAMES = ('fault_2000_01_01_00_31_10.log', 'fault_2000_01_01_00_46_57.log')
PARAMS = ('COM_ARM_HFLT_CHK', 'EKF2_EV_CTRL', 'EKF2_EV_DELAY', 'EKF2_HGT_REF', 'EKF2_RNG_CTRL')
HEADER = struct.Struct('<HBBBBBBI')
FILES = tuple(f'{base}/{name}' for base in (ROOT, ARCHIVE) for name in NAMES)


def nuttx_crc(data):
    # Exact firmware uses crc32part(data, len, 0), without final complement.
    return zlib.crc32(data, 0xffffffff) ^ 0xffffffff


def payload(seq, opcode, data=b'', *, session=0, offset=0, size=None):
    size = len(data) if size is None else size
    if not 0 <= size <= 239 or len(data) > 239:
        raise ValueError('FTP payload too large')
    return (HEADER.pack(seq, session, opcode, size, 0, 0, 0, offset) + data).ljust(251, b'\0')


def ftp_allowed(opcode, data, *, session=0, offset=0, size=None, archive_authorized=False, backed_up=False):
    size = len(data) if size is None else size
    if session != 0 or offset < 0:
        return False
    if opcode in (4, 14):
        return offset == 0 and data in tuple(p.encode() for p in FILES) and size == len(data)
    if opcode == 3:
        return data in (ROOT.encode(), ARCHIVE.encode()) and size == len(data)
    if opcode == 5:
        return not data and 1 <= size <= 239 and offset < 65536
    if opcode == 1:
        return not data and size == 0 and offset == 0
    if opcode == 9:
        return archive_authorized and backed_up and offset == 0 and data == ARCHIVE.encode() and size == len(data)
    if opcode == 13:
        pairs = tuple(f'{ROOT}/{n}'.encode() + b'\0' + f'{ARCHIVE}/{n}'.encode() for n in NAMES)
        return archive_authorized and backed_up and offset == 0 and data in pairs and size == len(data)
    return False


def normalized_fault(text):
    text = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', text).replace('\r', '')
    start = text.index('[hardfault_log]')
    # cat transcript includes a terminal prompt; a byte file does not.
    end = text.rfind('nsh>')
    return text[start:end if end > start else len(text)].rstrip()


class Nak(RuntimeError):
    def __init__(self, data):
        self.code = data[0] if data else None
        super().__init__(f'FTP NAK {list(data)}')


class Connection:
    def __init__(self, port, *, archive_authorized=False):
        self.port = port
        self.enc = m.MAVLink(None, srcSystem=250, srcComponent=198)
        self.dec = m.MAVLink(None)
        self.dec.robust_parsing = True
        self.ev = Evidence(250, 198, expected_parameters=PARAMS)
        self.start = time.monotonic()
        self.next_hb = 0
        self.seq = secrets.randbelow(20000) + 10000
        self.tx = Counter()
        self.log = []
        self.archive_authorized = archive_authorized
        self.backed_up = self.archived_verified = False

    def safe(self):
        return bool(self.ev.identity and self.ev.identity['uid2'] == UID2
                    and self.ev.identity['firmware'] == '1.14.3' and not self.ev.fatal
                    and self.ev.heartbeats and not self.ev.heartbeats[-1]['armed']
                    and self.now() - self.ev.heartbeats[-1]['elapsed_s'] < 2)

    def now(self):
        return time.monotonic() - self.start

    def send(self, msg):
        allowed = readonly_message(msg)
        if msg.get_type() == 'PARAM_REQUEST_READ':
            name = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
            allowed = self.safe() and name.rstrip('\0') in PARAMS and (
                msg.target_system, msg.target_component, msg.param_index) == (1, 1, -1)
        elif msg.get_type() == 'SERIAL_CONTROL':
            allowed = self.safe() and (msg.device, msg.flags, msg.timeout, msg.baudrate) == (10, 2, 0, 0) and (
                (msg.count == 0 and bytes(msg.data) == bytes(70)) or
                (msg.count == len(LISTENER) and bytes(msg.data) == LISTENER.ljust(70, b'\0')))
        elif msg.get_type() == 'FILE_TRANSFER_PROTOCOL':
            raw = bytes(msg.payload)
            _, session, opcode, size, req, burst, padding, offset = HEADER.unpack_from(raw)
            allowed = self.safe() and (msg.target_network, msg.target_system, msg.target_component) == (0, 1, 1) and (
                req, burst, padding) == (0, 0, 0) and ftp_allowed(
                opcode, raw[12:12 + size] if opcode != 5 else b'', session=session, offset=offset, size=size,
                archive_authorized=self.archive_authorized, backed_up=self.backed_up)
        elif msg.get_type() == 'COMMAND_LONG' and msg.command == m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            allowed = self.safe() and self.archived_verified and msg.confirmation == 0 and (
                msg.target_system, msg.target_component) == (1, 1) and msg.param1 == 1 and all(
                getattr(msg, f'param{i}') == 0 for i in range(2, 8))
        if not allowed:
            raise RuntimeError('Maintenance send gate rejected ' + msg.get_type())
        raw = msg.pack(self.enc)
        if self.port.write(raw) != len(raw):
            raise OSError('Short serial write')
        self.enc.seq = (self.enc.seq + 1) % 256
        self.tx[msg.get_type()] += 1

    def pump(self):
        now = self.now()
        if now > 110:
            raise RuntimeError('Bounded service session expired')
        if now >= self.next_hb:
            self.send(self.enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            self.next_hb = now + 1
        msgs = self.dec.parse_buffer(self.port.read(min(self.port.in_waiting, 65536))) or []
        for msg in msgs:
            self.ev.ingest(msg, now)
        if self.ev.fatal:
            raise RuntimeError(self.ev.fatal)
        if self.ev.identity and (self.ev.identity['uid2'] != UID2 or self.ev.identity['firmware'] != '1.14.3'):
            raise RuntimeError('Wrong vehicle identity')
        time.sleep(.002)
        return msgs

    def identify(self):
        next_info = 0
        while not self.safe():
            self.pump()
            if self.now() > 10:
                raise RuntimeError('No fresh disarmed expected-aircraft identity')
            if self.ev.heartbeats and self.now() >= next_info:
                self.ev.version_requested_at = self.now()
                self.send(self.enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                next_info = self.now() + 2

    def params(self):
        self.ev.parameters.clear()
        for name in PARAMS:
            self.ev.requests[name] = self.now()
            self.send(self.enc.param_request_read_encode(1, 1, name.encode(), -1))
        until = self.now() + 4
        while len(self.ev.parameters) != len(PARAMS):
            self.pump()
            if self.now() > until or not self.safe():
                raise RuntimeError('Incomplete fresh protection parameters')
        result = dict(self.ev.parameters)
        if result['COM_ARM_HFLT_CHK'] != 1 or result['EKF2_EV_CTRL'] != 0 or result['EKF2_EV_DELAY'] != 0:
            raise RuntimeError('Expected retained protection/fusion-off state absent')
        return result

    def actuator(self):
        buffer = ''
        self.send(self.enc.serial_control_encode(10, 2, 0, 0, len(LISTENER), LISTENER.ljust(70, b'\0')))
        until, poll = self.now() + 5, self.now() + .25
        while self.now() < until:
            for msg in self.pump():
                if (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1) and msg.get_type() == 'SERIAL_CONTROL':
                    buffer += bytes(msg.data[:msg.count]).decode('utf8', 'replace')
            state = parse_actuator_snapshot(buffer)
            if state and reply_complete(buffer, LISTENER):
                if state['armed'] or not state['lockdown']:
                    raise RuntimeError('Maintenance requires disarmed output-locked state')
                return state
            if len(buffer) > 16384 or not self.safe():
                raise RuntimeError('Unsafe/incomplete actuator read')
            if self.now() >= poll:
                self.send(self.enc.serial_control_encode(10, 2, 0, 0, 0, bytes(70)))
                poll = self.now() + .25
        raise RuntimeError('Actuator read expired')

    def ftp(self, opcode, data=b'', *, offset=0, size=None):
        seq = self.seq
        self.seq += 2
        msg = self.enc.file_transfer_protocol_encode(0, 1, 1, payload(seq, opcode, data, offset=offset, size=size))
        for attempt in range(3):
            self.send(msg)
            until = self.now() + 1.5
            while self.now() < until:
                for reply in self.pump():
                    if reply.get_type() != 'FILE_TRANSFER_PROTOCOL' or (reply.get_srcSystem(), reply.get_srcComponent()) != (1, 1):
                        continue
                    if (reply.target_system, reply.target_component) != (250, 198):
                        continue
                    raw = bytes(reply.payload)
                    rseq, sess, op, length, req, _, _, roff = HEADER.unpack_from(raw)
                    if rseq != seq + 1 or req != opcode:
                        continue
                    if op not in (128, 129) or length > 239 or sess != 0 or roff != offset:
                        raise RuntimeError('Malformed correlated FTP reply')
                    body = raw[12:12 + length]
                    self.log.append({'opcode': opcode, 'seq': seq, 'offset': offset, 'reply_opcode': op,
                                     'size': length, 'attempt': attempt})
                    if op == 129:
                        raise Nak(body)
                    return body
                if not self.safe():
                    raise RuntimeError('Fresh maintenance identity lost')
        raise RuntimeError(f'FTP timeout opcode={opcode} seq={seq}; stop without destructive retries')

    def listing(self, path):
        offset, rows = 0, []
        for _ in range(24):
            try:
                body = self.ftp(3, path.encode(), offset=offset)
            except Nak as exc:
                if exc.code == 6:
                    return rows
                raise
            chunks = body.split(b'\0')[:-1]
            if not chunks:
                return rows
            offset += len(chunks)
            for part in chunks:
                if not part or part[:1] == b'S':
                    continue
                if part[:1] not in (b'D', b'F'):
                    raise RuntimeError('Malformed directory entry')
                rows.append(part.decode('utf8'))
        raise RuntimeError('Directory pagination limit')

    def download(self, path):
        info = self.ftp(4, path.encode())
        if len(info) != 4:
            raise RuntimeError('Missing FTP size')
        size, = struct.unpack('<I', info)
        data = bytearray()
        try:
            if not 0 < size < 65536:
                raise RuntimeError('Unexpected fault log size')
            while len(data) < size:
                block = self.ftp(5, offset=len(data), size=min(239, size-len(data)))
                if not block or len(block) > size-len(data):
                    raise RuntimeError('Invalid download block')
                data.extend(block)
        finally:
            self.ftp(1)
        body = self.ftp(14, path.encode())
        if len(body) != 4 or struct.unpack('<I', body)[0] != nuttx_crc(data):
            raise RuntimeError('Byte backup CRC mismatch')
        return bytes(data)


def save(path, data):
    with path.open('xb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    if path.read_bytes() != data:
        raise RuntimeError('Local backup readback mismatch')


@contextmanager
def connect():
    if subprocess.run(['fuser', str(Path(USB).resolve())], capture_output=True).returncode == 0:
        raise RuntimeError('FC port occupied; no process killed')
    with serial.Serial(USB, 115200, timeout=0, write_timeout=.5, exclusive=True) as port:
        yield port


def run(args):
    out = args.output_dir
    result = {'scope': 'Authorized recoverable two-log archive/reboot only; no flight or parameter writes',
              'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'stage': args.stage, 'archive': ARCHIVE, 'files': [], 'error': None,
              'reboot_requested': False, 'reboot_ack_accepted': False, 'archive_verified': False,
              'parameter_writes': 0, 'files_deleted': 0, 'flight_ready': False}
    c = None
    try:
        with connect() as port:
            c = Connection(port, archive_authorized=args.stage == 'archive')
            c.identify()
            result['identity'] = c.ev.identity
            result['parameters_before'] = c.params()
            result['actuator_before'] = c.actuator()
            if args.stage == 'archive':
                prior = json.loads(args.review.read_text())
                if prior.get('error') or prior['identity']['uid2'] != UID2:
                    raise RuntimeError('Invalid previous health review')
                root = c.listing(ROOT)
                result['root_before'] = root
                faults = sorted(s[1:].split('\t')[0] for s in root if s.startswith('Ffault_') and '.log\t' in s)
                if faults != sorted(NAMES) or any(s == 'D' + ARCHIVE.split('/')[-1] for s in root):
                    raise RuntimeError('Root fault set changed or archive destination already exists')
                originals = {}
                for name, key in zip(NAMES, ('fault_first', 'fault_second')):
                    data = c.download(ROOT + '/' + name)
                    if normalized_fault(data.decode('utf8')) != normalized_fault(prior['command_records'][key]):
                        raise RuntimeError('Fault file differs from reviewed historical text')
                    save(out / name, data)
                    originals[name] = data
                    result['files'].append({'name': name, 'size': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
                                            'nuttx_crc32': f'{nuttx_crc(data):08x}', 'pc_backup_crc_verified': True,
                                            'archived': False, 'archive_readback_verified': False})
                # Durable evidence before the first SD mutation, retained on partial failure.
                save(out / 'backup_manifest.json', json.dumps(result, indent=2).encode())
                c.backed_up = True
                c.ftp(9, ARCHIVE.encode())
                if c.listing(ARCHIVE):
                    raise RuntimeError('New archive directory not empty')
                for record in result['files']:
                    name = record['name']
                    c.actuator()
                    c.ftp(13, (ROOT + '/' + name).encode() + b'\0' + (ARCHIVE + '/' + name).encode())
                    record['archived'] = True
                    data = c.download(ARCHIVE + '/' + name)
                    if data != originals[name]:
                        raise RuntimeError('Archived byte readback mismatch')
                    record['archive_readback_verified'] = True
                result['root_after'] = c.listing(ROOT)
                result['archive_listing'] = c.listing(ARCHIVE)
                if any(s.startswith('Ffault_') for s in result['root_after']):
                    raise RuntimeError('Root still contains fault logs')
                result['parameters_after'] = c.params()
                if result['parameters_before'] != result['parameters_after']:
                    raise RuntimeError('Monitored parameters changed unexpectedly')
                result['actuator_after'] = c.actuator()
                result['archive_verified'] = True
            else:
                receipt = json.loads(args.receipt.read_text())
                if (receipt.get('error') or not receipt['archive_verified'] or receipt['archive'] != ARCHIVE
                        or receipt['identity']['uid2'] != UID2 or len(receipt['files']) != 2):
                    raise RuntimeError('Missing verified archive receipt')
                result['root_before_reboot'] = c.listing(ROOT)
                if any(s.startswith('Ffault_') for s in result['root_before_reboot']):
                    raise RuntimeError('New root fault file; reboot service stopped')
                for rec in receipt['files']:
                    if rec['name'] not in NAMES or not rec['archive_readback_verified']:
                        raise RuntimeError('Invalid archived file record')
                    data = c.download(ARCHIVE + '/' + rec['name'])
                    if hashlib.sha256(data).hexdigest() != rec['sha256'] or data != (args.receipt.parent / rec['name']).read_bytes():
                        raise RuntimeError('Archived/local backup no longer matches')
                if result['parameters_before'] != receipt['parameters_after']:
                    raise RuntimeError('Protection parameters drift before reboot')
                c.archived_verified = True
                result['archive_verified'] = True
                c.send(c.enc.command_long_encode(1, 1, m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0))
                result['reboot_requested'] = True
                until = c.now() + .35
                while c.now() < until:
                    for msg in c.pump():
                        if msg.get_type() == 'COMMAND_ACK' and (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1) and (
                                msg.command == m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN and
                                (msg.target_system, msg.target_component) == (250, 198)):
                            if msg.result != m.MAV_RESULT_ACCEPTED:
                                raise RuntimeError('Reboot rejected')
                            result['reboot_ack_accepted'] = True
                result['reboot_requires_fresh_identity_and_boot_clock_verification'] = True
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or 'Interrupted'
    finally:
        if c:
            result['tx_counts'] = dict(c.tx)
            result['ftp_transactions'] = c.log
            result['duration_s'] = c.now()
        save(out / (args.stage + '.json'), json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False).encode())
    print(json.dumps({k: v for k, v in result.items() if k != 'ftp_transactions'}, ensure_ascii=False, indent=2), flush=True)
    return 0 if result['error'] is None else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('stage', choices=('archive', 'reboot'))
    ap.add_argument('--physical-confirmed', action='store_true', help='Battery off, props removed, USB only, QGC exited')
    ap.add_argument('--authorized', action='store_true', help='Explicit user approval of archive and USB-only reboot')
    ap.add_argument('--review', type=Path)
    ap.add_argument('--receipt', type=Path)
    ap.add_argument('--output-dir', type=Path, required=True)
    args = ap.parse_args()
    if not args.physical_confirmed or not args.authorized:
        ap.error('Explicit authorization and physical conditions required')
    if not args.output_dir.is_dir() or (args.output_dir / (args.stage + '.json')).exists():
        ap.error('Existing output directory and new receipt required')
    if args.stage == 'archive' and (not args.review or not args.review.is_file() or any((args.output_dir / n).exists() for n in NAMES)):
        ap.error('Archive requires historical review and new backup filenames')
    if args.stage == 'reboot' and (not args.receipt or not args.receipt.is_file()):
        ap.error('Reboot requires verified archive receipt')
    raise SystemExit(run(args))


if __name__ == '__main__':
    main()
