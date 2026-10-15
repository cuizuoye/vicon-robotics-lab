#!/usr/bin/env python3
"""MC-Link四轮只读联调；没有使能或非零速度命令入口。"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import secrets
import socket
import struct
import sys
import time
from link_timing import SerialLogCapture, summarize_timing

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/mecanum_vicon_control'))
from mecanum_vicon_control.transport_protocol import (
    MessageType, FrameFlags, ProtocolError, encode_frame, decode_frame,
    encode_uart_packet, decode_uart_packet, pack_cmd_vel, unpack_wheel_telemetry,
)
from mecanum_vicon_control.readonly_link_status import (
    decode_readonly_status, STATUS, TAG,
)

decode_status = decode_readonly_status


def decode_telemetry(frame, allow_timeout=False):
    t = unpack_wheel_telemetry(frame.payload)
    allowed_fault = 1 if allow_timeout else 0
    if frame.flags & FrameFlags.ENABLED or t.state_flags & 2:
        raise ValueError('unexpected enabled output')
    if t.state_flags & 0x6d != 0x6d or t.fault_bits & ~allowed_fault:
        raise ValueError(f'wrong readonly state or fault: {t.state_flags:#x}/{t.fault_bits:#x}')
    if bool(frame.flags & FrameFlags.FAULT) != bool(t.fault_bits):
        raise ValueError('fault header/payload mismatch')
    return asdict(t)


class Transport:
    def __init__(self, args):
        self.serial = bool(args.serial)
        self.buffer = bytearray()
        self.invalid_packets = 0
        if self.serial:
            import serial
            self.device = serial.Serial(port=None, baudrate=921600, timeout=0, write_timeout=0.1, exclusive=True)
            self.device.dtr = self.device.rts = False
            self.device.port = args.serial
            self.device.open()
        else:
            self.device = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if args.interface:
                # 仅限制本进程socket，不修改路由、代理或网卡配置。
                self.device.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                                       args.interface.encode() + b'\0')
            self.device.bind((args.bind_ip, 46001))
            self.device.setblocking(False)
            self.peer = (args.esp_ip, 46000)

    def send(self, raw):
        if self.serial:
            if self.device.write(encode_uart_packet(raw)) != len(encode_uart_packet(raw)):
                raise RuntimeError('short serial write')
        elif self.device.sendto(raw, self.peer) != len(raw):
            raise RuntimeError('short UDP write')

    def receive(self):
        result = []
        if self.serial:
            self.buffer.extend(self.device.read(4096))
            while 0 in self.buffer:
                end = self.buffer.index(0)
                packet = bytes(self.buffer[:end+1]); del self.buffer[:end+1]
                if packet != b'\0':
                    try:
                        result.append(decode_frame(decode_uart_packet(packet)))
                    except ProtocolError:
                        # 打开端口可能落在旧遥测的半帧中；允许分隔符重同步，记录错误。
                        self.invalid_packets += 1
                        if self.invalid_packets > 3:
                            raise RuntimeError('repeated invalid UART packets')
            if len(self.buffer) > 1024:
                raise RuntimeError('unframed serial overflow')
        else:
            for _ in range(100):
                try:
                    raw, peer = self.device.recvfrom(1024)
                except BlockingIOError:
                    break
                if peer[0] == self.peer[0]:
                    result.append(decode_frame(raw))
        return result

    def close(self):
        self.device.close()


def run(args, *, capture=None, started=None):
    transport = None
    record = dict(mode='serial' if args.serial else 'udp', baud=921600,
                  motion_enabled=False, battery_disconnected_confirmed=True, phases=[], sent=[])
    started = time.monotonic() if started is None else started
    sid = secrets.randbits(32) or 1
    seq = 0

    def send(kind, payload=b'', flags=0):
        nonlocal seq
        seq = (seq + 1) & 0xffffffff
        before = time.monotonic()-started
        transport.send(encode_frame(kind, seq, sid,
                        int(before*1000) & 0xffffffff, payload, flags))
        record['sent'].append(dict(kind=int(kind), sequence=seq, session_id=sid,
                                  start_s=before, end_s=time.monotonic()-started))
        return seq

    def collect(name, duration, command_stream, allow_timeout=False):
        until = time.monotonic()+duration
        next_send = time.monotonic()
        phase = dict(name=name, telemetry=[], diagnostics=[], hello_ack=[])
        record['phases'].append(phase)
        while time.monotonic() < until:
            now = time.monotonic()
            if command_stream and now >= next_send:
                send(MessageType.CMD_VEL, pack_cmd_vel(0, 0, 0, timeout_ms=150))
                next_send = now+0.02
            for frame in transport.receive():
                if frame.session_id != sid:
                    continue
                if frame.message_type == MessageType.HELLO_ACK:
                    if frame.flags != FrameFlags.RESPONSE or frame.payload:
                        raise ValueError('invalid HELLO_ACK')
                    phase['hello_ack'].append(frame.sequence)
                elif frame.message_type == MessageType.WHEEL_TELEMETRY:
                    # Retain the first rejected frame, not just its error text.
                    observed = asdict(unpack_wheel_telemetry(frame.payload))
                    if observed['fault_bits'] or frame.flags & FrameFlags.ENABLED:
                        record['timeout_telemetry' if allow_timeout else 'failure_telemetry'] = dict(observed,
                            host_time_s=time.monotonic()-started,
                            session_id=frame.session_id, header_flags=int(frame.flags))
                    t = decode_telemetry(frame, allow_timeout)
                    t['host_time_s'] = time.monotonic()-started
                    phase['telemetry'].append(t)
                elif frame.message_type == MessageType.LINK_STATUS:
                    s = decode_status(frame.payload)
                    s['sample_time_ms'] = frame.sender_time_ms
                    s['host_time_s'] = time.monotonic()-started
                    phase['diagnostics'].append(s)
            time.sleep(0.001)
        return phase

    try:
        transport = Transport(args)
        if capture is None and getattr(args, 'esp_log_serial', None):
            capture = SerialLogCapture(args.esp_log_serial, started)
        hello = send(MessageType.HELLO, flags=FrameFlags.ACK_REQUIRED)
        phase = collect('healthy', args.seconds, True)
        if hello not in phase['hello_ack'] or len(phase['telemetry']) < 10 or len(phase['diagnostics']) < 10:
            raise RuntimeError('no STM32 HELLO_ACK or insufficient four-wheel telemetry')
        seqs = {t['last_command_sequence'] for t in phase['telemetry']}
        if len(seqs - {0}) < 5:
            raise RuntimeError('zero-command roundtrip sequence does not progress')
        first, last = phase['diagnostics'][0], phase['diagnostics'][-1]
        elapsed = (phase['telemetry'][-1]['sample_time_ms']-phase['telemetry'][0]['sample_time_ms'])/1000
        diagnostic_elapsed = ((last['sample_time_ms']-first['sample_time_ms']) & 0xffffffff)/1000
        if diagnostic_elapsed <= 0:
            raise RuntimeError('diagnostics clock did not progress')
        phase['sample_hz'] = (last['ticks']-first['ticks'])/diagnostic_elapsed
        if not 90 <= phase['sample_hz'] <= 110 or last['ticks']-first['ticks'] != last['updates']-first['updates']:
            raise RuntimeError('four-wheel sampling is not progressing at 100Hz')
        phase['telemetry_hz'] = (len(phase['telemetry'])-1)/elapsed
        if args.check_timeout:
            timed = collect('no_commands', 0.45, False, True)
            if not timed['telemetry'] or timed['telemetry'][-1]['fault_bits'] != 1 or not timed['telemetry'][-1]['state_flags'] & 16:
                raise RuntimeError('150ms readonly command lease did not expire')
            sid = (sid + 1) & 0xffffffff or 1
            seq = 0
            hello = send(MessageType.HELLO, flags=FrameFlags.ACK_REQUIRED)
            recovered = collect('new_session', 1.0, True)
            if hello not in recovered['hello_ack'] or not recovered['telemetry']:
                raise RuntimeError('new session failed to recover the readonly link')
        record['success'] = True
        print(json.dumps(dict(success=True, mode=record['mode'], phases=[
            dict(name=p['name'], frames=len(p['telemetry']), diagnostics=len(p['diagnostics']),
                 last=p['telemetry'][-1] if p['telemetry'] else None,
                 sample_hz=p.get('sample_hz'), telemetry_hz=p.get('telemetry_hz'))
            for p in record['phases']]), ensure_ascii=False))
    except Exception as error:
        record['success'] = False
        record['error'] = str(error)
        record['error_host_time_s'] = time.monotonic()-started
        raise
    finally:
        record['command_stream_stopped_host_time_s'] = time.monotonic()-started
        record['invalid_uart_packets'] = transport.invalid_packets if transport else 0
        if transport:
            transport.close()
        record['esp_serial'] = capture.finish() if capture else dict(lines=[], error=None)
        record['timing'] = summarize_timing(record['sent'], record['esp_serial']['lines'])
        record['timing']['fault_telemetry_rejected_during_probe'] = 'failure_telemetry' in record
        record['timing']['post_run_lease_expiry_is_not_healthy_phase_failure'] = bool(
            record.get('success') and 'failure_telemetry' not in record)
        if args.output:
            # 联调工具输出数据文件，不是修改源代码。
            Path(args.output).write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    m = p.add_mutually_exclusive_group(required=True)
    m.add_argument('--serial'); m.add_argument('--esp-ip')
    p.add_argument('--bind-ip', default='0.0.0.0'); p.add_argument('--interface', default='')
    p.add_argument('--seconds', type=float, default=5.0)
    p.add_argument('--check-timeout', action='store_true'); p.add_argument('--output')
    p.add_argument('--esp-log-serial', help='Optional COM log capture at 115200; reads only, DTR/RTS false')
    p.add_argument('--confirm-battery-disconnected', action='store_true', required=True)
    args = p.parse_args()
    if not 2 <= args.seconds <= 30:
        p.error('--seconds must be 2..30')
    if args.serial and args.esp_log_serial:
        p.error('--esp-log-serial requires UDP mode')
    run(args)


if __name__ == '__main__':
    main()
