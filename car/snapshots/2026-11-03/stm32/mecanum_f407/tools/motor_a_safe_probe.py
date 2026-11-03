#!/usr/bin/env python3
"""单电机文本固件无动力验证：只发送 stop/status，绝不使能或设定速度。"""

import argparse
import json
import time

import serial


class SerialLineFramer:
    """先同步到换行边界，之后严格保留完整行；不将残片当状态。"""

    def __init__(self):
        self.buffer = bytearray()
        self.synchronized = False
        self.startup_discarded = None
        self.startup_truncated = False
        self.overflow = False
        self.invalid_lines = 0

    def feed(self, data):
        lines = []
        for byte in data:
            if not self.synchronized:
                if byte == 10:
                    self.startup_discarded = {
                        'text': bytes(self.buffer).rstrip(b'\r').decode('ascii', errors='replace'),
                        'truncated': self.startup_truncated,
                    }
                    self.buffer.clear()
                    self.synchronized = True
                elif len(self.buffer) < 512:
                    self.buffer.append(byte)
                else:
                    self.startup_truncated = True
                continue
            if byte == 10:
                if not self.overflow:
                    lines.append(bytes(self.buffer).rstrip(b'\r'))
                self.buffer.clear()
                self.overflow = False
            elif not self.overflow:
                self.buffer.append(byte)
                if len(self.buffer) > 512:
                    # 丢弃整个超长行直到下一换行，不能把行尾重新当成有效数据。
                    self.buffer.clear()
                    self.overflow = True
                    self.invalid_lines += 1
        return lines


def parse_status(line):
    if not line.startswith(('TEL,', 'STATUS,')):
        return None
    fields = dict(item.split('=', 1) for item in line.split(',')[1:])
    result = {'kind': line.split(',', 1)[0], 'mode': fields['mode']}
    for name in ('ms', 'en', 'hiz', 'target_cps', 'duty', 'enc', 'cps'):
        result[name] = int(fields[name], 10)
    result['fault'] = int(fields['fault'], 16)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--queries', type=int, default=10)
    args = parser.parse_args()
    if not 3 <= args.duration <= 60 or not 1 <= args.queries <= 30:
        parser.error('duration must be 3..60 seconds and queries must be 1..30')

    port = serial.Serial(port=None, baudrate=115200, timeout=0.02, exclusive=True)
    # 不用 DTR/RTS 操作开发板复位或 BOOT 引脚。
    port.dtr = False
    port.rts = False
    port.port = args.port
    port.open()
    framer = SerialLineFramer()
    records = []
    ready_lines = []
    error_lines = []
    unexpected_lines = []
    invalid_lines = 0
    stop_replies = 0
    sent_queries = 0
    stop_sent = False
    next_query = float('inf')
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            data = port.read(min(max(port.in_waiting, 1), 4096))
            for frame in framer.feed(data):
                try:
                    line = frame.decode('ascii')
                except UnicodeDecodeError:
                    invalid_lines += 1
                    continue
                if not line:
                    continue
                if line.startswith('READY,'):
                    ready_lines.append(line)
                elif line.startswith('ERR,'):
                    error_lines.append(line)
                elif line == 'OK,stopped':
                    stop_replies += 1
                elif line.startswith(('TEL,', 'STATUS,')):
                    try:
                        record = parse_status(line)
                    except (ValueError, KeyError):
                        invalid_lines += 1
                        continue
                    records.append(record)
                    if (record['en'] != 0 or record['hiz'] != 1
                            or record['duty'] != 0 or record['target_cps'] != 0):
                        # 异常时只请求解除使能，随后退出；没有运动测试分支。
                        port.write(b'stop\n')
                        port.flush()
                        error_lines.append('HOST,unexpected_enabled_or_output_state')
                        raise RuntimeError('unsafe reported state; stop requested')
                elif not line.startswith('INFO,'):
                    unexpected_lines.append(line)

            now = time.monotonic()
            # 等待 READY 或有效遥测，避免对未启动的固件发送命令。
            if not stop_sent and (ready_lines or records):
                port.write(b'stop\n')
                port.flush()
                stop_sent = True
                next_query = now + 0.3
            if stop_sent and sent_queries < args.queries and now >= next_query:
                port.write(b'status\n')
                port.flush()
                sent_queries += 1
                next_query = now + 0.75
    finally:
        try:
            if stop_sent:
                port.write(b'stop\n')
                port.flush()
        finally:
            port.close()

    telemetry = [row for row in records if row['kind'] == 'TEL']
    statuses = [row for row in records if row['kind'] == 'STATUS']
    sample_gaps = [
        (b['ms'] - a['ms']) & 0xFFFFFFFF
        for a, b in zip(telemetry, telemetry[1:])
    ]
    samples_span = ((telemetry[-1]['ms'] - telemetry[0]['ms']) & 0xFFFFFFFF
                    if len(telemetry) >= 2 else 0)
    all_safe = bool(records) and all(
        row['en'] == 0 and row['hiz'] == 1 and row['duty'] == 0
        and row['target_cps'] == 0 and row['fault'] == 0 for row in records)
    result = {
        'port': args.port, 'baudrate': 115200, 'sent': 'stop/status only',
        'duration_s': args.duration, 'ready_lines': ready_lines,
        'sent_status_queries': sent_queries, 'received_status_replies': len(statuses),
        'received_stop_replies': stop_replies, 'telemetry_count': len(telemetry),
        'telemetry_rate_hz': ((len(telemetry) - 1) * 1000.0 / samples_span
                              if samples_span else None),
        'max_telemetry_sample_gap_ms': max(sample_gaps, default=None),
        'startup_discarded': framer.startup_discarded,
        'invalid_lines': invalid_lines + framer.invalid_lines, 'error_lines': error_lines,
        'unexpected_lines': unexpected_lines[:10],
        'all_reported_states_safe_and_fault_free': all_safe,
        'first_telemetry': telemetry[0] if telemetry else None,
        'last_telemetry': telemetry[-1] if telemetry else None,
    }
    result['passed'] = (all_safe and len(telemetry) >= 10
                        and sent_queries == args.queries
                        and len(statuses) == sent_queries and stop_replies >= 1
                        and framer.synchronized
                        and not result['invalid_lines'] and not error_lines
                        and not unexpected_lines
                        and max(sample_gaps, default=0) <= 250)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
