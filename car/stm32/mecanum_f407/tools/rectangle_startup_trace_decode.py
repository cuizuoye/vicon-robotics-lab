#!/usr/bin/env python3
"""离线解析STM32启动RAM记录；不打开串口、不烧录、不控制电机。"""
import argparse
import json
import math
from pathlib import Path
import struct
import subprocess

MAGIC = 0x52535431
CAPACITY = 600
HEADER = struct.Struct('<6I4B4f2I')
ROW = struct.Struct('<I4i12f4h8HI8B')
WHEELS = ('FL', 'FR', 'RL', 'RR')


def symbol(elf, name):
    output = subprocess.check_output(
        ['arm-none-eabi-nm', '-S', str(elf)], text=True)
    found = [line.split() for line in output.splitlines()
             if len(line.split()) == 4 and line.split()[-1] == name]
    if len(found) != 1:
        raise ValueError(f'ELF symbol missing or ambiguous: {name}')
    address, size = int(found[0][0], 16), int(found[0][1], 16)
    if not 0x20000000 <= address < address + size <= 0x20020000:
        raise ValueError('symbol not in this board SRAM')
    return {'address': f'0x{address:08x}', 'bytes': size}


def delta32(after, before):
    value = (after - before) & 0xffffffff
    return value if value < 0x80000000 else value - 0x100000000


def decode(blob):
    if len(blob) != HEADER.size + CAPACITY * ROW.size:
        raise ValueError('wrong trace byte length')
    h = HEADER.unpack_from(blob)
    magic, version, row_bytes, capacity, count, post = h[:6]
    capturing, complete, overflow, reserved = h[6:10]
    targets, started, stopped = list(h[10:14]), h[14], h[15]
    if (magic, version, row_bytes, capacity) != (MAGIC, 1, ROW.size, CAPACITY):
        raise ValueError('trace ABI mismatch')
    if count > capacity or post > 20 or reserved or overflow:
        raise ValueError('invalid/overflowed trace metadata')
    if capturing not in (0, 1) or complete not in (0, 1) or (capturing and complete):
        raise ValueError('invalid capture state')
    if (complete and (not count or post != 20)) or (not count and (capturing or post)):
        raise ValueError('inconsistent completion/count')
    if count and not (capturing or complete):
        raise ValueError('recorded trace neither active nor complete')
    if any(not math.isfinite(v) or not 0 <= v <= 300 for v in targets):
        raise ValueError('invalid fixed-forward targets')
    rows = []
    for i in range(count):
        v = ROW.unpack_from(blob, HEADER.size + i * ROW.size)
        counts = list(v[1:5]); filtered = list(v[5:9])
        ramped = list(v[9:13]); integral = list(v[13:17])
        duty = list(v[17:21]); cc = list(v[21:29]); fault = v[29]
        state, enabled, key, assist, wheel, legs, before, pad = v[30:38]
        if (state > 9 or before > 9 or enabled not in (0, 1) or key not in (0, 1)
                or assist > 15 or wheel > 4 or legs > 1 or pad):
            raise ValueError(f'invalid state fields in row {i}')
        if enabled and (state != 5 or fault or legs):
            raise ValueError(f'enabled outside first leg in row {i}')
        for values, limit in ((filtered, 10000), (ramped, 300), (integral, 100)):
            if any(not math.isfinite(x) or abs(x) > limit for x in values):
                raise ValueError(f'nonfinite/out-of-bounds PI data in row {i}')
        if any(abs(x) > 100 or x % 10 for x in duty) or any(x > 100 for x in cc):
            raise ValueError(f'PWM out of bounds in row {i}')
        if not enabled and (any(duty) or any(cc) or assist):
            raise ValueError(f'nonzero disabled outputs in row {i}')
        for w, d in enumerate(duty):
            expected = ([100, 100 - d // 10] if d > 0 else
                        [100 + d // 10, 100] if d < 0 else [0, 0])
            if cc[2*w:2*w+2] != expected:
                raise ValueError(f'duty/actual CCR mismatch in row {i}, wheel {w}')
        row = {'index': i, 'ms': v[0], 'counts': counts,
               'filtered_cps': filtered, 'ramped_cps': ramped,
               'integral_permille': integral, 'duty_permille': duty,
               'actual_compares': cc, 'fault': fault, 'state': state,
               'enabled': bool(enabled), 'key': bool(key), 'assist_mask': assist,
               'fault_wheel': wheel, 'legs': legs, 'before_state': before}
        if rows:
            dt = (row['ms'] - rows[-1]['ms']) & 0xffffffff
            if not 0 < dt <= 30:
                raise ValueError(f'invalid sampling interval in row {i}')
            row['raw_cps_from_count_delta'] = [
                delta32(a, b) * 1000 / dt
                for a, b in zip(counts, rows[-1]['counts'])]
        rows.append(row)
    if complete and any(row['enabled'] for row in rows[-20:]):
        raise ValueError('completed trace does not end with 20 stopped rows')
    enabled_rows = [row for row in rows if row['enabled']]
    stopped_rows = [row for row in rows if not row['enabled']]
    if rows and started != rows[0]['ms']:
        raise ValueError('start timestamp does not match first row')
    if post != len(stopped_rows) or (stopped_rows and stopped != stopped_rows[0]['ms']):
        raise ValueError('stop timestamp/count does not match rows')
    if stopped_rows and any(r['enabled'] for r in rows[stopped_rows[0]['index']:]):
        raise ValueError('unexpected re-enable after stopping')
    wheel_summary = {}
    for i, wheel in enumerate(WHEELS):
        wheel_summary[wheel] = {
            'raw_count_change': delta32(rows[-1]['counts'][i], rows[0]['counts'][i]) if rows else 0,
            'maximum_abs_duty_permille': max((abs(r['duty_permille'][i]) for r in rows), default=0),
            'last_enabled_duty_permille': enabled_rows[-1]['duty_permille'][i] if enabled_rows else None,
            'assist_samples': sum(bool(r['assist_mask'] & (1 << i)) for r in rows),
        }
    return {
        'format_valid': True, 'count': count, 'capturing': bool(capturing),
        'complete': bool(complete), 'target_cps': targets,
        'started_ms': started, 'stopped_ms': stopped,
        'runtime_to_stop_ms': ((stopped-started) & 0xffffffff) if stopped_rows else None,
        'last_fault': rows[-1]['fault'] if rows else None,
        'fault_wheel': rows[-1]['fault_wheel'] if rows else None,
        'enabled_samples': len(enabled_rows), 'post_stop_rows': post,
        'wheels': wheel_summary, 'rows': rows,
        'boundary': 'RAM/encoder/PWM evidence only; not Vicon feedback, motor voltage or physical motion proof.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--elf', required=True, type=Path)
    parser.add_argument('--summary-only', action='store_true')
    args = parser.parse_args()
    location = symbol(args.elf, 'startup_trace')
    if location['bytes'] != HEADER.size + CAPACITY * ROW.size:
        raise ValueError('ELF trace size does not match decoder')
    result = decode(args.trace.read_bytes()); result['elf_symbol'] = location
    if args.summary_only:
        result.pop('rows')
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
