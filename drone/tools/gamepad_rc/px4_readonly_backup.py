#!/usr/bin/env python3
"""只读备份PX4参数：按索引验证完整性，保留准确数值，不允许覆盖旧备份。

只发送GCS心跳、版本信息请求、PARAM_REQUEST_LIST/READ；没有写参、解锁、
遥控、设定值、校准或重启入口。参数输出是诊断产物，不会自动恢复到飞控。
"""

import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import struct
import time

from pymavlink import mavutil
import mavlink_link
import px4_params


def exact_text(value, ptype):
    return str(int(value)) if ptype in px4_params.UNSIGNED | px4_params.SIGNED else format(value, '.17g')


def record_param(msg):
    name = str(msg.param_id).rstrip('\x00')
    index, count = int(msg.param_index), int(msg.param_count)
    # _HASH_CHECK不是可恢复参数，索引65535不能算作参数表的一项。
    if name == '_HASH_CHECK' or not 0 <= index < count:
        return None
    ptype = int(msg.param_type)
    value = px4_params.decode_value(float(msg.param_value), ptype)
    if not math.isfinite(value):
        raise ValueError(f'Non-finite parameter: {name}')
    return {'name': name, 'index': index, 'param_count': count, 'type': ptype,
            'value': value, 'exact_text': exact_text(value, ptype),
            'wire_value_hex': struct.pack('<f', float(msg.param_value)).hex()}


def wait_px4_heartbeat(mav, timeout=8):
    """TRS的GCS心跳可能转发到USB；只接受飞控本人，不把首个心跳当身份。"""
    deadline = time.monotonic() + timeout
    keepalive = mavlink_link.KeepAlive(mav)
    while time.monotonic() < deadline:
        keepalive.tick()
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=.1)
        if msg is None or (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
            continue
        if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            raise RuntimeError('Unexpected autopilot at system=1/component=1')
        mav.target_system, mav.target_component = 1, 1
        return msg
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--output-dir', type=Path, required=True)
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args()
    if not args.output_dir.is_dir():
        ap.error('Output directory must already exist')
    outputs = ['parameters_verified.json', 'parameters_verified.txt', 'parameters_manifest.json']
    if any((args.output_dir / name).exists() for name in outputs):
        ap.error('Refusing to overwrite a previous capture')
    if not 5 <= args.timeout <= 50:
        ap.error('Timeout must be between 5 and 50 seconds')

    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    indexed, expected, version = {}, None, None
    mav = mavlink_link.open_link(args.port, source_system=250)
    try:
        hb = wait_px4_heartbeat(mav, timeout=8)
        if hb is None or hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            raise RuntimeError('No PX4 heartbeat; no parameter capture performed')
        if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError('Armed vehicle: refusing this maintenance capture')
        sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()
        mav.mav.command_long_send(sysid, compid, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                  0, mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                                  0, 0, 0, 0, 0, 0)
        mav.param_fetch_all()
        keepalive = mavlink_link.KeepAlive(mav)
        deadline, next_retry = time.monotonic() + args.timeout, time.monotonic() + 2
        while time.monotonic() < deadline:
            keepalive.tick()
            msg = mav.recv_match(blocking=True, timeout=0.1)
            if msg is not None and msg.get_srcSystem() == sysid and msg.get_srcComponent() == compid:
                kind = msg.get_type()
                if kind == 'HEARTBEAT' and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    raise RuntimeError('Vehicle became armed; ended capture without control commands')
                if kind == 'AUTOPILOT_VERSION':
                    v = int(msg.flight_sw_version)
                    version = {'version': f'{v >> 24}.{(v >> 16) & 255}.{(v >> 8) & 255}',
                               'release_type': v & 255, 'board_version': msg.board_version}
                elif kind == 'PARAM_VALUE':
                    rec = record_param(msg)
                    if rec is not None:
                        if expected is not None and expected != rec['param_count']:
                            raise RuntimeError('Parameter table size changed during capture')
                        expected = rec['param_count']
                        indexed[rec['index']] = rec
            if expected and len(indexed) == expected and version is not None:
                break
            now = time.monotonic()
            if expected and now >= next_retry:
                missing = [i for i in range(expected) if i not in indexed]
                for i in missing[:24]:
                    mav.mav.param_request_read_send(sysid, compid, b'', i)
                next_retry = now + 2
        records = [indexed[i] for i in sorted(indexed)]
        missing = [i for i in range(expected or 0) if i not in indexed]
        complete = bool(expected) and not missing and len({r['name'] for r in records}) == expected
    finally:
        mav.close()

    # 本机新建备份产物；用x模式保证不会覆盖既有记录，不向飞控写任何内容。
    capture = {'captured_at_utc': started, 'port': args.port, 'system_id': sysid,
               'component_id': compid, 'firmware': version, 'complete': complete,
               'expected_count': expected, 'received_count': len(records),
               'missing_indices': missing, 'records': records,
               'scope': 'read-only maintenance; no flight or parameter-write commands'}
    texts = {
        'parameters_verified.json': json.dumps(capture, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
        'parameters_verified.txt': '# PX4 exact-value read-only backup; NAME VALUE TYPE\n' + ''.join(
            f"{r['name']:20} {r['exact_text']:>24} {px4_params.TYPE_NAME.get(r['type'], r['type'])}\n"
            for r in sorted(records, key=lambda r: r['name'])),
    }
    hashes = {}
    for filename, content in texts.items():
        with (args.output_dir / filename).open('x', encoding='utf-8') as f:
            f.write(content)
        hashes[filename] = hashlib.sha256((args.output_dir / filename).read_bytes()).hexdigest()
    manifest = {k: capture[k] for k in ('captured_at_utc', 'port', 'firmware', 'complete',
                                        'expected_count', 'received_count', 'missing_indices', 'scope')}
    manifest['sha256'] = hashes
    with (args.output_dir / 'parameters_manifest.json').open('x', encoding='utf-8') as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0 if complete else 2


if __name__ == '__main__':
    raise SystemExit(main())
