#!/usr/bin/env python3
"""独立8%/200ms启动、约10秒悬空观察。旧镜像拒绝，不自动重试/复位。"""
import argparse
import base64
import gzip
import json
import signal

from drive_pi_probe import (FIELDS as OLD_FIELDS, MODES as OLD_MODES,
                            Probe as DriveProbe, parse_status_for_firmware)
from speed_readonly_probe import WHEELS

FIRMWARE = 'drive_pi_start8_v1'
FIELDS = OLD_FIELDS | {'start_mask', 'fault_wheel'}
MODES = {name: (m[0], m[1].replace(' S5', ' S10'), m[2], m[3])
         for name, m in OLD_MODES.items()}


def parse_status(line):
    row = parse_status_for_firmware(line, FIRMWARE, FIELDS)
    if not 0 <= row['start_mask'] <= 15 or not 0 <= row['fault_wheel'] <= 4:
        raise ValueError('invalid startup/fault wheel')
    if (not row['en'] and row['start_mask']) or (not row['fault'] and row['fault_wheel']):
        raise ValueError('startup active while disabled or fault wheel without fault')
    for i, w in enumerate(WHEELS):
        if row['start_mask'] & (1 << i) and (
                row[f'want_{w}'] == 0 or abs(row[f'd_{w}']) != 80):
            raise ValueError('startup on zero wheel or wrong bounded duty')
    return row


def run_rates(before, after):
    elapsed = (after['ms'] - before['ms']) & 0xffffffff
    if not 8500 <= elapsed <= 10000:
        raise ValueError('incomplete ten-second observation interval')
    values = {k: ((after[k] - before[k]) & 0xffffffff) * 1000 / elapsed
              for k in ('ticks', 'updates')}
    if any(not 95 <= rate <= 105 for rate in values.values()):
        raise ValueError('sampling rate not near 100Hz')
    return dict(elapsed_ms=elapsed, timer_hz=values['ticks'], measurement_hz=values['updates'])


class Probe(DriveProbe):
    firmware = FIRMWARE
    modes = MODES
    host_run_seconds = 9.7
    firmware_run_limit_ms = 10000
    parse_frame = staticmethod(parse_status)
    run_rates = staticmethod(run_rates)

    def run(self):
        result = super().run()
        active = [r for r in self.rows if r['en']]
        end = result['end_state']
        # 失败仍保留部分启动信息，不把null trial/错误心跳文本当作零动作。
        result['observed_startup'] = {
            'active_samples': len(active),
            'maximum_sampled_duty_permille': max(
                (abs(r[f'd_{w}']) for r in active for w in WHEELS), default=0),
            'assist_seen_mask': sum(1 << i for i in range(4)
                                   if any(r['start_mask'] & (1 << i) for r in active)),
            'latched_fault': end['fault'] if end else None,
            'fault_wheel': end['fault_wheel'] if end else None,
            'not_full_motion_acceptance': not result['passed'],
        }
        result['startup_profile'] = {
            'floor_permille': 80, 'maximum_window_ms': 200,
            'early_finish_counts': 8, 'target_ramp_cps_per_s': 800,
            'maximum_output_permille': 100, 'initial_no_progress_ms': 800,
            'lost_progress_ms': 500, 'heartbeat_timeout_ms': 150,
            'host_stop_s': 9.7, 'firmware_hard_limit_ms': 10000,
            'not_torque_or_current_control': True,
        }
        return result


def compact_result(result, include_trace=False):
    """保留判据和终态，省略重复帧；只改变输出，不改变串口测试行为。"""
    compact = {k: v for k, v in result.items()
               if k not in ('telemetry', 'raw_serial_records')}
    compact['capture'] = {
        'summary_only': True,
        'telemetry_count': len(result['telemetry']),
        'raw_serial_record_count': len(result['raw_serial_records']),
        'full_telemetry_in_output': include_trace,
    }
    if include_trace:
        # 测试结束、串口关闭后压缩完整记录；不占用运行中的心跳时序。
        payload = json.dumps({k: result[k] for k in ('telemetry', 'raw_serial_records')},
                             ensure_ascii=False, separators=(',', ':')).encode('utf-8')
        compact['trace'] = {
            'encoding': 'gzip+base64',
            'content_type': 'application/json; charset=utf-8',
            'uncompressed_bytes': len(payload),
            'data': base64.b64encode(gzip.compress(payload, mtime=0)).decode('ascii'),
        }
    return compact


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port')
    p.add_argument('--mode', choices=('no-power', 'post-run', 'powered-stationary', *MODES), required=True)
    p.add_argument('--confirm-battery-disconnected', action='store_true')
    p.add_argument('--confirm-powered-suspended', action='store_true')
    p.add_argument('--summary-only', action='store_true',
                   help='omit repeated telemetry/raw frames to avoid tool output truncation')
    p.add_argument('--include-trace', action='store_true',
                   help='with --summary-only, include lossless gzip/base64 telemetry and raw records')
    args = p.parse_args()
    if args.include_trace and not args.summary_only:
        p.error('--include-trace requires --summary-only; serial unopened')
    if not ((args.mode in ('no-power', 'post-run') and args.confirm_battery_disconnected and
             not args.confirm_powered_suspended) or
            (args.mode in (*MODES, 'powered-stationary') and args.confirm_powered_suspended and
             not args.confirm_battery_disconnected)):
        p.error('exact mode-specific physical confirmation required; serial unopened')
    import serial
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    link = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.05, exclusive=True)
    link.dtr = link.rts = False
    link.port = args.port
    link.open()
    try:
        result = Probe(link, motion=args.mode if args.mode in MODES else None,
                       after_run=args.mode == 'post-run',
                       powered_stationary=args.mode == 'powered-stationary').run()
    finally:
        link.close()
    print(json.dumps(compact_result(result, include_trace=args.include_trace)
                     if args.summary_only else result,
                     ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
