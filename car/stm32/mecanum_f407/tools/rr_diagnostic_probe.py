#!/usr/bin/env python3
"""仅RR正向164cps的单次悬空诊断；无动力默认只stop/status，禁止自动重试/复位。"""
import argparse
import json
import signal
import time

from drive_pi_probe import delta_count, parse_status_for_firmware
from drive_startup_probe import FIELDS, Probe as StartupProbe, compact_result
from speed_readonly_probe import WHEELS

FIRMWARE = 'rr_diag_start8_v1'
MODES = {'rr164': (5, 'run RR164 S10', (0, 0, 0), (0, 0, 0, 0))}


def targets(_mode):
    return dict(fl=0, fr=0, rl=0, rr=164)


def parse_status(line):
    row = parse_status_for_firmware(line, FIRMWARE, FIELDS, MODES, 4, targets)
    if row['start_mask'] not in (0, 8) or not 0 <= row['fault_wheel'] <= 4:
        raise ValueError('RR-only startup/fault wheel mismatch')
    if ((not row['en'] and row['start_mask']) or
            (not row['fault'] and row['fault_wheel']) or
            (row['start_mask'] and row['d_rr'] != -80)):
        raise ValueError('invalid bounded RR startup')
    return row


TRACE_FIELDS = {'i', 'ms', 'enc', 'cps', 'fcps', 'target', 'im', 'd', 'p1', 'p2',
                'en', 'assist', 'other', 'fault'}


def parse_trace(line, duty_quantum=10, pwm_period_counts=100, target_limit=164):
    parts = line.split(',')
    if parts[0] != 'RRTRACE':
        raise ValueError('not RR trace')
    values = {}
    for part in parts[1:]:
        key, value = part.split('=', 1)
        if key in values:
            raise ValueError('duplicate RR trace field')
        values[key] = int(value, 16 if key == 'fault' else 10)
    if set(values) != TRACE_FIELDS:
        raise ValueError('exact RR trace schema required')
    if (not 0 <= values['i'] <= 1000 or not 0 <= values['ms'] <= 0xffffffff or
            not 0 <= values['fault'] <= 0xffffffff or
            any(not -(1 << 31) <= values[k] < (1 << 31)
                for k in ('enc', 'cps', 'fcps', 'target', 'im'))):
        raise ValueError('RR trace counter out of range')
    d = values['d']
    active_ticks = pwm_period_counts * abs(d) // 1000
    cc = (pwm_period_counts-active_ticks, pwm_period_counts) if d < 0 else (0, 0)
    if (values['other'] or values['en'] not in (0, 1) or values['assist'] not in (0, 8) or
            not -100 <= d <= 0 or d % duty_quantum or (values['p1'], values['p2']) != cc or
            not 0 <= values['target'] <= target_limit or abs(values['im']) > 1000000 or
            (values['en'] and values['fault']) or
            (values['en'] and (not -1000 <= values['cps'] <= 0 or
                               not 0 <= values['fcps'] <= 1000)) or
            (not values['en'] and any(values[k] for k in ('d', 'target', 'fcps', 'im', 'assist'))) or
            (values['assist'] and (not values['en'] or d != -80 or values['im']))):
        raise ValueError('unsafe RR trace output/state')
    return values


def parse_meta(line, prefix):
    parts = line.split(',')
    if len(parts) != 3 or parts[0] != prefix:
        raise ValueError('bad RR trace boundary')
    if not parts[1].startswith('n=') or parts[2] not in ('overflow=0', 'overflow=1'):
        raise ValueError('bad RR trace metadata')
    n = int(parts[1][2:])
    if not 1 <= n <= 1001:
        raise ValueError('bad RR trace length')
    return n, int(parts[2][-1])


def analyze_trace(rows, require_full=False):
    if not rows or rows[-1]['en'] or any(not r['en'] for r in rows[:-1]):
        raise ValueError('recording must end with exactly one disabled frame')
    for i, row in enumerate(rows):
        if row['i'] != i:
            raise ValueError('missing/duplicate/reordered RR trace sample')
        if i:
            dt = (row['ms'] - rows[i-1]['ms']) & 0xffffffff
            if not 1 <= dt <= 30:
                raise ValueError('RR trace sampling gap/reset')
    elapsed = (rows[-1]['ms'] - rows[0]['ms']) & 0xffffffff
    if require_full and not 9000 <= elapsed <= 10000:
        raise ValueError('RR full recording unexpectedly short')
    if require_full and rows[-1]['fault']:
        raise ValueError('RR full recording ended with latched fault')
    if require_full and (not elapsed or not 95 <= (len(rows)-1)*1000/elapsed <= 105):
        raise ValueError('RR trace rate not near 100Hz')
    # 100ms计数差用于观察速度变化，避免把10ms低速整数计数误当机械停顿。
    windows = []
    for i in range(100, len(rows)-1, 10):
        a, b = rows[i-10], rows[i]
        dt = (b['ms'] - a['ms']) & 0xffffffff
        if dt:
            windows.append(-delta_count(a['enc'], b['enc'])*1000/dt)
    active = rows[:-1]
    return dict(samples=len(rows), elapsed_ms=elapsed,
                recorded_hz=(len(rows)-1)*1000/elapsed if elapsed else None,
                max_output_permille=max((abs(r['d']) for r in active), default=0),
                filtered_cps_min=min((r['fcps'] for r in active), default=None),
                filtered_cps_max=max((r['fcps'] for r in active), default=None),
                duty_levels_permille=sorted({r['d'] for r in active}),
                count_window_100ms_cps_min=min(windows, default=None),
                count_window_100ms_cps_max=max(windows, default=None),
                root_cause='not determined; correlate recorded speed/output with operator observation',
                note='100ms count windows after first 1s; not waveform or ground/Vicon acceptance')


class Probe(StartupProbe):
    firmware = FIRMWARE
    modes = MODES
    parse_frame = staticmethod(parse_status)
    target_values = staticmethod(targets)
    parse_trace_frame = staticmethod(parse_trace)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.trace_phase = self.trace_requested = False

    def send(self, command):
        if command == 'trace':
            if not self.trace_phase or self.trace_requested:
                raise ValueError('trace only after completed one-shot, never retried')
            self.trace_requested = True
            self.commands.append(command)
            data = b'trace\n'
            if self.link.write(data) != len(data):
                raise ValueError('short trace request write')
            return
        if self.trace_phase and command not in ('stop', 'status'):
            raise ValueError('no motion/heartbeat during trace export')
        super().send(command)

    def collect_trace(self, require_full):
        # 即使动作失败，也可保留已停机的部分数据；不清故障、不复位、不重试。
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            self.event(0.03)
        end = self.query()
        if end['en'] or any(end[f'p_{w}{i}'] for w in WHEELS for i in (1, 2)):
            raise ValueError('cannot export while any output active')
        self.trace_phase = True
        self.send('trace')
        begin = self.wait(lambda x: isinstance(x, str) and x.startswith('RRBEGIN,'), timeout=1)
        n, overflow = parse_meta(begin, 'RRBEGIN')
        rows = []
        deadline, last_rx = time.monotonic() + 35, time.monotonic()
        while time.monotonic() < deadline:
            event = self.event(0.03)
            if isinstance(event, str) and event.startswith('RRTRACE,'):
                row = self.parse_trace_frame(event)
                if row['i'] != len(rows) or len(rows) >= n:
                    raise ValueError('trace sample order/count mismatch')
                rows.append(row)
                last_rx = time.monotonic()
            elif isinstance(event, str) and event.startswith('RREND,'):
                if parse_meta(event, 'RREND') != (n, overflow) or len(rows) != n:
                    raise ValueError('incomplete RR recording')
                if overflow:
                    raise ValueError('RR recording overflow; motion not accepted')
                return analyze_trace(rows, require_full)
            elif time.monotonic() - last_rx > 1:
                raise TimeoutError('RR recording stalled')
        raise TimeoutError('RR recording export deadline')

    def run(self):
        result = super().run()
        result['test'] = result['test'].replace('drive_pi_', 'rr_diagnostic_', 1)
        result['rr_recording'] = None
        if self.run_sent and result['end_state'] and not result['end_state']['en']:
            try:
                result['rr_recording'] = self.collect_trace(require_full=result['passed'])
            except (Exception, KeyboardInterrupt) as exc:
                result['errors'].append('RR export failed: ' + str(exc))
                result['passed'] = False
            finally:
                try:
                    self.send('stop'); self.wait(lambda x: x == 'OK,stopped_drive', timeout=0.5)
                    end = self.query()
                    result['end_state'] = end
                    if end['en'] or end['used'] != 1 or end['wdg_reset'] or end['fault']:
                        result['errors'].append('final RR output/fault/reset state invalid')
                        result['passed'] = False
                except (Exception, KeyboardInterrupt) as exc:
                    result['errors'].append('final RR stop/status failed: ' + str(exc))
                    result['passed'] = False
        result['scope'] = 'RR-only suspended low-speed diagnostic; no retuning, no other-wheel calibration'
        return result


def main(probe_class=Probe):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port')
    modes = probe_class.modes
    p.add_argument('--mode', choices=('no-power', 'post-run', 'powered-stationary', *modes), required=True)
    p.add_argument('--confirm-battery-disconnected', action='store_true')
    p.add_argument('--confirm-powered-suspended', action='store_true')
    args = p.parse_args()
    if not ((args.mode in ('no-power', 'post-run') and args.confirm_battery_disconnected and
             not args.confirm_powered_suspended) or
            (args.mode in (*modes, 'powered-stationary') and args.confirm_powered_suspended and
             not args.confirm_battery_disconnected)):
        p.error('exact mode-specific physical confirmation required; serial unopened')
    import serial
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    link = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.05, exclusive=True)
    link.dtr = link.rts = False
    link.port = args.port
    link.open()
    try:
        result = probe_class(link, motion=args.mode if args.mode in modes else None,
                       after_run=args.mode == 'post-run',
                       powered_stationary=args.mode == 'powered-stationary').run()
    finally:
        link.close()
    # 包含无损全部100Hz原始记录；停机/串口关闭后才压缩。
    print(json.dumps(compact_result(result, include_trace=True), ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
