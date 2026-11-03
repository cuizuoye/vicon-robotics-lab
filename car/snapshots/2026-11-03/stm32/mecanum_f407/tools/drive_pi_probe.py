#!/usr/bin/env python3
"""drive_pi_v1受限单次悬空试验。默认无动作；每次只测一种固定模式。"""
import argparse
import json
import math
import signal
import time

from forward_pi_probe import (PI_FIELDS, SIGNS, Probe as ForwardProbe,
                              healthy, rates, stationary)
from speed_readonly_probe import WHEELS

FIELDS = PI_FIELDS | {'mode', 'vx', 'vy', 'wz'}
CPR = (3179 / 3, 1049, 3167 / 3, 1061)
# body单位mm/s、mrad/s；wheel单位m/s，独立CPR不可复制左前数据。
MODES = {
    'forward50': (1, 'run F50 S5', (50, 0, 0), (0.05, 0.05, 0.05, 0.05)),
    'left50': (2, 'run L50 S5', (0, 50, 0), (-0.05, 0.05, 0.05, -0.05)),
    'yaw200': (3, 'run Y200 S5', (0, 0, 200), (-0.0292, 0.0292, -0.0292, 0.0292)),
    'diagonal25': (4, 'run D25 S5', (25, 25, 0), (0, 0.05, 0.05, 0)),
}
BY_ID = {mode[0]: mode for mode in MODES.values()}
HOST_RUN_SECONDS = 4.7


def targets(mode):
    return dict(zip(WHEELS, (int(speed * cpr / (math.pi * 0.06))
                            for speed, cpr in zip(mode[3], CPR))))


def parse_status_for_firmware(line, firmware='drive_pi_v1', fields=None,
                              modes=None, selected=5, target_function=targets,
                              duty_quantum=10, pwm_period_counts=100):
    fields = FIELDS if fields is None else fields
    by_id = BY_ID if modes is None else {m[0]: m for m in modes.values()}
    parts = line.split(',')
    if parts[0] not in ('TELSP', 'STATUSSP'):
        raise ValueError('not drive telemetry')
    raw = {}
    for part in parts[1:]:
        k, v = part.split('=', 1)
        if k in raw:
            raise ValueError('duplicate field')
        raw[k] = v
    if set(raw) != fields or raw['fw'] != firmware:
        raise ValueError('exact ' + firmware + ' schema required; no motion sent')
    row = {k: int(v, 16 if k == 'fault' else 10) for k, v in raw.items() if k != 'fw'}
    row.update(fw=raw['fw'], kind=parts[0])
    for k in ('ms', 'ticks', 'updates', 'fault'):
        if not 0 <= row[k] <= 0xffffffff:
            raise ValueError('unsigned counter out of range')
    if (row['ro'] or row['gpio_low'] or row['en'] not in (0, 1) or
            row['wdg_reset'] not in (0, 1) or row['used'] not in (0, 1) or
            not 0 <= row['stable_ms'] <= 500 or
            not 1 <= row['dt_min'] <= row['dt_max'] <= 30 or
            not 0 <= row['irq_us_max'] <= 1000):
        raise ValueError('invalid drive/sampling state')
    if row['en']:
        if row['mode'] not in by_id or row['sel'] != selected or not row['used'] or row['fault']:
            raise ValueError('invalid active group/mode/fault')
        mode = by_id[row['mode']]
        if tuple(row[k] for k in ('vx', 'vy', 'wz')) != mode[2]:
            raise ValueError('body command mismatch')
        expected_targets = target_function(mode)
    else:
        if any(row[k] for k in ('sel', 'mode', 'vx', 'vy', 'wz')):
            raise ValueError('disabled body command nonzero')
        expected_targets = {w: 0 for w in WHEELS}
    for w in WHEELS:
        for k in ('enc', 'cps', 'fcps', 'target', 'want', 'rpm', 'mms'):
            if not -(1 << 31) <= row[f'{k}_{w}'] < (1 << 31):
                raise ValueError('signed field out of range')
        duty, want, ramp = (row[f'{k}_{w}'] for k in ('d', 'want', 'target'))
        expected_want = expected_targets[w]
        if want != expected_want or not min(0, want) <= ramp <= max(0, want):
            raise ValueError('per-wheel target/ramp mismatch')
        direction = (want > 0) - (want < 0)
        if (not -100 <= duty <= 100 or duty % duty_quantum or
                (direction == 0 and duty != 0) or (direction and duty * SIGNS[w] * direction < 0)):
            raise ValueError('wrong duty direction/limit/quantization')
        active_ticks = pwm_period_counts * abs(duty) // 1000
        cc = ((pwm_period_counts, pwm_period_counts-active_ticks) if duty > 0 else
              (pwm_period_counts-active_ticks, pwm_period_counts) if duty < 0 else (0, 0))
        if (row[f'p_{w}1'], row[f'p_{w}2']) != cc:
            raise ValueError('actual CCR mapping mismatch')
        if not row['en'] and any(row[f'{k}_{w}'] for k in ('target', 'want', 'fcps', 'd')):
            raise ValueError('disabled control state nonzero')
    return row


def parse_status(line):
    return parse_status_for_firmware(line)


def delta_count(before, after):
    d = (after - before) & 0xffffffff
    return d - 0x100000000 if d >= 0x80000000 else d


def moving(before, row, motion, modes=None, target_function=targets):
    healthy(row)
    mode = (MODES if modes is None else modes)[motion]
    if row['en'] != 1 or row['mode'] != mode[0]:
        raise ValueError('run ended early or wrong motion mode')
    expected = target_function(mode)
    for w in WHEELS:
        direction = (expected[w] > 0) - (expected[w] < 0)
        delta = delta_count(before[f'enc_{w}'], row[f'enc_{w}']) * SIGNS[w]
        if direction == 0:
            if delta or any(row[f'{k}_{w}'] for k in ('cps', 'fcps', 'rpm', 'mms', 'd')):
                raise ValueError('zero-target wheel unexpectedly moved: ' + w)
        elif (delta * direction < 0 or not 0 <= row[f'cps_{w}'] * SIGNS[w] * direction <= 1000 or
              not 0 <= row[f'fcps_{w}'] * direction <= 1000 or
              row[f'rpm_{w}'] * direction < 0 or row[f'mms_{w}'] * direction < 0):
            raise ValueError('wrong direction/excessive speed: ' + w)


class Probe(ForwardProbe):
    # 复用经回归的有界串口读帧、静止观察和100Hz校验；不复用旧前进动作。
    firmware = 'drive_pi_v1'
    modes = MODES
    host_run_seconds = HOST_RUN_SECONDS
    firmware_run_limit_ms = 5000
    parse_frame = staticmethod(parse_status)
    run_rates = staticmethod(rates)
    target_values = staticmethod(targets)

    def validate_moving(self, before, row):
        moving(before, row, self.motion_name, self.modes, self.target_values)

    def __init__(self, link, motion=None, after_run=False, powered_stationary=False):
        if motion is not None and motion not in self.modes:
            raise ValueError('unknown fixed motion')
        super().__init__(link, motion=motion is not None, after_run=after_run,
                         powered_stationary=powered_stationary)
        self.motion_name = motion
        self.parser = self.parse_frame

    def send(self, command):
        run = self.modes[self.motion_name][1] if self.motion else None
        allowed = ('stop', 'status', 'hb', run) if self.motion else ('stop', 'status')
        if command not in allowed or (command == run and self.run_sent):
            raise ValueError('fixed command allowlist/one-shot violation')
        if command == run:
            self.run_sent = True  # 失败的写入也不能自动重试。
        self.commands.append(command)
        data = (command + '\n').encode('ascii')
        if self.link.write(data) != len(data):
            raise ValueError('short serial write')

    def trial(self, before):
        self.send('hb')
        self.wait(lambda x: x == 'OK,hb_drive', timeout=0.2)
        started = time.monotonic()
        self.send(self.modes[self.motion_name][1])
        deadline, next_hb, last_rx = started + self.host_run_seconds, started + 0.05, started
        ack, first, last, rows = False, None, None, []
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - next_hb > 0.08:
                raise TimeoutError('host heartbeat delayed')
            if now >= next_hb:
                self.send('hb'); next_hb = now + 0.05
            event = self.event(min(0.01, max(0, deadline-time.monotonic())))
            now = time.monotonic()
            if event == 'OK,running_drive':
                ack = True
            if isinstance(event, dict):
                if not event['en'] and not ack and now-started < 0.2:
                    stationary(before, event); continue
                self.validate_moving(before, event)
                if first is None:
                    first = event
                last, last_rx = event, now
                rows.append(event)
            if not ack and now-started > 0.3:
                raise TimeoutError('run acknowledgement missing')
            if now-last_rx > 0.3:
                raise TimeoutError('running telemetry gap exceeds 0.3s')
        self.send('stop')
        self.wait(lambda x: x == 'OK,stopped_drive', timeout=0.5)
        if not ack or len(rows) < 30 or last is None:
            raise ValueError('insufficient observed run')
        progress = {w: delta_count(before[f'enc_{w}'], last[f'enc_{w}']) * SIGNS[w] for w in WHEELS}
        for w, target in self.target_values(self.modes[self.motion_name]).items():
            if (target == 0 and progress[w] != 0) or (target != 0 and not
                    10 <= progress[w] * ((target > 0)-(target < 0)) <= 100000):
                raise ValueError('insufficient/unexpected progress: ' + w)
        return {'mode': self.motion_name, 'host_run_limit_s': self.host_run_seconds,
                'firmware_run_limit_ms': self.firmware_run_limit_ms, 'run_samples': len(rows),
                'signed_forward_progress': progress, 'cadence': self.run_rates(first, last),
                'last_running_state': last,
                'max_abs_duty_permille': max(abs(r[f'd_{w}']) for r in rows for w in WHEELS),
                'tracking_acceptance': 'suspended encoder observation only, not ground/Vicon performance'}

    def run(self):
        errors, before, end, sampling, result = [], None, None, None, None
        try:
            self.wait(lambda x: isinstance(x, dict))
            self.send('stop'); self.wait(lambda x: x == 'OK,stopped_drive')
            before, settled, sampling = self.observe_idle(0.8 if self.motion else 5)
            if self.motion:
                result = self.trial(settled)
        except (Exception, KeyboardInterrupt) as exc:
            errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            try:
                self.send('stop'); self.wait(lambda x: x == 'OK,stopped_drive', timeout=0.5)
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    event = self.event(0.03)
                    if isinstance(event, dict):
                        end = event
                        if not event['en'] and (event['fault'] or (event['stable_ms'] >= 500 and all(
                                event[f'{k}_{w}'] == 0 for w in WHEELS for k in ('cps', 'rpm', 'mms')))):
                            break
                else:
                    raise TimeoutError('final settling not observed')
                end = self.query()
                if end['en'] or end['sel'] or end['fault'] or end['wdg_reset']:
                    errors.append('final group not healthy/disabled')
                if self.run_sent and end['used'] != 1:
                    errors.append('one-shot state unexpectedly reset')
            except (Exception, KeyboardInterrupt) as exc:
                errors.append('final stop/status failed: ' + str(exc))
        return {'test': 'drive_pi_' + (self.motion_name or ('post_run_no_power' if self.after_run else
                         'powered_stationary' if self.powered_stationary else 'no_power')),
                'passed': not errors, 'errors': errors, 'firmware': self.firmware,
                'power_state': 'powered_suspended_confirmed' if self.motion else
                               'powered_stationary_confirmed' if self.powered_stationary else
                               'battery_disconnected_confirmed',
                'motion_command_sent': self.run_sent, 'command_order': self.commands,
                'start_state': before, 'end_state': end, 'idle_cadence': sampling, 'trial': result,
                'telemetry': self.rows, 'raw_serial_records': self.records,
                'scope': 'operator must confirm physical wheel directions/stop; not body motion/Vicon/waveform proof'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port')
    p.add_argument('--mode', choices=('no-power', 'post-run', 'powered-stationary', *MODES), required=True)
    p.add_argument('--confirm-battery-disconnected', action='store_true')
    p.add_argument('--confirm-powered-suspended', action='store_true')
    args = p.parse_args()
    if not ((args.mode in ('no-power', 'post-run') and args.confirm_battery_disconnected and not args.confirm_powered_suspended) or
            (args.mode in (*MODES, 'powered-stationary') and args.confirm_powered_suspended and not args.confirm_battery_disconnected)):
        p.error('exact mode-specific physical confirmation required; serial unopened')
    import serial
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    link = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.05, exclusive=True)
    link.dtr = link.rts = False
    link.port = args.port; link.open()
    try:
        result = Probe(link, motion=args.mode if args.mode in MODES else None,
                       after_run=args.mode == 'post-run', powered_stationary=args.mode == 'powered-stationary').run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
