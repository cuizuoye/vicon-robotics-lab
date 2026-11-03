#!/usr/bin/env python3
"""four_bench_v1: 默认stop/status；单轮固定±10%、5秒，不自动串行执行其它轮。

实际轮向/停止需操作者反馈；CCR是寄存器读数，不是PWM波形实测。未测CPR时不输出RPM。
主机计数保护：启动0.8秒仍无变化，或已有计数后连续0.5秒无变化，提前请求全停。
"""
import argparse
import json
import signal
import serial

from dual_motor_bench_probe import DualBenchProbe
from front_encoder_observe import count_delta
from motor_a_no_motor_pwm_probe import ProbeError

WHEELS = ('FL', 'FR', 'RL', 'RR')
INITIAL_COUNT_TIMEOUT_S = 0.8
NO_COUNT_CHANGE_TIMEOUT_S = 0.5
FIELDS = {'fw', 'ms', 'en', 'sel', 'limit_ms', 'fault'} | {
    key for w in WHEELS for key in (f'd_{w.lower()}', f'enc_{w.lower()}',
                                     f'cps_{w.lower()}', f'p_{w.lower()}1', f'p_{w.lower()}2')}


def stopped_four(row):
    return (row['en'] == 0 and row['sel'] == 0 and row['limit_ms'] == 0
            and all(row[f'd_{w.lower()}'] == row[f'p_{w.lower()}1'] == row[f'p_{w.lower()}2'] == 0
                    for w in WHEELS))


def parse_four_status(line):
    parts = line.split(',')
    if parts[0] not in ('TEL4', 'STATUS4'):
        raise ValueError('not four-wheel telemetry')
    fields = {}
    for part in parts[1:]:
        key, value = part.split('=', 1)
        if key in fields:
            raise ValueError('duplicate field')
        fields[key] = value
    if set(fields) != FIELDS or fields['fw'] != 'four_bench_v1':
        raise ValueError('wrong firmware/schema')
    row = {'kind': 'TEL' if parts[0] == 'TEL4' else 'STATUS', 'fw': fields['fw'], 'raw_kind': parts[0]}
    for key in FIELDS - {'fw'}:
        row[key] = int(fields[key], 16 if key == 'fault' else 10)
    if not (0 <= row['ms'] <= 0xffffffff and 0 <= row['fault'] <= 0xffffffff
            and row['en'] in (0, 1) and row['sel'] in range(5)
            and row['limit_ms'] == (5200 if row['en'] else 0)
            and bool(row['sel']) == bool(row['en'])):
        raise ValueError('invalid enable/selection/timing')
    for i, w in enumerate(WHEELS, 1):
        w = w.lower()
        duty = row[f'd_{w}']
        expected = {0: (0, 0), 100: (100, 90), -100: (90, 100)}.get(duty)
        if expected is None or (row[f'p_{w}1'], row[f'p_{w}2']) != expected:
            raise ValueError('unexpected actual CCR/duty mapping')
        if duty != 0 and row['sel'] != i:
            raise ValueError('nonselected wheel requested')
        if any(not -(1 << 31) <= row[f'{prefix}_{w}'] < (1 << 31) for prefix in ('enc', 'cps')):
            raise ValueError('invalid encoder value')
    if row['en'] == 0 and not stopped_four(row):
        raise ValueError('nonzero output while disabled')
    return row


class FourWheelProbe(DualBenchProbe):
    firmware_name = 'four_bench_v1'
    telemetry_prefixes = ('TEL4,', 'STATUS4,')
    parse_status = staticmethod(parse_four_status)

    def __init__(self, link, *, wheel=None, sign=1, **kwargs):
        if wheel not in (None,) + WHEELS or sign not in (-1, 1):
            raise ProbeError('invalid single-wheel request')
        super().__init__(link, test='status', **kwargs)
        self.wheel = wheel
        self.selected = WHEELS.index(wheel) + 1 if wheel else 0
        self.duty = sign * 100
        self.duration_s = 5.0
        self.arm_command = f'arm {wheel} A5'
        self.arm_reply = f'OK,armed={wheel}'
        self.duty_command = f'duty {wheel} {self.duty}'
        self.duty_reply = f'OK,duty={wheel}:{self.duty}'
        self.allowed_commands = frozenset({'stop', 'status'})
        if wheel:
            self.allowed_commands |= {'hb', self.arm_command, self.duty_command}
        self.baseline = None
        self.last_encoder_count = None
        self.last_encoder_change_at = None
        self.encoder_progress_seen = False
        self.encoder_guard_trip_reason = None

    def encoder_progress_deadline(self):
        if self.wheel is None or self.output_started is None:
            return None
        if not self.encoder_progress_seen:
            return self.output_started + INITIAL_COUNT_TIMEOUT_S
        return self.last_encoder_change_at + NO_COUNT_CHANGE_TIMEOUT_S

    def check_encoder_progress_deadline(self):
        deadline = self.encoder_progress_deadline()
        if deadline is not None and self.now() >= deadline:
            self.encoder_guard_trip_reason = ('selected encoder stopped changing'
                                             if self.encoder_progress_seen
                                             else 'selected encoder has no initial counts')
            raise ProbeError(self.encoder_guard_trip_reason + '; stop, do not increase duty')

    def observe_encoder_progress(self, row):
        # 先查截止时间，避免迟到的计数刷新已超时的保护；不依赖cps或假定编码器方向。
        self.check_encoder_progress_deadline()
        count = row[f'enc_{self.wheel.lower()}']
        if count_delta(count, self.last_encoder_count) != 0:
            self.last_encoder_change_at = self.now()
            self.encoder_progress_seen = True
        self.last_encoder_count = count

    def service_heartbeat(self):
        if self.keepalive:
            self.check_encoder_progress_deadline()
        super().service_heartbeat()

    def send(self, command):
        # 四路遥测长于旧双路：115200下最坏发送约40ms；60ms命令间隔
        # 避免旧单行接收槽在阻塞TX期间收到两条命令。仅本工具改变节拍。
        if self.last_send is not None:
            remaining = 0.06 - (self.now() - self.last_send)
            if remaining > 0:
                self.sleep(remaining)
        if command == 'hb':
            self.check_encoder_progress_deadline()
        super().send(command)

    def quiet(self, row):
        return self.baseline is None or all(
            row[f'enc_{w.lower()}'] == self.baseline[f'enc_{w.lower()}'] and row[f'cps_{w.lower()}'] == 0
            for w in WHEELS if w != self.wheel)

    def armed_zero(self, row):
        return (row['en'] == 1 and row['sel'] == self.selected and row['limit_ms'] == 5200
                and all(row[f'd_{w.lower()}'] == 0 for w in WHEELS) and self.quiet(row))

    def active_pwm(self, row):
        return (row['en'] == 1 and row['sel'] == self.selected and row['limit_ms'] == 5200
                and all(row[f'd_{w.lower()}'] == (self.duty if w == self.wheel else 0) for w in WHEELS)
                and self.quiet(row))

    def run(self):
        if self.has_run:
            raise ProbeError('single use; no automatic repeat')
        self.has_run = True
        completed = False
        final = None
        try:
            self.wait_for(lambda r: isinstance(r, dict), stopped_four, timeout=2.0)
            self.phase = 'preflight_stop'
            self.exchange('stop', 'OK,stopped_all', stopped_four)
            self.baseline = self.query(stopped_four, stopped_four)
            if self.wheel:
                if any(self.baseline[f'cps_{w.lower()}'] != 0 for w in WHEELS):
                    raise ProbeError('encoders not stationary at preflight')
                self.phase = 'arm'
                self.keepalive = True
                self.send('hb')
                self.arm_attempted = True
                transition = lambda r: stopped_four(r) or self.armed_zero(r)
                self.exchange(self.arm_command, self.arm_reply, transition)
                self.query(self.armed_zero, transition)
                self.phase = 'request_pwm'
                self.output_started = self.now()
                self.output_deadline = self.output_started + self.duration_s
                self.last_encoder_count = self.baseline[f'enc_{self.wheel.lower()}']
                self.last_encoder_change_at = self.output_started
                transition = lambda r: self.armed_zero(r) or self.active_pwm(r)
                self.exchange(self.duty_command, self.duty_reply, transition)
                first = self.wait_for(lambda r: isinstance(r, dict) and self.active_pwm(r), transition, timeout=0.3)
                self.observe_encoder_progress(first)
                self.active_samples = 1
                self.phase = 'active_pwm'
                self.announce(f'PULSE_ACTIVE: only {self.wheel}, raw {self.duty/10:g}%, 5 seconds.')
                while self.now() < self.output_deadline:
                    self.check_encoder_progress_deadline()
                    event = self.next_event(min(self.output_deadline, self.encoder_progress_deadline()))
                    if event is None:
                        self.check_encoder_progress_deadline()
                        break
                    if not isinstance(event, dict) or not self.active_pwm(event) or event['fault']:
                        raise ProbeError('unexpected output, fault or nonselected encoder movement')
                    self.active_samples += 1
                    self.observe_encoder_progress(event)
                if self.active_samples < 40 or self.now() - self.output_deadline > 0.2:
                    raise ProbeError('insufficient telemetry or exceeded pulse deadline')
            completed = True
        except (Exception, KeyboardInterrupt) as exc:
            self.errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            self.keepalive = False
            self.phase = 'final_stop'
            try:
                self.exchange('stop', 'OK,stopped_all', lambda r: True, allow_fault=True)
                final = self.query(stopped_four, lambda r: True, allow_fault=True)
                self.stop_verified = True
                if final['fault']:
                    self.errors.append('latched final fault: ' + hex(final['fault']))
                if self.wheel:
                    if not self.quiet(final):
                        self.errors.append('nonselected encoder changed before final stop')
                    if self.baseline and final[f'enc_{self.wheel.lower()}'] == self.baseline[f'enc_{self.wheel.lower()}']:
                        self.errors.append('zero selected encoder delta')
                    self.announce('STOP_REQUEST_CONFIRMED: all CCR zero; observe actual stop and disconnect battery.')
            except (Exception, KeyboardInterrupt) as exc:
                self.errors.append('stop verification failed: ' + str(exc))
                self.announce('STOP_UNCONFIRMED: disconnect propulsion battery now.')
        delta = ({w: count_delta(final[f'enc_{w.lower()}'], self.baseline[f'enc_{w.lower()}']) for w in WHEELS}
                 if final is not None and self.baseline is not None else None)
        return {'test': 'four_wheel_single_bench', 'firmware': self.firmware_name,
                'wheel': self.wheel, 'raw_duty_permille': self.duty if self.wheel else 0,
                'requested_duration_s': self.duration_s if self.wheel else 0,
                'passed': completed and self.stop_verified and not self.errors, 'errors': self.errors,
                'active_samples': self.active_samples, 'start_state': self.baseline, 'end_state': final,
                'raw_count_delta': delta, 'raw_samples': self.records,
                'command_order': [c['command'] for c in self.commands],
                'software_stop_verified': self.stop_verified, 'requires_power_disconnect': not self.stop_verified,
                'encoder_progress_guard': {
                    'host_only': True, 'enabled': self.wheel is not None,
                    'initial_count_timeout_s': INITIAL_COUNT_TIMEOUT_S,
                    'no_count_change_timeout_s': NO_COUNT_CHANGE_TIMEOUT_S,
                    'progress_observed': self.encoder_progress_seen,
                    'last_change_from_request_s': (self.last_encoder_change_at - self.output_started
                                                   if self.output_started is not None else None),
                    'trip_reason': self.encoder_guard_trip_reason},
                'physical_rotation_observed': None, 'physical_stop_confirmed': None,
                'counts_per_output_revolution': None,
                'measurement_scope': 'raw counts and actual CCR reports; not measured waveform, RPM, CPR or physical stop proof'}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port')
    p.add_argument('--wheel', choices=WHEELS)
    p.add_argument('--sign', type=int, choices=(-1, 1), default=1)
    p.add_argument('--confirm-battery-disconnected', action='store_true')
    p.add_argument('--confirm-suspended', action='store_true')
    p.add_argument('--confirm-stationary', action='store_true')
    p.add_argument('--confirm-single-pulse', action='store_true')
    args = p.parse_args(argv)
    if args.wheel:
        if not all((args.confirm_suspended, args.confirm_stationary, args.confirm_single_pulse)):
            p.error('single pulse needs suspended, stationary and explicit single-pulse confirmation; serial unopened')
        if args.confirm_battery_disconnected:
            p.error('motion and battery-disconnected modes cannot be combined')
    elif not args.confirm_battery_disconnected:
        p.error('default stop/status requires battery-disconnected confirmation; serial unopened')
    def interrupted(signum, frame):
        raise KeyboardInterrupt('termination requested')

    signal.signal(signal.SIGTERM, interrupted)
    probe = FourWheelProbe(None, wheel=args.wheel, sign=args.sign)
    link = serial.Serial(port=None, baudrate=115200, timeout=0.01, write_timeout=0.1, exclusive=True)
    link.dtr = link.rts = False
    link.port = args.port
    link.open()
    try:
        probe.link = link
        result = probe.run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
