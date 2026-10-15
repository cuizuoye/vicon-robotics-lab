#!/usr/bin/env python3
"""独立dual_v2固件台架工具；默认只stop/status，动作必须显式选择和确认。

不兼容dual_v1、旧单A或MC-Link。固定10%；普通2秒，专用A+/B-观察5秒。不自动重试、
换向、清故障或加速。两电机均悬空，未选通道也必须验证零指令。
"""

import argparse
import json
import signal

import serial

from motor_a_no_motor_pwm_probe import PWMProbe, ProbeError


TESTS = {
    'b-positive': ('B', 2, 0, 100),
    # 两路同电气正号用于诊断，不再把它称作两轮共同前进。
    'both-positive': ('AB', 3, 100, 100),
    # 用户按共同车头参考确认B安装轮向相反；只在主机请求层归一化。
    'both-forward': ('AB', 3, 100, -100),
    'both-forward-observe': ('AB', 3, 100, -100),
    'a-reverse': ('A', 1, -100, 0),
}
FIELDS = frozenset({'fw', 'ms', 'en', 'mask', 'a', 'b', 'az', 'bz', 'limit_ms',
                    'enc_a', 'cps_a', 'fault'})


def parse_dual_status(line, *, firmware='dual_v2', limits=(0, 2200, 5200),
                      forward_only_limit=5200):
    parts = line.split(',')
    if parts[0] not in {'TEL2', 'STATUS2'}:
        raise ValueError('not dual_v2 telemetry')
    fields = {}
    for part in parts[1:]:
        key, value = part.split('=', 1)
        if key in fields:
            raise ValueError('duplicate telemetry field')
        fields[key] = value
    if set(fields) != FIELDS or fields['fw'] != firmware:
        raise ValueError('wrong firmware/schema')
    row = {'kind': 'STATUS' if parts[0] == 'STATUS2' else 'TEL',
           'raw_kind': parts[0], 'fw': fields['fw']}
    for key in FIELDS - {'fw'}:
        row[key] = int(fields[key], 16 if key == 'fault' else 10)
    if not (0 <= row['ms'] <= 0xFFFFFFFF and 0 <= row['fault'] <= 0xFFFFFFFF
            and row['en'] in (0, 1) and row['mask'] in (0, 1, 2, 3)
            and row['a'] in (-100, 0, 100) and row['b'] in (-100, 0, 100)
            and row['az'] == int(row['a'] == 0) and row['bz'] == int(row['b'] == 0)
            and row['limit_ms'] in limits
            and -(1 << 31) <= row['enc_a'] < (1 << 31)
            and -(1 << 31) <= row['cps_a'] < (1 << 31)):
        raise ValueError('invalid dual telemetry values')
    if row['en'] == 0 and not stopped_dual(row):
        raise ValueError('disabled state has nonzero output')
    if row['en'] == 1 and (row['mask'] == 0
                          or row['limit_ms'] == 0
                          or (row['mask'] == 1 and row['b'] != 0)
                          or (row['mask'] == 2 and row['a'] != 0)):
        raise ValueError('output outside armed channel mask')
    if row['limit_ms'] == forward_only_limit and (row['mask'] != 3 or row['a'] not in (0, 100)
                                   or row['b'] not in (0, -100)):
        raise ValueError('5-second window is only for A+/B- observation')
    return row


def stopped_dual(row):
    return (row['en'] == 0 and row['mask'] == 0 and row['a'] == 0 and row['b'] == 0
            and row['az'] == 1 and row['bz'] == 1 and row['limit_ms'] == 0)


class DualBenchProbe(PWMProbe):
    duration_s = 2.0
    minimum_active_samples = 15
    firmware_name = 'dual_v2'
    parse_status = staticmethod(parse_dual_status)
    telemetry_prefixes = ('TEL2,', 'STATUS2,')

    def __init__(self, link, *, test='status', **kwargs):
        if test != 'status' and test not in TESTS:
            raise ProbeError('unknown fixed test')
        super().__init__(link, **kwargs)
        self.test = test
        self.has_run = False
        self.count_a_before = None
        long_observe = test == 'both-forward-observe'
        self.duration_s = 5.0 if long_observe else 2.0
        self.minimum_active_samples = 45 if long_observe else 15
        self.expected_limit_ms = 5200 if long_observe else 2200
        self.channel, self.mask, self.a, self.b = TESTS.get(test, ('NONE', 0, 0, 0))
        self.arm_command = f'arm {self.channel} A5'
        self.arm_reply = f'OK,armed={self.channel}'
        if long_observe:
            self.arm_command = 'arm AB A5 5s'
            self.arm_reply = 'OK,armed=AB5'
        if self.channel == 'AB':
            self.duty_command = f'duty AB {self.a} {self.b}'
        else:
            self.duty_command = f'duty {self.channel} {self.a or self.b}'
        self.duty_reply = f'OK,duty=A:{self.a},B:{self.b}'
        self.allowed_commands = frozenset({'stop', 'status'})
        if test != 'status':
            self.allowed_commands |= {'hb', self.arm_command, self.duty_command}

    def send(self, command):
        if command in {self.arm_command, self.duty_command}:
            expected = 'arm' if command == self.arm_command else 'request_pwm'
            if self.phase != expected or any(item['command'] == command for item in self.commands):
                raise ProbeError('motion retry/out-of-phase command forbidden')
        super().send(command)

    def armed_zero(self, row):
        return (row['en'] == 1 and row['mask'] == self.mask
                and row['a'] == 0 and row['b'] == 0 and row['az'] == 1 and row['bz'] == 1
                and row['limit_ms'] == self.expected_limit_ms
                and self.a_feedback_quiet(row))

    def a_feedback_quiet(self, row):
        # B单独试转时A编码器已验证接好；计数变化也应中止，而非只看A=0请求。
        return (self.test != 'b-positive' or self.count_a_before is None
                or (row['enc_a'] == self.count_a_before and row['cps_a'] == 0))

    def active_pwm(self, row):
        return (row['en'] == 1 and row['mask'] == self.mask
                and row['a'] == self.a and row['b'] == self.b
                and row['az'] == int(self.a == 0) and row['bz'] == int(self.b == 0)
                and row['limit_ms'] == self.expected_limit_ms
                and self.a_feedback_quiet(row))

    def next_event(self, deadline):
        while self.now() < deadline:
            self.service_heartbeat()
            if self.now() >= deadline:
                return None
            if not self.lines:
                count = min(max(self.link.in_waiting, 1), 2048)
                self.feed(self.link.read(count))
                continue
            line = self.lines.popleft()
            if line.startswith('ERR,'):
                raise ProbeError(line)
            if line.startswith('READY,'):
                raise ProbeError('MCU reset/startup; require a new physical readiness check')
            if line.startswith('INFO,'):
                continue
            if line.startswith(self.telemetry_prefixes):
                try:
                    row = self.parse_status(line)
                except (ValueError, KeyError) as exc:
                    raise ProbeError('malformed/wrong-channel dual telemetry') from exc
                self.last_status_at = self.now()
                self.records.append({'phase': self.phase, **row})
                return row
            if line in {'OK,stopped_all', self.arm_reply, self.duty_reply}:
                return line
            raise ProbeError('unexpected firmware/serial line: ' + line)
        return None

    def run(self):
        if self.has_run:
            raise ProbeError('single-use; automatic repeat forbidden')
        self.has_run = True
        completed = False
        try:
            self.wait_for(lambda row: isinstance(row, dict), stopped_dual, timeout=2.0)
            self.phase = 'preflight_stop'
            self.exchange('stop', 'OK,stopped_all', stopped_dual)
            baseline = self.query(stopped_dual, stopped_dual)
            if self.test == 'b-positive':
                if baseline['cps_a'] != 0:
                    raise ProbeError('A encoder not stationary before B-only test')
                self.count_a_before = baseline['enc_a']
            if self.test == 'status':
                completed = True
            else:
                self.phase = 'arm'
                self.keepalive = True
                self.send('hb')
                self.arm_attempted = True
                transition = lambda row: stopped_dual(row) or self.armed_zero(row)
                self.exchange(self.arm_command, self.arm_reply, transition)
                self.query(self.armed_zero, transition)
                self.phase = 'request_pwm'
                self.output_started = self.now()
                self.output_deadline = self.output_started + self.duration_s
                transition = lambda row: self.armed_zero(row) or self.active_pwm(row)
                self.exchange(self.duty_command, self.duty_reply, transition)
                self.wait_for(lambda row: isinstance(row, dict) and self.active_pwm(row),
                              transition, timeout=0.3)
                self.active_samples = 1
                self.phase = 'active_pwm'
                self.announce(f'PULSE_ACTIVE: {self.test}; A={self.a}, B={self.b}; 约{self.duration_s:g}秒。')
                while self.now() < self.output_deadline:
                    event = self.next_event(self.output_deadline)
                    if event is None:
                        break
                    if not isinstance(event, dict) or not self.active_pwm(event) or event['fault']:
                        raise ProbeError('unexpected dual-channel state during pulse')
                    self.active_samples += 1
                if self.now() - self.output_deadline > 0.2:
                    raise ProbeError('host exceeded pulse deadline')
                if self.active_samples < self.minimum_active_samples:
                    raise ProbeError('insufficient motion telemetry')
                completed = True
        except (Exception, KeyboardInterrupt) as exc:
            self.errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            self.keepalive = False
            self.phase = 'final_stop'
            try:
                self.exchange('stop', 'OK,stopped_all', lambda row: True, allow_fault=True)
                self.query(stopped_dual, lambda row: True, allow_fault=True)
                self.stop_verified = True
                if self.test != 'status':
                    self.announce('STOP_REQUEST_CONFIRMED: 两路回报零指令；请独立确认轮子停转并断电。')
            except (Exception, KeyboardInterrupt) as exc:
                self.errors.append('stop verification failed: ' + str(exc))
                self.announce('STOP_UNCONFIRMED: 请关闭驱动板开关并拔掉电池。')
        stop_at = next((item['at_s'] for item in reversed(self.commands)
                        if item['command'] == 'stop'), None)
        return {
            'test': self.test, 'firmware': self.firmware_name, 'baudrate': 115200,
            'requested_duration_s': 0.0 if self.test == 'status' else self.duration_s,
            'duty_a_permille': self.a, 'duty_b_permille': self.b,
            'command_order': [item['command'] for item in self.commands],
            'startup_fragment_discarded': self.startup_fragment,
            'output_request_to_stop_s': (stop_at - self.output_started
                                         if self.output_started is not None and stop_at is not None else None),
            'active_samples': self.active_samples, 'raw_samples': self.records,
            'last_reported_state': self.records[-1] if self.records else None,
            'software_stop_verified': self.stop_verified,
            'requires_power_disconnect': not self.stop_verified,
            'errors': self.errors, 'passed': completed and self.stop_verified and not self.errors,
            'physical_rotation_observed': None, 'physical_stop_confirmed': None,
            'counts_per_wheel_revolution': None,
            'measurement_scope': 'reported A/B requests and raw A counts; not measured PWM, RPM or physical stop proof',
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--test', choices=('status', *TESTS), default='status')
    parser.add_argument('--confirm-battery-disconnected', action='store_true')
    parser.add_argument('--confirm-suspended', action='store_true')
    parser.add_argument('--confirm-two-motors', action='store_true',
                        help='仅第一驱动板Motor_A/FL及Motor_B/FR接入，第二板不供电，逻辑电源不并联')
    parser.add_argument('--confirm-channel-isolation', action='store_true',
                        help='AIN/PE5/6和BIN/PE9/11接线明确，未选通道也应保持静止')
    parser.add_argument('--confirm-stationary', action='store_true')
    args = parser.parse_args(argv)
    if args.test == 'status':
        if not args.confirm_battery_disconnected:
            parser.error('status requires physically disconnected battery confirmation')
    elif not (args.confirm_suspended and args.confirm_two_motors
              and args.confirm_channel_isolation and args.confirm_stationary):
        parser.error('motion requires suspended/two-motors/channel-isolation/stationary confirmations')

    def interrupted(signum, frame):
        raise KeyboardInterrupt('termination requested')

    signal.signal(signal.SIGTERM, interrupted)
    link = serial.Serial(port=None, baudrate=115200, timeout=0.01,
                         write_timeout=0.1, exclusive=True)
    link.dtr = False
    link.rts = False
    link.port = args.port
    link.open()
    try:
        result = DualBenchProbe(link, test=args.test).run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
