#!/usr/bin/env python3
"""双前轮编码器只读检查；front_enc_ro_v1，115200，不发arm/duty/hb。

默认动力断开的stop/status；--observe仅在确认悬空、只读固件、两轮初始静止后读取。
--turns是操作者声明的输出轴完整圈数，不是自动检测，不据此自动写入CPR或启动PI。
"""

import argparse
import json
import signal

import serial

from dual_motor_bench_probe import DualBenchProbe, stopped_dual
from motor_a_no_motor_pwm_probe import ProbeError


FIELDS = frozenset({'fw', 'ms', 'en', 'mask', 'a', 'b', 'az', 'bz', 'limit_ms',
                    'enc_a', 'cps_a', 'enc_b', 'cps_b', 'fault'})


def parse_front_status(line):
    parts = line.split(',')
    if parts[0] not in {'TEL3', 'STATUS3'}:
        raise ValueError('wrong telemetry prefix')
    fields = {}
    for part in parts[1:]:
        key, value = part.split('=', 1)
        if key in fields:
            raise ValueError('duplicate field')
        fields[key] = value
    if set(fields) != FIELDS or fields['fw'] != 'front_enc_ro_v1':
        raise ValueError('wrong readonly firmware/schema')
    row = {'kind': 'TEL' if parts[0] == 'TEL3' else 'STATUS',
           'raw_kind': parts[0], 'fw': fields['fw']}
    for key in FIELDS - {'fw'}:
        row[key] = int(fields[key], 16 if key == 'fault' else 10)
    if not stopped_dual(row):
        raise ValueError('readonly firmware must report disabled zero output')
    if not (0 <= row['ms'] <= 0xffffffff and 0 <= row['fault'] <= 0xffffffff):
        raise ValueError('invalid timestamp/fault')
    if any(not -(1 << 31) <= row[key] < (1 << 31)
           for key in ('enc_a', 'cps_a', 'enc_b', 'cps_b')):
        raise ValueError('encoder value out of range')
    return row


def count_delta(after, before):
    return ((after - before + (1 << 31)) % (1 << 32)) - (1 << 31)


class FrontEncoderObserve(DualBenchProbe):
    firmware_name = 'front_enc_ro_v1'
    telemetry_prefixes = ('TEL3,', 'STATUS3,')
    parse_status = staticmethod(parse_front_status)

    def __init__(self, link, *, seconds=0, expect_stationary=False,
                 wheel=None, turns=None, **kwargs):
        if not (seconds == 0 or 1 <= seconds <= 60):
            raise ProbeError('observation must be 1..60 seconds, or zero for status')
        if wheel not in (None, 'A', 'B') or ((wheel is None) != (turns is None)):
            raise ProbeError('wheel and operator-declared turns must be supplied together')
        if turns is not None and (not isinstance(turns, int) or not 1 <= turns <= 20):
            raise ProbeError('turns must be an integer in 1..20')
        if (wheel is not None or expect_stationary) and seconds == 0:
            raise ProbeError('manual/stationary mode requires observation')
        if wheel is not None and expect_stationary:
            raise ProbeError('manual turns cannot be a stationary test')
        super().__init__(link, test='status', **kwargs)
        self.seconds = seconds
        self.expect_stationary = expect_stationary
        self.wheel = wheel
        self.turns = turns
        # 明确不继承任何运动白名单；只允许停止和查询。
        self.allowed_commands = frozenset({'stop', 'status'})

    def check_feedback(self, row, baseline):
        if not stopped_dual(row) or row['fault']:
            raise ProbeError('readonly output/fault changed')
        quiet = ('a', 'b') if self.expect_stationary else ()
        if self.wheel is not None:
            quiet = ('b' if self.wheel == 'A' else 'a',)
        for wheel in quiet:
            if row['enc_' + wheel] != baseline['enc_' + wheel] or row['cps_' + wheel] != 0:
                raise ProbeError('stationary/nonselected encoder changed: ' + wheel)

    def run(self):
        if self.has_run:
            raise ProbeError('single-use observation; no automatic retry')
        self.has_run = True
        completed = False
        baseline = end = None
        observation_samples = 0
        try:
            self.wait_for(lambda row: isinstance(row, dict), stopped_dual, timeout=2.0)
            self.phase = 'preflight_stop'
            self.exchange('stop', 'OK,stopped_all', stopped_dual)
            baseline = self.query(stopped_dual, stopped_dual)
            self.check_feedback(baseline, baseline)
            if self.seconds:
                self.phase = 'observe_only'
                deadline = self.now() + self.seconds
                self.announce(f'OBSERVE_ONLY: 约{self.seconds:g}秒；控制线保持低，仅记录原始计数。')
                while self.now() < deadline:
                    event = self.next_event(min(deadline, self.now() + 0.4))
                    if event is None:
                        if self.now() >= deadline:
                            break
                        raise ProbeError('observation telemetry timeout')
                    if not isinstance(event, dict):
                        raise ProbeError('unexpected acknowledgement during observation')
                    self.check_feedback(event, baseline)
                    observation_samples += 1
                if observation_samples < int(self.seconds * 8):
                    raise ProbeError('insufficient observation samples')
            end = self.query(stopped_dual, stopped_dual)
            self.check_feedback(end, baseline)
            if self.turns is not None and end['enc_' + self.wheel.lower()] == baseline['enc_' + self.wheel.lower()]:
                raise ProbeError('declared manual turns produced zero counts')
            completed = True
        except (Exception, KeyboardInterrupt) as exc:
            self.errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            self.phase = 'final_stop'
            try:
                self.exchange('stop', 'OK,stopped_all', lambda row: True, allow_fault=True)
                self.query(stopped_dual, lambda row: True, allow_fault=True)
                self.stop_verified = True
            except (Exception, KeyboardInterrupt) as exc:
                self.errors.append('stop verification failed: ' + str(exc))
                self.announce('STOP_UNCONFIRMED: 请关闭驱动开关并拔掉动力电池。')
        deltas = ({w: count_delta(end['enc_' + w.lower()], baseline['enc_' + w.lower()])
                   for w in ('A', 'B')} if baseline is not None and end is not None else None)
        passed = completed and self.stop_verified and not self.errors
        return {'test': 'front_encoder_readonly', 'firmware': self.firmware_name,
                'baudrate': 115200, 'passed': passed, 'errors': self.errors,
                'observation_seconds': self.seconds, 'observation_samples': observation_samples,
                'expect_stationary': self.expect_stationary,
                'operator_declared_wheel': self.wheel, 'operator_declared_output_turns': self.turns,
                'start_state': baseline, 'end_state': end, 'raw_count_delta': deltas,
                'counts_per_output_revolution_estimate':
                    abs(deltas[self.wheel]) / self.turns if passed and self.turns else None,
                'raw_samples': self.records, 'command_order': [c['command'] for c in self.commands],
                'software_stop_verified': self.stop_verified,
                'requires_power_disconnect': not self.stop_verified,
                'physical_rotation_observed': None, 'physical_stop_confirmed': None,
                'measurement_scope': 'raw A/B counts; manual turns declared by operator; not RPM or calibrated CPR'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--observe', action='store_true')
    parser.add_argument('--seconds', type=int, default=10, choices=range(1, 61), metavar='1..60')
    parser.add_argument('--expect-stationary', action='store_true')
    parser.add_argument('--wheel', choices=('A', 'B'))
    parser.add_argument('--turns', type=int, choices=range(1, 21), metavar='1..20')
    parser.add_argument('--confirm-battery-disconnected', action='store_true')
    parser.add_argument('--confirm-readonly-firmware', action='store_true')
    parser.add_argument('--confirm-suspended', action='store_true')
    parser.add_argument('--confirm-stationary', action='store_true')
    args = parser.parse_args(argv)
    if args.observe:
        if not all((args.confirm_readonly_firmware, args.confirm_suspended, args.confirm_stationary)):
            parser.error('观察需确认只读固件、车架固定悬空和初始静止；未打开串口')
    elif not args.confirm_battery_disconnected:
        parser.error('默认状态检查需确认动力电池拔掉；未打开串口')
    try:
        # 参数验证必须发生在构造/打开真实串口之前。
        probe = FrontEncoderObserve(None, seconds=args.seconds if args.observe else 0,
                                    expect_stationary=args.expect_stationary,
                                    wheel=args.wheel, turns=args.turns)
    except ProbeError as exc:
        parser.error(str(exc))

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
        probe.link = link
        result = probe.run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
