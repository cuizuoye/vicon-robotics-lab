#!/usr/bin/env python3
"""一台Motor_A悬空试转：仅正向10%、目标2秒、一次请求后stop。

须先确认车架固定且所有轮子离地，只连接Motor_A，动力上电零指令下始终静止。
不自动加速、反转、重试或清故障；串口停机反馈不是实际轮子停转的证明。
"""

import argparse
import json
import signal

import serial

from motor_a_no_motor_pwm_probe import PWMProbe, ProbeError


class UnloadedPulseProbe(PWMProbe):
    duration_s = 2.0
    minimum_active_samples = 15
    measurement_scope = ('serial control and stop reports only; '
                         'physical rotation and stop require user observation')

    def __init__(self, link, **kwargs):
        # 不提供运行时的反向、幅值或持续时间选择。
        super().__init__(link, direction='forward', **kwargs)
        self.has_run = False

    def send(self, command):
        if command in {'arm A5', self.duty_command}:
            expected_phase = 'arm' if command == 'arm A5' else 'request_pwm'
            if self.phase != expected_phase:
                raise ProbeError('motion command outside the single pulse phase')
            if any(item['command'] == command for item in self.commands):
                raise ProbeError('motion command retry is forbidden')
        super().send(command)

    def announce_active(self):
        self.announce('PULSE_ACTIVE: Motor_A正向10%，目标约2秒；'
                      '观察轮子，不移动接线、不用手接触轮子。')

    def announce_stopped(self):
        self.announce('PULSE_STOP_REQUEST_CONFIRMED: 已收到停机ACK和禁用状态；'
                      '请独立确认轮子实际停止。若仍转动，立即关开关并拔电池。')

    def test_description(self):
        return 'SINGLE Motor_A; suspended; forward 10%; one 2-second request'

    def run(self):
        if self.has_run:
            raise ProbeError('single-use probe; automatic repeat is forbidden')
        self.has_run = True
        result = super().run()
        # 仅保存原始计数，未接线/未标定时不能据此声称轮速或反馈正常。
        # 收集结果发生在stop检查后，不改变占空比、时限或动作次数。
        result['encoder_samples'] = [
            {name: row[name] for name in ('phase', 'kind', 'ms', 'enc', 'cps')}
            for row in self.records
        ]
        result['encoder_input_validation_required'] = True
        result['physical_rotation_observed'] = None
        result['physical_stop_confirmed'] = None
        return result


def require_conditions(suspended, single_motor_a, stationary):
    if not (suspended and single_motor_a and stationary):
        raise ProbeError('requires all three confirmations: suspended/stable chassis, '
                         'only Motor_A connected, stationary after power-up at zero command')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--confirm-suspended', action='store_true',
                        help='确认车架固定、所有轮子离地，人手和线材远离轮子且可立即断电')
    parser.add_argument('--confirm-single-motor-a', action='store_true',
                        help='确认仅本板Motor_A连接，Motor_B空置、另一驱动板未供电，电源不并联')
    parser.add_argument('--confirm-stationary', action='store_true',
                        help='确认当前上电零指令观察完全静止；MCU未复位、接线未改变')
    args = parser.parse_args(argv)
    try:
        require_conditions(args.confirm_suspended, args.confirm_single_motor_a,
                           args.confirm_stationary)
    except ProbeError as exc:
        parser.error(str(exc))

    def interrupted(signum, frame):
        raise KeyboardInterrupt('termination requested')

    signal.signal(signal.SIGTERM, interrupted)
    link = serial.Serial(port=None, baudrate=115200, timeout=0.01,
                         write_timeout=0.1, exclusive=True)
    # 不用DTR/RTS复位MCU；保持已验证的供电/启动状态。
    link.dtr = False
    link.rts = False
    link.port = args.port
    link.open()
    try:
        result = UnloadedPulseProbe(link).run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
