#!/usr/bin/env python3
"""两前轮悬空共同前进展示；独立demo30_v1固件，固定10%、单次30秒。

默认只stop/status且要求动力断开。必须显式--run及全部就绪确认才运行。
不兼容当前已烧录的dual_v2，不烧录、不复位、不修改网络，不自动重试。
"""

import argparse
import json
import signal

import serial

from dual_motor_bench_probe import DualBenchProbe, parse_dual_status


def parse_demo_status(line):
    return parse_dual_status(line, firmware='demo30_v1', limits=(0, 30200),
                             forward_only_limit=30200)


class FrontDemoProbe(DualBenchProbe):
    firmware_name = 'demo30_v1'
    parse_status = staticmethod(parse_demo_status)

    def __init__(self, link, *, run_demo=False, **kwargs):
        # 复用已验证的单次动作、串口节奏、心跳和finally全停路径。
        super().__init__(link, test='both-forward-observe' if run_demo else 'status', **kwargs)
        self.test = 'front-forward-30s' if run_demo else 'status'
        self.duration_s = 30.0
        self.minimum_active_samples = 280
        self.expected_limit_ms = 30200
        self.arm_command = 'arm AB A5 30s'
        self.arm_reply = 'OK,armed=AB30'
        self.allowed_commands = frozenset({'stop', 'status'})
        if run_demo:
            self.allowed_commands |= {'hb', self.arm_command, self.duty_command}
        self.next_progress_s = 5.0

    def next_event(self, deadline):
        event = super().next_event(deadline)
        if self.phase == 'active_pwm' and self.output_started is not None:
            elapsed = self.now() - self.output_started
            if elapsed >= self.next_progress_s:
                self.announce(f'DEMO_PROGRESS: 约{int(self.next_progress_s)} / 30秒，A=+10%，B=−10%。')
                self.next_progress_s += 5.0
        return event


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--run', action='store_true', help='显式启动唯一一次30秒展示')
    parser.add_argument('--confirm-battery-disconnected', action='store_true')
    parser.add_argument('--confirm-suspended', action='store_true')
    parser.add_argument('--confirm-two-motors', action='store_true',
                        help='只接第一板A/FL、B/FR，第二板不供电，逻辑电源不并联')
    parser.add_argument('--confirm-channel-isolation', action='store_true')
    parser.add_argument('--confirm-stationary', action='store_true')
    parser.add_argument('--confirm-30s', action='store_true', help='确认本轮允许固定10%%、30秒动作')
    args = parser.parse_args(argv)
    if args.run:
        if not all((args.confirm_suspended, args.confirm_two_motors,
                    args.confirm_channel_isolation, args.confirm_stationary, args.confirm_30s)):
            parser.error('缺少悬空、两电机、通道隔离、静止或30秒许可确认；未打开串口')
    elif not args.confirm_battery_disconnected:
        parser.error('默认状态检查要求动力电池已拔掉；未打开串口')

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
        result = FrontDemoProbe(link, run_demo=args.run).run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
