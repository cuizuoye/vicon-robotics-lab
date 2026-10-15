#!/usr/bin/env python3
"""Motor_A编码器静止观察：只发送stop/status，不使能、不清零、不驱动。

须确认编码器已安全接入且有供电，车轮实际静止。只报告计数与cps；
没有每圈计数标定时不换算RPM，也不把计数稳定当成转动反馈已验证。
"""

import argparse
import json
import signal

import serial

from motor_a_no_motor_pwm_probe import PWMProbe, ProbeError, stopped


def signed_count_delta(end, start):
    """按32位累计计数求差，允许跨过有符号边界。"""
    return ((end - start + (1 << 31)) % (1 << 32)) - (1 << 31)


def summarize_encoder(rows):
    if not rows:
        return {'sample_count': 0, 'stationary_stable': False}
    steps = [signed_count_delta(b['enc'], a['enc'])
             for a, b in zip(rows, rows[1:])]
    gaps = [(b['ms'] - a['ms']) & 0xFFFFFFFF
            for a, b in zip(rows, rows[1:])]
    max_step = max((abs(value) for value in steps), default=0)
    max_cps = max(abs(row['cps']) for row in rows)
    return {
        'sample_count': len(rows),
        'count_start': rows[0]['enc'], 'count_end': rows[-1]['enc'],
        'count_delta': signed_count_delta(rows[-1]['enc'], rows[0]['enc']),
        'max_abs_step': max_step, 'max_abs_cps': max_cps,
        'observation_span_ms': (rows[-1]['ms'] - rows[0]['ms']) & 0xFFFFFFFF,
        'max_sample_gap_ms': max(gaps, default=0),
        'stationary_stable': len(rows) >= 2 and max_step == 0 and max_cps == 0,
    }


class EncoderObserveProbe(PWMProbe):
    duration_s = 5.0

    def __init__(self, link, **kwargs):
        super().__init__(link, **kwargs)
        self.allowed_commands = frozenset({'stop', 'status'})
        self.has_run = False

    def handle_ready_line(self, line):
        # 本阶段要求保持已启动的MCU，复位后不能沿用就绪确认。
        raise ProbeError('MCU reset/startup during encoder observation')

    def run(self):
        if self.has_run:
            raise ProbeError('single-use observation; automatic repeat is forbidden')
        self.has_run = True
        completed = False
        samples = []
        try:
            self.wait_for(lambda event: isinstance(event, dict), stopped, timeout=2.0)
            self.phase = 'preflight_stop'
            self.exchange('stop', 'OK,stopped', stopped)
            samples.append(self.query(stopped, stopped))
            self.phase = 'observe_encoder'
            deadline = self.now() + self.duration_s
            while self.now() < deadline:
                event = self.next_event(min(deadline, self.now() + 0.1))
                if event is None:
                    if self.now() - self.last_status_at > 0.4:
                        raise ProbeError('encoder observation telemetry timeout')
                    continue
                if not isinstance(event, dict) or not stopped(event) or event['fault']:
                    raise ProbeError('unexpected state during stationary observation')
                samples.append(event)
            completed = True
        except (Exception, KeyboardInterrupt) as exc:
            self.errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            self.keepalive = False
            self.phase = 'final_stop'
            try:
                # 即使先前报告异常，也只能请求stop；最后必须取得新的禁用STATUS。
                self.exchange('stop', 'OK,stopped', lambda row: True, allow_fault=True)
                self.query(stopped, lambda row: True, allow_fault=True)
                self.stop_verified = True
            except (Exception, KeyboardInterrupt) as exc:
                self.errors.append('stop verification failed: ' + str(exc))
                self.announce('STOP_UNCONFIRMED: 请关闭驱动板开关并拔掉电池。')
        summary = summarize_encoder(samples)
        if completed and not summary['stationary_stable']:
            self.errors.append('encoder changed while user-confirmed stationary; no motion enabled')
        quality_ok = (summary['sample_count'] >= 30
                      and summary.get('observation_span_ms', 0) >= 3500
                      and summary.get('max_sample_gap_ms', 0xFFFFFFFF) <= 250)
        if completed and not quality_ok:
            self.errors.append('insufficient or discontinuous encoder observations')
        return {
            'test': 'Motor_A stationary encoder observation; stop/status only',
            'baudrate': 115200, 'requested_duration_s': self.duration_s,
            'command_order': [item['command'] for item in self.commands],
            'startup_fragment_discarded': self.startup_fragment,
            'encoder_observation': summary,
            'last_reported_state': self.records[-1] if self.records else None,
            'software_stop_verified': self.stop_verified,
            'requires_power_disconnect': not self.stop_verified,
            'errors': self.errors,
            'passed': completed and quality_ok and self.stop_verified and not self.errors,
            'rotation_feedback_verified': False,
            'counts_per_wheel_revolution': None,
            'measurement_scope': 'stationary counts/cps only; physical state requires user observation',
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--confirm-encoder-connected', action='store_true',
                        help='确认E1A/PB4、E1B/PB5已按已核对电气条件接好且编码器供电')
    parser.add_argument('--confirm-input-compatible', action='store_true',
                        help='确认板级引脚空闲、FT数字输入NOPULL、MCU先供电且不复位/掉电')
    parser.add_argument('--confirm-stationary', action='store_true',
                        help='确认驱动零指令、车轮悬空稳定且实际静止；不手动转动')
    args = parser.parse_args(argv)
    if not (args.confirm_encoder_connected and args.confirm_input_compatible
            and args.confirm_stationary):
        parser.error('requires encoder wiring/electrical compatibility/stationary confirmations')

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
        result = EncoderObserveProbe(link).run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
