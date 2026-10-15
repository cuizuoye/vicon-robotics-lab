#!/usr/bin/env python3
"""无电机心跳超时检查：反向10%约3秒，然后停止心跳并保留超时故障。

两路电机插头必须拔下，表笔固定AO1对Vin−。不拔USB，不复位，不自动清故障。
只有在主动断心跳后接受一次ERR,heartbeat_timeout，必须在主机stop前观察到
禁用/零输出/故障位1。串口回报不是PWM波形或带载停机的独立证据。
"""

import argparse
import json
import signal
import sys
import time

import serial

from motor_a_no_motor_pwm_probe import (
    MAX_STATUS_AGE_S, PWMProbe, ProbeError, armed_zero, require_no_motor, stopped,
)


ACTIVE_HOLD_S = 3.0
MAX_TIMEOUT_OBSERVE_S = 1.2
MIN_TIMEOUT_OBSERVE_S = 0.4
LATCH_HOLD_S = 1.0
EXPECTED_TIMEOUT_FAULT = 1


def timed_out(row):
    return stopped(row) and row['fault'] == EXPECTED_TIMEOUT_FAULT


class HeartbeatTimeoutProbe(PWMProbe):
    def __init__(self, link, *, now=time.monotonic, sleep=time.sleep,
                 announce=lambda message: print(message, file=sys.stderr, flush=True)):
        super().__init__(link, direction='reverse', now=now, sleep=sleep, announce=announce)
        self.heartbeat_withheld = False
        self.withheld_at = None
        self.timeout_observe_deadline = None
        self.expected_notices = []
        self.first_automatic_state = None
        self.first_automatic_at = None
        self.post_last_hb_status = None
        self.automatic_timeout_verified = False
        self.fault_latched_verified = False
        self.latch_samples = 0

    def send(self, command):
        if command == 'hb' and self.heartbeat_withheld:
            raise ProbeError('heartbeat must never resume after intentional cutoff')
        super().send(command)

    def service_heartbeat(self):
        if self.heartbeat_withheld:
            if self.phase in {'heartbeat_off_sync', 'await_timeout', 'verify_latched'}:
                if (self.last_status_at is None
                        or self.now() - self.last_status_at > MAX_STATUS_AGE_S):
                    raise ProbeError('telemetry lost during intentional heartbeat cutoff')
            return
        super().service_heartbeat()

    def handle_error_line(self, line):
        allowed_phase = self.phase in {
            'heartbeat_off_sync', 'await_timeout', 'verify_latched', 'final_stop',
        }
        if (line == 'ERR,heartbeat_timeout' and self.heartbeat_withheld
                and allowed_phase and not self.expected_notices):
            self.expected_notices.append({'line': line, 'received_at_s': self.now()})
            return
        # 同名通知在断心跳前出现、重复通知或其它故障都不能作为测试通过证据。
        super().handle_error_line(line)

    def next_event(self, deadline):
        event = super().next_event(deadline)
        if isinstance(event, dict):
            self.records[-1]['received_at_s'] = self.now()
        return event

    def wait_for(self, matches, allowed, timeout=1.0, *, allow_fault=False):
        if (self.timeout_observe_deadline is not None
                and self.phase in {'heartbeat_off_sync', 'await_timeout'}):
            remaining = self.timeout_observe_deadline - self.now()
            if remaining <= 0:
                raise ProbeError('intentional timeout observation deadline exceeded')
            timeout = min(timeout, remaining)
        return super().wait_for(matches, allowed, timeout, allow_fault=allow_fault)

    def run(self):
        completed = False
        try:
            self.wait_for(lambda event: isinstance(event, dict), stopped, timeout=2.0)
            self.phase = 'preflight_stop'
            self.exchange('stop', 'OK,stopped', stopped)
            self.query(stopped, stopped)

            self.phase = 'arm'
            self.keepalive = True
            self.service_heartbeat()
            self.arm_attempted = True
            transition = lambda row: stopped(row) or armed_zero(row)
            self.exchange('arm A5', 'OK,armed', transition)
            self.query(armed_zero, transition)

            self.phase = 'request_pwm'
            self.output_started = self.now()
            self.output_deadline = self.output_started + ACTIVE_HOLD_S + MAX_TIMEOUT_OBSERVE_S
            hold_until = self.output_started + ACTIVE_HOLD_S
            transition = lambda row: armed_zero(row) or self.active_pwm(row)
            self.exchange(self.duty_command, self.duty_reply, transition)
            self.wait_for(lambda event: isinstance(event, dict) and self.active_pwm(event),
                          transition, timeout=0.3)
            self.active_samples = 1
            self.phase = 'active_pwm'
            self.announce('PWM_ACTIVE: 反向10%约3秒，读取固定在AO1的表笔；电机仍须拔下。')
            while self.now() < hold_until:
                event = self.next_event(hold_until)
                if event is None:
                    break
                if (not isinstance(event, dict) or not self.active_pwm(event)
                        or event['fault'] != 0):
                    raise ProbeError('PWM telemetry changed before intentional cutoff')
                self.active_samples += 1
            if self.now() - hold_until > 0.2 or self.active_samples < 20:
                raise ProbeError('invalid active PWM observation window')

            # 发送最后一次心跳后，永久禁用本工具的hb发送。status不会续心跳。
            self.phase = 'heartbeat_off_sync'
            self.send('hb')
            self.keepalive = False
            self.heartbeat_withheld = True
            self.withheld_at = self.now()
            deadline = min(self.output_deadline, self.last_hb + MAX_TIMEOUT_OBSERVE_S)
            self.timeout_observe_deadline = deadline
            self.announce('HEARTBEAT_OFF: 已停止心跳，仅监听/查询；不要拔USB或复位。')
            self.post_last_hb_status = self.query(self.active_pwm, self.active_pwm)
            self.phase = 'await_timeout'
            while self.now() < deadline:
                event = self.next_event(deadline)
                if event is None:
                    break
                if not isinstance(event, dict):
                    raise ProbeError('unexpected acknowledgement while waiting for timeout')
                if self.active_pwm(event) and event['fault'] == 0:
                    continue
                if not timed_out(event):
                    raise ProbeError('timeout did not report disabled output and fault bit 1')
                delay = self.now() - self.last_hb
                if (not self.expected_notices
                        or not MIN_TIMEOUT_OBSERVE_S <= delay <= MAX_TIMEOUT_OBSERVE_S):
                    raise ProbeError('missing timeout notice or unexpected observed delay')
                self.first_automatic_state = event.copy()
                self.first_automatic_at = self.now()
                break
            if self.first_automatic_state is None:
                raise ProbeError('automatic timeout stop not observed before fallback deadline')

            # 主机尚未发送stop：先用status独立确认自动禁用及故障，再保持监听。
            self.phase = 'verify_latched'
            self.query(timed_out, timed_out, allow_fault=True)
            self.automatic_timeout_verified = True
            self.announce('AUTO_STOP_REPORTED: 已回报自动禁用、零输出、故障0x00000001；继续核对锁存。')
            latch_until = self.now() + LATCH_HOLD_S
            while self.now() < latch_until:
                event = self.next_event(latch_until)
                if event is None:
                    break
                if not isinstance(event, dict) or not timed_out(event):
                    raise ProbeError('timeout fault cleared or output restarted unexpectedly')
                if event['kind'] == 'TEL':
                    self.latch_samples += 1
            if self.now() - latch_until > 0.2 or self.latch_samples < 5:
                raise ProbeError('insufficient fault-latch observation')
            self.query(timed_out, timed_out, allow_fault=True)
            self.fault_latched_verified = True
            completed = True
        except (Exception, KeyboardInterrupt) as exc:
            self.errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            # 异常也只请求stop，不清故障、不再arm、不恢复心跳。
            self.keepalive = False
            self.phase = 'final_stop'
            known = lambda row: stopped(row) or armed_zero(row) or self.active_pwm(row)
            try:
                self.exchange('stop', 'OK,stopped', known, allow_fault=True)
                expected = timed_out if completed else stopped
                self.query(expected, known, allow_fault=True)
                self.stop_verified = True
                self.announce('TEST_STOPPED: 已取得stop确认和禁用状态，未清故障；关闭驱动板并拔电池。')
            except (Exception, KeyboardInterrupt) as exc:
                self.errors.append('stop verification failed: ' + str(exc))
                self.announce('STOP_UNCONFIRMED: 请立即关闭驱动板开关并拔掉电池。')

        last = self.records[-1] if self.records else None
        notice_at = (self.expected_notices[0]['received_at_s']
                     if self.expected_notices else None)
        result = {
            'test': 'NO MOTOR; reverse 10%; intentional heartbeat timeout',
            'baudrate': 115200,
            'duty_permille': self.duty_permille,
            'measurement_output': 'AO1',
            'active_hold_target_s': ACTIVE_HOLD_S,
            'configured_firmware_timeout_ms': 500,
            'maximum_timeout_observation_s': MAX_TIMEOUT_OBSERVE_S,
            'startup_fragment_discarded': self.startup_fragment,
            'attempted_command_counts': {
                command: sum(item['command'] == command for item in self.commands)
                for command in sorted(self.allowed_commands)
            },
            'command_order_without_heartbeats': [
                item['command'] for item in self.commands if item['command'] != 'hb'
            ],
            'active_samples': self.active_samples,
            'latch_samples': self.latch_samples,
            'telemetry_and_status_count': len(self.records),
            'heartbeat_withheld_at_s': self.withheld_at,
            'last_heartbeat_sent_at_s': self.last_hb,
            'expected_timeout_notices': self.expected_notices,
            'host_last_heartbeat_to_notice_s': (
                notice_at - self.last_hb if notice_at is not None and self.last_hb is not None else None),
            'host_last_heartbeat_to_disabled_report_s': (
                self.first_automatic_at - self.last_hb
                if self.first_automatic_at is not None and self.last_hb is not None else None),
            'post_last_heartbeat_status': self.post_last_hb_status,
            'first_automatic_stopped_state': self.first_automatic_state,
            'automatic_timeout_verified_before_host_stop': self.automatic_timeout_verified,
            'fault_latch_verified_without_clear': self.fault_latched_verified,
            'software_stop_verified': self.stop_verified,
            'requires_power_disconnect': not self.stop_verified,
            'last_reported_state': last,
            'errors': self.errors,
            'passed': completed and self.stop_verified and not self.errors,
            'measurement_scope': 'serial reports and host receipt timing only; not waveform or loaded stop proof',
        }
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--confirm-no-motor', action='store_true',
                        help='两路电机均拔下，AO1表笔固定，MCU和驱动板供电未并联')
    args = parser.parse_args(argv)
    try:
        require_no_motor(args.confirm_no_motor)
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
        result = HeartbeatTimeoutProbe(link).run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
