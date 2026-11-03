#!/usr/bin/env python3
"""动力断开且两路电机拔下时，仅复查/清除故障位1，绝不使能或发送心跳。

只允许stop、status、clear 5A。fault=1才清一次，fault=0跳过，其它故障拒绝清除。
用户确认标志不能自动检测电池和插头；清除后仍保持动力断开。
"""

import argparse
import json
import signal
import sys
import time

import serial

from motor_a_no_motor_pwm_probe import MAX_STATUS_AGE_S, PWMProbe, ProbeError, stopped


def eligible(row):
    return stopped(row) and row['fault'] in {0, 1}


def clean_stopped(row):
    return stopped(row) and row['fault'] == 0


class ClearTimeoutFaultProbe(PWMProbe):
    def __init__(self, link, *, now=time.monotonic, sleep=time.sleep,
                 announce=lambda message: print(message, file=sys.stderr, flush=True)):
        super().__init__(link, now=now, sleep=sleep, announce=announce)
        self.allowed_commands = frozenset({'stop', 'status', 'clear 5A'})
        self.preflight_started = False
        self.clear_permitted = False
        self.clear_attempted = False
        self.clear_acknowledged = False
        self.clear_verified = False
        self.fault_before = None
        self.clean_samples = 0

    def service_heartbeat(self):
        if self.keepalive:
            raise ProbeError('fault-clear tool must never enable heartbeat')

    def acknowledgement_lines(self):
        return {'OK,stopped', 'OK,faults_cleared'}

    def handle_ready_line(self, line):
        if self.preflight_started:
            raise ProbeError('MCU reset during fault-clear check')
        super().handle_ready_line(line)

    def send(self, command):
        if command == 'clear 5A':
            if (self.phase != 'clear_timeout_fault' or not self.clear_permitted
                    or self.clear_attempted):
                raise ProbeError('clear must be separately authorized once after fault=1 STATUS')
            if (self.last_status_at is None
                    or self.now() - self.last_status_at > MAX_STATUS_AGE_S):
                raise ProbeError('stale status; do not clear a fault')
            self.clear_attempted = True
        super().send(command)

    def run(self):
        completed = False
        try:
            self.wait_for(lambda event: isinstance(event, dict), eligible,
                          timeout=2.0, allow_fault=True)
            self.preflight_started = True
            self.phase = 'preflight_stop'
            self.exchange('stop', 'OK,stopped', eligible, allow_fault=True)
            before = self.query(eligible, eligible, allow_fault=True)
            self.fault_before = before['fault']

            if self.fault_before == 1:
                self.phase = 'clear_timeout_fault'
                self.clear_permitted = True
                self.exchange('clear 5A', 'OK,faults_cleared', eligible, allow_fault=True)
                self.clear_acknowledged = True
                self.query(clean_stopped, eligible, allow_fault=True)
                self.clear_verified = True

            self.phase = 'verify_clean_stopped'
            until = self.now() + 0.4
            while self.now() < until:
                event = self.next_event(until)
                if event is None:
                    break
                if not isinstance(event, dict) or not clean_stopped(event):
                    raise ProbeError('not continuously disabled, zero output, fault-free')
                if event['kind'] == 'TEL':
                    self.clean_samples += 1
            if self.clean_samples < 2:
                raise ProbeError('insufficient clean disabled telemetry')
            self.query(clean_stopped, clean_stopped)
            completed = True
        except (Exception, KeyboardInterrupt) as exc:
            self.errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            self.clear_permitted = False
            self.keepalive = False
            self.phase = 'final_stop'
            try:
                # 异常时也只stop/status；最终必须以新STATUS确认禁用，不凭ACK推断。
                self.exchange('stop', 'OK,stopped', lambda row: True, allow_fault=True)
                expected = clean_stopped if completed else stopped
                self.query(expected, lambda row: True, allow_fault=True)
                self.stop_verified = True
                self.announce('FAULT_CHECK_STOPPED: stop和禁用状态已确认；保持电池拔掉，不使能。')
            except (Exception, KeyboardInterrupt) as exc:
                self.errors.append('stop verification failed: ' + str(exc))
                self.announce('STOP_UNCONFIRMED: 保持电池拔掉和电机插头空置，不继续使能。')

        last = self.records[-1] if self.records else None
        return {
            'test': 'POWER DISCONNECTED, NO MOTOR; timeout fault 1 only',
            'baudrate': 115200,
            'startup_fragment_discarded': self.startup_fragment,
            'attempted_command_counts': {
                command: sum(item['command'] == command for item in self.commands)
                for command in sorted(self.allowed_commands)
            },
            'command_order': [item['command'] for item in self.commands],
            'fault_before': self.fault_before,
            'clear_attempted': self.clear_attempted,
            'clear_acknowledged': self.clear_acknowledged,
            'clear_verified': self.clear_verified,
            'clean_disabled_samples': self.clean_samples,
            'software_stop_verified': self.stop_verified,
            'last_reported_state': last,
            'errors': self.errors,
            'passed': completed and self.stop_verified and not self.errors,
            'measurement_scope': 'serial state only; battery/connector condition must be confirmed by user',
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--confirm-power-disconnected', action='store_true')
    parser.add_argument('--confirm-no-motor', action='store_true')
    args = parser.parse_args(argv)
    if not args.confirm_power_disconnected or not args.confirm_no_motor:
        parser.error('requires battery physically unplugged AND both motor connectors empty')

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
        result = ClearTimeoutFaultProbe(link).run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
