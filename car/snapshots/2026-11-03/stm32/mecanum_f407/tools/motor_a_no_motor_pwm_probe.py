#!/usr/bin/env python3
"""无电机台架专用：选定一个方向的10% PWM，限时10秒，结束请求stop。

这不是带电机测试工具。确认两路电机插头都拔下后才能使用。
默认正向测AO2；--direction reverse反向测AO1，不在同一轮中切换方向。
只验证串口回报；电压、波形和真实停机仍需独立测量。
"""

import argparse
from collections import deque
import json
import signal
import sys
import time

import serial

from motor_a_safe_probe import parse_status


DUTY_PERMILLE = 100
DURATION_S = 10.0
HEARTBEAT_INTERVAL_S = 0.1
MAX_HEARTBEAT_GAP_S = 0.3
MAX_STATUS_AGE_S = 0.4
COMMAND_GAP_S = 0.03
BASE_COMMANDS = frozenset({'stop', 'status', 'hb', 'arm A5'})


class ProbeError(RuntimeError):
    pass


def stopped(row):
    return (row['en'] == 0 and row['hiz'] == 1 and row['duty'] == 0
            and row['target_cps'] == 0 and row['mode'] == 'duty')


def armed_zero(row):
    return (row['en'] == 1 and row['hiz'] == 1 and row['duty'] == 0
            and row['target_cps'] == 0 and row['mode'] == 'duty')


def pwm_state(row, duty_permille):
    return (row['en'] == 1 and row['hiz'] == 0
            and row['duty'] == duty_permille
            and row['target_cps'] == 0 and row['mode'] == 'duty')


class PWMProbe:
    duration_s = DURATION_S
    minimum_active_samples = 80
    measurement_scope = 'serial reports only; not waveform or physical stop proof'

    def __init__(self, link, *, direction='forward', now=time.monotonic, sleep=time.sleep,
                 announce=lambda message: print(message, file=sys.stderr, flush=True)):
        if direction not in {'forward', 'reverse'}:
            raise ProbeError('direction must be forward or reverse')
        self.direction = direction
        self.duty_permille = DUTY_PERMILLE if direction == 'forward' else -DUTY_PERMILLE
        self.direction_name = '正向' if direction == 'forward' else '反向'
        self.measurement_output = 'AO2' if direction == 'forward' else 'AO1'
        self.duty_command = f'duty {self.duty_permille}'
        self.duty_reply = f'OK,duty={self.duty_permille}'
        # 每一轮只允许已选方向，不能在运行中反转或增大占空比。
        self.allowed_commands = BASE_COMMANDS | {self.duty_command}
        self.link = link
        self.now = now
        self.sleep = sleep
        self.announce = announce
        self.phase = 'synchronize'
        self.buffer = bytearray()
        self.lines = deque()
        self.rx_synchronized = False
        self.startup_fragment = None
        self.last_send = None
        self.last_status_at = None
        self.last_hb = None
        self.keepalive = False
        self.arm_attempted = False
        self.output_started = None
        self.output_deadline = None
        self.records = []
        self.commands = []
        self.active_samples = 0
        self.stop_verified = False
        self.errors = []

    def active_pwm(self, row):
        return pwm_state(row, self.duty_permille)

    def send(self, command):
        if command not in self.allowed_commands:
            raise ProbeError('command is outside the fixed no-motor test')
        if self.last_send is not None:
            remaining = COMMAND_GAP_S - (self.now() - self.last_send)
            if remaining > 0:
                self.sleep(remaining)
        # 防止线程暂停后补发心跳或超出本轮PWM时间窗口。
        if (command == self.duty_command and self.output_deadline is not None
                and self.now() >= self.output_deadline):
            raise ProbeError('PWM request would exceed the output deadline')
        if command == 'hb':
            if not self.keepalive:
                raise ProbeError('heartbeat outside an authorized phase')
            if self.output_deadline is not None and self.now() >= self.output_deadline:
                raise ProbeError('heartbeat would exceed the PWM deadline')
            if self.last_hb is not None and self.now() - self.last_hb > MAX_HEARTBEAT_GAP_S:
                raise ProbeError('host scheduling gap; do not renew heartbeat')
        sent_at = self.now()
        self.commands.append({'command': command, 'at_s': sent_at})
        self.last_send = sent_at
        data = (command + '\n').encode('ascii')
        if self.link.write(data) != len(data):
            raise ProbeError('incomplete serial write')
        if command == 'hb':
            self.last_hb = sent_at

    def service_heartbeat(self):
        if not self.keepalive:
            return
        now = self.now()
        if self.output_deadline is not None and now >= self.output_deadline:
            return
        if self.last_status_at is None or now - self.last_status_at > MAX_STATUS_AGE_S:
            raise ProbeError('telemetry timeout; stop heartbeat')
        if self.last_hb is not None and now - self.last_hb > MAX_HEARTBEAT_GAP_S:
            raise ProbeError('host scheduling gap; stop heartbeat')
        if self.last_hb is None or now - self.last_hb >= HEARTBEAT_INTERVAL_S:
            self.send('hb')

    def feed(self, data):
        for byte in data:
            if byte != 10:
                self.buffer.append(byte)
                if len(self.buffer) > 512:
                    raise ProbeError('serial line exceeds 512 bytes')
                continue
            raw = bytes(self.buffer).rstrip(b'\r')
            self.buffer.clear()
            # 打开串口可能位于一行中间，只丢弃第一行并明确记录；之后严格解析。
            if not self.rx_synchronized:
                self.startup_fragment = raw.decode('ascii', errors='backslashreplace')
                self.rx_synchronized = True
                continue
            try:
                line = raw.decode('ascii')
            except UnicodeDecodeError as exc:
                raise ProbeError('non-ASCII serial data') from exc
            if line:
                self.lines.append(line)

    def handle_error_line(self, line):
        # 普通PWM检查不接受任何ERR；专门的超时检查仅重载这一处通知处理。
        raise ProbeError(line)

    def handle_ready_line(self, line):
        if self.arm_attempted:
            raise ProbeError('MCU reset after arm was attempted')
        if line != 'READY,motor_a_safe_off':
            raise ProbeError('unexpected firmware READY line')

    def acknowledgement_lines(self):
        return {'OK,stopped', 'OK,armed', self.duty_reply}

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
                self.handle_error_line(line)
                continue
            if line.startswith('READY,'):
                self.handle_ready_line(line)
                continue
            if line.startswith('INFO,'):
                continue
            if line.startswith(('TEL,', 'STATUS,')):
                try:
                    row = parse_status(line)
                except (ValueError, KeyError) as exc:
                    raise ProbeError('malformed telemetry') from exc
                self.last_status_at = self.now()
                self.records.append({'phase': self.phase, **row})
                return row
            if line in self.acknowledgement_lines():
                return line
            raise ProbeError('unexpected serial line: ' + line)
        return None

    def wait_for(self, matches, allowed, timeout=1.0, *, allow_fault=False):
        deadline = self.now() + timeout
        if self.keepalive and self.output_deadline is not None:
            deadline = min(deadline, self.output_deadline)
        while self.now() < deadline:
            event = self.next_event(deadline)
            if event is None:
                break
            if isinstance(event, dict):
                if not allowed(event):
                    raise ProbeError('unexpected reported output state: ' + json.dumps(event))
                if not allow_fault and event['fault'] != 0:
                    raise ProbeError('MCU fault: ' + hex(event['fault']))
            if matches(event):
                return event
            if isinstance(event, str):
                raise ProbeError('unexpected acknowledgement: ' + event)
        raise ProbeError('reply timeout in ' + self.phase)

    def exchange(self, command, reply, allowed, *, allow_fault=False):
        self.send(command)
        return self.wait_for(lambda event: event == reply, allowed,
                             allow_fault=allow_fault)

    def query(self, expected, allowed, *, allow_fault=False):
        self.send('status')
        row = self.wait_for(
            lambda event: isinstance(event, dict) and event['kind'] == 'STATUS',
            allowed, allow_fault=allow_fault)
        if not expected(row):
            raise ProbeError('queried state does not match the requested phase')
        return row

    def announce_active(self):
        self.announce(f'PWM_ACTIVE: {self.direction_name}10%，请读取固定在'
                      f'{self.measurement_output}的表笔；不要移动表笔。')

    def announce_stopped(self):
        self.announce('PWM_STOPPED: 已收到stop确认和禁用状态；仍不要连接电机。')

    def test_description(self):
        return f'NO MOTOR; {self.direction} 10% only'

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
            # 从请求输出之前开始计时，而不是收到ACK后再额外运行一段时间。
            self.output_started = self.now()
            self.output_deadline = self.output_started + self.duration_s
            transition = lambda row: armed_zero(row) or self.active_pwm(row)
            self.exchange(self.duty_command, self.duty_reply, transition)
            self.wait_for(lambda event: isinstance(event, dict) and self.active_pwm(event),
                          transition, timeout=0.3)
            self.active_samples = 1
            self.phase = 'active_pwm'
            self.announce_active()
            while self.now() < self.output_deadline:
                event = self.next_event(self.output_deadline)
                if event is None:
                    break
                if (not isinstance(event, dict) or not self.active_pwm(event)
                        or event['fault'] != 0):
                    raise ProbeError('PWM telemetry changed unexpectedly')
                self.active_samples += 1
            if self.now() - self.output_deadline > 0.2:
                raise ProbeError('host exceeded the output deadline')
            if self.active_samples < self.minimum_active_samples:
                raise ProbeError('insufficient PWM telemetry samples')
            completed = True
        except (Exception, KeyboardInterrupt) as exc:
            self.errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            # 正常结束、通信错误、Ctrl-C均先停止心跳，再请求停机；不自动清故障。
            self.keepalive = False
            self.phase = 'final_stop'
            known = lambda row: stopped(row) or armed_zero(row) or self.active_pwm(row)
            try:
                self.exchange('stop', 'OK,stopped', known, allow_fault=True)
                self.query(stopped, known, allow_fault=True)
                self.stop_verified = True
                self.announce_stopped()
            except (Exception, KeyboardInterrupt) as exc:
                self.errors.append('stop verification failed: ' + str(exc))
                self.announce('STOP_UNCONFIRMED: 请关闭驱动板开关并拔掉电池。')
        last = self.records[-1] if self.records else None
        stop_at = next((item['at_s'] for item in reversed(self.commands)
                        if item['command'] == 'stop'
                        and (self.output_started is None
                             or item['at_s'] >= self.output_started)), None)
        result = {
            'test': self.test_description(),
            'direction': self.direction,
            'measurement_output': self.measurement_output,
            'baudrate': 115200,
            'duty_permille': self.duty_permille,
            'requested_duration_s': self.duration_s,
            'startup_fragment_discarded': self.startup_fragment,
            'attempted_command_counts': {
                command: sum(item['command'] == command for item in self.commands)
                for command in sorted(self.allowed_commands)
            },
            'command_order_without_heartbeats': [
                item['command'] for item in self.commands if item['command'] != 'hb'
            ],
            'output_request_to_stop_s': (
                stop_at - self.output_started
                if self.output_started is not None and stop_at is not None else None),
            'telemetry_and_status_count': len(self.records),
            'active_samples': self.active_samples,
            'output_requested_at_s': self.output_started,
            'output_deadline_s': self.output_deadline,
            'software_stop_verified': self.stop_verified,
            'requires_power_disconnect': not self.stop_verified,
            'last_reported_state': last,
            'errors': self.errors,
            'passed': completed and self.stop_verified and not self.errors,
            'measurement_scope': self.measurement_scope,
        }
        return result


def require_no_motor(confirmed):
    if not confirmed:
        raise ProbeError('requires --confirm-no-motor; both motor connectors must be empty')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--direction', choices=('forward', 'reverse'), default='forward',
                        help='固定一个方向：forward测AO2，reverse测AO1，均为10%%、10秒')
    parser.add_argument('--confirm-no-motor', action='store_true',
                        help='确认Motor_A、Motor_B均拔下，STM32与驱动板供电未并联')
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
    # 不用DTR/RTS复位开发板，也不连接这些信号到BOOT/RESET。
    link.dtr = False
    link.rts = False
    link.port = args.port
    link.open()
    try:
        result = PWMProbe(link, direction=args.direction).run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
