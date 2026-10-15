"""纯离线串口/时钟替身；这些测试绝不打开真实串口。"""

import contextlib
import io
import unittest
from unittest import mock

from motor_a_no_motor_pwm_probe import PWMProbe, ProbeError, main, require_no_motor


class Clock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class FakeSerial:
    def __init__(self, clock, scenario='normal', chunk=2048):
        self.clock = clock
        self.scenario = scenario
        self.chunk = chunk
        self.rx = bytearray(b'partial startup line\n')
        self.sent = []
        self.en = 0
        self.hiz = 1
        self.duty = 0
        self.fault = 2 if scenario == 'preflight_fault' else 0
        self.next_tel = 0.0
        self.output_started = None
        self.triggered = False
        self.armed_once = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def line(self, text):
        self.rx.extend((text + '\r\n').encode('ascii'))

    def status(self, kind='TEL'):
        self.line(f'{kind},ms={int(self.clock.now()*1000)},en={self.en},'
                  f'hiz={self.hiz},mode=duty,target_cps=0,duty={self.duty},'
                  f'enc=17,cps=0,fault=0x{self.fault:08x}')

    def write(self, data):
        command = data.decode('ascii').strip()
        self.sent.append((self.clock.now(), command))
        if command == 'stop':
            if self.scenario == 'ignore_final_stop' and self.armed_once:
                return len(data)
            self.en, self.hiz, self.duty = 0, 1, 0
            self.line('OK,stopped')
        elif command == 'status':
            self.status('STATUS')
        elif command == 'arm A5':
            if self.scenario == 'reject_arm':
                self.line('ERR,heartbeat_required')
            else:
                self.en = 1
                self.armed_once = True
                self.line('OK,armed')
        elif command in {'duty 100', 'duty -100'}:
            if self.scenario == 'write_failure':
                raise OSError('simulated USB failure during duty write')
            self.hiz, self.duty = 0, int(command.split()[1])
            self.output_started = self.clock.now()
            reply_duty = -self.duty if self.scenario == 'wrong_ack_sign' else self.duty
            self.line(f'OK,duty={reply_duty}')
        elif command != 'hb':
            raise AssertionError('test sent an unexpected command')
        return len(data)

    def read(self, count):
        # 有缓冲数据时真实read不会等待完整timeout；分片不等于限速700B/s。
        self.clock.sleep(0.001 if self.rx else 0.01)
        elapsed = (self.clock.now() - self.output_started
                   if self.output_started is not None else 0)
        if abs(self.duty) == 100 and elapsed > 0.6 and not self.triggered:
            self.triggered = True
            if self.scenario == 'reset':
                self.line('READY,motor_a_safe_off')
                self.en, self.hiz, self.duty = 0, 1, 0
            elif self.scenario == 'fault':
                self.fault = 1
                self.en, self.hiz, self.duty = 0, 1, 0
            elif self.scenario == 'wrong_duty':
                self.duty = 200
            elif self.scenario == 'wrong_sign':
                self.duty = -self.duty
            elif self.scenario == 'corrupt_line':
                self.line('unrecognized data')
            elif self.scenario == 'host_stall':
                self.clock.sleep(0.6)
            elif self.scenario == 'keyboard_interrupt':
                raise KeyboardInterrupt('simulated Ctrl-C')
        if self.clock.now() >= self.next_tel:
            self.next_tel = self.clock.now() + 0.1
            if not (self.scenario == 'lost_telemetry' and abs(self.duty) == 100
                    and elapsed > 0.6):
                self.status()
        count = min(count, self.chunk, len(self.rx))
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result


class PWMProbeTests(unittest.TestCase):
    direction = 'forward'
    duty = 100

    def execute(self, scenario='normal', chunk=2048):
        clock = Clock()
        link = FakeSerial(clock, scenario, chunk)
        messages = []
        probe = PWMProbe(link, direction=self.direction, now=clock.now, sleep=clock.sleep,
                         announce=messages.append)
        return probe.run(), link, messages

    def test_confirmation_required(self):
        with self.assertRaises(ProbeError):
            require_no_motor(False)
        require_no_motor(True)

    def test_cli_without_confirmation_never_opens_serial(self):
        with mock.patch('motor_a_no_motor_pwm_probe.serial.Serial') as constructor:
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    main(['/dev/offline-test-only', '--direction', self.direction])
            self.assertEqual(error.exception.code, 2)
            constructor.assert_not_called()

    def test_commands_outside_whitelist_never_written(self):
        clock = Clock()
        link = FakeSerial(clock)
        probe = PWMProbe(link, direction=self.direction, now=clock.now, sleep=clock.sleep)
        for command in (f'duty {-self.duty}', 'duty 200', 'duty -200',
                        'speed 100', 'clear 5A', 'brake'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        self.assertEqual(link.sent, [])

    def test_normal_fixed_selected_pwm_and_final_stop(self):
        result, link, _ = self.execute()
        self.assertTrue(result['passed'], result)
        self.assertGreaterEqual(result['active_samples'], 80)
        self.assertTrue(result['software_stop_verified'])
        self.assertEqual((link.en, link.hiz, link.duty), (0, 1, 0))
        commands = [cmd for _, cmd in link.sent]
        self.assertEqual(commands.count(f'duty {self.duty}'), 1)
        self.assertNotIn(f'duty {-self.duty}', commands)
        self.assertEqual(result['direction'], self.direction)
        self.assertEqual(result['duty_permille'], self.duty)
        output = 'AO2' if self.direction == 'forward' else 'AO1'
        self.assertEqual(result['measurement_output'], output)
        self.assertEqual(commands.count('arm A5'), 1)
        self.assertEqual(commands[-2:], ['stop', 'status'])
        self.assertFalse(any(cmd.startswith(('clear', 'speed', 'brake', 'zero'))
                             for cmd in commands))
        deadline = result['output_deadline_s']
        self.assertTrue(all(at < deadline for at, cmd in link.sent if cmd == 'hb'))
        stop_time = [at for at, cmd in link.sent if cmd == 'stop'][-1]
        self.assertLessEqual(stop_time, deadline + 0.04)

    def test_chunked_serial_lines(self):
        result, _, _ = self.execute(chunk=7)
        self.assertTrue(result['passed'], result)

    def test_preflight_fault_prevents_arm(self):
        result, link, _ = self.execute('preflight_fault')
        self.assertFalse(result['passed'])
        self.assertNotIn('arm A5', [cmd for _, cmd in link.sent])
        self.assertNotIn(f'duty {self.duty}', [cmd for _, cmd in link.sent])

    def test_arm_rejection_prevents_pwm(self):
        result, link, _ = self.execute('reject_arm')
        self.assertFalse(result['passed'])
        self.assertNotIn(f'duty {self.duty}', [cmd for _, cmd in link.sent])
        self.assertTrue(result['software_stop_verified'])

    def test_lost_telemetry_stops_early(self):
        result, link, _ = self.execute('lost_telemetry')
        self.assertFalse(result['passed'])
        self.assertTrue(result['software_stop_verified'])
        self.assertLess(link.sent[-1][0] - link.output_started, 2)

    def test_reset_stops_early(self):
        result, _, _ = self.execute('reset')
        self.assertFalse(result['passed'])
        self.assertTrue(any('reset' in error for error in result['errors']))

    def test_fault_is_not_automatically_cleared(self):
        result, link, _ = self.execute('fault')
        self.assertFalse(result['passed'])
        self.assertTrue(result['software_stop_verified'])
        self.assertEqual(link.fault, 1)
        self.assertFalse(any(cmd.startswith('clear') for _, cmd in link.sent))

    def test_wrong_duty_is_rejected(self):
        result, _, _ = self.execute('wrong_duty')
        self.assertFalse(result['passed'])
        self.assertTrue(result['software_stop_verified'])

    def test_wrong_reported_direction_is_rejected(self):
        result, _, _ = self.execute('wrong_sign')
        self.assertFalse(result['passed'])
        self.assertTrue(result['software_stop_verified'])

    def test_wrong_acknowledgement_direction_is_rejected(self):
        result, link, _ = self.execute('wrong_ack_sign')
        self.assertFalse(result['passed'])
        self.assertTrue(result['software_stop_verified'])
        self.assertEqual([cmd for _, cmd in link.sent].count(f'duty {self.duty}'), 1)

    def test_corrupt_serial_line_is_rejected(self):
        result, _, _ = self.execute('corrupt_line')
        self.assertFalse(result['passed'])
        self.assertTrue(result['software_stop_verified'])

    def test_stalled_host_does_not_renew_heartbeat(self):
        result, link, _ = self.execute('host_stall')
        self.assertFalse(result['passed'])
        last_hb = [at for at, cmd in link.sent if cmd == 'hb'][-1]
        self.assertLess(last_hb, link.output_started + 0.8)

    def test_keyboard_interrupt_requests_stop(self):
        result, _, _ = self.execute('keyboard_interrupt')
        self.assertFalse(result['passed'])
        self.assertTrue(result['software_stop_verified'])

    def test_unconfirmed_stop_requires_power_disconnect(self):
        result, _, messages = self.execute('ignore_final_stop')
        self.assertFalse(result['passed'])
        self.assertFalse(result['software_stop_verified'])
        self.assertTrue(result['requires_power_disconnect'])
        self.assertTrue(any('STOP_UNCONFIRMED' in message for message in messages))

    def test_write_failure_requests_stop(self):
        result, _, _ = self.execute('write_failure')
        self.assertFalse(result['passed'])
        self.assertTrue(result['software_stop_verified'])


class ReversePWMProbeTests(PWMProbeTests):
    """对反向重复全部正常/故障场景，不降低已有正向工具的保护。"""

    direction = 'reverse'
    duty = -100


class DirectionValidationTests(unittest.TestCase):
    def test_invalid_direction_rejected_before_writing(self):
        clock = Clock()
        link = FakeSerial(clock)
        with self.assertRaises(ProbeError):
            PWMProbe(link, direction='both', now=clock.now, sleep=clock.sleep)
        self.assertEqual(link.sent, [])

    def test_cli_invalid_direction_never_opens_serial(self):
        with mock.patch('motor_a_no_motor_pwm_probe.serial.Serial') as constructor:
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    main(['/dev/offline-test-only', '--direction', 'both',
                          '--confirm-no-motor'])
            self.assertEqual(error.exception.code, 2)
            constructor.assert_not_called()


if __name__ == '__main__':
    unittest.main()
