"""双通道工具离线回归；只使用模拟时钟和串口，不打开设备。"""

import contextlib
import io
import unittest
from unittest import mock

from dual_motor_bench_probe import DualBenchProbe, TESTS, main, parse_dual_status, stopped_dual
from motor_a_no_motor_pwm_probe import ProbeError
from test_motor_a_no_motor_pwm_probe import Clock, FakeSerial


class DualFakeSerial(FakeSerial):
    def __init__(self, clock, scenario='normal', chunk=2048):
        super().__init__(clock, scenario, chunk)
        self.mask = 0
        self.a = self.b = 0
        self.limit_ms = 0

    def status(self, kind='TEL'):
        if self.en == 0:
            self.mask = self.a = self.b = 0
            self.limit_ms = 0
        elif self.output_started is not None:
            if self.mask == 2:
                self.b = self.duty
            else:
                self.a = self.duty
        if self.scenario == 'old_firmware':
            super().status(kind)
            return
        mask = self.mask
        a, b = self.a, self.b
        enc_a, cps_a = 17, 50 if self.scenario == 'preflight_cps' else 0
        if self.output_started is not None and self.clock.now() - self.output_started > 0.6 and self.en:
            if self.scenario == 'cross_channel':
                a = 100  # B测试时，A输出意外非零。
            elif self.scenario == 'wrong_channel':
                mask, a, b = 1, 100, 0
            elif self.scenario == 'a_count_changed':
                enc_a = 18
        prefix = 'TEL2' if kind == 'TEL' else 'STATUS2'
        limit_ms = self.limit_ms
        if self.output_started is not None and self.en and self.scenario == 'wrong_limit':
            limit_ms = 2200 if limit_ms == 5200 else 5200
        self.line(f'{prefix},fw=dual_v2,ms={int(self.clock.now()*1000)},en={self.en},'
                  f'mask={mask},a={a},b={b},az={int(a == 0)},bz={int(b == 0)},'
                  f'limit_ms={limit_ms},'
                  f'enc_a={enc_a},cps_a={cps_a},fault=0x{self.fault:08x}')

    def write(self, data):
        command = data.decode('ascii').strip()
        self.sent.append((self.clock.now(), command))
        if command == 'stop':
            if self.scenario == 'ignore_final_stop' and self.armed_once:
                return len(data)
            self.en, self.hiz, self.duty = 0, 1, 0
            self.mask = self.a = self.b = 0
            self.limit_ms = 0
            self.line('OK,stopped_all')
        elif command == 'status':
            self.status('STATUS')
        elif command.startswith('arm '):
            if self.scenario == 'reject_arm':
                self.line('ERR,heartbeat_required')
            else:
                channel = command.split()[1]
                self.en = 1
                self.mask = {'A': 1, 'B': 2, 'AB': 3}[channel]
                long_observe = command == 'arm AB A5 5s'
                self.limit_ms = 5200 if long_observe else 2200
                self.armed_once = True
                self.line('OK,armed=' + ('AB5' if long_observe else channel))
        elif command.startswith('duty '):
            if self.scenario == 'write_failure':
                raise OSError('simulated USB write failure')
            _, channel, *values = command.split()
            if channel == 'A':
                self.a, self.b = int(values[0]), 0
            elif channel == 'B':
                self.a, self.b = 0, int(values[0])
            else:
                self.a, self.b = map(int, values)
            self.duty = self.a or self.b
            self.hiz = 0
            self.output_started = self.clock.now()
            reply_a, reply_b = self.a, self.b
            if self.scenario == 'wrong_ack_sign':
                reply_a, reply_b = -reply_a, -reply_b
            self.line(f'OK,duty=A:{reply_a},B:{reply_b}')
        elif command != 'hb':
            raise AssertionError('unexpected simulated command: ' + command)
        return len(data)


class DualBenchTests(unittest.TestCase):
    def execute(self, test='b-positive', scenario='normal', chunk=2048):
        clock = Clock()
        link = DualFakeSerial(clock, scenario, chunk)
        messages = []
        probe = DualBenchProbe(link, test=test, now=clock.now, sleep=clock.sleep,
                               announce=messages.append)
        return probe.run(), link, probe, messages

    def test_status_only_never_arms_or_heartbeats(self):
        result, link, _, _ = self.execute('status')
        self.assertTrue(result['passed'], result)
        self.assertEqual([cmd for _, cmd in link.sent], ['stop', 'status', 'stop', 'status'])
        self.assertTrue(stopped_dual(result['last_reported_state']))

    def test_all_fixed_pulses_one_arm_one_duty_then_stop_both(self):
        for test, (channel, mask, a, b) in TESTS.items():
            with self.subTest(test=test):
                result, link, probe, _ = self.execute(test)
                self.assertTrue(result['passed'], result)
                self.assertEqual((result['duty_a_permille'], result['duty_b_permille']), (a, b))
                self.assertGreaterEqual(result['active_samples'], probe.minimum_active_samples)
                self.assertGreaterEqual(result['output_request_to_stop_s'], probe.duration_s)
                self.assertLessEqual(result['output_request_to_stop_s'], probe.duration_s + 0.04)
                commands = [cmd for _, cmd in link.sent]
                self.assertEqual(commands.count(probe.arm_command), 1)
                self.assertEqual(commands.count(probe.duty_command), 1)
                self.assertEqual(commands[-2:], ['stop', 'status'])
                self.assertEqual((link.en, link.a, link.b), (0, 0, 0))
                self.assertTrue(all(at < probe.output_deadline for at, cmd in link.sent if cmd == 'hb'))
                self.assertTrue(all(row['mask'] == mask and row['a'] == a and row['b'] == b
                                    for row in result['raw_samples'] if row['phase'] == 'active_pwm'))
                self.assertIsNone(result['physical_stop_confirmed'])

    def test_chunked_lines(self):
        result, _, _, _ = self.execute(chunk=7)
        self.assertTrue(result['passed'], result)

    def test_body_forward_uses_opposite_raw_polarities_in_one_command(self):
        result, link, _, _ = self.execute('both-forward')
        self.assertTrue(result['passed'], result)
        commands = [cmd for _, cmd in link.sent]
        self.assertEqual(commands.count('arm AB A5'), 1)
        self.assertEqual(commands.count('duty AB 100 -100'), 1)
        self.assertNotIn('duty AB 100 100', commands)
        self.assertEqual((result['duty_a_permille'], result['duty_b_permille']), (100, -100))
        self.assertTrue(stopped_dual(result['last_reported_state']))

    def test_long_forward_observation_has_explicit_arm_and_five_second_deadline(self):
        result, link, _, _ = self.execute('both-forward-observe')
        self.assertTrue(result['passed'], result)
        commands = [cmd for _, cmd in link.sent]
        self.assertEqual(commands.count('arm AB A5 5s'), 1)
        self.assertNotIn('arm AB A5', commands)
        self.assertEqual(commands.count('duty AB 100 -100'), 1)
        self.assertEqual(result['requested_duration_s'], 5.0)
        self.assertGreaterEqual(result['active_samples'], 45)
        self.assertLessEqual(result['output_request_to_stop_s'], 5.04)

    def test_reported_limit_must_match_selected_short_or_long_test(self):
        for test in ('b-positive', 'both-forward-observe'):
            result, link, _, _ = self.execute(test, scenario='wrong_limit')
            self.assertFalse(result['passed'], result)
            self.assertTrue(result['software_stop_verified'], result)
            self.assertEqual(sum(cmd.startswith('duty') for _, cmd in link.sent), 1)

    def test_wrong_firmware_prevents_enable(self):
        result, link, _, _ = self.execute(scenario='old_firmware')
        self.assertFalse(result['passed'])
        self.assertFalse(any(cmd.startswith(('arm', 'duty', 'hb')) for _, cmd in link.sent))

    def test_fault_or_rejected_arm_never_writes_duty(self):
        for scenario in ('preflight_fault', 'reject_arm', 'preflight_cps'):
            with self.subTest(scenario=scenario):
                result, link, _, _ = self.execute(scenario=scenario)
                self.assertFalse(result['passed'])
                self.assertFalse(any(cmd.startswith('duty') for _, cmd in link.sent))

    def test_fault_injections_stop_without_retry_clear_or_change_channel(self):
        for scenario in ('lost_telemetry', 'reset', 'fault', 'wrong_duty', 'wrong_sign',
                         'wrong_ack_sign', 'corrupt_line', 'host_stall', 'keyboard_interrupt',
                         'write_failure', 'wrong_channel', 'cross_channel', 'a_count_changed'):
            with self.subTest(scenario=scenario):
                result, link, probe, _ = self.execute(scenario=scenario)
                self.assertFalse(result['passed'], result)
                self.assertTrue(result['software_stop_verified'], result)
                commands = [cmd for _, cmd in link.sent]
                self.assertEqual(commands.count(probe.arm_command), 1)
                self.assertEqual(commands.count(probe.duty_command), 1)
                self.assertTrue(set(commands) <= probe.allowed_commands)
                self.assertFalse(any(cmd.startswith(('clear', 'speed', 'zero')) for cmd in commands))

    def test_stall_does_not_renew_heartbeat(self):
        result, link, _, _ = self.execute(scenario='host_stall')
        self.assertFalse(result['passed'])
        self.assertLess([at for at, cmd in link.sent if cmd == 'hb'][-1], link.output_started + 0.8)

    def test_missing_stop_confirmation_requires_disconnect(self):
        result, _, _, messages = self.execute(scenario='ignore_final_stop')
        self.assertFalse(result['passed'])
        self.assertTrue(result['requires_power_disconnect'])
        self.assertTrue(any('STOP_UNCONFIRMED' in msg for msg in messages))

    def test_single_use_and_motion_whitelist(self):
        _, link, probe, _ = self.execute()
        previous = list(link.sent)
        with self.assertRaises(ProbeError):
            probe.run()
        for command in ('arm A A5', 'arm A5', 'duty B -100', 'duty B 200', 'clear 5A'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        self.assertEqual(previous, link.sent)

    def test_duplicate_motion_not_written(self):
        clock = Clock()
        link = DualFakeSerial(clock)
        probe = DualBenchProbe(link, test='b-positive', now=clock.now, sleep=clock.sleep)
        probe.phase = 'request_pwm'
        probe.send(probe.duty_command)
        previous = list(link.sent)
        with self.assertRaises(ProbeError):
            probe.send(probe.duty_command)
        self.assertEqual(previous, link.sent)

    def test_cli_confirmation_missing_never_opens_serial(self):
        flags = ['--confirm-suspended', '--confirm-two-motors',
                 '--confirm-channel-isolation', '--confirm-stationary']
        for missing in range(len(flags)):
            with mock.patch('dual_motor_bench_probe.serial.Serial') as constructor:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(['/dev/offline-test-only', '--test', 'b-positive'] +
                         [flag for i, flag in enumerate(flags) if i != missing])
                constructor.assert_not_called()
        with mock.patch('dual_motor_bench_probe.serial.Serial') as constructor:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(['/dev/offline-test-only'])
            constructor.assert_not_called()

    def test_cli_cannot_change_duration_or_amplitude(self):
        for option in ('--duration', '--duty', '--direction'):
            with mock.patch('dual_motor_bench_probe.serial.Serial') as constructor:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(['/dev/offline-test-only', '--confirm-battery-disconnected', option, '10'])
                constructor.assert_not_called()

    def test_expired_deadline_after_spacing_no_duty_write(self):
        clock = Clock()
        link = DualFakeSerial(clock)
        probe = DualBenchProbe(link, test='b-positive', now=clock.now, sleep=clock.sleep)
        probe.phase = 'request_pwm'
        probe.last_send = 0.0
        probe.output_deadline = 0.02
        with self.assertRaises(ProbeError):
            probe.send(probe.duty_command)
        self.assertEqual(link.sent, [])

    def test_strict_parser_schema_ranges_and_channel_isolation(self):
        line = ('STATUS2,fw=dual_v2,ms=100,en=0,mask=0,a=0,b=0,az=1,bz=1,limit_ms=0,'
                'enc_a=-1,cps_a=0,fault=0x00000000')
        self.assertTrue(stopped_dual(parse_dual_status(line)))
        malformed = [line + ',a=0', line + ',extra=0', line.replace('dual_v2', 'dual_v1'),
                     line.replace('ms=100', 'ms=-1'), line.replace('a=0,b=0', 'a=100,b=0'),
                     line.replace('az=1', 'az=0'), line.replace('fault=0x00000000', 'fault=-1'),
                     line.replace('enc_a=-1', 'enc_a=2147483648'),
                     line.replace('limit_ms=0', 'limit_ms=5200'),
                     line.replace('en=0,mask=0,a=0,b=0,az=1', 'en=1,mask=2,a=100,b=0,az=0')]
        for sample in malformed:
            with self.subTest(sample=sample), self.assertRaises(ValueError):
                parse_dual_status(sample)


if __name__ == '__main__':
    unittest.main()
