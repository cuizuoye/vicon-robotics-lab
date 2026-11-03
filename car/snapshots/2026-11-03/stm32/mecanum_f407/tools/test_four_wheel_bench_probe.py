"""四轮串口工具离线回归；真实设备绝不打开。"""
import contextlib
import io
import unittest
from unittest import mock

from four_wheel_bench_probe import (FourWheelProbe, INITIAL_COUNT_TIMEOUT_S,
                                   NO_COUNT_CHANGE_TIMEOUT_S, WHEELS, main, parse_four_status)
from motor_a_no_motor_pwm_probe import ProbeError
from test_motor_a_no_motor_pwm_probe import Clock, FakeSerial


class FourFake(FakeSerial):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.selected = 0
        self.enc = [17, 23, 29, 31]
        if self.scenario == 'count_wrap':
            self.enc = [(1 << 31) - 3] * 4
        self.initial = list(self.enc)

    def update_encoder(self):
        if (self.en and self.duty and self.output_started is not None
                and self.scenario not in ('no_counts', 'no_counts_stale_cps')):
            i = self.selected - 1
            elapsed = self.clock.now() - self.output_started
            if self.scenario in ('stall_after_counts', 'stall_stale_cps', 'resume_after_gap'):
                elapsed = min(elapsed, 0.25)
            elif self.scenario == 'late_stall':
                elapsed = min(elapsed, 4.0)
            elif self.scenario == 'delayed_start':
                elapsed = max(0, elapsed - 0.6)
            steps = int(elapsed / 0.3) if self.scenario == 'slow_regular_counts' else int(elapsed * 100)
            if self.scenario == 'resume_after_gap' and self.clock.now() - self.output_started >= 1.2:
                steps += int((self.clock.now() - self.output_started - 1.2) * 100)
            count = self.initial[i] + steps * (1 if self.duty > 0 else -1)
            self.enc[i] = ((count + (1 << 31)) % (1 << 32)) - (1 << 31)

    def status(self, kind='TEL'):
        self.update_encoder()
        active = self.en and self.duty and self.output_started is not None
        elapsed = self.clock.now() - self.output_started if active else 0
        values = {'fw': 'four_bench_v1', 'ms': int(self.clock.now() * 1000), 'en': self.en,
                  'sel': self.selected, 'limit_ms': 5200 if self.en else 0, 'fault': f'0x{self.fault:08x}'}
        for i, w in enumerate(WHEELS, 1):
            w = w.lower()
            duty = self.duty if self.en and self.selected == i else 0
            p = {0: (0, 0), 100: (100, 90), -100: (90, 100)}.get(duty, (0, 0))
            values.update({f'd_{w}': duty, f'enc_{w}': self.enc[i-1],
                           f'cps_{w}': (100 if duty > 0 else -100) if duty and self.scenario != 'no_counts' else 0,
                           f'p_{w}1': p[0], f'p_{w}2': p[1]})
        if self.scenario == 'old_firmware':
            values['fw'] = 'front_enc_ro_v1'
        if self.scenario == 'preflight_cps':
            values['cps_rr'] = 1
        if elapsed > 0.6:
            if self.scenario == 'cross_encoder':
                other = WHEELS[self.selected % 4].lower()
                values[f'enc_{other}'] += 1
            elif self.scenario == 'wrong_ccr':
                values['p_fl1'] = 1
            elif self.scenario == 'wrong_selection':
                values['sel'] = self.selected % 4 + 1
        prefix = 'TEL4' if kind == 'TEL' else 'STATUS4'
        self.line(prefix + ',' + ','.join(f'{k}={v}' for k, v in values.items()))

    def write(self, data):
        command = data.decode('ascii').strip()
        self.sent.append((self.clock.now(), command))
        if command == 'stop':
            if self.scenario == 'ignore_final_stop' and self.armed_once:
                return len(data)
            self.update_encoder()
            self.en = self.selected = self.duty = 0
            self.line('OK,stopped_all')
        elif command == 'status':
            self.status('STATUS')
        elif command.startswith('arm '):
            if self.scenario == 'reject_arm':
                self.line('ERR,heartbeat_required')
            else:
                wheel = command.split()[1]
                self.selected = WHEELS.index(wheel) + 1
                self.en = 1
                self.armed_once = True
                self.line('OK,armed=' + wheel)
        elif command.startswith('duty '):
            if self.scenario == 'write_failure':
                raise OSError('offline write failure')
            _, wheel, duty = command.split()
            self.duty = int(duty)
            self.output_started = self.clock.now()
            self.initial = list(self.enc)
            self.line(f'OK,duty={wheel}:{-self.duty if self.scenario == "wrong_ack_sign" else self.duty}')
        elif command != 'hb':
            raise AssertionError('unexpected command: ' + command)
        return len(data)


class FourWheelTests(unittest.TestCase):
    def execute(self, scenario='normal', wheel='FL', sign=1, chunk=2048):
        clock = Clock()
        link = FourFake(clock, scenario, chunk)
        probe = FourWheelProbe(link, wheel=wheel, sign=sign, now=clock.now,
                               sleep=clock.sleep, announce=lambda message: None)
        return probe.run(), link, probe

    def test_default_never_arms(self):
        result, link, _ = self.execute(wheel=None)
        self.assertTrue(result['passed'], result)
        self.assertEqual([c for _, c in link.sent], ['stop', 'status', 'stop', 'status'])
        self.assertFalse(result['encoder_progress_guard']['enabled'])

    def test_brief_counts_then_stall_stops_all_eight_pulses_early(self):
        for wheel in WHEELS:
            for sign in (-1, 1):
                with self.subTest(wheel=wheel, sign=sign):
                    result, link, probe = self.execute('stall_after_counts', wheel=wheel, sign=sign)
                    self.assertFalse(result['passed'], result)
                    self.assertEqual(result['encoder_progress_guard']['trip_reason'],
                                     'selected encoder stopped changing')
                    self.assertTrue(result['software_stop_verified'])
                    self.assertGreater(abs(result['raw_count_delta'][wheel]), 0)
                    stop_at = [at for at, command in link.sent if command == 'stop'][-1]
                    self.assertLess(stop_at - link.output_started, 1.0)
                    deadline = probe.last_encoder_change_at + NO_COUNT_CHANGE_TIMEOUT_S
                    self.assertTrue(all(at < deadline for at, command in link.sent if command == 'hb'))
                    commands = [command for _, command in link.sent]
                    self.assertEqual(commands.count(probe.arm_command), 1)
                    self.assertEqual(commands.count(probe.duty_command), 1)
                    self.assertEqual(commands[-2:], ['stop', 'status'])
                    end = result['end_state']
                    self.assertEqual((end['en'], end['sel']), (0, 0))
                    self.assertTrue(all(end[f'p_{w.lower()}{i}'] == 0 for w in WHEELS for i in (1, 2)))

    def test_stall_guard_does_not_trust_nonzero_cps(self):
        for scenario, reason in (('stall_stale_cps', 'selected encoder stopped changing'),
                                 ('no_counts_stale_cps', 'selected encoder has no initial counts')):
            result, link, _ = self.execute(scenario, wheel='RL')
            self.assertFalse(result['passed'], result)
            self.assertEqual(result['encoder_progress_guard']['trip_reason'], reason)
            self.assertTrue(result['software_stop_verified'])
            stop_at = [at for at, command in link.sent if command == 'stop'][-1]
            self.assertLess(stop_at - link.output_started, 1.0)

    def test_initial_no_counts_deadline_stops_before_one_second(self):
        result, link, probe = self.execute('no_counts', wheel='RR')
        self.assertFalse(result['passed'], result)
        self.assertEqual(result['encoder_progress_guard']['trip_reason'],
                         'selected encoder has no initial counts')
        stop_at = [at for at, command in link.sent if command == 'stop'][-1]
        self.assertLessEqual(stop_at - link.output_started, INITIAL_COUNT_TIMEOUT_S + 0.07)
        self.assertTrue(all(at < probe.output_started + INITIAL_COUNT_TIMEOUT_S
                            for at, command in link.sent if command == 'hb'))

    def test_late_stall_and_later_resume_do_not_complete_five_seconds(self):
        for scenario, latest_stop_s in (('late_stall', 4.8), ('resume_after_gap', 1.0)):
            result, link, _ = self.execute(scenario, wheel='RL')
            self.assertFalse(result['passed'], result)
            self.assertEqual(result['encoder_progress_guard']['trip_reason'],
                             'selected encoder stopped changing')
            self.assertTrue(result['software_stop_verified'])
            stop_at = [at for at, command in link.sent if command == 'stop'][-1]
            self.assertLess(stop_at - link.output_started, latest_stop_s)

    def test_delayed_start_and_slow_regular_counts_are_allowed(self):
        for scenario in ('delayed_start', 'slow_regular_counts'):
            result, _, _ = self.execute(scenario, wheel='RL')
            self.assertTrue(result['passed'], result)
            self.assertTrue(result['encoder_progress_guard']['progress_observed'])
            self.assertIsNone(result['encoder_progress_guard']['trip_reason'])

    def test_count_wrap_does_not_trigger_no_progress(self):
        for sign in (-1, 1):
            result, _, _ = self.execute('count_wrap', wheel='RR', sign=sign)
            self.assertTrue(result['passed'], result)
            self.assertGreater(result['raw_count_delta']['RR'] * sign, 0)

    def test_count_arriving_after_deadline_cannot_refresh_progress(self):
        clock = Clock()
        probe = FourWheelProbe(None, wheel='RL', now=clock.now, sleep=clock.sleep)
        probe.output_started = 0.0
        probe.last_encoder_count = 1
        probe.last_encoder_change_at = 0.2
        probe.encoder_progress_seen = True
        clock.value = 0.2 + NO_COUNT_CHANGE_TIMEOUT_S + 0.001
        with self.assertRaisesRegex(ProbeError, 'stopped changing'):
            probe.observe_encoder_progress({'enc_rl': 2})
        self.assertEqual(probe.last_encoder_count, 1)

    def test_heartbeat_after_count_deadline_is_not_written(self):
        clock = Clock()
        link = FourFake(clock)
        probe = FourWheelProbe(link, wheel='RR', now=clock.now, sleep=clock.sleep)
        probe.output_started = 0.0
        probe.last_encoder_count = 0
        probe.last_encoder_change_at = 0.0
        probe.keepalive = True
        probe.last_send = INITIAL_COUNT_TIMEOUT_S - 0.03
        clock.value = probe.last_send
        with self.assertRaisesRegex(ProbeError, 'no initial counts'):
            probe.send('hb')
        self.assertEqual(link.sent, [])

    def test_eight_single_pulses_stop_and_isolate(self):
        for wheel in WHEELS:
            for sign in (-1, 1):
                with self.subTest(wheel=wheel, sign=sign):
                    result, link, probe = self.execute(wheel=wheel, sign=sign)
                    self.assertTrue(result['passed'], result)
                    self.assertGreaterEqual(result['active_samples'], 40)
                    self.assertGreater(result['raw_count_delta'][wheel] * sign, 0)
                    self.assertTrue(all(v == 0 for w, v in result['raw_count_delta'].items() if w != wheel))
                    commands = [c for _, c in link.sent]
                    self.assertEqual(commands.count(probe.arm_command), 1)
                    self.assertEqual(commands.count(probe.duty_command), 1)
                    self.assertEqual(commands[-2:], ['stop', 'status'])
                    self.assertTrue(all(after[0] - before[0] >= 0.06 - 1e-9
                                        for before, after in zip(link.sent, link.sent[1:])))
                    self.assertTrue(all(at < probe.output_deadline for at, c in link.sent if c == 'hb'))
                    self.assertIsNone(result['physical_stop_confirmed'])
                    self.assertIsNone(result['counts_per_output_revolution'])

    def test_fragmented_lines(self):
        result, _, _ = self.execute(chunk=7, wheel='RR', sign=-1)
        self.assertTrue(result['passed'], result)

    def test_preflight_failures_never_request_duty(self):
        for scenario in ('old_firmware', 'preflight_fault', 'preflight_cps', 'reject_arm'):
            result, link, _ = self.execute(scenario)
            self.assertFalse(result['passed'])
            self.assertFalse(any(c.startswith('duty') for _, c in link.sent))

    def test_runtime_failures_stop_without_retry_or_direction_change(self):
        scenarios = ('lost_telemetry', 'reset', 'fault', 'wrong_duty', 'wrong_sign',
                     'wrong_ack_sign', 'corrupt_line', 'host_stall', 'keyboard_interrupt',
                     'write_failure', 'wrong_ccr', 'wrong_selection', 'cross_encoder', 'no_counts')
        for scenario in scenarios:
            with self.subTest(scenario=scenario):
                result, link, probe = self.execute(scenario, wheel='FR', sign=-1)
                self.assertFalse(result['passed'], result)
                self.assertTrue(result['software_stop_verified'], result)
                commands = [c for _, c in link.sent]
                self.assertEqual(commands.count(probe.arm_command), 1)
                self.assertEqual(commands.count(probe.duty_command), 1)
                self.assertTrue(set(commands) <= probe.allowed_commands)

    def test_missing_stop_confirmation_requires_disconnect(self):
        result, _, _ = self.execute('ignore_final_stop')
        self.assertFalse(result['passed'])
        self.assertFalse(result['software_stop_verified'])
        self.assertTrue(result['requires_power_disconnect'])

    def test_single_use_and_forbidden_commands(self):
        _, link, probe = self.execute(wheel='RL')
        before = list(link.sent)
        with self.assertRaises(ProbeError):
            probe.run()
        for command in ('arm ALL A5', 'duty RL 200', 'duty RR 100', 'clear 5A', 'zero', 'speed 100', 'duty RL -100'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        self.assertEqual(before, link.sent)

    def test_cli_checks_before_serial_open(self):
        for flags in ([], ['--wheel', 'FL'], ['--confirm-battery-disconnected', '--wheel', 'FR'],
                      ['--wheel', 'RR', '--confirm-suspended', '--confirm-stationary'],
                      ['--wheel', 'ALL'], ['--sign', '2'], ['--seconds', '30']):
            with mock.patch('four_wheel_bench_probe.serial.Serial') as constructor:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(['/dev/offline-only'] + flags)
                constructor.assert_not_called()

    def test_schema_limits_and_line_capacity(self):
        _, _, probe = self.execute(wheel=None)
        row = {k: v for k, v in probe.records[-1].items() if k not in ('kind', 'raw_kind', 'phase')}
        row.update(ms=0xffffffff, fault='0xffffffff')
        for wheel in WHEELS:
            row['enc_' + wheel.lower()] = -(1 << 31)
            row['cps_' + wheel.lower()] = -(1 << 31)
        line = 'STATUS4,' + ','.join(f'{k}={v}' for k, v in row.items())
        self.assertLess(len(line + '\r\n'), 512)
        self.assertEqual(parse_four_status(line)['enc_rr'], -(1 << 31))
        for bad in (line + ',en=0', line.replace('en=0', 'en=1'),
                    line.replace('p_fl1=0', 'p_fl1=1'), line.replace('four_bench_v1', 'dual_v2'),
                    line.replace('enc_rl=-2147483648', 'enc_rl=2147483648'),
                    line.replace(',cps_rr=-2147483648', '')):
            with self.assertRaises(ValueError):
                parse_four_status(bad)


if __name__ == '__main__':
    unittest.main()
