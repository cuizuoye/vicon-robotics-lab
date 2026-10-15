"""悬空单电机脉冲工具的离线测试；绝不打开硬件串口。"""

import contextlib
import io
import unittest
from unittest import mock

from motor_a_no_motor_pwm_probe import ProbeError
from motor_a_unloaded_pulse import UnloadedPulseProbe, main, require_conditions
from test_motor_a_no_motor_pwm_probe import Clock, FakeSerial


class UnloadedPulseTests(unittest.TestCase):
    def execute(self, scenario='normal', chunk=2048):
        clock = Clock()
        link = FakeSerial(clock, scenario, chunk)
        messages = []
        probe = UnloadedPulseProbe(link, now=clock.now, sleep=clock.sleep,
                                   announce=messages.append)
        return probe.run(), link, messages, probe

    def test_normal_one_positive_two_second_request_and_stop(self):
        result, link, messages, _ = self.execute()
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['requested_duration_s'], 2.0)
        self.assertEqual(result['duty_permille'], 100)
        self.assertGreaterEqual(result['active_samples'], 15)
        self.assertLessEqual(result['output_request_to_stop_s'], 2.04)
        self.assertGreaterEqual(result['output_request_to_stop_s'], 2.0)
        commands = [command for _, command in link.sent]
        self.assertEqual(commands.count('arm A5'), 1)
        self.assertEqual(commands.count('duty 100'), 1)
        self.assertEqual(commands[-2:], ['stop', 'status'])
        self.assertTrue(set(commands) <= {'stop', 'status', 'hb', 'arm A5', 'duty 100'})
        self.assertTrue(all(at < result['output_deadline_s'] for at, command in link.sent
                            if command == 'hb'))
        self.assertEqual((link.en, link.hiz, link.duty), (0, 1, 0))
        self.assertIsNone(result['physical_stop_confirmed'])
        self.assertIsNone(result['physical_rotation_observed'])
        self.assertTrue(any('独立确认' in message for message in messages))

    def test_chunked_serial(self):
        result, _, _, _ = self.execute(chunk=7)
        self.assertTrue(result['passed'], result)

    def test_raw_encoder_trace_does_not_claim_feedback_verified(self):
        result, link, _, _ = self.execute()
        samples = result['encoder_samples']
        self.assertEqual(len(samples), result['telemetry_and_status_count'])
        self.assertTrue(all(set(row) == {'phase', 'kind', 'ms', 'enc', 'cps'}
                            for row in samples))
        self.assertTrue(any(row['phase'] == 'active_pwm' for row in samples))
        self.assertTrue(all(row['enc'] == 17 and row['cps'] == 0 for row in samples))
        self.assertTrue(result['encoder_input_validation_required'])
        self.assertIsNone(result['physical_rotation_observed'])
        self.assertIsNone(result['physical_stop_confirmed'])
        self.assertEqual([command for _, command in link.sent].count('duty 100'), 1)

    def test_preflight_fault_prevents_enable(self):
        result, link, _, _ = self.execute('preflight_fault')
        self.assertFalse(result['passed'])
        commands = [command for _, command in link.sent]
        self.assertNotIn('arm A5', commands)
        self.assertNotIn('duty 100', commands)

    def test_rejected_arm_prevents_pwm(self):
        result, link, _, _ = self.execute('reject_arm')
        self.assertFalse(result['passed'])
        self.assertNotIn('duty 100', [command for _, command in link.sent])
        self.assertTrue(result['software_stop_verified'])

    def test_active_faults_stop_without_retry_or_clear(self):
        for scenario in ('lost_telemetry', 'reset', 'fault', 'wrong_duty', 'wrong_sign',
                         'wrong_ack_sign', 'corrupt_line', 'host_stall',
                         'keyboard_interrupt', 'write_failure'):
            with self.subTest(scenario=scenario):
                result, link, _, _ = self.execute(scenario)
                self.assertFalse(result['passed'], result)
                self.assertTrue(result['software_stop_verified'], result)
                commands = [command for _, command in link.sent]
                self.assertEqual(commands.count('duty 100'), 1)
                self.assertEqual(commands.count('arm A5'), 1)
                self.assertFalse(any(command.startswith(('clear', 'speed'))
                                     or command == 'duty -100' for command in commands))

    def test_host_stall_does_not_renew_heartbeat(self):
        result, link, _, _ = self.execute('host_stall')
        last_hb = [at for at, command in link.sent if command == 'hb'][-1]
        self.assertLess(last_hb, link.output_started + 0.8)
        self.assertFalse(result['passed'])

    def test_unconfirmed_stop_requires_physical_disconnect(self):
        result, _, messages, _ = self.execute('ignore_final_stop')
        self.assertFalse(result['passed'])
        self.assertFalse(result['software_stop_verified'])
        self.assertTrue(result['requires_power_disconnect'])
        self.assertTrue(any('STOP_UNCONFIRMED' in message for message in messages))

    def test_probe_cannot_run_twice(self):
        _, link, _, probe = self.execute()
        previous = list(link.sent)
        with self.assertRaises(ProbeError):
            probe.run()
        self.assertEqual(link.sent, previous)

    def test_motion_command_duplicates_or_wrong_phase_never_written(self):
        clock = Clock()
        link = FakeSerial(clock)
        probe = UnloadedPulseProbe(link, now=clock.now, sleep=clock.sleep)
        for command in ('arm A5', 'duty 100', 'duty -100', 'duty 200', 'clear 5A'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        self.assertEqual(link.sent, [])
        probe.phase = 'request_pwm'
        probe.send('duty 100')
        with self.assertRaises(ProbeError):
            probe.send('duty 100')
        self.assertEqual(len(link.sent), 1)

    def test_each_missing_confirmation_prevents_serial_open(self):
        flags = ['--confirm-suspended', '--confirm-single-motor-a', '--confirm-stationary']
        for missing in range(3):
            with self.subTest(missing=missing):
                with mock.patch('motor_a_unloaded_pulse.serial.Serial') as constructor:
                    with contextlib.redirect_stderr(io.StringIO()):
                        with self.assertRaises(SystemExit) as error:
                            main(['/dev/offline-test-only'] +
                                 [flag for index, flag in enumerate(flags) if index != missing])
                    self.assertEqual(error.exception.code, 2)
                    constructor.assert_not_called()
        require_conditions(True, True, True)

    def test_expired_deadline_after_command_spacing_prevents_pwm_write(self):
        clock = Clock()
        link = FakeSerial(clock)
        probe = UnloadedPulseProbe(link, now=clock.now, sleep=clock.sleep)
        probe.phase = 'request_pwm'
        probe.output_deadline = 0.02
        probe.last_send = 0.0
        with self.assertRaises(ProbeError):
            probe.send('duty 100')
        self.assertEqual(link.sent, [])

    def test_cli_cannot_select_reverse_or_longer_pulse(self):
        for option in ('--direction', '--duration'):
            with self.subTest(option=option):
                with mock.patch('motor_a_unloaded_pulse.serial.Serial') as constructor:
                    with contextlib.redirect_stderr(io.StringIO()):
                        with self.assertRaises(SystemExit):
                            main(['/dev/offline-test-only', '--confirm-suspended',
                                  '--confirm-single-motor-a', '--confirm-stationary',
                                  option, '10'])
                    constructor.assert_not_called()


if __name__ == '__main__':
    unittest.main()
