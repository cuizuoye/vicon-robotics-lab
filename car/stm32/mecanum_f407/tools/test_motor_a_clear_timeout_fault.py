"""只清超时故障工具的纯离线测试，不打开真实串口。"""

import contextlib
import io
import unittest
from unittest import mock

from motor_a_clear_timeout_fault import ClearTimeoutFaultProbe, main
from motor_a_no_motor_pwm_probe import ProbeError
from test_motor_a_no_motor_pwm_probe import Clock, FakeSerial


class ClearSerial(FakeSerial):
    def __init__(self, clock, scenario='normal', chunk=2048):
        super().__init__(clock, scenario, chunk)
        self.fault = {'already_clear': 0, 'other_fault': 2, 'mixed_faults': 3}.get(scenario, 1)
        self.clear_received = False
        self.cleared_at = None
        if scenario == 'unexpected_enabled':
            self.en, self.hiz, self.duty = 1, 0, 100

    def write(self, data):
        command = data.decode('ascii').strip()
        if command == 'stop' and self.scenario == 'ignore_final_stop' and self.clear_received:
            self.sent.append((self.clock.now(), command))
            return len(data)
        if command != 'clear 5A':
            return super().write(data)
        self.sent.append((self.clock.now(), command))
        self.clear_received = True
        if self.scenario == 'clear_write_failure':
            raise OSError('simulated USB failure')
        if self.scenario == 'keyboard_interrupt':
            raise KeyboardInterrupt('simulated Ctrl-C')
        if self.scenario == 'clear_rejected':
            self.line('ERR,stop_before_clear')
            return len(data)
        if self.scenario != 'clear_no_effect':
            self.fault = 0
            self.cleared_at = self.clock.now()
        if self.scenario == 'reset_during_clear':
            self.line('READY,motor_a_safe_off')
        if self.scenario == 'corrupt_during_clear':
            self.line('unknown serial data')
        if self.scenario != 'clear_ack_missing':
            self.line('OK,faults_cleared')
        return len(data)

    def status(self, kind='TEL'):
        if self.scenario == 'silent_after_clear' and self.clear_received and kind == 'TEL':
            return
        super().status(kind)

    def read(self, count):
        if self.cleared_at is not None and self.clock.now() - self.cleared_at > 0.15:
            if self.scenario == 'fault_returns':
                self.fault = 1
            elif self.scenario == 'new_fault_after_clear':
                self.fault = 2
        return super().read(count)


class ClearTimeoutFaultTests(unittest.TestCase):
    def execute(self, scenario='normal', chunk=2048):
        clock = Clock()
        link = ClearSerial(clock, scenario, chunk)
        messages = []
        probe = ClearTimeoutFaultProbe(link, now=clock.now, sleep=clock.sleep,
                                       announce=messages.append)
        return probe.run(), link, messages

    def assert_no_motion_or_retry(self, link):
        commands = [command for _, command in link.sent]
        self.assertTrue(all(command in {'stop', 'status', 'clear 5A'} for command in commands))
        self.assertLessEqual(commands.count('clear 5A'), 1)
        self.assertFalse(link.armed_once)

    def test_timeout_only_cleared_once_and_remains_disabled(self):
        result, link, _ = self.execute()
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['fault_before'], 1)
        self.assertTrue(result['clear_acknowledged'])
        self.assertTrue(result['clear_verified'])
        self.assertTrue(result['software_stop_verified'])
        self.assertEqual((link.en, link.hiz, link.duty, link.fault), (0, 1, 0, 0))
        self.assertGreaterEqual(result['clean_disabled_samples'], 2)
        self.assertEqual([command for _, command in link.sent].count('clear 5A'), 1)
        self.assert_no_motion_or_retry(link)

    def test_fragmented_serial(self):
        result, link, _ = self.execute(chunk=7)
        self.assertTrue(result['passed'], result)
        self.assert_no_motion_or_retry(link)

    def test_already_clear_skips_clear_command(self):
        result, link, _ = self.execute('already_clear')
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['fault_before'], 0)
        self.assertFalse(result['clear_attempted'])
        self.assertNotIn('clear 5A', [command for _, command in link.sent])
        self.assert_no_motion_or_retry(link)

    def test_other_or_mixed_faults_never_cleared(self):
        for scenario in ('other_fault', 'mixed_faults'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'])
                self.assertNotIn('clear 5A', [command for _, command in link.sent])
                self.assertIn(link.fault, {2, 3})
                self.assert_no_motion_or_retry(link)

    def test_enabled_state_is_not_cleared(self):
        result, link, _ = self.execute('unexpected_enabled')
        self.assertFalse(result['passed'])
        self.assertNotIn('clear 5A', [command for _, command in link.sent])
        self.assert_no_motion_or_retry(link)

    def test_ack_without_state_change_is_failure(self):
        result, link, _ = self.execute('clear_no_effect')
        self.assertFalse(result['passed'])
        self.assertTrue(result['clear_acknowledged'])
        self.assertFalse(result['clear_verified'])
        self.assertEqual(link.fault, 1)
        self.assert_no_motion_or_retry(link)

    def test_missing_ack_does_not_retry(self):
        result, link, _ = self.execute('clear_ack_missing')
        self.assertFalse(result['passed'])
        self.assertFalse(result['clear_acknowledged'])
        self.assert_no_motion_or_retry(link)

    def test_returning_fault_is_not_cleared_again(self):
        for scenario in ('fault_returns', 'new_fault_after_clear'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'])
                self.assert_no_motion_or_retry(link)

    def test_no_telemetry_after_clear_is_failure(self):
        result, link, _ = self.execute('silent_after_clear')
        self.assertFalse(result['passed'])
        self.assert_no_motion_or_retry(link)

    def test_reset_corruption_interrupt_rejection_and_write_failure(self):
        for scenario in ('reset_during_clear', 'corrupt_during_clear', 'keyboard_interrupt',
                         'clear_rejected', 'clear_write_failure'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'], result)
                self.assert_no_motion_or_retry(link)

    def test_missing_final_stop_ack_blocks_continuation(self):
        result, link, messages = self.execute('ignore_final_stop')
        self.assertFalse(result['passed'])
        self.assertFalse(result['software_stop_verified'])
        self.assertTrue(any('STOP_UNCONFIRMED' in message for message in messages))
        self.assert_no_motion_or_retry(link)

    def test_forbidden_commands_never_written(self):
        clock = Clock()
        link = ClearSerial(clock)
        probe = ClearTimeoutFaultProbe(link, now=clock.now, sleep=clock.sleep)
        for command in ('hb', 'arm A5', 'duty 100', 'duty -100', 'speed 100', 'clear 5A'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        self.assertEqual(link.sent, [])

    def test_stale_status_prevents_clear(self):
        clock = Clock()
        link = ClearSerial(clock)
        probe = ClearTimeoutFaultProbe(link, now=clock.now, sleep=clock.sleep)
        probe.phase = 'clear_timeout_fault'
        probe.clear_permitted = True
        probe.last_status_at = 0.0
        clock.sleep(1.0)
        with self.assertRaises(ProbeError):
            probe.send('clear 5A')
        self.assertEqual(link.sent, [])

    def test_clear_command_cannot_repeat(self):
        clock = Clock()
        link = ClearSerial(clock)
        probe = ClearTimeoutFaultProbe(link, now=clock.now, sleep=clock.sleep)
        probe.phase = 'clear_timeout_fault'
        probe.clear_permitted = True
        probe.last_status_at = 0.0
        probe.send('clear 5A')
        with self.assertRaises(ProbeError):
            probe.send('clear 5A')
        self.assertEqual([command for _, command in link.sent], ['clear 5A'])

    def test_cli_requires_both_confirmations_before_opening_serial(self):
        for flags in ([], ['--confirm-no-motor'], ['--confirm-power-disconnected']):
            with self.subTest(flags=flags):
                with mock.patch('motor_a_clear_timeout_fault.serial.Serial') as constructor:
                    with contextlib.redirect_stderr(io.StringIO()):
                        with self.assertRaises(SystemExit) as error:
                            main(['/dev/offline-test-only', *flags])
                    self.assertEqual(error.exception.code, 2)
                    constructor.assert_not_called()


if __name__ == '__main__':
    unittest.main()
