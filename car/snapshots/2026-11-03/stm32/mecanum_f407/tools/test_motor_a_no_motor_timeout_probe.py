"""心跳超时工具的纯离线检查；不打开真实串口，不影响任何硬件。"""

import contextlib
import io
import unittest
from unittest import mock

from motor_a_no_motor_pwm_probe import ProbeError
from motor_a_no_motor_timeout_probe import HeartbeatTimeoutProbe, main
from test_motor_a_no_motor_pwm_probe import Clock, FakeSerial


class TimeoutSerial(FakeSerial):
    def __init__(self, clock, scenario='normal', chunk=2048):
        super().__init__(clock, scenario, chunk)
        self.last_hb = None
        self.timeout_at = None
        self.cutoff_injected = False
        self.ignored_sync_status = False

    def write(self, data):
        if data == b'hb\n':
            self.last_hb = self.clock.now()
        if (data == b'status\n' and self.scenario == 'ignore_post_hb_status'
                and self.output_started is not None and not self.ignored_sync_status):
            self.ignored_sync_status = True
            self.sent.append((self.clock.now(), 'status'))
            return len(data)
        return super().write(data)

    def tick_timeout(self):
        now = self.clock.now()
        age = now - self.last_hb if self.last_hb is not None else 0
        if self.timeout_at is not None:
            if now - self.timeout_at > 0.2:
                if self.scenario == 'fault_cleared_later':
                    self.fault = 0
                elif self.scenario == 'uncommanded_restart':
                    self.en, self.hiz, self.duty, self.fault = 1, 0, -100, 0
            return
        if (self.scenario == 'corrupt_after_cutoff' and self.en and age > 0.2
                and not self.cutoff_injected):
            self.cutoff_injected = True
            self.line('unrecognized serial data')
        threshold = 0.2 if self.scenario == 'premature_timeout' else 0.5
        if not self.en or age <= threshold or self.scenario == 'timeout_missing':
            return
        self.timeout_at = now
        if self.scenario == 'reset_on_timeout':
            self.line('READY,motor_a_safe_off')
            self.en, self.hiz, self.duty, self.fault = 0, 1, 0, 0
            return
        if self.scenario != 'timeout_without_notice':
            notice = ('ERR,uart_overflow' if self.scenario == 'timeout_wrong_notice'
                      else 'ERR,heartbeat_timeout')
            self.line(notice)
            if self.scenario == 'duplicate_notice':
                self.line(notice)
        if self.scenario != 'timeout_without_stop':
            self.en, self.hiz, self.duty = 0, 1, 0
        self.fault = (0 if self.scenario == 'timeout_without_fault' else
                      4 if self.scenario == 'timeout_wrong_fault' else 1)

    def read(self, count):
        self.clock.sleep(0.001 if self.rx else 0.01)
        elapsed = (self.clock.now() - self.output_started
                   if self.output_started is not None else 0)
        if abs(self.duty) == 100 and elapsed > 0.6 and not self.triggered:
            self.triggered = True
            if self.scenario == 'unexpected_notice_while_active':
                self.line('ERR,heartbeat_timeout')
                self.en, self.hiz, self.duty, self.fault = 0, 1, 0, 1
            elif self.scenario == 'host_stall':
                self.clock.sleep(0.6)
            elif self.scenario == 'keyboard_interrupt':
                raise KeyboardInterrupt('simulated Ctrl-C')
        self.tick_timeout()
        if self.clock.now() >= self.next_tel:
            self.next_tel = self.clock.now() + 0.1
            age = (self.clock.now() - self.last_hb
                   if self.last_hb is not None else 0)
            lost = ((self.scenario == 'lost_telemetry' and elapsed > 0.6)
                    or (self.scenario == 'silent_after_cutoff' and self.en and age > 0.1))
            if not lost:
                self.status()
        count = min(count, self.chunk, len(self.rx))
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result


class HeartbeatTimeoutProbeTests(unittest.TestCase):
    def execute(self, scenario='normal', chunk=2048):
        clock = Clock()
        link = TimeoutSerial(clock, scenario, chunk)
        messages = []
        probe = HeartbeatTimeoutProbe(link, now=clock.now, sleep=clock.sleep,
                                      announce=messages.append)
        return probe.run(), link, messages

    def assert_no_resume_or_clear(self, result, link):
        withheld = result['heartbeat_withheld_at_s']
        if withheld is not None:
            self.assertFalse(any(at > withheld and command in {'hb', 'arm A5', 'duty -100'}
                                 for at, command in link.sent))
        commands = [command for _, command in link.sent]
        self.assertFalse(any(command.startswith(('clear', 'speed', 'brake', 'zero'))
                             for command in commands))
        self.assertNotIn('duty 100', commands)
        self.assertLessEqual(commands.count('arm A5'), 1)
        self.assertLessEqual(commands.count('duty -100'), 1)

    def test_normal_automatic_stop_before_host_stop_and_fault_retained(self):
        result, link, _ = self.execute()
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['automatic_timeout_verified_before_host_stop'])
        self.assertTrue(result['fault_latch_verified_without_clear'])
        self.assertTrue(result['software_stop_verified'])
        self.assertEqual(result['last_reported_state']['fault'], 1)
        self.assertEqual((link.en, link.hiz, link.duty, link.fault), (0, 1, 0, 1))
        self.assertGreaterEqual(result['active_samples'], 20)
        self.assertGreaterEqual(result['latch_samples'], 5)
        self.assertEqual(len(result['expected_timeout_notices']), 1)
        self.assertGreaterEqual(result['host_last_heartbeat_to_disabled_report_s'], 0.5)
        self.assertLess(result['host_last_heartbeat_to_disabled_report_s'], 0.7)
        automatically_disabled_at = (result['last_heartbeat_sent_at_s']
                                     + result['host_last_heartbeat_to_disabled_report_s'])
        host_stop_at = [at for at, command in link.sent if command == 'stop'][-1]
        self.assertLess(automatically_disabled_at, host_stop_at)
        self.assert_no_resume_or_clear(result, link)

    def test_fragmented_serial_keeps_timeout_detection(self):
        result, link, _ = self.execute(chunk=7)
        self.assertTrue(result['passed'], result)
        self.assert_no_resume_or_clear(result, link)

    def test_missing_automatic_timeout_uses_fallback_stop(self):
        result, link, _ = self.execute('timeout_missing')
        self.assertFalse(result['passed'])
        self.assertFalse(result['automatic_timeout_verified_before_host_stop'])
        self.assertTrue(result['software_stop_verified'])
        final_stop_at = [at for at, command in link.sent if command == 'stop'][-1]
        self.assertLessEqual(final_stop_at - result['last_heartbeat_sent_at_s'], 1.25)
        self.assert_no_resume_or_clear(result, link)

    def test_missing_sync_status_fails_without_renewing_heartbeat(self):
        result, link, _ = self.execute('ignore_post_hb_status')
        self.assertFalse(result['passed'])
        self.assertFalse(result['automatic_timeout_verified_before_host_stop'])
        final_stop_at = [at for at, command in link.sent if command == 'stop'][-1]
        self.assertLessEqual(final_stop_at - result['last_heartbeat_sent_at_s'], 1.25)
        self.assert_no_resume_or_clear(result, link)

    def test_sync_status_wait_is_bounded_by_observation_deadline(self):
        clock = Clock()
        link = TimeoutSerial(clock, scenario='timeout_missing')
        probe = HeartbeatTimeoutProbe(link, now=clock.now, sleep=clock.sleep,
                                      announce=lambda message: None)
        probe.heartbeat_withheld = True
        probe.phase = 'heartbeat_off_sync'
        probe.timeout_observe_deadline = 0.15
        probe.last_status_at = 0.0
        with self.assertRaises(ProbeError):
            probe.wait_for(lambda event: False, lambda row: True, timeout=1.0)
        self.assertLessEqual(clock.now(), 0.17)
        self.assertEqual(link.sent, [])

    def test_bad_timeout_evidence_never_passes(self):
        for scenario in ('timeout_without_fault', 'timeout_without_stop',
                         'timeout_wrong_fault', 'timeout_without_notice',
                         'timeout_wrong_notice', 'premature_timeout', 'duplicate_notice',
                         'reset_on_timeout', 'unexpected_notice_while_active',
                         'corrupt_after_cutoff', 'fault_cleared_later', 'uncommanded_restart'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'], result)
                self.assertGreaterEqual([command for _, command in link.sent].count('stop'), 1)
                self.assert_no_resume_or_clear(result, link)

    def test_telemetry_silence_is_failure_not_timeout_proof(self):
        for scenario in ('lost_telemetry', 'silent_after_cutoff'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'], result)
                self.assertFalse(result['automatic_timeout_verified_before_host_stop'])
                self.assert_no_resume_or_clear(result, link)

    def test_preflight_fault_and_arm_rejection_prevent_pwm(self):
        for scenario in ('preflight_fault', 'reject_arm'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'], result)
                self.assertNotIn('duty -100', [command for _, command in link.sent])
                self.assert_no_resume_or_clear(result, link)

    def test_wrong_ack_sign_and_write_failure_request_stop(self):
        for scenario in ('wrong_ack_sign', 'write_failure'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'], result)
                self.assertTrue(result['software_stop_verified'])
                self.assert_no_resume_or_clear(result, link)

    def test_host_stall_and_interrupt_do_not_restart(self):
        for scenario in ('host_stall', 'keyboard_interrupt'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'], result)
                self.assert_no_resume_or_clear(result, link)

    def test_missing_stop_confirmation_requires_power_disconnect(self):
        result, link, messages = self.execute('ignore_final_stop')
        self.assertFalse(result['passed'])
        self.assertFalse(result['software_stop_verified'])
        self.assertTrue(result['requires_power_disconnect'])
        self.assertTrue(any('STOP_UNCONFIRMED' in message for message in messages))
        self.assert_no_resume_or_clear(result, link)

    def test_heartbeat_cannot_resume_after_cutoff(self):
        clock = Clock()
        link = TimeoutSerial(clock)
        probe = HeartbeatTimeoutProbe(link, now=clock.now, sleep=clock.sleep)
        probe.heartbeat_withheld = True
        probe.keepalive = True
        with self.assertRaises(ProbeError):
            probe.send('hb')
        self.assertEqual(link.sent, [])

    def test_non_test_commands_are_never_written(self):
        clock = Clock()
        link = TimeoutSerial(clock)
        probe = HeartbeatTimeoutProbe(link, now=clock.now, sleep=clock.sleep)
        for command in ('duty 100', 'duty -200', 'clear 5A', 'speed 100', 'brake'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        self.assertEqual(link.sent, [])

    def test_cli_without_confirmation_never_opens_serial(self):
        with mock.patch('motor_a_no_motor_timeout_probe.serial.Serial') as constructor:
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    main(['/dev/offline-test-only'])
            self.assertEqual(error.exception.code, 2)
            constructor.assert_not_called()


if __name__ == '__main__':
    unittest.main()
