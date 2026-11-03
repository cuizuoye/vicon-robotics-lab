"""编码器观察工具离线测试；不打开真实串口。"""

import contextlib
import io
import unittest
from unittest import mock

from motor_a_encoder_observe import (
    EncoderObserveProbe, main, signed_count_delta, summarize_encoder,
)
from motor_a_no_motor_pwm_probe import ProbeError
from test_motor_a_no_motor_pwm_probe import Clock, FakeSerial


class EncoderSerial(FakeSerial):
    def __init__(self, clock, scenario='normal', chunk=2048):
        super().__init__(clock, scenario, chunk)
        self.count = -700000
        self.cps = 0

    def status(self, kind='TEL'):
        if self.scenario == 'lost_telemetry' and kind == 'TEL' and self.clock.now() > 0.6:
            return
        if self.scenario == 'drift' and kind == 'TEL' and self.clock.now() > 0.2:
            self.count += 1
            self.cps = 10
        if self.scenario == 'cps_only' and kind == 'TEL':
            self.cps = 200
        if self.scenario == 'reset' and kind == 'TEL' and self.clock.now() > 0.6:
            self.line('READY,motor_a_safe_off')
        if self.scenario == 'unexpected_enabled' and kind == 'TEL' and self.clock.now() > 0.6:
            self.en = 1
        self.line(f'{kind},ms={int(self.clock.now()*1000)},en={self.en},'
                  f'hiz={self.hiz},mode=duty,target_cps=0,duty={self.duty},'
                  f'enc={self.count},cps={self.cps},fault=0x{self.fault:08x}')


class EncoderObserveTests(unittest.TestCase):
    def execute(self, scenario='normal', chunk=2048):
        clock = Clock()
        link = EncoderSerial(clock, scenario, chunk)
        probe = EncoderObserveProbe(link, now=clock.now, sleep=clock.sleep,
                                    announce=lambda message: None)
        return probe.run(), link, probe

    def test_normal_only_stop_status_and_no_zero(self):
        result, link, _ = self.execute()
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['encoder_observation']['stationary_stable'])
        self.assertEqual(result['encoder_observation']['count_delta'], 0)
        self.assertEqual(result['encoder_observation']['count_start'], -700000)
        self.assertFalse(result['rotation_feedback_verified'])
        self.assertIsNone(result['counts_per_wheel_revolution'])
        self.assertTrue(set(cmd for _, cmd in link.sent) <= {'stop', 'status'})
        self.assertEqual([cmd for _, cmd in link.sent][-2:], ['stop', 'status'])

    def test_chunked_frames(self):
        result, _, _ = self.execute(chunk=7)
        self.assertTrue(result['passed'], result)

    def test_drift_and_nonzero_cps_fail_without_enable(self):
        for scenario in ('drift', 'cps_only'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'], result)
                self.assertFalse(result['encoder_observation']['stationary_stable'])
                self.assertTrue(set(cmd for _, cmd in link.sent) <= {'stop', 'status'})

    def test_fault_does_not_clear_or_enable(self):
        result, link, _ = self.execute('preflight_fault')
        self.assertFalse(result['passed'])
        self.assertEqual(link.fault, 2)
        self.assertTrue(set(cmd for _, cmd in link.sent) <= {'stop', 'status'})

    def test_telemetry_timeout_stops_observation(self):
        result, link, _ = self.execute('lost_telemetry')
        self.assertFalse(result['passed'])
        self.assertTrue(any('timeout' in error for error in result['errors']))
        self.assertLess(link.clock.now(), 2.0)
        self.assertTrue(result['software_stop_verified'])

    def test_reset_fails_without_motion(self):
        result, link, _ = self.execute('reset')
        self.assertFalse(result['passed'])
        self.assertTrue(any('reset' in error for error in result['errors']))
        self.assertTrue(set(cmd for _, cmd in link.sent) <= {'stop', 'status'})

    def test_unexpected_enable_requests_stop(self):
        result, link, _ = self.execute('unexpected_enabled')
        self.assertFalse(result['passed'])
        self.assertTrue(set(cmd for _, cmd in link.sent) <= {'stop', 'status'})
        self.assertTrue(any('stop' == cmd for _, cmd in link.sent))

    def test_motion_commands_never_written(self):
        clock = Clock()
        link = EncoderSerial(clock)
        probe = EncoderObserveProbe(link, now=clock.now, sleep=clock.sleep)
        for command in ('hb', 'arm A5', 'duty 100', 'duty -100', 'zero', 'clear 5A', 'speed 100'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        self.assertEqual(link.sent, [])

    def test_cannot_repeat(self):
        _, link, probe = self.execute()
        previous = list(link.sent)
        with self.assertRaises(ProbeError):
            probe.run()
        self.assertEqual(link.sent, previous)

    def test_missing_confirmation_never_opens_serial(self):
        flags = ['--confirm-encoder-connected', '--confirm-input-compatible', '--confirm-stationary']
        for missing in range(3):
            with self.subTest(missing=missing):
                with mock.patch('motor_a_encoder_observe.serial.Serial') as constructor:
                    with contextlib.redirect_stderr(io.StringIO()):
                        with self.assertRaises(SystemExit):
                            main(['/dev/offline-test-only'] +
                                 [flag for index, flag in enumerate(flags) if index != missing])
                    constructor.assert_not_called()

    def test_empty_samples_not_verified(self):
        self.assertFalse(summarize_encoder([])['stationary_stable'])

    def test_signed_boundary_delta(self):
        self.assertEqual(signed_count_delta(-2147483648, 2147483647), 1)
        self.assertEqual(signed_count_delta(2147483647, -2147483648), -1)

    def test_summary_does_not_hide_motion_by_net_zero(self):
        rows = [{'ms': 100, 'enc': 20, 'cps': 0},
                {'ms': 200, 'enc': 21, 'cps': 10},
                {'ms': 300, 'enc': 20, 'cps': -10}]
        result = summarize_encoder(rows)
        self.assertEqual(result['count_delta'], 0)
        self.assertFalse(result['stationary_stable'])
        self.assertEqual(result['max_abs_step'], 1)


if __name__ == '__main__':
    unittest.main()
