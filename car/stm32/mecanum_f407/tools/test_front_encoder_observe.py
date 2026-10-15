"""双编码器只读工具离线测试；绝不打开真实串口。"""

import contextlib
import io
import unittest
from unittest import mock

from front_encoder_observe import FrontEncoderObserve, count_delta, main, parse_front_status
from motor_a_no_motor_pwm_probe import ProbeError
from test_dual_motor_bench_probe import DualFakeSerial
from test_motor_a_no_motor_pwm_probe import Clock


class EncoderFakeSerial(DualFakeSerial):
    def status(self, kind='TEL'):
        elapsed = self.clock.now()
        if self.scenario == 'lost_telemetry' and elapsed > 0.6:
            return
        if self.scenario == 'reset' and elapsed > 0.6 and not self.triggered:
            self.triggered = True
            self.line('READY,front_encoder_readonly_v1')
        if self.scenario == 'fault' and elapsed > 0.6:
            self.fault = 128
        a, b, ca, cb = 17, 23, 0, 0
        if self.scenario in ('manual_a', 'cross_channel'):
            a += int(elapsed * 20)
            ca = 20
        if self.scenario in ('manual_b', 'cross_channel'):
            b -= int(elapsed * 30)
            cb = -30
        prefix = 'TEL3' if kind == 'TEL' else 'STATUS3'
        line = (f'{prefix},fw=front_enc_ro_v1,ms={int(elapsed*1000)},en=0,mask=0,'
                f'a=0,b=0,az=1,bz=1,limit_ms=0,enc_a={a},cps_a={ca},'
                f'enc_b={b},cps_b={cb},fault=0x{self.fault:08x}')
        if self.scenario == 'old_firmware':
            line = line.replace('front_enc_ro_v1', 'demo30_v1')
        if self.scenario == 'enabled':
            line = line.replace('en=0', 'en=1')
        if self.scenario == 'truncated':
            line = line[:-15]
        self.line(line)

    def read(self, count):
        if self.scenario == 'interrupt' and self.clock.now() > 0.6 and not self.triggered:
            self.triggered = True
            raise KeyboardInterrupt('simulated Ctrl-C')
        return super().read(count)


class FrontEncoderTests(unittest.TestCase):
    def execute(self, scenario='normal', chunk=2048, **options):
        clock = Clock()
        link = EncoderFakeSerial(clock, scenario, chunk)
        probe = FrontEncoderObserve(link, now=clock.now, sleep=clock.sleep,
                                    announce=lambda message: None, **options)
        return probe.run(), link, probe

    def test_status_only(self):
        result, link, _ = self.execute()
        self.assertTrue(result['passed'], result)
        self.assertEqual([cmd for _, cmd in link.sent], ['stop', 'status', 'status', 'stop', 'status'])
        self.assertEqual(result['raw_count_delta'], {'A': 0, 'B': 0})

    def test_stationary_observation_and_fragmentation(self):
        for chunk in (2048, 7):
            result, link, _ = self.execute(seconds=2, expect_stationary=True, chunk=chunk)
            self.assertTrue(result['passed'], result)
            self.assertGreaterEqual(result['observation_samples'], 16)
            self.assertTrue(set(cmd for _, cmd in link.sent) <= {'stop', 'status'})

    def test_manual_a_and_b_raw_signs_and_declared_turns(self):
        for wheel, scenario, sign in (('A', 'manual_a', 1), ('B', 'manual_b', -1)):
            result, link, _ = self.execute(scenario, seconds=2, wheel=wheel, turns=5)
            self.assertTrue(result['passed'], result)
            self.assertGreater(result['raw_count_delta'][wheel] * sign, 0)
            self.assertGreater(result['counts_per_output_revolution_estimate'], 0)
            self.assertEqual(result['raw_count_delta']['B' if wheel == 'A' else 'A'], 0)
            self.assertTrue(set(cmd for _, cmd in link.sent) <= {'stop', 'status'})
            self.assertIsNone(result['physical_rotation_observed'])

    def test_moving_stationary_or_nonselected_channel_fails(self):
        for scenario, options in (
                ('manual_a', {'expect_stationary': True}),
                ('manual_b', {'expect_stationary': True}),
                ('cross_channel', {'wheel': 'A', 'turns': 5}),
                ('cross_channel', {'wheel': 'B', 'turns': 5})):
            result, _, _ = self.execute(scenario, seconds=2, **options)
            self.assertFalse(result['passed'])
            self.assertIsNone(result['counts_per_output_revolution_estimate'])

    def test_declared_turns_without_counts_not_calibrated(self):
        result, _, _ = self.execute(seconds=2, wheel='A', turns=5)
        self.assertFalse(result['passed'])
        self.assertIsNone(result['counts_per_output_revolution_estimate'])

    def test_wrong_firmware_or_nonzero_state_never_enables(self):
        for scenario in ('old_firmware', 'enabled', 'truncated', 'preflight_fault'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'])
                self.assertTrue(set(cmd for _, cmd in link.sent) <= {'stop', 'status'})

    def test_runtime_failures_exit_with_stop_without_retry(self):
        for scenario in ('fault', 'reset', 'lost_telemetry', 'interrupt'):
            result, link, _ = self.execute(scenario, seconds=2)
            self.assertFalse(result['passed'], result)
            if scenario == 'lost_telemetry':
                self.assertFalse(result['software_stop_verified'])
                self.assertTrue(result['requires_power_disconnect'])
            else:
                self.assertTrue(result['software_stop_verified'], result)
            self.assertTrue(set(cmd for _, cmd in link.sent) <= {'stop', 'status'})

    def test_single_use_and_no_motion_commands(self):
        _, link, probe = self.execute()
        before = list(link.sent)
        with self.assertRaises(ProbeError):
            probe.run()
        for command in ('hb', 'arm B A5', 'duty B -100', 'clear 5A', 'zero', 'speed 100'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        self.assertEqual(before, link.sent)

    def test_invalid_calibration_options(self):
        for options in ({'seconds': 61}, {'seconds': -1}, {'wheel': 'A'},
                        {'turns': 5}, {'seconds': 2, 'wheel': 'A', 'turns': 0},
                        {'wheel': 'A', 'turns': 5},
                        {'seconds': 2, 'expect_stationary': True, 'wheel': 'A', 'turns': 5}):
            with self.assertRaises(ProbeError):
                FrontEncoderObserve(None, **options)

    def test_cli_missing_confirmations_and_bad_options_never_opens(self):
        invalid = ([], ['--observe'], ['--confirm-battery-disconnected', '--run'],
                   ['--confirm-battery-disconnected', '--wheel', 'A'],
                   ['--confirm-battery-disconnected', '--expect-stationary'])
        for flags in invalid:
            with mock.patch('front_encoder_observe.serial.Serial') as constructor:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(['/dev/offline-only'] + flags)
                constructor.assert_not_called()

    def test_help_never_opens_serial(self):
        with mock.patch('front_encoder_observe.serial.Serial') as constructor:
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as done:
                main(['--help'])
            self.assertEqual(done.exception.code, 0)
            constructor.assert_not_called()

    def test_schema_limits_duplicates_missing_fields_and_line_size(self):
        line = ('STATUS3,fw=front_enc_ro_v1,ms=4294967295,en=0,mask=0,a=0,b=0,az=1,bz=1,'
                'limit_ms=0,enc_a=-2147483648,cps_a=-2147483648,enc_b=-2147483648,'
                'cps_b=-2147483648,fault=0xffffffff')
        self.assertLess(len(line + '\r\n'), 224)
        self.assertEqual(parse_front_status(line)['enc_b'], -(1 << 31))
        for wrong in (line + ',a=0', line.replace(',enc_b=-2147483648', ''),
                      line.replace('enc_b=-2147483648', 'enc_b=2147483648'),
                      line.replace('en=0', 'en=1'), line.replace('a=0,', 'a=100,'),
                      line.replace('STATUS3', 'STATUS2')):
            with self.assertRaises(ValueError):
                parse_front_status(wrong)

    def test_signed_32bit_difference_wrap(self):
        self.assertEqual(count_delta(-2147483647, 2147483647), 2)
        self.assertEqual(count_delta(2147483647, -2147483647), -2)


if __name__ == '__main__':
    unittest.main()
