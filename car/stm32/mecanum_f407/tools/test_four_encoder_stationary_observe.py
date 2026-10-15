"""纯离线；不打开串口，不给电机发动作。"""
import contextlib
import io
import unittest
from unittest import mock

from four_encoder_stationary_observe import FourStationaryObserve, main
from motor_a_no_motor_pwm_probe import ProbeError
from test_four_wheel_bench_probe import FourFake
from test_motor_a_no_motor_pwm_probe import Clock


class StationaryFake(FourFake):
    def status(self, kind='TEL'):
        late = self.clock.now() > 0.7
        if late and self.scenario == 'count_change':
            self.enc[2] += 1
        if late and self.scenario == 'fault_late':
            self.fault = 4
        if late and self.scenario == 'lost_observation' and kind == 'TEL':
            return
        if late and self.scenario == 'reset_late' and not self.triggered:
            self.triggered = True
            self.line('READY,four_bench_safe_off_v1')
            return
        if late and self.scenario == 'ignore_stops' and kind == 'TEL':
            self.en, self.selected, self.duty = 1, 1, 100
        super().status(kind)

    def write(self, data):
        command = data.decode().strip()
        if self.clock.now() > 0.7 and self.scenario == 'ignore_stops' and command == 'stop':
            self.sent.append((self.clock.now(), command))
            return len(data)
        return super().write(data)


class StationaryTests(unittest.TestCase):
    def execute(self, scenario='normal', chunk=2048):
        clock = Clock()
        link = StationaryFake(clock, scenario, chunk)
        probe = FourStationaryObserve(link, now=clock.now, sleep=clock.sleep, announce=lambda m: None)
        return probe.run(), link, probe

    def test_normal_fixed_five_seconds_no_motion(self):
        result, link, _ = self.execute()
        self.assertTrue(result['passed'], result)
        self.assertGreaterEqual(result['observation_samples'], 45)
        self.assertGreaterEqual(result['elapsed_observation_seconds'], 5.0)
        self.assertEqual(result['raw_count_delta'], dict.fromkeys(('FL', 'FR', 'RL', 'RR'), 0))
        self.assertEqual([c for _, c in link.sent], ['stop', 'status', 'stop', 'status'])
        self.assertIsNone(result['physical_stationary_confirmed_after'])

    def test_fragmented(self):
        result, _, _ = self.execute(chunk=7)
        self.assertTrue(result['passed'], result)

    def test_faults_and_count_change_never_arm(self):
        for scenario in ('preflight_fault', 'preflight_cps', 'old_firmware', 'count_change',
                         'fault_late', 'lost_observation', 'reset_late'):
            with self.subTest(scenario=scenario):
                result, link, _ = self.execute(scenario)
                self.assertFalse(result['passed'], result)
                self.assertTrue(result['requires_power_disconnect'])
                self.assertTrue(all(c in {'stop', 'status'} for _, c in link.sent))

    def test_missing_stop_confirmation(self):
        result, _, _ = self.execute('ignore_stops')
        self.assertFalse(result['passed'])
        self.assertFalse(result['software_stop_verified'])

    def test_forbidden_commands_and_rerun(self):
        _, link, probe = self.execute()
        before = list(link.sent)
        for command in ('hb', 'arm FR A5', 'duty FR -100', 'clear 5A', 'zero'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        with self.assertRaises(ProbeError):
            probe.run()
        self.assertEqual(link.sent, before)

    def test_cli_rejects_missing_confirmation_and_motion_options(self):
        for flags in ([], ['--confirm-suspended'], ['--confirm-powered-stationary'],
                      ['--confirm-powered-stationary', '--confirm-suspended', '--wheel', 'FR'],
                      ['--confirm-powered-stationary', '--confirm-suspended', '--seconds', '30']):
            with mock.patch('four_encoder_stationary_observe.serial.Serial') as constructor:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(['/dev/offline-only'] + flags)
                constructor.assert_not_called()


if __name__ == '__main__':
    unittest.main()
