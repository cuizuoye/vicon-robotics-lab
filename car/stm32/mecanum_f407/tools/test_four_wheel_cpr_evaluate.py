"""CPR换算纯离线测试；不打开真实串口。"""
import copy
import unittest
from unittest import mock

from four_wheel_cpr_evaluate import WHEELS, evaluate


def snapshot(ms, counts):
    row = {'fw': 'four_bench_v1', 'ms': ms, 'en': 0, 'sel': 0, 'limit_ms': 0, 'fault': 0}
    for w, count in zip(WHEELS, counts):
        w = w.lower()
        row.update({f'enc_{w}': count, f'cps_{w}': 0, f'd_{w}': 0,
                    f'p_{w}1': 0, f'p_{w}2': 0})
    return {'test': 'four_wheel_powered_stationary_observation', 'firmware': 'four_bench_v1',
            'passed': True, 'errors': [], 'software_stop_verified': True,
            'requires_power_disconnect': False, 'command_order': ['stop', 'status', 'stop', 'status'],
            'observation_samples': 50, 'start_state': dict(row), 'end_state': dict(row),
            'raw_samples': [dict(row) for _ in range(50)]}


class CPRTests(unittest.TestCase):
    def setUp(self):
        self.before = snapshot(6000, [100, 200, 300, 400])
        self.after = snapshot(16000, [5100, -4800, 5300, -4600])
        self.kw = dict(turns=dict.fromkeys(WHEELS, 5),
                       encoder_signs={'FL': 1, 'FR': -1, 'RL': 1, 'RR': -1},
                       direction='forward', manual_only=True,
                       no_reset_or_power_change=True, integer_output_turns=True)

    def test_four_forward_estimates_no_hardware_or_parameter_write(self):
        with mock.patch('serial.Serial') as serial:
            result = evaluate(self.before, self.after, **self.kw)
            serial.assert_not_called()
        self.assertTrue(result['passed'])
        self.assertFalse(result['runtime_parameters_written'])
        self.assertFalse(result['repeatability_verified'])
        for value in result['estimates'].values():
            self.assertEqual(value['counts_per_output_revolution_estimate'], 1000.0)

    def test_single_reverse_and_signed_count_wrap(self):
        kw = dict(self.kw, turns={'FL': 5}, encoder_signs={'FL': 1}, direction='reverse')
        result = evaluate(self.before, snapshot(16000, [-4900, 200, 300, 400]), **kw)
        self.assertEqual(result['estimates']['FL']['normalized_count_delta'], -5000)
        kw['direction'] = 'forward'
        result = evaluate(snapshot(6000, [(1 << 31) - 20, 200, 300, 400]),
                          snapshot(16000, [-(1 << 31) + 4980, 200, 300, 400]), **kw)
        self.assertEqual(result['estimates']['FL']['raw_count_delta'], 5000)

    def test_reject_missing_operator_measurements(self):
        for key in ('manual_only', 'no_reset_or_power_change', 'integer_output_turns'):
            with self.subTest(key=key), self.assertRaises(ValueError):
                evaluate(self.before, self.after, **dict(self.kw, **{key: False}))
        for turns in ({}, {'FL': 0}, {'FL': 1.5}, {'FL': True}, {'FL': 21}, {'A': 5}):
            with self.subTest(turns=turns), self.assertRaises(ValueError):
                evaluate(self.before, self.after, **dict(self.kw, turns=turns))

    def test_reject_zero_counts_wrong_direction_and_unmeasured_motion(self):
        for after, kw in ((snapshot(16000, [100, 200, 300, 400]), self.kw),
                          (self.after, dict(self.kw, direction='reverse')),
                          (self.after, dict(self.kw, turns={'FL': 5}, encoder_signs={'FL': 1}))):
            with self.assertRaises(ValueError):
                evaluate(self.before, after, **kw)

    def test_reject_active_commands_wrong_firmware_and_unstable_snapshot(self):
        changes = ({'passed': False}, {'errors': ['timeout']}, {'software_stop_verified': False},
                   {'command_order': ['stop', 'hb', 'status']}, {'firmware': 'front_enc_ro_v1'},
                   {'observation_samples': 2}, {'raw_samples': []})
        for changeset in changes:
            bad = dict(self.after, **changeset)
            with self.subTest(changes=changeset), self.assertRaises(ValueError):
                evaluate(self.before, bad, **self.kw)
        for field, value in (('en', 1), ('fault', 1), ('cps_fl', 1), ('p_rr1', 1),
                             ('enc_fr', -4799), ('enc_fl', 1.5), ('ms', -1)):
            bad = copy.deepcopy(self.after)
            bad['raw_samples'][25][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                evaluate(self.before, bad, **self.kw)

    def test_timestamp_reset_or_large_gap_rejected_and_ms_wrap_allowed(self):
        for ms in (5000, 6000, 4000000):
            with self.subTest(ms=ms), self.assertRaises(ValueError):
                evaluate(self.before, snapshot(ms, [5100, -4800, 5300, -4600]), **self.kw)
        result = evaluate(snapshot(0xffffff00, [100, 200, 300, 400]), self.after, **self.kw)
        self.assertTrue(result['passed'])


if __name__ == '__main__':
    unittest.main()
