import unittest

from speed_readonly_probe import FIELDS, Probe, cadence, parse_status, stationary


def fixture(**updates):
    fields = {k: 0 for k in FIELDS}
    fields.update(fw='speed_ro_v1', ms=100, ticks=10, updates=10, dt_min=10,
                  dt_max=10, irq_us_max=10, ro=1, gpio_low=1, fault='0x00000000')
    fields.update(updates)
    return 'TELSP,' + ','.join(f'{k}={v}' for k, v in sorted(fields.items()))


class ReadOnlyTests(unittest.TestCase):
    def test_parse(self):
        row = parse_status(fixture())
        self.assertEqual(row['fw'], 'speed_ro_v1')
        self.assertEqual(row['ticks'], 10)

    def test_reject_unsafe(self):
        for change in ({'fw': 'speed_pi_v1'}, {'ro': 0}, {'gpio_low': 0}, {'en': 1},
                       {'p_fl1': 100}, {'d_fr': -100}, {'target_rl': 1}, {'fault': '0x1'},
                       {'wdg_reset': 1}, {'dt_max': 31}, {'irq_us_max': 1001}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                parse_status(fixture(**change))

    def test_schema(self):
        with self.assertRaises(ValueError):
            parse_status(fixture() + ',ro=1')
        with self.assertRaises(ValueError):
            parse_status(fixture() + ',extra=1')

    def test_cadence(self):
        before = parse_status(fixture())
        after = parse_status(fixture(ms=5100, ticks=510, updates=510))
        self.assertEqual(cadence(before, after)['timer_hz'], 100)
        after['updates'] = 260
        with self.assertRaises(ValueError):
            cadence(before, after)

    def test_allowlist(self):
        class Fake:
            def write(self, value):
                return len(value)
        p = Probe(Fake())
        p.send('stop')
        p.send('status')
        for command in ('hb', 'run FL 300 S5', 'arm FL A5', 'duty FL 100'):
            with self.assertRaises(ValueError):
                p.send(command)
        self.assertEqual(p.commands, ['stop', 'status'])

    def test_powered_stationary(self):
        baseline = parse_status(fixture(enc_fl=123, enc_rr=-345))
        stationary(baseline, parse_status(fixture(enc_fl=123, enc_rr=-345, ms=5000)))
        for change in ({'enc_fl': 124}, {'enc_rr': -344}, {'cps_rl': 100},
                       {'rpm_fr': 1}, {'mms_fl': -1}):
            row = dict(baseline, **change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                stationary(baseline, row)


if __name__ == '__main__':
    unittest.main()
