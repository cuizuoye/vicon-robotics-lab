import unittest
from unittest.mock import patch

from forward_pi_probe import PI_FIELDS, Probe, RUN_COMMAND, SIGNS, TARGET_CPS, moving, parse_status
from speed_pi_probe import parse_status as old_pi_parser
from test_speed_pi_probe import Clock, FakeSerial as OldSerial


def fixture(**updates):
    fields = {key: 0 for key in PI_FIELDS}
    fields.update(fw='forward_pi_v1', ms=1000, ticks=100, updates=100, dt_min=10,
                  dt_max=10, irq_us_max=180, stable_ms=500, fault='0x0')
    fields.update(updates)
    return 'TELSP,' + ','.join(f'{k}={v}' for k, v in sorted(fields.items()))


class FakeSerial(OldSerial):
    """只模拟协议/时间，不模拟真实四轮动力学。"""
    def __init__(self, clock, wrong_firmware=False, reverse=None, drop_ack=False, no_progress=None):
        super().__init__(clock)
        self.wrong_firmware = wrong_firmware
        self.reverse = reverse
        self.drop_ack = drop_ack
        self.no_progress = no_progress
        self.counts = {w: 0 for w in SIGNS}

    def row(self):
        ms = int(self.clock.now * 1000)
        fields = dict(ms=ms, ticks=ms//10, updates=ms//10, used=int(self.used),
                      stable_ms=min(500, int((self.clock.now-self.stop_time)*1000)))
        if self.wrong_firmware:
            fields['fw'] = 'speed_pi_v2'
        if self.started is not None:
            elapsed = self.clock.now-self.started
            fields.update(en=1, sel=5, used=1, stable_ms=0)
            for w, sign in SIGNS.items():
                direction = -1 if self.reverse == w and elapsed > 0.5 else 1
                self.counts[w] = int(elapsed * TARGET_CPS[w]) * sign * direction
                if self.no_progress == w:
                    self.counts[w] = 0
                duty = 40 * sign
                fields.update({f'cps_{w}': TARGET_CPS[w] * sign * direction,
                               f'fcps_{w}': TARGET_CPS[w] * direction,
                               f'target_{w}': min(TARGET_CPS[w], int(elapsed * 200)),
                               f'want_{w}': TARGET_CPS[w], f'rpm_{w}': 15915,
                               f'mms_{w}': 50, f'd_{w}': duty,
                               f'p_{w}1': 100 if duty > 0 else 96,
                               f'p_{w}2': 96 if duty > 0 else 100})
        fields.update({f'enc_{w}': count for w, count in self.counts.items()})
        return fixture(**fields)

    def write(self, data):
        command = data.decode().strip()
        self.commands.append(command)
        if command == 'stop':
            self.started = None
            self.stop_time = self.clock.now
            self.queue('OK,stopped_forward')
        elif command == 'status':
            self.queue(self.row().replace('TELSP,', 'STATUSSP,', 1))
        elif command == 'hb':
            self.queue('OK,hb_forward')
        elif command == RUN_COMMAND:
            if self.used:
                self.queue('ERR,one_shot_used')
            else:
                self.used = True
                self.started = self.clock.now
                if not self.drop_ack:
                    self.queue('OK,running_forward')
        else:
            raise AssertionError(command)
        return len(data)


class ForwardProbeTests(unittest.TestCase):
    def test_parser_wrong_protocol_or_registers(self):
        row = parse_status(fixture())
        self.assertEqual(row['fw'], 'forward_pi_v1')
        for change in ({'fw': 'speed_pi_v2'}, {'fw': 'speed_ro_v1'}, {'ro': 1}, {'gpio_low': 1},
                       {'sel': 1}, {'dt_max': 31}, {'irq_us_max': 1001}, {'stable_ms': 501},
                       {'d_fr': 10}, {'d_fl': -10}, {'p_fl1': 1}, {'want_rr': 1},
                       {'cps_fr': 1 << 31}, {'used': 2}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                parse_status(fixture(**change))
        with self.assertRaises(ValueError):
            parse_status(fixture() + ',used=0')
        with self.assertRaises(ValueError):
            old_pi_parser(fixture())

    def test_every_wheel_direction_and_fixed_target(self):
        clock = Clock()
        link = FakeSerial(clock)
        link.started = 0
        link.used = True
        active = parse_status(link.row())
        before = parse_status(fixture())
        moving(before, active)
        for w in SIGNS:
            with self.subTest(w=w), self.assertRaises(ValueError):
                moving(before, dict(active, **{f'cps_{w}': -SIGNS[w]}))
            with self.subTest(w=w), self.assertRaises(ValueError):
                parse_status(link.row().replace(f'want_{w}={TARGET_CPS[w]}', f'want_{w}=300'))

    def test_allowlist_and_failed_write_no_retry(self):
        link = FakeSerial(Clock())
        p = Probe(link)
        for command in ('hb', RUN_COMMAND, 'estop'):
            with self.assertRaises(ValueError):
                p.send(command)
        p = Probe(link, motion=True)
        for command in ('run FL 300 S5', 'run F50 S30', 'run F-50 S5', 'clear 5A'):
            with self.assertRaises(ValueError):
                p.send(command)
        with patch.object(link, 'write', return_value=0):
            with self.assertRaises(ValueError):
                p.send(RUN_COMMAND)
        with self.assertRaises(ValueError):
            p.send(RUN_COMMAND)

    def exercise(self, motion=False, after_run=False, **kwargs):
        clock = Clock()
        link = FakeSerial(clock, **kwargs)
        link.used = after_run
        with patch('forward_pi_probe.time.monotonic', clock):
            result = Probe(link, motion=motion, after_run=after_run).run()
        self.assertLessEqual(link.commands.count(RUN_COMMAND), 1)
        return result, link

    def test_only_stop_status_no_power_and_post_run(self):
        for after in (False, True):
            result, link = self.exercise(after_run=after)
            self.assertTrue(result['passed'], result['errors'])
            self.assertEqual(set(link.commands), {'stop', 'status'})
            self.assertFalse(result['motion_command_sent'])

    def test_simulated_forward_then_all_stop(self):
        result, link = self.exercise(motion=True)
        self.assertTrue(result['passed'], result['errors'])
        self.assertEqual(link.commands.count(RUN_COMMAND), 1)
        self.assertEqual(result['end_state']['en'], 0)
        self.assertEqual(result['end_state']['used'], 1)
        self.assertEqual(result['trial']['max_abs_duty_permille'], 40)
        self.assertTrue(all(v > 10 for v in result['trial']['normalized_forward_progress'].values()))

    def test_powered_stationary_is_read_only(self):
        clock = Clock()
        link = FakeSerial(clock)
        with patch('forward_pi_probe.time.monotonic', clock):
            result = Probe(link, powered_stationary=True).run()
        self.assertTrue(result['passed'], result['errors'])
        self.assertEqual(set(link.commands), {'stop', 'status'})
        self.assertEqual(result['power_state'], 'powered_stationary_confirmed')
        self.assertFalse(result['motion_command_sent'])
        with self.assertRaises(ValueError):
            Probe(link, motion=True, powered_stationary=True)

    def test_each_reverse_wheel_aborts_without_retry(self):
        for w in SIGNS:
            result, link = self.exercise(motion=True, reverse=w)
            self.assertFalse(result['passed'])
            self.assertEqual(link.commands.count(RUN_COMMAND), 1)
            self.assertEqual(result['end_state']['en'], 0)

    def test_old_firmware_missing_ack_and_stuck_wheel(self):
        for kwargs in ({'wrong_firmware': True}, {'drop_ack': True}, {'no_progress': 'rr'}):
            result, link = self.exercise(motion=True, **kwargs)
            self.assertFalse(result['passed'], kwargs)
            self.assertLessEqual(link.commands.count(RUN_COMMAND), 1)
            if kwargs.get('wrong_firmware'):
                self.assertNotIn(RUN_COMMAND, link.commands)
                self.assertIsNone(result['end_state'])
            else:
                self.assertEqual(result['end_state']['en'], 0)


if __name__ == '__main__':
    unittest.main()
