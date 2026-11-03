import unittest
from unittest.mock import patch

from speed_pi_probe import PI_FIELDS, Probe, RUN_COMMAND, moving, parse_status, rates, stationary


def fixture(**updates):
    fields = {key: 0 for key in PI_FIELDS}
    fields.update(fw='speed_pi_v2', ms=1000, ticks=100, updates=100, dt_min=10,
                  dt_max=10, irq_us_max=40, stable_ms=500, fault='0x00000000')
    fields.update(updates)
    return 'TELSP,'+', '.join(f'{k}={v}' for k, v in sorted(fields.items())).replace(', ', ',')


class Clock:
    def __init__(self):
        self.now = 1.0

    def __call__(self):
        return self.now


class FakeSerial:
    """仅模拟串口协议和定时，不模拟真实电机、波形或控制稳定性。"""
    def __init__(self, clock, wrong_firmware=False, isolation_error=False, drop_ack=False, fault=False):
        self.clock = clock
        self.pending = bytearray()
        self.commands = []
        self.started = None
        self.stop_time = clock.now-1
        self.next_telemetry = clock.now
        self.used = False
        self.count = 0
        self.wrong_firmware = wrong_firmware
        self.isolation_error = isolation_error
        self.drop_ack = drop_ack
        self.fault = fault
        self.sent_after_fault = False

    def queue(self, text):
        self.pending.extend((text+'\n').encode('ascii'))

    def row(self):
        ms = int(self.clock.now*1000)
        fields = dict(ms=ms, ticks=ms//10, updates=ms//10, used=int(self.used),
                      stable_ms=min(500, int((self.clock.now-self.stop_time)*1000)), enc_fl=self.count)
        if self.wrong_firmware:
            fields['fw'] = 'speed_ro_v1'
        if self.started is not None:
            elapsed = self.clock.now-self.started
            self.count = int(elapsed*300)
            fields.update(en=1, sel=1, used=1, stable_ms=0, enc_fl=self.count,
                          cps_fl=300, fcps_fl=300, target_fl=min(300, int(elapsed*200)),
                          rpm_fl=16986, mms_fl=53, d_fl=40, p_fl1=100, p_fl2=96)
            if self.isolation_error and elapsed > 0.5:
                fields['enc_fr'] = 1
            if self.fault and elapsed > 0.5:
                fields.update(en=0, sel=0, cps_fl=0, fcps_fl=0, target_fl=0, rpm_fl=0,
                              mms_fl=0, d_fl=0, p_fl1=0, p_fl2=0, fault='0x20')
                self.sent_after_fault = True
        if self.sent_after_fault:
            fields['fault'] = '0x20'
        return fixture(**fields)

    def write(self, data):
        command = data.decode().strip()
        self.commands.append(command)
        if command == 'stop':
            self.started = None
            self.stop_time = self.clock.now
            self.queue('OK,stopped_speed')
        elif command == 'status':
            self.queue(self.row().replace('TELSP,', 'STATUSSP,', 1))
        elif command == 'hb':
            self.queue('OK,hb_speed')
        elif command == RUN_COMMAND:
            if self.used:
                self.queue('ERR,one_shot_used')
            else:
                self.started = self.clock.now
                self.used = True
                if not self.drop_ack:
                    self.queue('OK,running_speed')
        else:
            raise AssertionError(command)
        return len(data)

    def read(self, size):
        if not self.pending:
            self.clock.now += 0.005
            if self.clock.now >= self.next_telemetry:
                self.next_telemetry = self.clock.now+0.1
                self.queue(self.row())
        chunk = bytes(self.pending[:size])
        del self.pending[:size]
        return chunk


class SpeedPiTests(unittest.TestCase):
    def test_parser_and_output_map(self):
        row = parse_status(fixture())
        self.assertEqual(row['fw'], 'speed_pi_v2')
        parse_status(fixture(en=1, sel=2, used=1, d_fr=-40, p_fr1=96, p_fr2=100))
        for change in ({'fw': 'speed_pi_v1'}, {'fw': 'speed_ro_v1'}, {'ro': 1},
                       {'gpio_low': 1}, {'d_fl': 110}, {'p_fl1': 1}, {'en': 1},
                       {'dt_max': 31}, {'irq_us_max': 1001}, {'stable_ms': 501},
                       {'sel': 3}, {'cps_rr': 1 << 31}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                parse_status(fixture(**change))
        with self.assertRaises(ValueError):
            parse_status(fixture()+',used=0')

    def test_idle_motion_guards(self):
        before = parse_status(fixture())
        stationary(before, before)
        for change in ({'enc_rr': 1}, {'cps_fl': 1}, {'used': 1}, {'fault': 32}, {'wdg_reset': 1}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                stationary(before, dict(before, **change))
        active = parse_status(fixture(en=1, sel=1, used=1, target_fl=300, fcps_fl=300,
                                      cps_fl=300, enc_fl=30, d_fl=40, p_fl1=100, p_fl2=96))
        moving(before, active)
        for change in ({'enc_fr': 1}, {'cps_fl': -1}, {'fcps_fl': -1},
                       {'target_fl': 301}, {'sel': 2}, {'en': 0}, {'fault': 16}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                moving(before, dict(active, **change))
        self.assertEqual(rates(before, dict(before, ms=6000, ticks=600, updates=600))['timer_hz'], 100)

    def test_one_shot_allowlist(self):
        clock = Clock()
        link = FakeSerial(clock)
        p = Probe(link)
        for command in ('hb', RUN_COMMAND, 'estop'):
            with self.assertRaises(ValueError):
                p.send(command)
        p = Probe(link, motion=True)
        for command in ('run FR 300 S5', 'run FL -300 S5', 'duty 100', 'run FL 300 S30'):
            with self.assertRaises(ValueError):
                p.send(command)
        p.send(RUN_COMMAND)
        with self.assertRaises(ValueError):
            p.send(RUN_COMMAND)

    def exercise(self, motion, after_run=False, **kwargs):
        clock = Clock()
        link = FakeSerial(clock, **kwargs)
        link.used = after_run
        with patch('speed_pi_probe.time.monotonic', clock):
            result = Probe(link, motion=motion, after_run=after_run).run()
        if kwargs.get('wrong_firmware'):
            self.assertEqual(link.commands[-1], 'stop')
            self.assertIsNone(result['end_state'])  # 错误协议不能伪装成已验证停机。
        else:
            self.assertEqual(link.commands[-2:], ['stop', 'status'])
        return result, link

    def test_simulated_no_power_flow(self):
        result, link = self.exercise(False)
        self.assertTrue(result['passed'], result['errors'])
        self.assertFalse(result['motion_command_sent'])
        self.assertNotIn('hb', link.commands)

    def test_simulated_post_run_flow(self):
        result, link = self.exercise(False, after_run=True)
        self.assertTrue(result['passed'], result['errors'])
        self.assertEqual(result['end_state']['used'], 1)
        self.assertEqual(result['test'], 'speed_pi_post_run_no_power')
        self.assertNotIn(RUN_COMMAND, link.commands)
        with self.assertRaises(ValueError):
            Probe(link, motion=True, after_run=True)

    def test_simulated_one_run_flow(self):
        result, link = self.exercise(True)
        self.assertTrue(result['passed'], result['errors'])
        self.assertEqual(link.commands.count(RUN_COMMAND), 1)
        self.assertEqual(result['end_state']['en'], 0)
        self.assertEqual(result['end_state']['used'], 1)
        self.assertGreater(result['trial']['raw_fl_progress'], 10)

    def test_wrong_firmware_and_failure_cleanup(self):
        result, link = self.exercise(True, wrong_firmware=True)
        self.assertFalse(result['passed'])
        self.assertNotIn(RUN_COMMAND, link.commands)
        for kwargs in ({'isolation_error': True}, {'drop_ack': True}, {'fault': True}):
            with self.subTest(kwargs=kwargs):
                result, link = self.exercise(True, **kwargs)
                self.assertFalse(result['passed'])
                self.assertEqual(link.commands.count(RUN_COMMAND), 1)
                self.assertLess(result['end_state']['ms'], 4000)


if __name__ == '__main__':
    unittest.main()
