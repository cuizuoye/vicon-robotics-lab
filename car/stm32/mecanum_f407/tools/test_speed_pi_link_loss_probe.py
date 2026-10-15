import unittest
from unittest.mock import patch

from speed_pi_link_loss_probe import EXPECTED_FAULT, Probe, fault_stopped
from speed_pi_probe import RUN_COMMAND, parse_status
from test_speed_pi_probe import Clock, FakeSerial, fixture


class LossSerial(FakeSerial):
    def __init__(self, clock, suppress_timeout=False, wrong_fault=False, **kwargs):
        super().__init__(clock, **kwargs)
        self.last_hb = clock.now
        self.fault_word = 0
        self.suppress_timeout = suppress_timeout
        self.wrong_fault = wrong_fault
        self.command_times = []

    def row(self):
        if self.started is not None and self.clock.now-self.last_hb >= 0.15 and not self.suppress_timeout:
            self.count = int((self.clock.now-self.started)*300)
            self.started = None
            self.fault_word = 0x20 if self.wrong_fault else EXPECTED_FAULT
        line = super().row()
        if self.fault_word:
            line = line.replace('fault=0x00000000', f'fault=0x{self.fault_word:x}')
        return line

    def write(self, data):
        self.command_times.append((data.decode().strip(), self.clock.now))
        if data.decode().strip() == 'hb':
            self.last_hb = self.clock.now
        return super().write(data)


class LossTests(unittest.TestCase):
    def test_fault_guard_and_allowlist(self):
        row = parse_status(fixture(used=1, fault='0x4'))
        fault_stopped(row, row)
        for change in ({'fault': 0}, {'fault': 32}, {'wdg_reset': 1}, {'used': 0},
                       {'p_fl1': 100}, {'en': 1}, {'enc_fr': 1}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                fault_stopped(row, dict(row, **change))
        clock = Clock()
        p = Probe(LossSerial(clock))
        p.hb_allowed = False
        with self.assertRaises(ValueError):
            p.send('hb')
        p = Probe(LossSerial(clock), latched=True)
        with self.assertRaises(ValueError):
            p.send(RUN_COMMAND)

    def exercise(self, latched=False, **kwargs):
        clock = Clock()
        link = LossSerial(clock, **kwargs)
        if latched:
            link.used = True
            link.fault_word = EXPECTED_FAULT
        with patch('speed_pi_link_loss_probe.time.monotonic', clock):
            result = Probe(link, latched=latched).run()
        return result, link

    def test_simulated_loss_and_latch(self):
        result, link = self.exercise()
        self.assertTrue(result['passed'], result['errors'])
        self.assertTrue(result['autonomous_fault_proven'])
        self.assertEqual(result['end_state']['fault'], EXPECTED_FAULT)
        self.assertEqual(link.commands.count(RUN_COMMAND), 1)
        self.assertFalse(result['trial']['stop_command_sent_before_fault_evidence'])
        self.assertTrue(result['trial']['no_command_during_fault_observation'])
        self.assertLess(result['trial']['first_fault_host_delay_s'], 0.5)

    def test_simulated_fault_no_power(self):
        result, link = self.exercise(latched=True)
        self.assertTrue(result['passed'], result['errors'])
        self.assertFalse(result['motion_command_sent'])
        self.assertNotIn('hb', link.commands)
        self.assertEqual(result['end_state']['fault'], EXPECTED_FAULT)

    def test_no_timeout_and_wrong_fault_fail(self):
        for kwargs in ({'suppress_timeout': True}, {'wrong_fault': True}, {'drop_ack': True}):
            with self.subTest(kwargs=kwargs):
                result, link = self.exercise(**kwargs)
                self.assertFalse(result['passed'])
                self.assertFalse(result['autonomous_fault_proven'])
                self.assertEqual(link.commands.count(RUN_COMMAND), 1)
                self.assertEqual(link.commands[-2:], ['stop', 'status'])
                run_at = next(t for c, t in link.command_times if c == RUN_COMMAND)
                cleanup_stop_at = next(t for c, t in link.command_times if c == 'stop' and t > run_at)
                self.assertLess(cleanup_stop_at-run_at, 2.33)  # 1.8秒驱动+0.5秒等待故障回报。
                self.assertLess(result['end_state']['ms'], 5200)  # 随后的清理/静止观察不是继续运行。


if __name__ == '__main__':
    unittest.main()
