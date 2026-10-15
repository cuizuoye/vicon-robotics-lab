import base64
import gzip
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from drive_pi_probe import Probe as OldProbe, parse_status as old_parse
from drive_startup_probe import (FIELDS, FIRMWARE, MODES, Probe, compact_result,
                                 parse_status, run_rates)
from test_drive_pi_probe import FakeSerial as OldSerial
from test_speed_pi_probe import Clock


def fixture(**updates):
    f = dict.fromkeys(FIELDS, 0)
    f.update(fw=FIRMWARE, ms=1000, ticks=100, updates=100, dt_min=10, dt_max=10,
             irq_us_max=180, stable_ms=500, fault='0x0')
    f.update(updates)
    return 'TELSP,' + ','.join(f'{k}={v}' for k, v in sorted(f.items()))


class FakeSerial(OldSerial):
    """假串口时间/帧，不模拟启动门槛或四轮动力学。"""
    def __init__(self, clock, stall=False, **kwargs):
        super().__init__(clock, **kwargs)
        self.stall = stall
        self.latched = False

    def row(self):
        f = old_parse(super().row())
        f.pop('kind')
        f['fw'] = FIRMWARE
        f['start_mask'] = f['fault_wheel'] = 0
        if self.started is not None:
            elapsed = self.clock.now - self.started
            if elapsed < 0.2:
                f['start_mask'] = 6 if self.motion == 'diagonal25' else 15
                for w in ('fl', 'fr', 'rl', 'rr'):
                    if f[f'want_{w}']:
                        sign = 1 if f[f'd_{w}'] > 0 else -1
                        f[f'd_{w}'] = 80 * sign
                        f[f'p_{w}1'], f[f'p_{w}2'] = (100, 92) if sign > 0 else (92, 100)
            if self.stall and elapsed > 0.7:
                self.latched = True
        if self.latched:
            f.update(en=0, sel=0, mode=0, vx=0, vy=0, wz=0, fault='0x20',
                     start_mask=0, fault_wheel=3, stable_ms=0)
            for w in ('fl', 'fr', 'rl', 'rr'):
                for k in ('cps', 'fcps', 'target', 'want', 'rpm', 'mms', 'd'):
                    f[f'{k}_{w}'] = 0
                f[f'p_{w}1'] = f[f'p_{w}2'] = 0
        return fixture(**f)

    def write(self, data):
        c = data.decode().strip()
        if c == 'hb' and self.latched:
            self.commands.append(c)
            self.queue('ERR,fault_latched')
            return len(data)
        old_c = c.replace(' S10', ' S5') if c.startswith('run ') else c
        super().write((old_c + '\n').encode())
        self.commands[-1] = c
        return len(data)


class StartupProbeTests(unittest.TestCase):
    def exercise(self, motion=None, after_run=False, powered_stationary=False, **kw):
        clock = Clock(); link = FakeSerial(clock, **kw); link.used = after_run
        with patch('drive_pi_probe.time.monotonic', clock):
            result = Probe(link, motion=motion, after_run=after_run,
                           powered_stationary=powered_stationary).run()
        self.assertLessEqual(sum(c.startswith('run ') for c in link.commands), 1)
        return result, link

    def test_exact_schema_and_bounded_mask(self):
        self.assertEqual(parse_status(fixture())['fw'], FIRMWARE)
        for kw in ({'fw': 'drive_pi_v1'}, {'start_mask': 16}, {'fault_wheel': 5},
                   {'start_mask': 1}, {'fault_wheel': 1}):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                parse_status(fixture(**kw))
        with self.assertRaises(ValueError):
            old_parse(fixture())
        with self.assertRaises(ValueError):
            parse_status(fixture() + ',start_mask=0')
        clock = Clock(); link = FakeSerial(clock); link.motion = 'diagonal25'; link.started = clock.now - 0.1
        link.used = True
        r = parse_status(link.row())
        self.assertEqual(r['start_mask'], 6)
        bad = dict(r, start_mask=7); bad.pop('kind')
        with self.assertRaises(ValueError):
            parse_status(fixture(**bad))

    def test_readonly_has_no_heartbeat_or_motion(self):
        for options in ({}, {'after_run': True}, {'powered_stationary': True}):
            r, link = self.exercise(**options)
            self.assertTrue(r['passed'], r['errors'])
            self.assertEqual(set(link.commands), {'stop', 'status'})
            self.assertFalse(r['motion_command_sent'])

    def test_compact_output_preserves_result_and_does_not_mutate_it(self):
        for options in ({}, {'motion': 'left50'}, {'motion': 'left50', 'stall': True}):
            r, _ = self.exercise(**options)
            compact = compact_result(r)
            self.assertNotIn('telemetry', compact)
            self.assertNotIn('raw_serial_records', compact)
            self.assertEqual(compact['capture']['telemetry_count'], len(r['telemetry']))
            self.assertEqual(compact['capture']['raw_serial_record_count'],
                             len(r['raw_serial_records']))
            for k in ('passed', 'errors', 'command_order', 'trial',
                      'end_state', 'observed_startup', 'startup_profile'):
                self.assertEqual(compact[k], r[k])
            self.assertIn('telemetry', r)
            self.assertIn('raw_serial_records', r)

    def test_all_four_modes_have_ten_second_profile(self):
        for name, mode in MODES.items():
            r, link = self.exercise(motion=name)
            self.assertTrue(r['passed'], (name, r['errors']))
            self.assertEqual(link.commands.count(mode[1]), 1)
            self.assertEqual(r['firmware'], FIRMWARE)
            self.assertEqual(r['trial']['host_run_limit_s'], 9.7)
            self.assertEqual(r['trial']['firmware_run_limit_ms'], 10000)
            self.assertGreaterEqual(r['trial']['run_samples'], 90)
            self.assertEqual(r['observed_startup']['maximum_sampled_duty_permille'], 80)
            self.assertEqual(r['end_state']['en'], 0)
            self.assertEqual(r['end_state']['used'], 1)

    def test_compressed_trace_is_lossless_and_preserves_failed_trial(self):
        for options in ({'after_run': True}, {'motion': 'yaw200'},
                        {'motion': 'left50', 'stall': True}):
            r, link = self.exercise(**options)
            compact = compact_result(r, include_trace=True)
            encoded = compact['trace']['data']
            payload = gzip.decompress(base64.b64decode(encoded, validate=True))
            self.assertEqual(len(payload), compact['trace']['uncompressed_bytes'])
            trace = json.loads(payload)
            for k in ('telemetry', 'raw_serial_records'):
                self.assertEqual(trace[k], r[k])
            self.assertTrue(compact['capture']['full_telemetry_in_output'])
            self.assertEqual(compact['passed'], r['passed'])
            self.assertEqual(compact['errors'], r['errors'])
            self.assertEqual(compact['end_state'], r['end_state'])
            self.assertEqual(compact['command_order'], link.commands)
            self.assertEqual(compact_result(r, include_trace=True)['trace'], compact['trace'])
            self.assertLess(len(encoded), len(payload))
            self.assertNotIn('trace', r)

    def test_trace_requires_compact_output_before_serial_is_opened(self):
        script = Path(__file__).with_name('drive_startup_probe.py')
        r = subprocess.run([sys.executable, '-B', str(script), 'unused', '--mode',
                            'no-power', '--confirm-battery-disconnected', '--include-trace'],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 2)
        self.assertIn('--include-trace requires --summary-only; serial unopened', r.stderr)

    def test_stall_preserves_partial_startup_and_does_not_retry(self):
        r, link = self.exercise(motion='left50', stall=True)
        self.assertFalse(r['passed'])
        self.assertTrue(r['motion_command_sent'])
        self.assertEqual(r['observed_startup']['latched_fault'], 32)
        self.assertEqual(r['observed_startup']['fault_wheel'], 3)
        self.assertGreater(r['observed_startup']['active_samples'], 0)
        self.assertEqual(r['end_state']['start_mask'], 0)
        self.assertEqual(link.commands.count('run L50 S10'), 1)
        self.assertEqual(link.started, None)

    def test_one_shot_allowlist_and_old_tool_rejects_new(self):
        link = FakeSerial(Clock()); p = Probe(link, motion='left50')
        for c in ('run L50 S5', 'run L50 S30', 'run F50 S10', 'clear 5A'):
            with self.assertRaises(ValueError):
                p.send(c)
        with patch.object(link, 'write', return_value=0), self.assertRaises(ValueError):
            p.send('run L50 S10')
        with self.assertRaises(ValueError):
            p.send('run L50 S10')
        clock = Clock(); link = FakeSerial(clock)
        with patch('drive_pi_probe.time.monotonic', clock):
            r = OldProbe(link, motion='left50').run()
        self.assertFalse(r['passed'])
        self.assertFalse(any(c.startswith('run ') for c in link.commands))

    def test_ten_second_sampling_interval_rejects_short_or_wrong_rate(self):
        a = dict(ms=1000, ticks=100, updates=100)
        self.assertEqual(run_rates(a, dict(ms=10600, ticks=1060, updates=1060))['timer_hz'], 100)
        for b in (dict(ms=5500, ticks=550, updates=550),
                  dict(ms=11001, ticks=1100, updates=1100),
                  dict(ms=10600, ticks=1500, updates=1060)):
            with self.assertRaises(ValueError):
                run_rates(a, b)

    def test_actual_c_startup_and_fault_telemetry(self):
        root = Path(__file__).resolve().parents[1]
        sources = ('tests/startup_schema_fixture.c', 'Src/drive_bench_control.c',
                   'Src/four_drive_group.c', 'Src/four_wheel_speed.c',
                   'Src/mecanum_drive_plan.c', 'Src/mecanum_kinematics.c')
        with tempfile.TemporaryDirectory(prefix='startup_schema_') as tmp:
            executable = Path(tmp) / 'fixture'
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-DDRIVE_STARTUP_OBSERVE=1', '-DSPEED_UART_FRAME_BYTES=1152U',
                            '-IInc', *sources, '-lm', '-o', str(executable)],
                           cwd=root, check=True, capture_output=True)
            result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 9)
        for i, name in enumerate(MODES):
            r, stopped = parse_status(lines[2*i]), parse_status(lines[2*i+1])
            self.assertEqual(r['start_mask'], 6 if name == 'diagonal25' else 15)
            self.assertEqual(stopped['start_mask'], 0)
            self.assertEqual(stopped['en'], 0)
        fault = parse_status(lines[-1])
        self.assertEqual((fault['fault'], fault['fault_wheel'], fault['start_mask']), (32, 3, 0))


if __name__ == '__main__':
    unittest.main()
