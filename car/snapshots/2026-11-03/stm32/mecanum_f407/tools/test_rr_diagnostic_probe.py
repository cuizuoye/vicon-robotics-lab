import base64
import gzip
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from drive_startup_probe import compact_result, parse_status as old_parse
from rr_diagnostic_probe import (FIELDS, FIRMWARE, Probe, analyze_trace, parse_meta,
                                  parse_status, parse_trace)
from test_speed_pi_probe import Clock, FakeSerial as BaseSerial


def fixture(**updates):
    f = dict.fromkeys(FIELDS, 0)
    f.update(fw=FIRMWARE, ms=1000, ticks=100, updates=100, dt_min=10, dt_max=10,
             irq_us_max=180, stable_ms=500, fault='0x0')
    f.update(updates)
    return 'TELSP,' + ','.join(f'{k}={v}' for k, v in sorted(f.items()))


def trace_fixture(**updates):
    f = dict(i=0, ms=1000, enc=-1, cps=-100, fcps=100, target=164, im=20000,
             d=-40, p1=96, p2=100, en=1, assist=0, other=0, fault='0x0')
    f.update(updates)
    return 'RRTRACE,' + ','.join(f'{k}={v}' for k, v in f.items())


class FakeSerial(BaseSerial):
    """仅协议/时序假设备，不模拟真实电机力矩或卡顿。"""
    def __init__(self, clock, fault=False, old=False, export_error=None, zero_moves=False, drop_ack=False):
        super().__init__(clock)
        self.inject_fault, self.old, self.export_error = fault, old, export_error
        self.zero_moves, self.drop_ack = zero_moves, drop_ack
        self.run_start = self.run_stop = None
        self.latched = False

    def row(self):
        ms = int(self.clock.now*1000)
        f = dict(ms=ms, ticks=ms//10, updates=ms//10, used=int(self.used), enc_rr=self.count,
                 stable_ms=min(500, int((self.clock.now-self.stop_time)*1000)))
        if self.old:
            f['fw'] = 'drive_pi_start8_v1'
        if self.started is not None and not self.latched:
            elapsed = self.clock.now-self.started
            self.count = -int(elapsed*164)
            if self.inject_fault and elapsed > 0.7:
                self.latched = True
            else:
                f.update(en=1, sel=4, mode=5, used=1, stable_ms=0, enc_rr=self.count,
                         cps_rr=-164, fcps_rr=164, want_rr=164, target_rr=min(164, int(elapsed*800)),
                         rpm_rr=9274, mms_rr=29, d_rr=-40, p_rr1=96, p_rr2=100)
                if elapsed < 0.2:
                    f.update(start_mask=8, d_rr=-80, p_rr1=92, p_rr2=100)
                if self.zero_moves and elapsed > 0.5:
                    f['enc_fl'] = 1
        if self.latched:
            f.update(fault='0x20', fault_wheel=4)
        return fixture(**f)

    def write(self, data):
        command = data.decode().strip()
        self.commands.append(command)
        if command == 'stop':
            if self.started is not None:
                self.run_stop = self.clock.now
            self.started = None; self.stop_time = self.clock.now
            self.queue('OK,stopped_drive')
        elif command == 'status':
            self.queue(self.row().replace('TELSP,', 'STATUSSP,', 1))
        elif command == 'hb':
            self.queue('ERR,fault_latched' if self.latched else 'OK,hb_drive')
        elif command == 'run RR164 S10':
            if self.used:
                self.queue('ERR,one_shot_used')
            else:
                self.started = self.run_start = self.clock.now; self.used = True
                if not self.drop_ack:
                    self.queue('OK,running_drive')
        elif command == 'trace':
            n = min(1000, int((self.run_stop-self.run_start)*100)+1)
            overflow = int(self.export_error == 'overflow')
            self.queue(f'RRBEGIN,n={n},overflow={overflow}')
            for i in range(n):
                if self.export_error == 'missing' and i == 2:
                    continue
                args = dict(i=i, ms=int(self.run_start*1000)+(i+1)*10, enc=-(i+1))
                if i == n-1:
                    args.update(en=0, d=0, p1=0, p2=0, target=0, fcps=0, im=0,
                                fault='0x20' if self.latched else '0x0')
                self.queue(trace_fixture(**args))
            if self.export_error != 'missing_end':
                self.queue(f'RREND,n={n},overflow={overflow}')
        else:
            raise AssertionError(command)
        return len(data)


class RRProbeTests(unittest.TestCase):
    def exercise(self, motion=None, after_run=False, powered_stationary=False, **kw):
        clock = Clock(); link = FakeSerial(clock, **kw); link.used = after_run
        with patch('drive_pi_probe.time.monotonic', clock):
            r = Probe(link, motion=motion, after_run=after_run,
                      powered_stationary=powered_stationary).run()
        self.assertLessEqual(link.commands.count('run RR164 S10'), 1)
        self.assertLessEqual(link.commands.count('trace'), 1)
        return r, link

    def test_exact_rr_identity_and_target_schema(self):
        self.assertEqual(parse_status(fixture())['fw'], FIRMWARE)
        active = dict(en=1, sel=4, used=1, mode=5, want_rr=164, target_rr=100,
                      d_rr=-80, p_rr1=92, p_rr2=100, start_mask=8)
        parse_status(fixture(**active))
        for bad in ({'fw': 'drive_pi_start8_v1'}, {'sel': 5}, {'vx': 50}, {'mode': 3},
                    {'want_fl': 164}, {'want_rr': 165}, {'d_rr': 80}, {'start_mask': 15}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                parse_status(fixture(**(active | bad)))
        with self.assertRaises(ValueError): old_parse(fixture())
        with self.assertRaises(ValueError): parse_status(fixture() + ',used=0')

    def test_readonly_modes_never_send_run_heartbeat_or_trace(self):
        for options in ({}, {'after_run': True}, {'powered_stationary': True}):
            r, link = self.exercise(**options)
            self.assertTrue(r['passed'], r['errors'])
            self.assertEqual(set(link.commands), {'stop', 'status'})
            self.assertIsNone(r['rr_recording'])

    def test_single_rr_run_full_recording_and_lossless_capture(self):
        r, link = self.exercise(motion='rr164')
        self.assertTrue(r['passed'], r['errors'])
        self.assertEqual(r['end_state']['used'], 1)
        self.assertEqual(r['trial']['signed_forward_progress']['fl'], 0)
        self.assertGreater(r['rr_recording']['samples'], 900)
        self.assertAlmostEqual(r['rr_recording']['recorded_hz'], 100)
        self.assertEqual(link.commands.count('run RR164 S10'), 1)
        tail = link.commands[link.commands.index('trace')+1:]
        self.assertEqual(set(tail), {'stop', 'status'})
        compact = compact_result(r, include_trace=True)
        decoded = json.loads(gzip.decompress(base64.b64decode(compact['trace']['data'])))
        self.assertEqual(decoded['raw_serial_records'], r['raw_serial_records'])
        self.assertEqual(sum(s.startswith('RRTRACE,') for s in decoded['raw_serial_records']),
                         r['rr_recording']['samples'])

    def test_fault_zero_wheel_missing_ack_do_not_retry(self):
        for kw in ({'fault': True}, {'zero_moves': True}, {'drop_ack': True}):
            r, link = self.exercise(motion='rr164', **kw)
            self.assertFalse(r['passed'])
            self.assertEqual(link.commands.count('run RR164 S10'), 1)
            self.assertIsNotNone(r['rr_recording'])
            self.assertIsNone(link.started)

    def test_old_firmware_prevents_run(self):
        r, link = self.exercise(motion='rr164', old=True)
        self.assertFalse(r['passed'])
        self.assertNotIn('run RR164 S10', link.commands)

    def test_export_errors_preserve_raw_data_and_never_retry(self):
        for error in ('missing', 'overflow', 'missing_end'):
            r, link = self.exercise(motion='rr164', export_error=error)
            self.assertFalse(r['passed'])
            self.assertTrue(any(s.startswith('RRTRACE,') for s in r['raw_serial_records']))
            self.assertEqual(link.commands.count('trace'), 1)
            self.assertEqual(r['end_state']['en'], 0)

    def test_trace_parser_rejects_unsafe_or_bad_values(self):
        parse_trace(trace_fixture())
        for bad in ({'other': 1}, {'d': -110}, {'d': -41}, {'p1': 95}, {'target': 165},
                    {'assist': 1}, {'assist': 8}, {'fault': '0x20'}, {'en': 0},
                    {'im': 1000001}, {'ms': -1}, {'enc': 1 << 31}, {'i': 1001}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                parse_trace(trace_fixture(**bad))
        with self.assertRaises(ValueError): parse_trace(trace_fixture() + ',en=1')
        for meta in ('RRBEGIN,n=0,overflow=0', 'RRBEGIN,n=1002,overflow=0',
                     'RRBEGIN,n=10,overflow=3', 'RREND,n=10,overflow=0'):
            with self.assertRaises(ValueError): parse_meta(meta, 'RRBEGIN')

    def test_trace_sequence_requires_stopped_endpoint_and_contiguous_times(self):
        a = parse_trace(trace_fixture())
        b = parse_trace(trace_fixture(i=1, ms=1010, en=0, d=0, p1=0, p2=0,
                                      target=0, fcps=0, im=0))
        self.assertEqual(analyze_trace([a, b])['samples'], 2)
        for rows in ([], [a], [b, a], [a, b | {'i': 2}], [a, b | {'ms': 1031}]):
            with self.assertRaises(ValueError): analyze_trace(rows)
        with self.assertRaises(ValueError): analyze_trace([a, b], require_full=True)

    def test_command_whitelist_and_short_run_write_consumes_attempt(self):
        link = FakeSerial(Clock()); p = Probe(link, motion='rr164')
        for c in ('run Y200 S10', 'run RR164 S5', 'run RR200 S10', 'trace', 'clear 5A'):
            with self.assertRaises(ValueError): p.send(c)
        with patch.object(link, 'write', return_value=0), self.assertRaises(ValueError):
            p.send('run RR164 S10')
        with self.assertRaises(ValueError): p.send('run RR164 S10')

    def test_cli_confirmation_required_before_opening_serial(self):
        script = Path(__file__).with_name('rr_diagnostic_probe.py')
        for flags in ([], ['--confirm-battery-disconnected'],
                      ['--confirm-battery-disconnected', '--confirm-powered-suspended']):
            r = subprocess.run([sys.executable, '-B', str(script), 'unused', '--mode', 'rr164', *flags],
                               capture_output=True, text=True)
            self.assertEqual(r.returncode, 2)
            self.assertIn('serial unopened', r.stderr)

    def test_real_c_schema_and_native_safety(self):
        root = Path(__file__).resolve().parents[1]
        sources = ['Src/rr_diagnostic_trace.c', 'Src/drive_bench_control.c', 'Src/four_drive_group.c',
                   'Src/four_wheel_speed.c', 'Src/mecanum_drive_plan.c', 'Src/mecanum_kinematics.c']
        with tempfile.TemporaryDirectory(prefix='rr_schema_') as tmp:
            for entry in ('rr_schema_fixture', 'test_rr_diagnostic'):
                exe = Path(tmp)/entry
                subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                '-DDRIVE_STARTUP_OBSERVE=1', '-DRR_DIAGNOSTIC=1', '-IInc',
                                f'tests/{entry}.c', *sources, '-lm', '-o', str(exe)],
                               cwd=root, check=True, capture_output=True)
                out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
                if entry == 'rr_schema_fixture':
                    lines = out.splitlines()
                    active, stopped = parse_status(lines[0]), parse_status(lines[2])
                    rows = [parse_trace(lines[1]), parse_trace(lines[3])]
                    self.assertEqual(active['sel'], 4)
                    self.assertEqual(active['start_mask'], 8)
                    self.assertEqual(stopped['en'], 0)
                    analyze_trace(rows)


if __name__ == '__main__':
    unittest.main()
