import unittest
import subprocess
import tempfile
from pathlib import Path
from unittest.mock import patch

from drive_pi_probe import FIELDS, MODES, Probe, SIGNS, moving, parse_status, targets
from test_speed_pi_probe import Clock, FakeSerial as OldSerial


def fixture(**updates):
    f = dict.fromkeys(FIELDS, 0)
    f.update(fw='drive_pi_v1', ms=1000, ticks=100, updates=100, dt_min=10, dt_max=10,
             irq_us_max=180, stable_ms=500, fault='0x0')
    f.update(updates)
    return 'TELSP,' + ','.join(f'{k}={v}' for k, v in sorted(f.items()))


class FakeSerial(OldSerial):
    """假时间/协议验证，不是四轮动力学仿真或硬件验收。"""
    def __init__(self, clock, wrong_firmware=False, reverse=None, no_progress=None,
                 drop_ack=False, zero_moves=False, wrong_mode=False, telemetry_loss=False):
        super().__init__(clock)
        self.motion = None
        self.wrong_firmware = wrong_firmware
        self.reverse, self.no_progress = reverse, no_progress
        self.drop_ack, self.zero_moves, self.wrong_mode = drop_ack, zero_moves, wrong_mode
        self.telemetry_loss = telemetry_loss
        self.counts = dict.fromkeys(SIGNS, 0)

    def row(self):
        ms = int(self.clock.now * 1000)
        f = dict(ms=ms, ticks=ms//10, updates=ms//10, used=int(self.used),
                 stable_ms=min(500, int((self.clock.now-self.stop_time)*1000)))
        if self.wrong_firmware:
            f['fw'] = 'forward_pi_v1'
        if self.started is not None:
            elapsed = self.clock.now-self.started
            mode = MODES[self.motion]
            f.update(en=1, sel=5, used=1, mode=mode[0], stable_ms=0)
            f.update(zip(('vx', 'vy', 'wz'), mode[2]))
            for w, target in targets(mode).items():
                direction = (target > 0)-(target < 0)
                actual = -direction if self.reverse == w and elapsed > 0.5 else direction
                self.counts[w] = int(elapsed * abs(target)) * SIGNS[w] * actual
                if self.no_progress == w:
                    self.counts[w] = 0
                if self.zero_moves and target == 0:
                    self.counts[w] = 1
                duty = direction * SIGNS[w] * 40
                cc = (100, 96) if duty > 0 else (96, 100) if duty < 0 else (0, 0)
                f.update({f'cps_{w}': abs(target) * SIGNS[w] * actual,
                          f'fcps_{w}': abs(target) * actual, f'want_{w}': target,
                          f'target_{w}': min(abs(target), int(elapsed * 200)) * direction,
                          f'rpm_{w}': 15000 * actual, f'mms_{w}': 50 * actual,
                          f'd_{w}': duty, f'p_{w}1': cc[0], f'p_{w}2': cc[1]})
            if self.wrong_mode:
                f['mode'] = 0
        f.update({f'enc_{w}': n for w, n in self.counts.items()})
        return fixture(**f)

    def write(self, data):
        command = data.decode().strip()
        self.commands.append(command)
        if command == 'stop':
            self.started = None; self.stop_time = self.clock.now
            self.queue('OK,stopped_drive')
        elif command == 'status':
            self.queue(self.row().replace('TELSP,', 'STATUSSP,', 1))
        elif command == 'hb':
            self.queue('OK,hb_drive')
        elif command in (m[1] for m in MODES.values()):
            if self.used:
                self.queue('ERR,one_shot_used')
            else:
                self.motion = next(name for name, mode in MODES.items() if mode[1] == command)
                self.used = True; self.started = self.clock.now
                if not self.drop_ack:
                    self.queue('OK,running_drive')
        else:
            raise AssertionError(command)
        return len(data)

    def read(self, size):
        # 接收断流不应被心跳ACK冒充速度遥测；保留stop ACK以验证finally路径。
        if self.telemetry_loss and self.started is not None and self.clock.now-self.started > 0.5:
            self.clock.now += 0.005
            return b''
        return super().read(size)


class DriveProbeTests(unittest.TestCase):
    def exercise(self, motion=None, after_run=False, powered_stationary=False, **kwargs):
        clock = Clock(); link = FakeSerial(clock, **kwargs); link.used = after_run
        with patch('drive_pi_probe.time.monotonic', clock):
            result = Probe(link, motion=motion, after_run=after_run,
                           powered_stationary=powered_stationary).run()
        self.assertLessEqual(sum(c.startswith('run ') for c in link.commands), 1)
        return result, link

    def test_parser_rejects_old_schema_duplicate_and_limits(self):
        self.assertEqual(parse_status(fixture())['mode'], 0)
        for update in ({'fw': 'forward_pi_v1'}, {'en': 2}, {'mode': 1}, {'vx': 1}, {'sel': 5},
                       {'used': 2}, {'ro': 1}, {'gpio_low': 1}, {'d_fl': 10}, {'p_fr1': 1},
                       {'want_fl': 1}, {'cps_rr': 1 << 31}, {'ms': -1}, {'dt_max': 31},
                       {'irq_us_max': 1001}, {'stable_ms': 501}, {'fcps_rl': 1}):
            with self.subTest(update=update), self.assertRaises(ValueError):
                parse_status(fixture(**update))
        with self.assertRaises(ValueError):
            parse_status(fixture() + ',used=0')

    def test_all_mode_targets_and_directions(self):
        before = parse_status(fixture())
        for name in MODES:
            clock = Clock(); clock.now = 1
            link = FakeSerial(clock); link.motion = name; link.started = 0; link.used = True
            row = parse_status(link.row()); moving(before, row, name)
            for w, target in targets(MODES[name]).items():
                if target:
                    with self.subTest(mode=name, wheel=w), self.assertRaises(ValueError):
                        moving(before, dict(row, **{f'cps_{w}': -row[f'cps_{w}']}), name)
                    with self.assertRaises(ValueError):
                        parse_status(link.row().replace(f'want_{w}={target}', f'want_{w}={target+1}'))
                else:
                    with self.assertRaises(ValueError):
                        moving(before, dict(row, **{f'enc_{w}': 1}), name)

    def test_readonly_modes_send_only_stop_status(self):
        for mode in ({}, {'after_run': True}, {'powered_stationary': True}):
            r, link = self.exercise(**mode)
            self.assertTrue(r['passed'], r['errors'])
            self.assertFalse(r['motion_command_sent'])
            self.assertEqual(set(link.commands), {'stop', 'status'})

    def test_all_fixed_trials_end_stopped(self):
        for mode in MODES:
            r, link = self.exercise(motion=mode)
            self.assertTrue(r['passed'], (mode, r['errors']))
            self.assertEqual(link.commands.count(MODES[mode][1]), 1)
            self.assertEqual(r['end_state']['en'], 0)
            self.assertEqual(r['end_state']['used'], 1)
            self.assertGreaterEqual(r['trial']['run_samples'], 30)

    def test_no_retry_after_short_write_or_other_motion(self):
        link = FakeSerial(Clock()); p = Probe(link, motion='left50')
        for c in ('run F50 S5', 'run L50 S30', 'clear 5A', 'estop'):
            with self.assertRaises(ValueError):
                p.send(c)
        with patch.object(link, 'write', return_value=0), self.assertRaises(ValueError):
            p.send('run L50 S5')
        with self.assertRaises(ValueError):
            p.send('run L50 S5')
        p = Probe(link)
        with self.assertRaises(ValueError):
            p.send('hb')

    def test_each_wheel_fault_stops_group(self):
        for mode in ('left50', 'yaw200'):
            for w in SIGNS:
                for issue in ('reverse', 'no_progress'):
                    r, link = self.exercise(motion=mode, **{issue: w})
                    self.assertFalse(r['passed'], (mode, w, issue))
                    self.assertEqual(r['end_state']['en'], 0)
                    self.assertEqual(link.commands.count(MODES[mode][1]), 1)

    def test_protocol_ack_wrong_mode_and_zero_wheel_fault(self):
        for issue in ({'wrong_firmware': True}, {'drop_ack': True}, {'wrong_mode': True},
                      {'zero_moves': True}, {'telemetry_loss': True}):
            r, link = self.exercise(motion='diagonal25', **issue)
            self.assertFalse(r['passed'], issue)
            if issue.get('wrong_firmware'):
                self.assertFalse(any(c.startswith('run ') for c in link.commands))
            self.assertEqual(link.started, None)

    def test_bad_mode_and_conflicting_state(self):
        link = FakeSerial(Clock())
        for kw in ({'motion': 'unknown'}, {'motion': 'left50', 'after_run': True},
                   {'motion': 'yaw200', 'powered_stationary': True},
                   {'after_run': True, 'powered_stationary': True}):
            with self.assertRaises(ValueError):
                Probe(link, **kw)

    def test_actual_c_telemetry_matches_host(self):
        root = Path(__file__).resolve().parents[1]
        sources = ('tests/drive_schema_fixture.c', 'Src/drive_bench_control.c',
                   'Src/four_drive_group.c', 'Src/mecanum_drive_plan.c',
                   'Src/mecanum_kinematics.c', 'Src/four_wheel_speed.c')
        with tempfile.TemporaryDirectory(prefix='drive_schema_') as tmp:
            executable = Path(tmp) / 'fixture'
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-DSPEED_UART_FRAME_BYTES=1152U', '-IInc', *sources,
                            '-lm', '-o', str(executable)], cwd=root, check=True, capture_output=True)
            result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 8)
        for i, name in enumerate(MODES):
            active = parse_status(lines[2*i]); stopped = parse_status(lines[2*i+1])
            moving(parse_status(fixture()), active, name)
            self.assertEqual(active['mode'], MODES[name][0])
            self.assertEqual(stopped['en'], 0)
            self.assertEqual(stopped['used'], 1)


if __name__ == '__main__':
    unittest.main()
