"""30秒演示的纯离线串口/时间测试，不打开真实设备。"""

import contextlib
import io
import unittest
from unittest import mock

from front_wheels_demo30 import FrontDemoProbe, main, parse_demo_status
from dual_motor_bench_probe import parse_dual_status, stopped_dual
from motor_a_no_motor_pwm_probe import ProbeError
from test_dual_motor_bench_probe import DualFakeSerial
from test_motor_a_no_motor_pwm_probe import Clock


class DemoFakeSerial(DualFakeSerial):
    def line(self, text):
        if self.scenario != 'bench_firmware':
            text = text.replace('fw=dual_v2', 'fw=demo30_v1')
        text = text.replace('OK,armed=AB5', 'OK,armed=AB30')
        if self.scenario == 'wrong_b' and self.output_started is not None and self.en:
            text = text.replace(',b=-100,', ',b=100,')
        super().line(text)

    def write(self, data):
        if data.decode('ascii').strip() == 'arm AB A5 30s':
            count = super().write(b'arm AB A5 5s\n')
            self.sent[-1] = (self.sent[-1][0], 'arm AB A5 30s')
            if self.en:
                self.limit_ms = 30200
            return len(data) if count else count
        return super().write(data)


class Demo30Tests(unittest.TestCase):
    def execute(self, run_demo=True, scenario='normal', chunk=2048):
        clock = Clock()
        link = DemoFakeSerial(clock, scenario, chunk)
        messages = []
        probe = FrontDemoProbe(link, run_demo=run_demo, now=clock.now,
                               sleep=clock.sleep, announce=messages.append)
        return probe.run(), link, probe, messages

    def test_fixed_thirty_seconds_single_forward_command(self):
        result, link, probe, messages = self.execute()
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['firmware'], 'demo30_v1')
        self.assertEqual(result['requested_duration_s'], 30.0)
        self.assertEqual((result['duty_a_permille'], result['duty_b_permille']), (100, -100))
        self.assertGreaterEqual(result['output_request_to_stop_s'], 30.0)
        self.assertLessEqual(result['output_request_to_stop_s'], 30.04)
        self.assertGreaterEqual(result['active_samples'], 280)
        commands = [cmd for _, cmd in link.sent]
        self.assertEqual(commands.count('arm AB A5 30s'), 1)
        self.assertEqual(commands.count('duty AB 100 -100'), 1)
        self.assertEqual(commands[-2:], ['stop', 'status'])
        self.assertTrue(stopped_dual(result['last_reported_state']))
        self.assertIsNone(result['physical_stop_confirmed'])
        self.assertTrue(any('25 / 30' in text for text in messages))
        self.assertTrue(all(at < probe.output_deadline for at, cmd in link.sent if cmd == 'hb'))

    def test_default_status_never_enables(self):
        result, link, _, _ = self.execute(run_demo=False)
        self.assertTrue(result['passed'], result)
        self.assertEqual([cmd for _, cmd in link.sent], ['stop', 'status', 'stop', 'status'])

    def test_wrong_firmware_and_preflight_fault_block_enable(self):
        for scenario in ('bench_firmware', 'old_firmware', 'preflight_fault'):
            result, link, _, _ = self.execute(scenario=scenario)
            self.assertFalse(result['passed'], result)
            self.assertFalse(any(cmd.startswith(('hb', 'arm', 'duty')) for _, cmd in link.sent))

    def test_faults_stop_without_retry(self):
        for scenario in ('reset', 'fault', 'wrong_duty', 'wrong_sign', 'wrong_b', 'wrong_limit',
                         'wrong_ack_sign', 'lost_telemetry', 'host_stall', 'keyboard_interrupt',
                         'write_failure', 'wrong_channel', 'corrupt_line'):
            with self.subTest(scenario=scenario):
                result, link, probe, _ = self.execute(scenario=scenario)
                self.assertFalse(result['passed'], result)
                self.assertTrue(result['software_stop_verified'], result)
                self.assertLess(result['output_request_to_stop_s'], 30)
                commands = [cmd for _, cmd in link.sent]
                self.assertEqual(commands.count(probe.arm_command), 1)
                self.assertEqual(commands.count(probe.duty_command), 1)
                self.assertTrue(set(commands) <= probe.allowed_commands)

    def test_single_use_and_no_direction_or_duration_change(self):
        _, link, probe, _ = self.execute()
        before = list(link.sent)
        with self.assertRaises(ProbeError):
            probe.run()
        for command in ('duty AB 100 100', 'duty A 100', 'arm AB A5 60s', 'clear 5A'):
            with self.assertRaises(ProbeError):
                probe.send(command)
        self.assertEqual(before, link.sent)

    def test_fragmented_serial_and_missing_stop_ack(self):
        self.assertTrue(self.execute(chunk=7)[0]['passed'])
        result, _, _, messages = self.execute(scenario='ignore_final_stop')
        self.assertFalse(result['passed'])
        self.assertTrue(result['requires_power_disconnect'])
        self.assertTrue(any('STOP_UNCONFIRMED' in text for text in messages))

    def test_cli_requires_all_confirmations_before_open(self):
        flags = ['--confirm-suspended', '--confirm-two-motors', '--confirm-channel-isolation',
                 '--confirm-stationary', '--confirm-30s']
        for missing in range(len(flags)):
            with mock.patch('front_wheels_demo30.serial.Serial') as constructor:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(['/dev/offline-only', '--run'] + [flag for i, flag in enumerate(flags) if i != missing])
                constructor.assert_not_called()
        with mock.patch('front_wheels_demo30.serial.Serial') as constructor:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(['/dev/offline-only'])
            constructor.assert_not_called()

    def test_help_is_readable_and_never_opens_serial(self):
        output = io.StringIO()
        with mock.patch('front_wheels_demo30.serial.Serial') as constructor:
            with contextlib.redirect_stdout(output), self.assertRaises(SystemExit) as ended:
                main(['--help'])
            self.assertEqual(ended.exception.code, 0)
            constructor.assert_not_called()
        self.assertIn('固定10%、30秒', output.getvalue())

    def test_demo_schema_does_not_loosen_bench_schema_or_overflow_tx_buffer(self):
        line = ('STATUS2,fw=demo30_v1,ms=4294967295,en=1,mask=3,a=100,b=-100,az=0,bz=0,'
                'limit_ms=30200,enc_a=-2147483648,cps_a=-2147483648,fault=0xffffffff')
        self.assertLess(len(line + '\r\n'), 160)
        self.assertEqual(parse_demo_status(line)['limit_ms'], 30200)
        with self.assertRaises(ValueError):
            parse_dual_status(line)
        for wrong in (line.replace('b=-100', 'b=100'), line.replace('limit_ms=30200', 'limit_ms=5200'),
                      line + ',a=100', line.replace('mask=3', 'mask=1')):
            with self.assertRaises(ValueError):
                parse_demo_status(wrong)


if __name__ == '__main__':
    unittest.main()
