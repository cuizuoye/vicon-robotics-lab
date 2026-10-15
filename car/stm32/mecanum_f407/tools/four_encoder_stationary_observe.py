#!/usr/bin/env python3
"""四轮上电静止观察：固定5秒，只允许stop/status，不使能、不发心跳或转动命令。"""
import argparse
import json
import signal

import serial

from four_wheel_bench_probe import FourWheelProbe, WHEELS, stopped_four
from front_encoder_observe import count_delta
from motor_a_no_motor_pwm_probe import ProbeError


class FourStationaryObserve(FourWheelProbe):
    def __init__(self, link, **kwargs):
        # 独立入口不接受wheel/sign，也不借用“电池拔掉”声明来进行带电观察。
        super().__init__(link, wheel=None, **kwargs)
        self.allowed_commands = frozenset({'stop', 'status'})
        self.last_ms = None
        self.observation_samples = 0

    def next_event(self, deadline):
        event = super().next_event(deadline)
        if isinstance(event, dict):
            if self.last_ms is not None and (event['ms'] - self.last_ms) & 0xffffffff > 0x7fffffff:
                raise ProbeError('MCU time moved backwards; possible reset')
            self.last_ms = event['ms']
        return event

    def stationary(self, row):
        return (stopped_four(row) and self.quiet(row)
                and all(row[f'cps_{w.lower()}'] == 0 for w in WHEELS))

    def run(self):
        if self.has_run:
            raise ProbeError('single use; no automatic repeat')
        self.has_run = True
        completed, final = False, None
        elapsed = 0.0
        try:
            self.wait_for(lambda r: isinstance(r, dict), stopped_four, timeout=2.0)
            self.phase = 'preflight_stop'
            self.exchange('stop', 'OK,stopped_all', stopped_four)
            self.baseline = self.query(self.stationary, stopped_four)
            self.phase = 'stationary_observation'
            started = last_rx = self.now()
            deadline = started + 5.0
            while self.now() < deadline:
                event = self.next_event(min(deadline, last_rx + 0.4))
                current = self.now()
                elapsed = current - started
                if current - last_rx > 0.4 + 1e-6:
                    raise ProbeError('stationary telemetry/host scheduling gap exceeded 0.4s')
                if event is None:
                    if current >= deadline:
                        break
                    raise ProbeError('stationary telemetry timeout')
                if not isinstance(event, dict) or not self.stationary(event) or event['fault']:
                    raise ProbeError('counts/CPS/output/fault changed during stationary observation')
                last_rx = current
                self.observation_samples += 1
            if self.observation_samples < 45 or self.now() - deadline > 0.2:
                raise ProbeError('insufficient samples or delayed observation')
            completed = True
        except (Exception, KeyboardInterrupt) as exc:
            self.errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            self.keepalive = False
            self.phase = 'final_stop'
            try:
                self.exchange('stop', 'OK,stopped_all', lambda r: True, allow_fault=True)
                final = self.query(stopped_four, lambda r: True, allow_fault=True)
                self.stop_verified = True
                if final['fault'] or not self.stationary(final):
                    self.errors.append('final fault, count change or nonzero CPS')
            except (Exception, KeyboardInterrupt) as exc:
                self.errors.append('stop verification failed: ' + str(exc))
            if self.errors:
                self.announce('OBSERVATION_FAILED: close driver switches and disconnect propulsion battery.')
        passed = completed and self.stop_verified and not self.errors
        return {
            'test': 'four_wheel_powered_stationary_observation', 'firmware': self.firmware_name,
            'passed': passed, 'errors': self.errors, 'observation_seconds': 5.0,
            'elapsed_observation_seconds': elapsed, 'observation_samples': self.observation_samples,
            'start_state': self.baseline, 'end_state': final,
            'raw_count_delta': ({w: count_delta(final[f'enc_{w.lower()}'], self.baseline[f'enc_{w.lower()}'])
                                 for w in WHEELS} if self.baseline and final else None),
            'raw_samples': self.records, 'command_order': [c['command'] for c in self.commands],
            'software_stop_verified': self.stop_verified, 'requires_power_disconnect': not passed,
            'physical_stationary_confirmed_after': None, 'counts_per_output_revolution': None,
            'measurement_scope': 'powered stationary raw counts/CPS and actual CCR reports only; not dynamic encoder, direction, measured PWM or CPR validation',
        }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port')
    p.add_argument('--confirm-powered-stationary', action='store_true')
    p.add_argument('--confirm-suspended', action='store_true')
    args = p.parse_args(argv)
    if not (args.confirm_powered_stationary and args.confirm_suspended):
        p.error('requires explicit powered-stationary and suspended confirmation; serial unopened')

    def interrupted(signum, frame):
        raise KeyboardInterrupt('termination requested')

    signal.signal(signal.SIGTERM, interrupted)
    probe = FourStationaryObserve(None)
    link = serial.Serial(port=None, baudrate=115200, timeout=0.01, write_timeout=0.1, exclusive=True)
    link.dtr = link.rts = False
    link.port = args.port
    link.open()
    try:
        probe.link = link
        result = probe.run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
