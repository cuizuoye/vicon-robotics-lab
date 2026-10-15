#!/usr/bin/env python3
"""仅RR隔离0.1%PWM镜像；不降低安全阈值、不自动重试、不接受旧镜像。"""
from drive_pi_probe import parse_status_for_firmware
from rr_diagnostic_probe import (FIELDS as BASE_FIELDS, MODES, Probe as BaseProbe,
                                  main, parse_trace as base_trace, targets)

FIRMWARE = 'rr_diag_pwm01_v1'
FIELDS = BASE_FIELDS | {'pwm_rr_arr', 'pwm_rr_psc'}


def parse_status(line):
    row = parse_status_for_firmware(line, FIRMWARE, FIELDS, MODES, 4, targets,
                                    duty_quantum=1, pwm_period_counts=8400)
    if row['pwm_rr_arr'] != 8399 or row['pwm_rr_psc'] != 0:
        raise ValueError('actual RR timer period/prescaler mismatch')
    if (row['start_mask'] not in (0, 8) or not 0 <= row['fault_wheel'] <= 4 or
            (not row['en'] and row['start_mask']) or
            (not row['fault'] and row['fault_wheel']) or
            (row['start_mask'] and row['d_rr'] != -80)):
        raise ValueError('invalid isolated RR startup/fault state')
    return row


def parse_trace(line):
    return base_trace(line, duty_quantum=1, pwm_period_counts=8400)


class Probe(BaseProbe):
    firmware = FIRMWARE
    parse_frame = staticmethod(parse_status)
    parse_trace_frame = staticmethod(parse_trace)

    def run(self):
        result = super().run()
        result['test'] = result['test'].replace('rr_diagnostic_', 'rr_pwm_fine_', 1)
        result['pwm_profile'] = dict(period_counts=8400, duty_quantum_permille=1,
                                     configured_hz=10000, max_output_permille=100,
                                     acceptance='not smoothness validation')
        return result


if __name__ == '__main__':
    raise SystemExit(main(Probe))
