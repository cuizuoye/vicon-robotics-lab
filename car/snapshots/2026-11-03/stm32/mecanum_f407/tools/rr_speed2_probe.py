#!/usr/bin/env python3
"""仅RR正向328cps对照；2倍目标不等于实测2倍，输出仍限10%。"""
from drive_pi_probe import parse_status_for_firmware
from rr_diagnostic_probe import main, parse_trace as base_trace
from rr_pwm_fine_probe import FIELDS, Probe as FineProbe

FIRMWARE = 'rr_diag_328_v1'
MODES = {'rr328': (5, 'run RR328 S10', (0, 0, 0), (0, 0, 0, 0))}


def targets(_mode):
    return dict(fl=0, fr=0, rl=0, rr=328)


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
    return base_trace(line, duty_quantum=1, pwm_period_counts=8400, target_limit=328)


class Probe(FineProbe):
    firmware = FIRMWARE
    modes = MODES
    target_values = staticmethod(targets)
    parse_frame = staticmethod(parse_status)
    parse_trace_frame = staticmethod(parse_trace)

    def run(self):
        result = super().run()
        result['test'] = result['test'].replace('rr_pwm_fine_', 'rr_speed2_', 1)
        result['speed_profile'] = dict(target_cps=328, reference_target_cps=164,
                                       target_ratio=2, cpr=1061, target_rpm=328*60/1061,
                                       max_output_permille=100,
                                       note='actual speed and physical smoothness require observation')
        result['scope'] = 'RR-only suspended 2x-target comparison; other wheels zero; no automatic retry'
        return result


if __name__ == '__main__':
    raise SystemExit(main(Probe))
