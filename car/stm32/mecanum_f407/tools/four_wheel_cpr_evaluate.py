#!/usr/bin/env python3
"""两个静止只读记录 + 操作者实数完整输出轴圈数 -> CPR初测估计。

纯离线：不打开串口、不写固件/参数。手转区间没有连续采样，不能当作动态验收。
"""
import argparse
import json
from pathlib import Path

from four_wheel_bench_probe import WHEELS, stopped_four
from front_encoder_observe import count_delta


def validate_snapshot(record):
    if (record.get('test') != 'four_wheel_powered_stationary_observation'
            or record.get('firmware') != 'four_bench_v1'
            or record.get('passed') is not True or record.get('errors') != []
            or record.get('software_stop_verified') is not True
            or record.get('requires_power_disconnect') is not False
            or record.get('command_order') != ['stop', 'status', 'stop', 'status']
            or record.get('observation_samples', 0) < 45):
        raise ValueError('requires successful powered-stationary stop/status-only record')
    end = record['end_state']
    samples = record['raw_samples']
    if len(samples) < 45:
        raise ValueError('missing raw stationary telemetry')
    for row in [record['start_state'], end] + samples:
        if (row.get('fw') != 'four_bench_v1' or not stopped_four(row)
                or type(row['ms']) is not int or not 0 <= row['ms'] <= 0xffffffff
                or row['fault'] != 0):
            raise ValueError('wrong firmware, output, timestamp or fault')
        for wheel in WHEELS:
            key = 'enc_' + wheel.lower()
            if (type(row[key]) is not int or not -(1 << 31) <= row[key] < (1 << 31)
                    or row['cps_' + wheel.lower()] != 0 or row[key] != end[key]):
                raise ValueError('snapshot counts/CPS must be stationary and valid')
    return end


def evaluate(before, after, *, turns, encoder_signs, direction,
             manual_only=False, no_reset_or_power_change=False,
             integer_output_turns=False):
    if not (manual_only is True and no_reset_or_power_change is True
            and integer_output_turns is True):
        raise ValueError('operator must confirm manual-only, no reset/power change and actual integer output turns')
    if (not turns or not set(turns) <= set(WHEELS)
            or set(encoder_signs) != set(turns) or direction not in ('forward', 'reverse')):
        raise ValueError('invalid wheels, direction or incomplete encoder signs')
    for wheel, value in turns.items():
        if type(value) is not int or not 1 <= value <= 20:
            raise ValueError('actual output turns must be integers 1..20')
        if type(encoder_signs[wheel]) is not int or encoder_signs[wheel] not in (-1, 1):
            raise ValueError('encoder signs must be independently established +/-1')
    start, end = validate_snapshot(before), validate_snapshot(after)
    elapsed = (end['ms'] - start['ms']) & 0xffffffff
    if not 0 < elapsed <= 3600000:
        raise ValueError('timestamps inconsistent with a no-reset manual interval of at most one hour')
    deltas = {w: count_delta(end['enc_' + w.lower()], start['enc_' + w.lower()]) for w in WHEELS}
    if any(deltas[w] != 0 for w in WHEELS if w not in turns):
        raise ValueError('unmeasured wheel count changed; do not infer its CPR')
    estimates = {}
    expected_sign = 1 if direction == 'forward' else -1
    for wheel, number in turns.items():
        normalized = deltas[wheel] * encoder_signs[wheel]
        if normalized * expected_sign <= 0:
            raise ValueError('zero counts or direction/sign disagreement for ' + wheel)
        estimates[wheel] = {'operator_counted_output_turns': number,
                            'operator_observed_direction': direction,
                            'encoder_sign': encoder_signs[wheel],
                            'raw_count_delta': deltas[wheel],
                            'normalized_count_delta': normalized,
                            'counts_per_output_revolution_estimate': abs(deltas[wheel]) / number}
    return {'test': 'four_wheel_manual_cpr_initial_estimate', 'passed': True,
            'firmware': 'four_bench_v1', 'start_ms': start['ms'], 'end_ms': end['ms'],
            'elapsed_between_endpoints_ms': elapsed, 'raw_count_delta': deltas,
            'estimates': estimates, 'runtime_parameters_written': False,
            'repeatability_verified': False,
            'operator_confirmations': {'manual_only': True, 'no_reset_or_power_change': True,
                                       'integer_output_turns': True},
            'measurement_scope': 'operator-counted complete output turns and stationary endpoints; no continuous telemetry between endpoints, no automatic CPR write or speed-loop enable'}


def pairs(items, parser):
    result = {}
    for item in items:
        try:
            wheel, text = item.split('=', 1)
            value = int(text)
        except ValueError:
            parser.error('use WHEEL=integer')
        if wheel not in WHEELS or wheel in result:
            parser.error('unknown or duplicate wheel')
        result[wheel] = value
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('before', type=Path)
    p.add_argument('after', type=Path)
    p.add_argument('--turns', nargs='+', required=True)
    p.add_argument('--encoder-signs', nargs='+', required=True)
    p.add_argument('--direction', choices=('forward', 'reverse'), required=True)
    p.add_argument('--confirm-manual-only', action='store_true')
    p.add_argument('--confirm-no-reset-or-power-change', action='store_true')
    p.add_argument('--confirm-integer-output-turns', action='store_true')
    args = p.parse_args(argv)
    turns, signs = pairs(args.turns, p), pairs(args.encoder_signs, p)
    try:
        result = evaluate(json.loads(args.before.read_text()), json.loads(args.after.read_text()),
                          turns=turns, encoder_signs=signs, direction=args.direction,
                          manual_only=args.confirm_manual_only,
                          no_reset_or_power_change=args.confirm_no_reset_or_power_change,
                          integer_output_turns=args.confirm_integer_output_turns)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        result = {'passed': False, 'error': str(exc), 'runtime_parameters_written': False}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
