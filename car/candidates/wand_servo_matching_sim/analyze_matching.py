#!/usr/bin/env python3
"""Review saved pure-model sweep; select two SIM ONLY gesture envelopes.

The minimum .30-.50m/s smooth-gesture setting and the extended .75m/s gesture
setting are intentionally distinguished. Neither is a live/hardware profile.
"""
import argparse
import csv
import json
from pathlib import Path

import matching_sim as m


def select_extended(results):
    by = {(r['limits'], r['case']): r for r in results}
    names = {r['limits'] for r in results}
    candidates = []
    for name in names:
        group = [r for r in results if r['limits'] == name]
        if len(group) != 9 or not all(r['passed'] for r in group):
            continue
        fast = by[name, 'fast_075_challenge']
        normal = [by[name, key] for key in ('gentle_030', 'normal_050', 'circle_040')]
        if (fast['rms_error_m'] <= .08 and fast['p95_error_m'] <= .12
                and all(v is not None for v in fast['reversal_delays_s'])
                and fast['reversal_delay_median_s'] <= .25
                and all(r['rms_error_m'] <= .07 and r['p95_error_m'] <= .10 for r in normal)
                and all(by[name, f'real{i}_center']['rms_error_m'] <=
                        .5*by['v0.20_a0.08', f'real{i}_center']['rms_error_m'] for i in range(1, 4))):
            candidates.append((fast['speed_mps'], fast['accel_mps2'], name))
    return min(candidates)[2] if candidates else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError('fresh review evidence required')
    source = json.loads((args.comparison/'summary.json').read_text())
    results = json.loads((args.comparison/'sweep.json').read_text())
    assert source['simulation_only'] is True and source['hardware_started'] is False
    for path, sha in source['input_sha256'].items():
        assert m.digest(Path(path)) == sha, f'input drift: {path}'
    cases = m.synthetic_cases()+m.recorded_cases()
    expected = {(m.SimOnlyLimits(v, a).name, c.name) for v in m.SPEEDS for a in m.ACCELS for c in cases}
    actual = {(r['limits'], r['case']) for r in results}
    assert len(results) == len(actual) == 225 and actual == expected
    minimum = m.select_candidate(results)
    assert minimum == source['selected_sim_only_limits']
    extended = select_extended(results)
    assert extended, 'no extended model preset met the screen'
    config = next(m.SimOnlyLimits(v, a) for v in m.SPEEDS for a in m.ACCELS
                  if m.SimOnlyLimits(v, a).name == extended)
    args.output.mkdir(parents=True)
    traces = {}
    sensitivity = []
    for name in dict.fromkeys(('v0.20_a0.08', minimum, extended, 'v1.00_a2.00')):
        c = next(m.SimOnlyLimits(v, a) for v in m.SPEEDS for a in m.ACCELS
                 if m.SimOnlyLimits(v, a).name == name)
        for case in cases:
            result, rows = m.run_case(c, case, keep_rows=True)
            assert result['passed']
            traces[name, case.name] = rows
            if name == extended:
                with (args.output/f'{case.name}_{name}.csv').open('w') as stream:
                    writer = csv.DictWriter(stream, fieldnames=rows[0])
                    writer.writeheader()
                    writer.writerows(rows)
    for lag, gains in ((.04, (1., 1., 1.)), (.12, (.98, .88, .95)), (.25, (.85, .75, .95))):
        for case in cases:
            if case.name in ('normal_050', 'fast_075_challenge', 'circle_040', 'real1_center'):
                result, _ = m.run_case(config, case, lag_s=lag, gains=gains)
                sensitivity.append(result)
    by = {(r['limits'], r['case']): r for r in results}
    concise = []
    for case in cases:
        baseline = by['v0.20_a0.08', case.name]
        r = by[extended, case.name]
        concise.append(dict(case=case.name, old_rms_m=baseline['rms_error_m'], new_rms_m=r['rms_error_m'],
                            improvement_fraction=1-r['rms_error_m']/baseline['rms_error_m'],
                            new_p95_m=r['p95_error_m'], new_peak_m=r['peak_error_m'],
                            new_moving_rms_m=r['moving_rms_error_m'],
                            target_clamped_fraction=r['target_clamped_fraction']))
    report = dict(simulation_only=True, hardware_started=False, firmware_changed=False,
        input_comparison=str(args.comparison.resolve()), input_sweep_sha256=m.digest(args.comparison/'sweep.json'),
        mapping_scale=1., nominal_region_m=[0., 3., 0., 3.], buffer_each_side_m=.50,
        minimum_smooth_gesture_preset=minimum, recommended_extended_gesture_preset=extended,
        extended_speed_mps=config.max_speed_mps, extended_accel_mps2=config.max_accel_mps2,
        extended_criteria='smooth .75m/s gesture RMS<=.08m, P95<=.12m, reversal median<=.25s; normal RMS<=.07m/P95<=.10m; each center real replay >=50% RMS reduction',
        classification='Engineering screens, not user acceptance or proof that every recorded hand jerk is followed',
        recommended_usage='Next step: real Sign -> SIM ONLY subjective A/B; no automatic real-vehicle migration',
        ambitious_5cm_all_normal_cases_passed=source['ambitious_5cm_all_normal_cases_passed'],
        unchanged_input_sha256=all(m.digest(Path(p)) == sha for p, sha in source['input_sha256'].items()),
        cases=concise, sensitivity=sensitivity,
        source_code_sha256={str(Path(p).resolve()): m.digest(Path(p)) for p in (m.__file__, __file__)})
    assert report['unchanged_input_sha256']
    (args.output/'recommendation.SIM_ONLY.json').write_text(json.dumps(report, indent=2)+'\n')
    plot(traces, concise, minimum, extended, args.output/'extended_comparison.png')
    print(json.dumps({k: report[k] for k in ('minimum_smooth_gesture_preset',
                     'recommended_extended_gesture_preset', 'unchanged_input_sha256')}, indent=2))


def plot(traces, concise, minimum, extended, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for ax, case in zip(axes.flat[:2], ('fast_075_challenge', 'real1_center')):
        base = traces['v0.20_a0.08', case]
        ax.plot([r['t'] for r in base], [r['hand_f'] for r in base], 'k--', label='1:1 rod target')
        for name in ('v0.20_a0.08', minimum, extended, 'v1.00_a2.00'):
            rows = traces[name, case]
            ax.plot([r['t'] for r in rows], [r['car_local_f']-1.5 for r in rows], label=name)
        ax.set(title=case, xlabel='Virtual time (s)', ylabel='Forward displacement (m)')
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    ax = axes[1, 0]
    group = [r for r in concise if not r['case'].endswith('corner')]
    ix = list(range(len(group)))
    ax.bar([i-.18 for i in ix], [r['old_rms_m']*100 for r in group], width=.36, label='old v.20/a.08')
    ax.bar([i+.18 for i in ix], [r['new_rms_m']*100 for r in group], width=.36, label=extended)
    ax.set(xticks=ix, xticklabels=[r['case'].replace('_center', '') for r in group],
           ylabel='Same-time XY RMS error (cm)', title='All unclipped center cases')
    ax.tick_params(axis='x', rotation=30)
    ax.legend(fontsize=8)
    ax.grid(axis='y', alpha=.25)
    ax = axes[1, 1]
    case = 'fast_075_challenge'
    for name in ('v0.20_a0.08', minimum, extended):
        rows = traces[name, case]
        ax.plot([r['t'] for r in rows], [r['plant_vf'] for r in rows], label=name)
    rows = traces[extended, case]
    ax.plot([r['t'] for r in rows], [.75*__import__('math').sin(2.5*r['t']) for r in rows],
            'k--', label='rod velocity')
    ax.set(title='Fast gesture reversal', xlabel='Virtual time (s)', ylabel='Forward velocity (m/s)')
    ax.grid(alpha=.25)
    ax.legend(fontsize=8)
    fig.suptitle('SIM ONLY | 1:1, fixed 3x3m | same inputs, no time-shift alignment | NOT physical capability')
    fig.savefig(path, dpi=140)
    plt.close(fig)


if __name__ == '__main__':
    main()
