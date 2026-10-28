#!/usr/bin/env python3
"""Analyze recorded physical Sign loss; no ROS connections or commands.

The bridge journal contains filtered wand offsets, not independent goal poses.
Plots explicitly reconstruct the expected 2x goal from those offsets. The
separate resume probe and automatic acceptance provide the goal-pose checks.
"""

import argparse
import ast
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(folder, manual_start):
    with (folder/'live_simulation.csv').open() as stream:
        raw = list(csv.DictReader(stream))
    # Take a snapshot while the live writer may still be running.
    rows = [r for r in raw if all(r.get(k) not in (None, '') for k in
            ('elapsed_s', 'virtual_x', 'virtual_y', 'wand_forward_m',
             'wand_left_m', 'speed_mps', 'source_stamp_ns'))]
    for r in rows:
        for k in ('elapsed_s', 'virtual_x', 'virtual_y', 'wand_forward_m',
                  'wand_left_m', 'speed_mps', 'real_wand_x', 'real_wand_y'):
            r[k] = float(r[k])
    summary = json.loads((folder/'summary.json').read_text())
    resume = json.loads((folder/'physical_resume_acceptance.json').read_text())
    fault = next(r for previous, r in zip(rows, rows[1:])
                 if r['elapsed_s'] > manual_start and r['synthetic_occlusion'] == '0'
                 and previous['state'] == 'ACTIVE' and r['state'] == 'FAULT')
    fault_t = fault['elapsed_s']
    origin = next(r for previous, r in reversed(list(zip(rows, rows[1:])))
                  if r['elapsed_s'] < fault_t and previous['state'] != 'ACTIVE'
                  and r['state'] == 'ACTIVE')
    resumed = next(r for r in rows if r['elapsed_s'] > fault_t and r['state'] == 'ACTIVE')
    manual = [r for r in rows if origin['elapsed_s'] <= r['elapsed_s'] < fault_t]
    latched = [r for r in rows if fault_t <= r['elapsed_s'] < resumed['elapsed_s']]
    intervals, start = [], None
    for r in rows:
        if not (fault_t <= r['elapsed_s'] <= resumed['elapsed_s']):
            continue
        missing = r['real_wand_fresh'] == '0'
        if missing and start is None:
            start = r
        elif not missing and start is not None:
            intervals.append({'freshness_false_start_s': start['elapsed_s'],
                'fresh_again_s': r['elapsed_s'],
                'duration_s': r['elapsed_s']-start['elapsed_s'],
                'already_latched_at_start': start is not fault})
            start = None
    last_frame_first_seen = next(r for r in rows
                                if r['source_stamp_ns'] == fault['source_stamp_ns'])
    # Only the first ACTIVE -> FAULT has a stop latency. Later gaps occur
    # while already latched; assigning them another latency is misleading.
    reason = next(e['reason'] for e in summary['events']
                  if e.get('controller_state') == 'FAULT'
                  and abs(e['elapsed_s']-fault_t) < .05)
    source_ages = ast.literal_eval(reason.split('source_age_s=', 1)[1])
    scale = summary['last_controller_status']['scale']
    yaw = summary['last_controller_status']['anchor_yaw_rad']
    c, s = math.cos(yaw), math.sin(yaw)

    def car_local(row):
        dx, dy = row['virtual_x']-origin['virtual_x'], row['virtual_y']-origin['virtual_y']
        return c*dx+s*dy, -s*dx+c*dy

    report = {
        'verification': 'REAL HAND MOTION AND PHYSICAL SIGN OCCLUSION; SOFTWARE CAR ONLY',
        'passed': False,
        'scale_configured': scale,
        'manual_origin_elapsed_s': origin['elapsed_s'],
        'physical_fault_elapsed_s': fault_t,
        'physical_fault_reason': reason,
        'wand_source_age_at_fault_s': source_ages['wand'],
        'car_source_age_at_fault_s': source_ages['car'],
        'journal_first_observation_of_last_frame_to_fault_approx_s': fault_t-last_frame_first_seen['elapsed_s'],
        'maximum_command_speed_mps': max(r['speed_mps'] for r in rows),
        'manual_maximum_command_speed_mps': max(r['speed_mps'] for r in manual),
        'manual_max_filtered_wand_radius_m': max(math.hypot(r['wand_forward_m'], r['wand_left_m']) for r in manual),
        'manual_max_virtual_displacement_m': max(math.hypot(*car_local(r)) for r in manual),
        'physical_source_missing_intervals': intervals,
        'synthetic_occlusion_during_latched_period': any(r['synthetic_occlusion'] != '0' for r in latched),
        'zero_output_throughout_latched_period': all(r['state'] == 'FAULT' and r['speed_mps'] == 0 for r in latched),
        'fresh_source_did_not_auto_resume': any(r['real_wand_fresh'] == '1' for r in latched)
            and all(r['state'] == 'FAULT' for r in latched),
        'latched_until_explicit_resume_s': resumed['elapsed_s']-fault_t,
        'explicit_resume': resume,
        'limitations': [
            'Stop time is output-zero/FAULT, not instantaneous physical braking.',
            'Plot targets are reconstructed from logged filtered wand displacement and configured scale.',
            'The late-started live_observer did not record the physical loss; use the bridge journal for it.',
            'This does not validate physical car dynamics, wireless transport, or actuation.'
        ],
    }
    report['passed'] = (report['maximum_command_speed_mps'] <= .15+1e-9
        and report['manual_max_virtual_displacement_m'] > .1 and scale == 2
        and .2 <= source_ages['wand'] < .3 and source_ages['car'] < .2
        and not report['synthetic_occlusion_during_latched_period']
        and report['zero_output_throughout_latched_period']
        and report['fresh_source_did_not_auto_resume'] and resume['passed'])
    selected = [r for r in rows if manual_start <= r['elapsed_s'] <= fault_t+12]
    moving = [r for r in selected if r['speed_mps'] > .01]
    plot_start = max(manual_start, moving[0]['elapsed_s']-3) if moving else manual_start
    selected = [r for r in selected if r['elapsed_s'] >= plot_start]
    t = [r['elapsed_s']-plot_start for r in selected]
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    for i, (key, label) in enumerate((('wand_forward_m', 'Forward'), ('wand_left_m', 'Left'))):
        axes[i].plot(t, [scale*r[key] if r['state'] == 'ACTIVE' else math.nan for r in selected],
                     color='darkorange', label='Expected goal: 2 x filtered wand')
        axes[i].plot(t, [car_local(r)[i] for r in selected], color='royalblue', label='Virtual car')
        axes[i].set_ylabel(f'{label} (m)')
    axes[0].legend(loc='best')
    axes[2].plot(t, [r['speed_mps'] for r in selected], color='royalblue', label='Command speed')
    axes[2].axhline(.15, color='darkorange', linestyle='--', label='0.15 m/s limit')
    axes[2].set_ylabel('Speed (m/s)')
    axes[2].set_xlabel('Seconds from displayed motion window')
    axes[2].legend(loc='best')
    for ax in axes:
        ax.axvspan(fault_t-plot_start, t[-1], color='gray', alpha=.12)
        for interval in intervals:
            ax.axvspan(interval['freshness_false_start_s']-plot_start,
                       min(interval['fresh_again_s']-plot_start, t[-1]), color='red', alpha=.15)
        ax.grid(alpha=.2)
    axes[0].set_title('Real Sign -> simulated car | physical occlusion, no relay injection\n'
                      'Gray: latched stop; red: actual source stale; no auto-resume')
    fig.savefig(folder/'physical_occlusion_validation.png', dpi=150)
    plt.close(fig)
    (folder/'physical_occlusion_acceptance.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--manual-start', type=float, default=30)
    args = parser.parse_args()
    raise SystemExit(analyze(args.folder, args.manual_start))
