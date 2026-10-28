#!/usr/bin/env python3
"""Analyze recorded real Sign movement, independent of movement order."""

import argparse
import csv
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-dir', type=Path, required=True)
    args = parser.parse_args()
    folder = args.record_dir.resolve()
    with (folder/'mapping.csv').open() as stream:
        all_rows = [r for r in csv.DictReader(stream) if r.get('markers_visible') is not None]
    source_summary = json.loads((folder/'summary.json').read_text())
    rows = []
    fields = ('elapsed_s', 'wand_x', 'wand_y', 'wand_z', 'wand_forward_m',
              'wand_left_m', 'target_x', 'target_y', 'target_forward_m', 'target_left_m')
    for row in all_rows:
        if row['state'] != 'PREVIEW' or not row['target_x']:
            continue
        try:
            value = {key: float(row[key]) for key in fields}
            value['markers_visible'] = int(row['markers_visible'])
        except (ValueError, TypeError):
            continue
        if all(math.isfinite(value[key]) for key in fields):
            rows.append(value)
    if len(rows) < 10:
        raise RuntimeError('not enough valid preview rows')
    if source_summary['last_status']['capture_count'] != 1:
        raise RuntimeError('multiple anchors require separate segment analysis')

    first = rows[0]
    numerator = denominator = max_error = max_step = 0.0
    active_times = []
    for old, row in zip(rows, rows[1:]):
        dt = row['elapsed_s']-old['elapsed_s']
        step = math.hypot(row['wand_x']-old['wand_x'], row['wand_y']-old['wand_y'])
        max_step = max(max_step, step)
        if dt > 0 and step/dt > .03:
            active_times.append(row['elapsed_s'])
    for row in rows:
        dx, dy = row['wand_x']-first['wand_x'], row['wand_y']-first['wand_y']
        gx, gy = row['target_x']-first['target_x'], row['target_y']-first['target_y']
        numerator += dx*gx+dy*gy
        denominator += dx*dx+dy*dy
        max_error = max(max_error, math.hypot(gx-2*dx, gy-2*dy))
    fitted_scale = numerator/denominator if denominator else None
    max_wand = max(math.hypot(r['wand_forward_m'], r['wand_left_m']) for r in rows)
    max_target = max(math.hypot(r['target_forward_m'], r['target_left_m']) for r in rows)
    last = rows[-1]
    observed_bad_states = sorted({r['state'] for r in all_rows}
                                 - {'WAITING', 'PREVIEW'})
    report = {
        'verification': 'REAL SIGN MOVEMENT; READONLY PREVIEW ONLY',
        'movement_order_required': False,
        'snapshot_end_elapsed_s': last['elapsed_s'],
        'valid_mapped_samples': len(rows),
        'motion_window_s': [min(active_times)-2, max(active_times)+2] if active_times else None,
        'large_motion_observed': max_wand > .1,
        'fitted_world_displacement_scale': fitted_scale,
        'max_independent_world_mapping_error_m': max_error,
        'max_wand_radial_displacement_m': max_wand,
        'max_target_radial_displacement_m': max_target,
        'wand_forward_range_m': [min(r['wand_forward_m'] for r in rows),
                                 max(r['wand_forward_m'] for r in rows)],
        'wand_left_range_m': [min(r['wand_left_m'] for r in rows),
                              max(r['wand_left_m'] for r in rows)],
        'final_wand_offset_m': [last['wand_forward_m'], last['wand_left_m']],
        'final_target_offset_m': [last['target_forward_m'], last['target_left_m']],
        'final_wand_distance_from_original_origin_m': math.hypot(
            last['wand_forward_m'], last['wand_left_m']),
        'final_target_distance_from_original_origin_m': math.hypot(
            last['target_forward_m'], last['target_left_m']),
        'all_five_markers_visible_fraction': sum(r['markers_visible'] == 5 for r in rows)/len(rows),
        'unexpected_states': observed_bad_states,
        'source_streams': source_summary['streams'],
        'first_10s_baseline': source_summary['first_10s_baseline'],
        'limitations': [
            'Human hand motion order and distance were approximate, not surveyed.',
            'A remaining target offset is twice the remaining wand offset, not a tracking error.',
            'No actuator or physical car following is verified by this report.',
            'Intervals shorter than the sampling period may not be visible in the 20Hz CSV.',
        ],
    }
    report['passed'] = (
        report['large_motion_observed'] and abs(fitted_scale-2) < 1e-9
        and max_error < 1e-9 and not observed_bad_states
        and all(s['invalid'] == 0 and s['max_receive_gap_s'] < .20
                for s in source_summary['streams'].values()))
    with (folder/'movement_snapshot.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (folder/'movement_acceptance.json').write_text(json.dumps(report, indent=2)+'\n')
    draw(rows, report, folder/'real_sign_movement.png')
    print(json.dumps({k: v for k, v in report.items()
                      if k not in ('first_10s_baseline', 'source_streams')}, indent=2))
    return 0 if report['passed'] else 1


def draw(rows, report, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    interval = report['motion_window_s']
    selected = [r for r in rows if interval[0] <= r['elapsed_s'] <= interval[1]] if interval else rows
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), constrained_layout=True)
    axes[0].plot([r['wand_forward_m'] for r in selected],
                 [r['wand_left_m'] for r in selected], label='Real wand', color='tab:green')
    axes[0].plot([r['target_forward_m'] for r in selected],
                 [r['target_left_m'] for r in selected], label='Preview target', color='darkorange')
    axes[0].scatter([0], [0], marker='+', color='tab:blue', s=80, label='Fixed car anchor')
    axes[0].set(xlabel='Start-frame forward (m)', ylabel='Start-frame left (m)',
                title='Real hand movement; target = 2 x displacement')
    axes[0].axis('equal')
    t = [r['elapsed_s'] for r in selected]
    for ax, name in zip(axes[1:], ('forward', 'left')):
        ax.plot(t, [r[f'wand_{name}_m'] for r in selected], label='Real wand', color='tab:green')
        ax.plot(t, [r[f'target_{name}_m'] for r in selected],
                label='Preview target', color='darkorange')
        ax.set(xlabel='Elapsed time from preview start (s)', ylabel='Displacement (m)',
               title=f'{name.capitalize()} displacement; scale {report["fitted_world_displacement_scale"]:.6f}')
    for ax in axes:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.suptitle('Real Sign / Vicon read-only preview - physical car stayed unpowered', fontsize=13)
    fig.savefig(output, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    raise SystemExit(main())
