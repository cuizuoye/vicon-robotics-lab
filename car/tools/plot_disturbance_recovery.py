#!/usr/bin/env python3
"""Plot the deterministic disturbance-recovery CSV verification artifact."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_rows(path):
    with Path(path).open(newline='') as file:
        rows = list(csv.DictReader(file))
    required = {
        'time_s', 'x_m', 'y_m', 'cross_track_m', 'state',
        'cmd_vx_mps', 'cmd_vy_mps', 'safety_enabled',
    }
    if not rows or not required.issubset(rows[0]):
        raise ValueError('CSV does not contain disturbance-recovery samples')
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rows = read_rows(args.input)

    time_s = [float(row['time_s']) for row in rows]
    x_m = [float(row['x_m']) for row in rows]
    y_m = [float(row['y_m']) for row in rows]
    cross = [float(row['cross_track_m']) if row['cross_track_m'] else float('nan')
             for row in rows]
    vx = [float(row['cmd_vx_mps']) for row in rows]
    vy = [float(row['cmd_vy_mps']) for row in rows]
    recovery = [row['state'] == 'RECOVERY' for row in rows]

    figure, axes = plt.subplots(3, 1, figsize=(10, 10), constrained_layout=True)
    axes[0].plot(x_m, y_m, color='#22bb55', linewidth=2.0, label='simulated actual path')
    axes[0].plot([min(x_m), max(x_m)], [0.0, 0.0], '--', color='#3377ff',
                 linewidth=1.5, label='active reference segment')
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].set_xlabel('world x [m]')
    axes[0].set_ylabel('world y [m]')
    axes[0].set_title('20 cm lateral disturbance and return to the same segment')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best')

    axes[1].plot(time_s, cross, color='#cc3344', linewidth=1.8, label='cross-track error')
    for value, color, label in [
            (0.08, '#ee8833', 'recovery enter'),
            (0.03, '#44aa66', 'recovery exit')]:
        axes[1].axhline(value, linestyle='--', color=color, linewidth=1.0, label=label)
        axes[1].axhline(-value, linestyle='--', color=color, linewidth=1.0)
    axes[1].fill_between(time_s, -0.24, 0.24, where=recovery,
                         color='#ff6666', alpha=0.12, label='RECOVERY state')
    axes[1].set_ylim(-0.24, 0.24)
    axes[1].set_ylabel('cross-track error [m]')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best', ncol=2)

    axes[2].plot(time_s, vx, label='safe cmd vx', color='#2255aa')
    axes[2].plot(time_s, vy, label='safe cmd vy', color='#aa33aa')
    axes[2].axhline(0.10, linestyle=':', color='#555555', label='translation cap')
    axes[2].axhline(-0.10, linestyle=':', color='#555555')
    axes[2].set_xlabel('time [s]')
    axes[2].set_ylabel('body velocity [m/s]')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='best')

    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    print(output)


if __name__ == '__main__':
    main()
