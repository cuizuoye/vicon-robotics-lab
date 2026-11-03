"""Deterministic virtual-time scenarios; never creates a ROS node or socket."""

import argparse
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path

from mecanum_vicon_control.math_utils import wrap_angle, world_error_to_body
from .core import Pose, Velocity, WandSession
from .scenario import interpolate


class Plant:
    """Same 2D lag/gain model as the existing simulated_vicon_car node.

    This approximates tracking dynamics, not tire contact or motor torque.
    """
    def __init__(self, pose):
        self.pose = pose
        self.velocity = Velocity()

    def step(self, command, dt):
        alpha = -math.expm1(-dt / 0.12)
        self.velocity = Velocity(*(a + alpha*(b-a) for a, b in zip(
            asdict(self.velocity).values(), asdict(command).values())))
        vx, vy, wz = self.velocity.vx*.98, self.velocity.vy*.88, self.velocity.wz*.95
        c, s = math.cos(self.pose.yaw), math.sin(self.pose.yaw)
        self.pose = Pose(
            self.pose.x + (c*vx-s*vy)*dt,
            self.pose.y + (s*vx+c*vy)*dt,
            self.pose.z, wrap_angle(self.pose.yaw+wz*dt))


def wand_at(local, yaw, t):
    c, s = math.cos(yaw), math.sin(yaw)
    f, l = local
    return Pose(-.4+c*f-s*l, -.8+s*f+c*l, 1+.12*math.sin(t), math.sin(t))


def simulate_case(name, yaw, keyframes, disturbance=None):
    dt = .02
    plant = Plant(Pose(1.0, -.5, 0.0, yaw))
    origin = plant.pose
    session = WandSession()
    session.observe('car', plant.pose, 0.0)
    session.observe('wand', wand_at((0, 0), yaw, 0), 0.0)
    assert session.engage(0.0)[0]
    checkpoints = {b[0] for a, b in zip(keyframes, keyframes[1:]) if a[1:] == b[1:]}
    rows, errors = [], []
    max_speed = max_accel = 0.0
    old_command = Velocity()
    disturbed = False
    for step in range(1, round(keyframes[-1][0]/dt)+1):
        t = step*dt
        if disturbance and t >= disturbance[0] and not disturbed:
            _, dx, dy, dyaw = disturbance
            plant.pose = Pose(plant.pose.x+dx, plant.pose.y+dy, 0,
                              wrap_angle(plant.pose.yaw+dyaw))
            disturbed = True
        local = interpolate(t, keyframes)
        session.observe('car', plant.pose, t)
        session.observe('wand', wand_at(local, yaw, t), t)
        command = session.tick(t)
        if session.state != 'ACTIVE':
            raise AssertionError(f'{name}: {session.reason} at t={t}')
        speed = math.hypot(command.vx, command.vy)
        accel = math.hypot(command.vx-old_command.vx, command.vy-old_command.vy)/dt
        max_speed, max_accel = max(max_speed, speed), max(max_accel, accel)
        old_command = command
        plant.step(command, dt)
        actual = world_error_to_body(plant.pose.x-origin.x, plant.pose.y-origin.y, yaw)
        goal = world_error_to_body(session.goal.x-origin.x, session.goal.y-origin.y, yaw)
        rows.append({
            'time_s': t, 'wand_forward_m': local[0], 'wand_left_m': local[1],
            'goal_forward_m': goal[0], 'goal_left_m': goal[1],
            'car_forward_m': actual[0], 'car_left_m': actual[1],
            'speed_mps': speed, 'yaw_error_rad': wrap_angle(yaw-plant.pose.yaw),
        })
        if any(abs(t-point) < dt/2 for point in checkpoints):
            errors.append({'time_s': t, 'error_m': math.hypot(
                actual[0]-2*local[0], actual[1]-2*local[1])})
    summary = {
        'name': name, 'initial_yaw_deg': math.degrees(yaw),
        'duration_virtual_s': keyframes[-1][0], 'checkpoints': errors,
        'max_settled_error_m': max(e['error_m'] for e in errors),
        'max_speed_mps': max_speed, 'max_acceleration_mps2': max_accel,
        'final_yaw_error_rad': rows[-1]['yaw_error_rad'],
        'disturbance_applied': disturbed,
    }
    summary['passed'] = (
        summary['max_settled_error_m'] <= .025
        and max_speed <= .15+1e-9 and max_accel <= .30+1e-9
        and abs(summary['final_yaw_error_rad']) <= .03)
    return summary, rows


def scenarios():
    return [
        ('forward_return', 0.0,
         [(0, 0, 0), (10, .5, 0), (16, .5, 0), (26, 0, 0), (32, 0, 0)], None),
        ('left_return_yaw90', math.pi/2,
         [(0, 0, 0), (5, 0, .25), (11, 0, .25), (16, 0, 0), (22, 0, 0)], None),
        ('diagonal_return_yaw60', math.pi/3,
         [(0, 0, 0), (2, .4, .3), (16, .4, .3), (18, 0, 0), (32, 0, 0)], None),
        ('disturbance_hold_yaw_minus40', math.radians(-40),
         [(0, 0, 0), (10, .5, 0), (24, .5, 0), (34, 0, 0), (40, 0, 0)],
         (12, .25, .20, .30)),
    ]


def plot_results(all_rows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(all_rows), 3, figsize=(13, 11), constrained_layout=True)
    for row_axes, (name, rows) in zip(axes, all_rows):
        t = [r['time_s'] for r in rows]
        ax = row_axes[0]
        ax.plot([r['goal_forward_m'] for r in rows], [r['goal_left_m'] for r in rows],
                '--', color='darkorange', label='target = 2 x wand')
        ax.plot([r['car_forward_m'] for r in rows], [r['car_left_m'] for r in rows],
                color='tab:blue', label='simulated car')
        ax.set(xlabel='Start-frame forward (m)', ylabel='Start-frame left (m)', title=name)
        ax.axis('equal')
        ax.legend(fontsize=8)
        ax = row_axes[1]
        for axis, color in [('forward', 'tab:blue'), ('left', 'tab:green')]:
            ax.plot(t, [2*r[f'wand_{axis}_m'] for r in rows], '--', color=color)
            ax.plot(t, [r[f'car_{axis}_m'] for r in rows], color=color, label=f'car {axis}')
        ax.set(xlabel='Virtual time (s)', ylabel='Displacement (m)')
        ax.legend(fontsize=8)
        ax = row_axes[2]
        ax.plot(t, [r['speed_mps'] for r in rows], color='tab:purple')
        ax.axhline(.15, color='red', linestyle='--', label='0.15 m/s limit')
        ax.set(xlabel='Virtual time (s)', ylabel='Command speed (m/s)', ylim=(-.01, .18))
        ax.legend(fontsize=8)
        for ax in row_axes:
            ax.grid(alpha=.25)
    fig.suptitle('Offline wand control: scale 2, fixed engage-time car heading', fontsize=14)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='wand_offline_results')
    args = parser.parse_args()
    destination = Path(args.output).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    summaries, all_rows = [], []
    for case in scenarios():
        summary, rows = simulate_case(*case)
        summaries.append(summary)
        all_rows.append((summary['name'], rows))
        with (destination / (summary['name']+'.csv')).open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    report = {
        'verification': 'SOFTWARE SIMULATION ONLY; not physical Vicon or ground validation',
        'scale': 2.0, 'config': asdict(WandSession().config), 'cases': summaries,
        'passed': all(s['passed'] for s in summaries),
    }
    (destination / 'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    plot_results(all_rows, destination / 'wand_mapping_simulation.png')
    print(json.dumps(report, indent=2))
    print(f'Results: {destination}')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
