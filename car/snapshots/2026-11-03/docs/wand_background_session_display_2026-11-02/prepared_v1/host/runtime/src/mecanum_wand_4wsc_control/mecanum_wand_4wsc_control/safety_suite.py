"""Deterministic fault-injection simulation; no ROS or hardware I/O."""

import argparse
import csv
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

from .core import Pose, Velocity
from .offline_suite import Plant
from .safety_core import FinalGate, Lease, SafetyConfig, Supervisor, ZERO


class Harness:
    def __init__(self, config=None, origin=None):
        self.c = config or SafetyConfig()
        self.sup, self.gate = Supervisor(self.c), FinalGate(self.c)
        self.plant = Plant(origin or Pose(0, 0, self.c.ground_z_m))
        self.now, self.ros = 10., 1000.
        self.sequence = 0
        self.hand = Pose(-.4, -.4, 1.)
        self.held = False
        self.pose_on = {'car': True, 'wand': True}
        self.lease_on = self.feedback_on = self.supervisor_on = self.gate_on = True
        self.command_link_on = True
        self.frozen_pose, self.frozen_lease = {}, None
        self.roll = self.pitch = 0.
        self.fault_bits = 0
        self.last_gate_receive = self.now
        self.plant_target = ZERO
        self.rows = []
        self.modify_frame = lambda f: f
        self.output = ZERO

    def step(self, dt=.02, ros_step=None):
        self.now += dt
        self.ros += dt if ros_step is None else ros_step
        self.sequence += 1
        for inputs in (self.sup.inputs, self.gate.inputs):
            for source, pose in (('car', self.plant.pose), ('wand', self.hand)):
                if self.pose_on[source]:
                    inputs.observe_pose(source, pose, self.frozen_pose.get(source, self.ros),
                                        self.now, self.ros,
                                        self.roll if source == 'car' else 0.,
                                        self.pitch if source == 'car' else 0.)
            if self.lease_on:
                lease = self.frozen_lease or Lease('operator-test', self.sequence,
                                                  self.now, self.ros, self.held)
                inputs.observe_lease(lease, self.now, self.ros)
            if self.feedback_on:
                inputs.observe_feedback(self.sequence, self.now, self.ros, self.now, self.ros,
                    output_zero=self.plant_target == ZERO, enabled=self.plant_target != ZERO,
                    speed=math.hypot(self.plant.velocity.vx, self.plant.velocity.vy),
                    yaw_rate=abs(self.plant.velocity.wz), fault_bits=self.fault_bits)
        if self.supervisor_on:
            self.frame = self.sup.tick(self.now, self.ros)
            if self.command_link_on:
                self.gate.receive(self.modify_frame(self.frame), self.now, self.ros)
        if self.gate_on:
            self.output = self.gate.tick(self.now, self.ros)
            self.plant_target, self.last_gate_receive = self.output, self.now
            self.sup.observe_gate(steady=self.now, ros=self.ros,
                                  epoch=self.gate.primed_epoch, fault=self.gate.fault,
                                  hard_fault=self.gate.hard_fault)
        if self.now-self.last_gate_receive > .15:
            self.plant_target = ZERO  # Independent simulated actuator command watchdog.
        self.plant.step(self.plant_target, max(0., dt))
        goal = self.sup.follower.goal
        self.rows.append(dict(time_s=self.now-10, state=self.sup.state,
            reason=self.sup.reason, gate_fault=self.gate.fault, held=self.held,
            car_x=self.plant.pose.x, car_y=self.plant.pose.y,
            goal_x=goal.x if goal else math.nan, goal_y=goal.y if goal else math.nan,
            command_speed_mps=math.hypot(self.output.vx, self.output.vy),
            plant_speed_mps=math.hypot(self.plant.velocity.vx, self.plant.velocity.vy),
            plant_target_speed_mps=math.hypot(self.plant_target.vx, self.plant_target.vy),
            stop_confirmed=self.sup.stop_confirmed))
        return self.output

    def run(self, seconds):
        for _ in range(round(seconds/.02)):
            self.step()

    def start(self):
        self.run(1.8)  # 0.5s motion history + 0.5s still + 0.6s released-ready.
        ok, reason = self.sup.request_start(self.now, self.ros)
        assert ok, (reason, self.sup.inputs.health(self.now, self.ros))
        self.run(.10)
        assert self.gate.primed_epoch == self.sup.epoch
        self.held = True
        self.step()
        assert self.sup.state == 'ACTIVE', self.sup.reason

    def moving(self):
        self.start()
        for i in range(1, 51):
            self.hand = Pose(-.4+.002*i, -.4, 1.)
            self.step()
        assert math.hypot(self.output.vx, self.output.vy) > .03


def normal_case():
    h = Harness()
    h.start()
    origin = h.plant.pose
    for i in range(1, 101):
        h.hand = Pose(-.4+.001*i, -.4+.0005*i, 1.)
        h.step()
    h.run(6.)
    expected = (origin.x+.2, origin.y+.1)
    error = math.dist((h.plant.pose.x, h.plant.pose.y), expected)
    goal = h.sup.follower.goal
    assert goal and math.dist((goal.x, goal.y), expected) < 1e-6
    assert error < .025 and h.sup.state == 'ACTIVE', (error, h.sup.reason, h.gate.fault)
    for i in range(1, 101):
        h.hand = Pose(-.3-.001*i, -.35-.0005*i, 1.)
        h.step()
    h.run(6.)
    returned = math.dist((h.plant.pose.x, h.plant.pose.y), (origin.x, origin.y))
    assert returned < .025
    max_speed = max(r['command_speed_mps'] for r in h.rows)
    assert max_speed <= h.c.max_speed_mps+1e-9
    h.held = False
    h.run(2.)
    assert h.sup.state == 'PAUSED' and h.sup.stop_confirmed
    h.hand = Pose(.5, .3, 1.)  # New origin after explicit restart, not an old goal chase.
    h.run(.8)
    parked = h.plant.pose
    ok, reason = h.sup.request_start(h.now, h.ros)
    assert ok, reason
    h.run(.1)
    h.held = True
    h.run(.3)
    drift = math.dist((h.plant.pose.x, h.plant.pose.y), (parked.x, parked.y))
    assert drift < .001 and h.sup.state == 'ACTIVE'
    return {'name': '2x_diagonal_return_pause_rebase', 'passed': True,
            'settled_error_m': error, 'return_error_m': returned,
            'rebase_drift_m': drift, 'max_speed_mps': max_speed}, h.rows


FAULT_CASES = (
    'wand_loss', 'car_loss', 'frozen_pose', 'operator_release', 'operator_loss',
    'operator_replay', 'supervisor_freeze', 'command_link_loss', 'delayed_command',
    'command_replay', 'diagonal_overspeed', 'oversized_yaw', 'acceleration_jump',
    'lift', 'tilt', 'car_jump', 'car_boundary', 'goal_boundary', 'mode_conflict',
    'duplicate_source', 'feedback_loss', 'actuator_fault', 'estop',
    'ros_clock_backwards', 'ros_clock_forward', 'steady_clock_backwards', 'timer_gap',
    'session_timeout', 'gate_process_loss',
)


def fault_case(name):
    c = SafetyConfig(max_session_s=2.5) if name == 'session_timeout' else SafetyConfig()
    h = Harness(c, Pose(.90, 0., c.ground_z_m) if name == 'goal_boundary' else None)
    h.moving()
    started, index = h.now, len(h.rows)
    if name == 'wand_loss': h.pose_on['wand'] = False
    elif name == 'car_loss': h.pose_on['car'] = False
    elif name == 'frozen_pose': h.frozen_pose['wand'] = h.ros
    elif name == 'operator_release': h.held = False
    elif name == 'operator_loss': h.lease_on = False
    elif name == 'operator_replay':
        h.frozen_lease = Lease('operator-test', h.sequence, h.now, h.ros, True)
    elif name == 'supervisor_freeze': h.supervisor_on = False
    elif name == 'command_link_loss': h.command_link_on = False
    elif name == 'command_replay': h.modify_frame = lambda _: h.frame_at_injection
    elif name == 'delayed_command':
        h.modify_frame = lambda f: replace(f, issued_steady=f.issued_steady-1,
                                          issued_ros=f.issued_ros-1)
    elif name == 'diagonal_overspeed':
        h.modify_frame = lambda f: replace(f, command=Velocity(.04, .04, 0))
    elif name == 'oversized_yaw':
        h.modify_frame = lambda f: replace(f, command=Velocity(0, 0, .21))
    elif name == 'acceleration_jump':
        h.modify_frame = lambda f: replace(f, command=Velocity(-.05, 0, 0))
    elif name == 'lift': h.plant.pose = replace(h.plant.pose, z=.40)
    elif name == 'tilt': h.pitch = .50
    elif name == 'car_jump': h.plant.pose = replace(h.plant.pose, x=h.plant.pose.x+.40)
    elif name == 'car_boundary': h.plant.pose = replace(h.plant.pose, x=1.30)
    elif name == 'goal_boundary':
        # Gradual wand move: goal breaches fixed arena before relative .5 m radius.
        h.hand = Pose(-.20, -.4, 1.)
    elif name == 'mode_conflict':
        for inputs in (h.sup.inputs, h.gate.inputs): inputs.mode = 'rectangle'
    elif name == 'duplicate_source':
        for inputs in (h.sup.inputs, h.gate.inputs): inputs.graph_ok = False
    elif name == 'feedback_loss': h.feedback_on = False
    elif name == 'actuator_fault': h.fault_bits = 1
    elif name == 'estop': h.gate.trip('ESTOP', hard=True)
    elif name == 'ros_clock_backwards': h.step(ros_step=-1.)
    elif name == 'ros_clock_forward': h.step(ros_step=1.)
    elif name == 'steady_clock_backwards': h.step(dt=-.02, ros_step=.02)
    elif name == 'timer_gap': h.step(dt=.30)
    elif name == 'gate_process_loss': h.gate_on = False
    h.frame_at_injection = h.frame
    h.run(2.)
    trace = h.rows[index:]
    field = 'plant_target_speed_mps' if name == 'gate_process_loss' else 'command_speed_mps'
    zero = next((r for r in trace if r[field] == 0.), None)
    assert zero is not None, (name, h.sup.state, h.sup.reason, h.gate.fault)
    zero_latency = max(0., zero['time_s']+10-started)
    bound = 2.0 if name == 'session_timeout' else .32
    assert zero_latency <= bound+1e-8, (name, zero_latency)
    zero_index = trace.index(zero)
    assert all(r[field] == 0 for r in trace[zero_index:]), name
    assert h.sup.state != 'ACTIVE' or not h.supervisor_on, (name, h.sup.state)
    if name == 'feedback_loss':
        assert h.sup.state == 'STOP_UNCONFIRMED' and not h.sup.stop_confirmed
    # Restore communication, but never call start/acknowledge. No auto-resume.
    h.pose_on = {'car': True, 'wand': True}
    h.frozen_pose, h.frozen_lease = {}, None
    h.lease_on = h.feedback_on = h.command_link_on = True
    h.modify_frame = lambda f: f
    mark = len(h.rows)
    h.run(.6)
    assert all(r['plant_target_speed_mps'] == 0 for r in h.rows[mark:]), name
    return {'name': name, 'passed': True, 'zero_output_latency_s': zero_latency,
            'final_state': h.sup.state, 'reason': h.sup.reason,
            'gate_fault': h.gate.fault, 'no_automatic_resume': True,
            'stop_confirmed': h.sup.stop_confirmed}, h.rows


def startup_case():
    h = Harness()
    h.held = True
    for i in range(150):
        h.hand = Pose(-.4+.001*i, -.4, 1.)
        h.step()
    assert h.sup.state == 'STANDBY' and all(r['command_speed_mps'] == 0 for r in h.rows)
    assert not h.sup.request_start(h.now, h.ros)[0]
    return {'name': 'startup_held_enable_and_moving_wand_cannot_arm', 'passed': True}, h.rows


def run_all():
    cases = [startup_case(), normal_case()]
    cases.extend(fault_case(name) for name in FAULT_CASES)
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = run_all()
    for summary, rows in results:
        with (args.output/(summary['name']+'.csv')).open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    report = {'verification': 'OFFLINE VIRTUAL TIME ONLY; no hardware connection',
              'config': asdict(SafetyConfig()), 'cases': [s for s, _ in results],
              'passed': all(s['passed'] for s, _ in results)}
    (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    plot(results, args.output/'safety_validation.png')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


def plot(results, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected = {'2x_diagonal_return_pause_rebase', 'wand_loss', 'operator_loss',
                'supervisor_freeze', 'lift', 'feedback_loss'}
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), constrained_layout=True)
    for ax, (summary, rows) in zip(axes.flat, ((s, r) for s, r in results if s['name'] in selected)):
        t = [r['time_s'] for r in rows]
        ax.plot(t, [r['command_speed_mps'] for r in rows], label='Final command')
        ax.plot(t, [r['plant_speed_mps'] for r in rows], label='Virtual inertia', alpha=.7)
        ax.axhline(.05, color='orange', linestyle='--', label='0.05 m/s cap')
        ax.set(title=summary['name'], xlabel='Virtual seconds', ylabel='m/s', ylim=(-.003, .06))
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('Dedicated supervisor + independent gate | OFFLINE SOFTWARE ONLY')
    fig.savefig(path, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    raise SystemExit(main())
