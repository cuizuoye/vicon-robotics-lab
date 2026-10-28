#!/usr/bin/env python3
"""SIM ONLY: virtual-clock speed/acceleration sweep; no ROS/transport startup.

Composition deliberately keeps the accepted ServoSafetyConfig immutable. The
experimental limits exist only in this process, driving the software Plant via
the unchanged pure Supervisor and FinalGate. They are NOT hardware profiles.
"""
import argparse
from bisect import bisect_left
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import sys

WORKSPACE = Path(__file__).resolve().parents[2]
VERIFIED = WORKSPACE / 'docs/wand_low_latency_servo_2026-10-23/verified_runtime/src'
PROFILE = VERIFIED / 'mecanum_wand_servo_hardware/config/servo.3x3_1to1.OFFLINE_ONLY.json'
PROFILE_SHA = '242f97b9b5bd6a47c5317af18a06d2206a21131bb102d70180a6dab5d25e5a99'
sys.path[:0] = [str(VERIFIED / 'mecanum_wand_servo_control'),
                str(WORKSPACE / 'src/mecanum_vicon_control')]
from mecanum_wand_servo_control.offline_validation import Harness, load_config, quantile
from mecanum_wand_servo_control.safety_core import ZERO

SPEEDS = (.20, .35, .50, .70, 1.00)
ACCELS = (.08, .30, .60, 1.20, 2.00)
SESSIONS = ('live_sign_VnpmhP', 'live_sign_fgrxqG', 'live_sign_tXC5YX')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def accepted_config():
    if digest(PROFILE) != PROFILE_SHA:
        raise ValueError('accepted profile drift; refuse comparison')
    return load_config(PROFILE)


@dataclass(frozen=True)
class SimOnlyLimits:
    """Not serializable as a hardware profile; no mode/IP/transport interface.

    Delegate every guard/geometry to a hash-pinned, fully validated immutable
    accepted config. Only two explicitly enumerated model experiments differ.
    The existing SafetyConfig/ServoSafetyConfig hard limits remain untouched.
    """
    max_speed_mps: float
    max_accel_mps2: float

    def __post_init__(self):
        if (type(self.max_speed_mps) not in (float, int)
                or type(self.max_accel_mps2) not in (float, int)
                or self.max_speed_mps not in SPEEDS or self.max_accel_mps2 not in ACCELS):
            raise ValueError('only enumerated SIM ONLY speed/acceleration experiments supported')
        object.__setattr__(self, '_accepted', accepted_config())

    def __getattr__(self, key):
        if key.startswith('_'):
            raise AttributeError(key)
        return getattr(self._accepted, key)

    def follower_config(self):
        return replace(self._accepted.follower_config(), max_speed_mps=self.max_speed_mps,
                       max_accel_mps2=self.max_accel_mps2)

    @property
    def name(self):
        return f'v{self.max_speed_mps:.2f}_a{self.max_accel_mps2:.2f}'


@dataclass(frozen=True)
class Sample:
    t: float
    forward: float
    left: float
    age: float = 0.
    z: float = 1.
    yaw: float = 0.
    source_offset: float = None


@dataclass(frozen=True)
class Case:
    name: str
    samples: tuple
    origin: tuple = (1.5, 1.5)
    reversals: tuple = ()
    kind: str = 'synthetic'
    provenance: dict = None


def synthetic_cases():
    cases = []
    # Smooth bounded gestures, not discontinuous/infinite-acceleration corners.
    for name, amplitude, omega in (('gentle_030', .25, 1.2),
                                   ('normal_050', .30, 5/3),
                                   ('fast_075_challenge', .30, 2.5)):
        period = 2*math.pi/omega
        duration = 3*period
        samples = tuple(Sample(i*.02, amplitude*(1-math.cos(omega*i*.02)), 0.,
                               z=1+.15*math.sin(i*.02), yaw=.5*math.sin(i*.02))
                        for i in range(1, round(duration/.02)+1))
        cases.append(Case(name, samples, reversals=tuple(period*k/2 for k in range(1, 6)),
                          provenance=dict(peak_hand_speed_mps=amplitude*omega,
                                          peak_hand_accel_mps2=amplitude*omega**2)))
    # Circular XY gesture, angular speed eased in over one second (C1 position).
    def circle(t):
        phase = (4/3)*(.5*t-math.sin(math.pi*t)/(2*math.pi)) if t < 1 else (4/3)*(t-.5)
        return .30*math.sin(phase), .30*(1-math.cos(phase))
    cases.append(Case('circle_040', tuple(Sample(i*.02, *circle(i*.02)) for i in range(1, 801))))
    return cases


def recorded_cases():
    """Replay source poses actually selected by recorded ACTIVE control ticks.

    Keep measured tick intervals and source age/stamp; repeated stamps stay
    repeated. No interpolation, smoothing, speed rescaling, or gap filling.
    A center placement is a translation of the SIM car origin ONLY. Original
    corner runs are also retained to expose nominal-target clipping separately.
    Recorded car poses/commands never feed the new simulated feedback.
    """
    result = []
    for index, session in enumerate(SESSIONS, 1):
        path = WORKSPACE / 'docs/wand_low_latency_servo_2026-10-23' / session / 'session/events.jsonl'
        statuses, poses = [], {}
        with path.open() as stream:
            for line in stream:
                event = json.loads(line)
                v = event['value']
                if event['topic'] == 'wand_pose':
                    poses[v['source_ros']] = v
                elif event['topic'] == 'supervisor_status' and v['state'] == 'ACTIVE':
                    statuses.append(v)
        epochs = {s['epoch'] for s in statuses}
        if len(epochs) != 1 or not statuses:
            raise ValueError('recording must contain exactly one known ACTIVE session')
        stamps = sorted(poses)
        anchor = statuses[0]['anchors']
        hand = anchor['wand_origin']
        yaw = anchor['car_origin']['yaw']
        c, s = math.cos(yaw), math.sin(yaw)
        start = statuses[0]['issued_steady']-.02
        samples = []
        for status in statuses:
            age = status['pose_source_ages_s']['wand']
            stamp = status['issued_ros']-age
            at = bisect_left(stamps, stamp)
            choices = stamps[max(0, at-1):at+1]
            match = min(choices, key=lambda value: abs(value-stamp))
            if abs(match-stamp) > 2e-6:
                raise ValueError('selected source pose missing; do not fabricate input')
            wand = poses[match]
            dx, dy = wand['x']-hand['x'], wand['y']-hand['y']
            samples.append(Sample(status['issued_steady']-start,
                                  c*dx+s*dy, -s*dx+c*dy, age, wand['z'], wand['yaw'],
                                  match-statuses[0]['issued_ros']+.02))
        # No replay spans a source loss: exact timestamp ages below timeout are
        # retained; loss/return/manual recovery are separate fault tests.
        if any(b.t-a.t > .10 for a, b in zip(samples, samples[1:])):
            raise ValueError('control gap in ACTIVE recording; split explicitly')
        provenance = dict(path=str(path), sha256=digest(path), original_anchors=anchor,
                          source_age_max_s=max(x.age for x in samples),
                          transform='same XY displacements, same yaw; car origin translation only')
        result.append(Case(f'real{index}_center', tuple(samples), kind='recorded', provenance=provenance))
        if index < 3:
            result.append(Case(f'real{index}_original_corner', tuple(samples), origin=(0., 0.),
                               kind='recorded', provenance=provenance))
    return result


def rms(values):
    return math.sqrt(sum(v*v for v in values)/len(values)) if values else None


def run_case(config, case, lag_s=.12, gains=(.98, .88, .95), keep_rows=False):
    h = Harness(config, origin_local=case.origin, lag_s=lag_s, gains=gains)
    h.start()
    source_epoch = h.ros
    prev_t = 0.
    old_command = ZERO
    old_goal = h.sup.follower.goal
    max_accel = max_reference_speed = mapping_error = 0.
    clamp_count = speed_count = accel_count = 0
    rows = []
    for sample in case.samples:
        dt = sample.t-prev_t
        if not 0 < dt <= .10:
            raise ValueError('invalid replay interval')
        h.set_hand(sample.forward, sample.left, sample.z, sample.yaw)
        # Remap the clock epoch, not the original source age. Duplicate source
        # stamps stay duplicates and cannot refresh Inputs or the estimator.
        h.pose_stamp['wand'] = (source_epoch+sample.source_offset if sample.source_offset is not None
                                else h.ros+dt-sample.age)
        before_velocity = h.plant.velocity
        command = h.step(dt=dt)
        row = dict(h.rows[-1], t=sample.t, plant_vf=before_velocity.vx,
                   plant_vl=before_velocity.vy, hand_f=sample.forward, hand_l=sample.left)
        f = h.sup.follower
        raw = f.mapping.goal(h.hand)
        expected = config.geometry().world(case.origin[0]+sample.forward,
                                            case.origin[1]+sample.left)
        mapping_error = max(mapping_error, math.hypot(raw.x-expected[0], raw.y-expected[1]))
        clipped = math.hypot(raw.x-row['target_x'], raw.y-row['target_y']) > 1e-7
        row['clamped'] = clipped
        clamp_count += clipped
        accel = math.hypot(command.vx-old_command.vx, command.vy-old_command.vy)/dt
        max_accel = max(max_accel, accel)
        speed_count += row['command_speed_mps'] >= config.max_speed_mps*.99
        accel_count += accel >= config.max_accel_mps2*.99
        goal = f.goal
        ref_speed = math.hypot(goal.x-old_goal.x, goal.y-old_goal.y)/dt
        max_reference_speed = max(max_reference_speed, ref_speed)
        row['accel_mps2'] = accel
        rows.append(row)
        if h.sup.state != 'ACTIVE' or h.gate.fault:
            # Fail, don't compute a flattering error after an unexpected stop.
            return dict(case=case.name, limits=config.name, passed=False, at_s=sample.t,
                        reason=h.sup.reason, gate_fault=h.gate.fault), rows if keep_rows else []
        assert config.inside(h.plant.pose), 'model footprint outside hard fence'
        assert config.goal_inside(f.mapped_goal), 'mapped target outside 3x3'
        assert row['command_speed_mps'] <= config.max_speed_mps+1e-7
        assert accel <= config.max_accel_mps2+1e-6
        assert ref_speed <= config.max_speed_mps+1e-6
        old_command, old_goal, prev_t = command, goal, sample.t
    errors = [row['same_time_error_m'] for row in rows]
    # Pauses/stillness can dilute RMS: also report hand-moving samples.
    moving_errors, hand_speeds = [], []
    first = 0
    for i, sample in enumerate(case.samples):
        while first+1 < i and case.samples[first+1].t <= sample.t-.10:
            first += 1
        old = case.samples[first]
        if sample.t-old.t >= .08:
            speed = math.hypot(sample.forward-old.forward, sample.left-old.left)/(sample.t-old.t)
            hand_speeds.append(speed)
            if speed > .02:
                moving_errors.append(errors[i])
    reversals = []
    for at in case.reversals:
        before = next((r for r in reversed(rows) if r['t'] < at-.02), None)
        if before:
            initial_sign = before['plant_vf']
            crossing = next((r for r in rows if at <= r['t'] <= at+2.
                             and r['plant_vf']*initial_sign <= 0.), None)
            reversals.append(crossing['t']-at if crossing else None)
    # Release is a hard zero request, intentionally NOT subject to normal
    # acceleration shaping. Model coast is measured separately from output.
    release_pose = h.plant.pose
    release_speed = math.hypot(h.plant.velocity.vx, h.plant.velocity.vy)
    # Replay has ended. The following is an explicitly SYNTHETIC release
    # fixture with a fresh stationary rod, not a fabricated continuation of
    # the real recording (in particular not its later occlusion).
    h.pose_stamp.clear()
    h.held = False
    release_start = h.now
    zero_immediate = h.step() == ZERO
    first_still = None
    for _ in range(149):
        if first_still is None and math.hypot(h.plant.velocity.vx, h.plant.velocity.vy) <= .005:
            first_still = h.now-release_start
        h.step()
    coast = math.hypot(h.plant.pose.x-release_pose.x, h.plant.pose.y-release_pose.y)
    assert zero_immediate and h.output == ZERO and h.sup.state == 'PAUSED' and h.sup.stop_confirmed, (
        case.name, h.sup.reason, h.gate.fault, h.sup.state)
    result = dict(case=case.name, limits=config.name, passed=True,
        speed_mps=config.max_speed_mps, accel_mps2=config.max_accel_mps2,
        model_lag_s=lag_s, model_gains=gains, duration_s=case.samples[-1].t,
        rms_error_m=rms(errors), p95_error_m=quantile(errors, .95), peak_error_m=max(errors),
        moving_rms_error_m=rms(moving_errors), end_error_m=errors[-1],
        hand_speed_p95_mps=quantile(hand_speeds, .95), hand_speed_max_mps=max(hand_speeds),
        target_clamped_fraction=clamp_count/len(rows),
        speed_saturated_fraction=speed_count/len(rows), accel_saturated_fraction=accel_count/len(rows),
        reference_limited_fraction=sum(r['reference_limited'] for r in rows)/len(rows),
        max_command_speed_mps=max(r['command_speed_mps'] for r in rows),
        max_command_accel_mps2=max_accel, max_reference_speed_mps=max_reference_speed,
        raw_mapping_max_error_m=mapping_error, reversal_delays_s=reversals,
        reversal_delay_median_s=quantile([x for x in reversals if x is not None], .5) if reversals else None,
        release_gate_zero_within_s=.02, release_at_model_speed_mps=release_speed,
        release_fixture='synthetic fresh stationary rod after replay ends',
        release_model_coast_m=coast, release_model_still_within_s=first_still)
    return result, rows if keep_rows else []


def snapshot_inputs(cases):
    paths = list((WORKSPACE/'candidates/wand_low_latency_servo').glob('*.py'))
    paths += list(VERIFIED.rglob('*.py')) + list(VERIFIED.rglob('*.json'))
    paths += [Path(case.provenance['path']) for case in cases if case.kind == 'recorded']
    return {str(p): digest(p) for p in sorted(set(paths))}


def select_candidate(results):
    """Practical model screen, not time-shifted/filtered trajectory fitting.

    Normal smooth motions: <=7cm RMS, <=10cm P95, <=250ms median reversal
    delay, RMS within 10% of the best swept result for each normal case.
    Center replays: >=50% RMS reduction each. No failures anywhere. Prefer
    the lowest speed, then acceleration that passes. The separate ambitious
    5cm screen remains reported even when model dynamics prevent passing.
    """
    by = {(r['limits'], r['case']): r for r in results}
    normal = ('gentle_030', 'normal_050', 'circle_040')
    if not results or any(('v0.20_a0.08', f'real{i}_center') not in by for i in range(1, 4)):
        return None
    best = {c: min(r['rms_error_m'] for r in results if r['case'] == c and r['passed']) for c in normal}
    accepted = []
    for speed in SPEEDS:
        for accel in ACCELS:
            name = SimOnlyLimits(speed, accel).name
            group = [r for r in results if r['limits'] == name]
            if len(group) != 9 or not all(r['passed'] for r in group):
                continue
            good = all(by[name, c]['rms_error_m'] <= min(.07, best[c]*1.10)
                       and by[name, c]['p95_error_m'] <= .10
                       and (not by[name, c]['reversal_delays_s'] or
                            (all(x is not None for x in by[name, c]['reversal_delays_s'])
                             and by[name, c]['reversal_delay_median_s'] <= .25)) for c in normal)
            real = all(by[name, f'real{i}_center']['rms_error_m'] <=
                       .5*by['v0.20_a0.08', f'real{i}_center']['rms_error_m'] for i in range(1, 4))
            if good and real:
                accepted.append((speed, accel, name))
    return min(accepted)[2] if accepted else None


def render_plot(results, traces, selected, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for ax, case in zip(axes.flat[:2], ('normal_050', 'real3_center')):
        baseline = traces['v0.20_a0.08', case]
        ax.plot([r['t'] for r in baseline], [r['hand_f'] for r in baseline], 'k--', label='1:1 target')
        for name in dict.fromkeys(('v0.20_a0.08', 'v0.70_a0.08', 'v0.20_a1.20', selected)):
            if name is None:
                continue
            rows = traces[name, case]
            ax.plot([r['t'] for r in rows], [r['car_local_f']-1.5 for r in rows], label=name)
        ax.set(title=case, xlabel='Virtual time (s)', ylabel='Forward displacement (m)')
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    ax = axes[1, 0]
    grid = [[next(r['rms_error_m']*100 for r in results
                  if r['case'] == 'normal_050' and r['limits'] == SimOnlyLimits(v, a).name)
             for v in SPEEDS] for a in ACCELS]
    img = ax.imshow(grid, origin='lower', aspect='auto', cmap='magma_r')
    ax.set(xticks=range(len(SPEEDS)), xticklabels=SPEEDS,
           yticks=range(len(ACCELS)), yticklabels=ACCELS,
           xlabel='Speed cap (m/s)', ylabel='Acceleration cap (m/s2)', title='Normal 0.50m/s gesture: RMS (cm)')
    for j, row in enumerate(grid):
        for i, value in enumerate(row):
            ax.text(i, j, f'{value:.1f}', ha='center', va='center', color='black' if value < 10 else 'white')
    fig.colorbar(img, ax=ax)
    ax = axes[1, 1]
    for name in ('v0.20_a0.08', selected):
        if name:
            rows = traces[name, 'normal_050']
            ax.plot([r['t'] for r in rows], [r['same_time_error_m']*100 for r in rows], label=name)
    ax.set(title='Unshifted, same-time error', xlabel='Virtual time (s)', ylabel='Position error (cm)')
    ax.grid(alpha=.3)
    ax.legend()
    fig.suptitle('SIM ONLY | 1:1 XY | fixed 3x3m | model lag .12s, gains .98/.88 | NOT hardware acceptance')
    fig.savefig(output, dpi=140)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError('fresh comparison directory required')
    cases = synthetic_cases()+recorded_cases()
    preserved = snapshot_inputs(cases)
    args.output.mkdir(parents=True)
    (args.output/'preserved_before.json').write_text(json.dumps(preserved, indent=2)+'\n')
    results = []
    for speed in SPEEDS:
        for accel in ACCELS:
            config = SimOnlyLimits(speed, accel)
            for case in cases:
                report, _ = run_case(config, case)
                results.append(report)
            print(config.name, 'complete', flush=True)
    selected = select_candidate(results)
    (args.output/'sweep.json').write_text(json.dumps(results, indent=2)+'\n')
    traces = {}
    import csv
    for name in dict.fromkeys(('v0.20_a0.08', 'v0.70_a0.08', 'v0.20_a1.20', selected)):
        if name is None:
            continue
        config = next(SimOnlyLimits(v, a) for v in SPEEDS for a in ACCELS if SimOnlyLimits(v, a).name == name)
        for case in cases:
            _, rows = run_case(config, case, keep_rows=True)
            traces[name, case.name] = rows
            with (args.output/f'{case.name}_{name}.csv').open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0])
                writer.writeheader()
                writer.writerows(rows)
    sensitivity = []
    if selected:
        config = next(SimOnlyLimits(v, a) for v in SPEEDS for a in ACCELS if SimOnlyLimits(v, a).name == selected)
        for lag, gains in ((.04, (1., 1., 1.)), (.12, (.98, .88, .95)), (.25, (.85, .75, .95))):
            for case in cases:
                if case.name in ('normal_050', 'circle_040', 'real3_center'):
                    result, _ = run_case(config, case, lag_s=lag, gains=gains)
                    sensitivity.append(result)
    unchanged = preserved == snapshot_inputs(cases)
    assert unchanged, 'accepted inputs changed during comparison'
    report = dict(simulation_only=True, hardware_started=False, firmware_changed=False,
        mapping_scale=1., nominal_region_m=[0., 3., 0., 3.], buffer_each_side_m=.50,
        unchanged_accepted_sources=unchanged, input_sha256=preserved,
        compared_parameter_pairs=len(SPEEDS)*len(ACCELS), cases_per_pair=len(cases),
        model='Unidentified first-order velocity lag .12s, XY gains .98/.88; virtual time, no wire/ROS latency model',
        selected_sim_only_limits=selected,
        selection_criteria='normal RMS<=.07m and within 10% of grid best, P95<=.10m, reversal median<=.25s; every center replay RMS improvement>=50%; no unexpected stop',
        ambitious_5cm_all_normal_cases_passed=any(
            all(r['passed'] and r.get('rms_error_m', math.inf) <= .05
                for r in results if r['limits'] == name and r['case'] in ('gentle_030', 'normal_050', 'circle_040'))
            for name in {r['limits'] for r in results}),
        control_algorithms_unchanged=['ContinuousSession', 'Supervisor', 'FinalGate'],
        case_metadata=[dict(name=c.name, kind=c.kind, origin=c.origin, provenance=c.provenance) for c in cases],
        sensitivity=sensitivity,
        limitations=['No hardware, ROS signed output wire, battery/current/thermal/braking acceptance',
                     'No camera-to-wheel latency measurement; no communication jitter added to synthetic cases',
                     'Recorded inputs replay selected real samples/source ages; old car feedback never reused',
                     'Only XY is mapped; rod Z/yaw intentionally ignored; vehicle heading held',
                     'Corner clipping reported separately; no shifting curves to reduce tracking errors',
                     'New limits do not change any existing profile, live UI or firmware'])
    (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    render_plot(results, traces, selected, args.output/'comparison.png')
    print(json.dumps(dict(selected=selected, cases=len(results), failures=sum(not r['passed'] for r in results),
                         unchanged_accepted_sources=unchanged), indent=2))


if __name__ == '__main__':
    main()
