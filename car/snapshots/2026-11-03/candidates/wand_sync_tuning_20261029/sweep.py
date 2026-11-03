"""Current-controller paired SIM sweep, including immutable real Sign replay."""
import argparse
from collections import defaultdict
from dataclasses import asdict
import json
import math
from pathlib import Path
import numpy as np
from model import (ROOT, Limits, Harness, SPEEDS, ACCELS, RPMS, Sample,
                   synthetic_cases, ZERO, wheel_rpm, protected_hashes, save, digest)

REPORT = ROOT/'docs/wand_sync_tuning_2026-10-29'
REAL = ROOT/'docs/wand_fast_follow_ros_sim_2026-10-29/real_sign_5gzsjthu'


def cases():
    base = synthetic_cases()
    chosen = {name:base[name] for name in ('gentle', 'normal', 'fast_reversal', 'circle',
                                          'mixed', 'burst_hold_0', 'burst_hold_1', 'jitter')}
    chosen['fast_diagonal'] = [Sample(s.t, s.x/math.sqrt(2), s.x/math.sqrt(2))
                               for s in base['fast_reversal']]
    # Actual source selections from the previously accepted real-Sign run.
    # Exact tick join to mapped_goal; do not re-use its old car feedback.
    statuses = {}
    commands = {}
    with (REAL/'events.jsonl').open() as stream:
        for line in stream:
            event = json.loads(line)
            v = event['value']
            if event['topic'] == 'supervisor_status' and v['state'] == 'ACTIVE':
                statuses[v['issued_steady']] = v
            elif event['topic'] == 'candidate' and v['frame']['state'] == 'ACTIVE':
                commands[v['frame']['issued_steady']] = v
    if set(statuses) != set(commands):
        raise ValueError('exact recorded source/control match missing')
    ordered = sorted(statuses)
    first = statuses[ordered[0]]['anchors']['car_origin']
    co, si = math.cos(first['yaw']), math.sin(first['yaw'])
    start, start_ros = ordered[0], statuses[ordered[0]]['issued_ros']
    samples = []
    for at in ordered:
        status, command = statuses[at], commands[at]
        goal = status['mapped_goal']
        dx, dy = goal['x']-first['x'], goal['y']-first['y']
        samples.append(Sample(at-start+.02, co*dx+si*dy, -si*dx+co*dy,
            goal['yaw']-first['yaw'], status['issued_ros']-command['sign_stamp'],
            command['sign_stamp']-start_ros+.02))
    # Full XY+yaw input (not silently discarded), plus a deliberately declared
    # XY-only ablation to separate fixed low yaw limit from XY parameter tuning.
    chosen['real_sign_xy_yaw'] = samples
    chosen['real_sign_xy_only'] = [Sample(s.t, s.x, s.y, 0., s.age, s.source_offset) for s in samples]
    return chosen, dict(source=str(REAL/'events.jsonl'), sha256=digest(REAL/'events.jsonl'),
                       exact_control_samples=len(samples), original_car_origin=first,
                       normalized_source_clock_only=True, old_car_feedback_reused=False,
                       coordinate_transform='rigid transform to original car start frame; displacement scale/time unchanged',
                       xy_only_ablation_declared=True)


def quantile(values, q):
    return float(np.quantile(values, q)) if len(values) else None


def run_case(profile, name, samples, lag=.12, gains=(.98, .88, .95), keep=False):
    h = Harness(profile, lag, gains)
    h.start()
    origin_ros = h.ros
    previous, previous_t = ZERO, 0.
    errors, moving_errors, rows = [], [], []
    duration = speed_time = accel_time = reference_time = 0.
    max_accel = max_speed = max_rpm = 0.
    for i, s in enumerate(samples):
        dt = s.t-previous_t
        if not 0 < dt <= .10:
            raise ValueError('gap: split, never interpolate replay')
        h.gesture(s.x, s.y, s.yaw)
        h.pose_stamp['wand'] = origin_ros+s.source_offset if s.source_offset is not None else h.ros+dt-s.age
        v = h.step(dt)
        if h.sup.state != 'ACTIVE' or h.gate.fault:
            return dict(profile=profile.name, case=name, passed=False, at_s=s.t,
                        reason=h.sup.reason, gate_fault=h.gate.fault), rows
        speed = math.hypot(v.vx, v.vy)
        accel = math.hypot(v.vx-previous.vx, v.vy-previous.vy)/dt
        rpm = max(abs(value) for value in wheel_rpm(v))
        assert speed <= profile.max_speed_mps+1e-8
        assert accel <= profile.max_accel_mps2+1e-6
        assert rpm <= profile.sim_max_rpm+1e-6
        max_speed, max_accel, max_rpm = max(max_speed,speed), max(max_accel,accel), max(max_rpm,rpm)
        error = math.hypot(s.x-h.plant.pose.x, s.y-h.plant.pose.y)
        errors.append(error)
        j = i
        while j > 0 and s.t-samples[j].t < .08:
            j -= 1
        hand_speed = (math.hypot(s.x-samples[j].x, s.y-samples[j].y)/(s.t-samples[j].t)) if j < i else 0.
        if hand_speed > .02:
            moving_errors.append(error)
        duration += dt
        speed_time += dt*(speed >= .98*profile.max_speed_mps)
        accel_time += dt*(accel >= .98*profile.max_accel_mps2)
        reference_time += dt*h.sup.follower.reference_limited
        rows.append(dict(t=s.t, goal_x=s.x, goal_y=s.y, car_x=h.plant.pose.x, car_y=h.plant.pose.y,
                         error_m=error, speed_mps=speed, accel_mps2=accel, rpm=rpm,
                         hand_speed_mps=hand_speed, reference_limited=h.sup.follower.reference_limited,
                         car_vx=h.plant.velocity.vx, car_vy=h.plant.velocity.vy))
        previous, previous_t = v, s.t
    # Independent synthetic stop fixture, not claimed to be recorded human Esc.
    h.pose_stamp.clear()
    h.held = False
    stopped = h.step() == ZERO
    stop_unconfirmed = False
    for _ in range(120):
        h.step()
        stop_unconfirmed |= h.sup.state == 'STOP_UNCONFIRMED'
    passed = bool(stopped and h.sup.stop_confirmed and h.output == ZERO and not stop_unconfirmed)
    result = dict(profile=profile.name, case=name, passed=passed,
        speed_mps=profile.max_speed_mps, accel_mps2=profile.max_accel_mps2,
        wheel_cap_rpm=profile.sim_max_rpm, duration_s=duration, model_tau_s=lag, model_gains=gains,
        rms_m=math.sqrt(float(np.mean(np.square(errors)))), p95_m=quantile(errors,.95),
        moving_rms_m=math.sqrt(float(np.mean(np.square(moving_errors)))) if moving_errors else None,
        max_error_m=max(errors), final_error_m=errors[-1], peak_speed_mps=max_speed,
        peak_accel_mps2=max_accel, peak_rpm=max_rpm,
        speed_cap_fraction=speed_time/duration, accel_cap_fraction=accel_time/duration,
        reference_hold_fraction=reference_time/duration, output_stop_immediate=stopped,
        model_stop_confirmed=h.sup.stop_confirmed, stop_confirmation_deadline_met=not stop_unconfirmed)
    if name in ('gentle','normal','fast_reversal','fast_diagonal'):
        omega = {'gentle':1.2,'normal':5/3,'fast_reversal':2.5,'fast_diagonal':2.5}[name]
        delays = []
        unit = (1/math.sqrt(2),1/math.sqrt(2)) if name == 'fast_diagonal' else (1.,0.)
        for k in range(1, int(samples[-1].t*omega/math.pi)):
            at = k*math.pi/omega
            initial = next((r['car_vx']*unit[0]+r['car_vy']*unit[1] for r in reversed(rows) if r['t'] < at-.04), 0.)
            crossing = next((r['t']-at for r in rows if at <= r['t'] <= at+1.5 and
                             (r['car_vx']*unit[0]+r['car_vy']*unit[1])*initial <= 0.), None)
            delays.append(crossing)
        result['reversal_delay_median_s'] = quantile([v for v in delays if v is not None], .5)
        result['reversals_missing'] = sum(v is None for v in delays)
    return result, rows if keep else []


def ranking(results):
    by_profile = defaultdict(list)
    for r in results:
        by_profile[r['profile']].append(r)
    ranked = []
    for name, group in by_profile.items():
        good = len(group) == 11 and all(r['passed'] for r in group)
        # Equal case weights, not sample counts; jitter must not dilute motion.
        moving = [r['moving_rms_m'] for r in group if r['passed'] and r.get('moving_rms_m') is not None]
        primary = [r for r in group if not r['case'].startswith('burst_') and r['case'] != 'jitter']
        primary_moving = [r['moving_rms_m'] for r in primary if r['passed'] and r.get('moving_rms_m') is not None]
        ranked.append(dict(profile=name, all_cases_passed=good,
            mean_case_moving_rms_m=float(np.mean(moving)) if good and moving else None,
            primary_mean_moving_rms_m=float(np.mean(primary_moving)) if good and primary_moving else None,
            primary_worst_p95_m=max(r['p95_m'] for r in primary) if good else None,
            worst_case_p95_m=max(r['p95_m'] for r in group) if good else None,
            speed_mps=group[0].get('speed_mps'), accel_mps2=group[0].get('accel_mps2'),
            wheel_cap_rpm=group[0].get('wheel_cap_rpm')))
    return sorted(ranked, key=lambda r:(not r['all_cases_passed'], r['primary_mean_moving_rms_m'] or 1e9))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--extend-from', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = protected_hashes()
    save(args.output/'PROTECTED_BEFORE.json', before)
    inputs, provenance = cases()
    cached = {}
    reused = 0
    if args.extend_from:
        prior = json.loads((args.extend_from/'RESULT.json').read_text())
        if not prior['original_unchanged'] or prior['input_provenance'] != provenance:
            raise ValueError('cannot reuse changed source/model evidence')
        for row in json.loads((args.extend_from/'SWEEP.json').read_text()):
            cached[(row['profile'],row['case'])] = row
    results = []
    for rpm in RPMS:
        for v in SPEEDS:
            for a in ACCELS:
                c = Limits(v, a, rpm)
                for name, samples in inputs.items():
                    if (c.name,name) in cached:
                        r = cached[c.name,name];reused += 1
                    else:r, _ = run_case(c, name, samples)
                    results.append(r)
                print(c.name, sum(r['passed'] for r in results[-11:]), '/11', flush=True)
    save(args.output/'SWEEP.json', results)
    ranked = ranking(results)
    # Smallest near-best profile (<=5% mean error of the minimum), per RPM tier.
    selected = {}
    for rpm in RPMS:
        valid = [r for r in ranked if r['all_cases_passed'] and r['wheel_cap_rpm'] == rpm]
        if valid:
            best = valid[0]['primary_mean_moving_rms_m']
            selected[str(int(rpm))] = min((r for r in valid if r['primary_mean_moving_rms_m'] <= best*1.05),
                key=lambda r:(r['speed_mps'],r['accel_mps2']))
    sensitivity = []
    for choice in selected.values():
        c = Limits(choice['speed_mps'],choice['accel_mps2'],choice['wheel_cap_rpm'])
        for tau, gains in ((.06,(1.,1.,1.)),(.12,(.98,.88,.95)),(.20,(.90,.80,.90))):
            for name in ('normal','fast_reversal','fast_diagonal','real_sign_xy_yaw','burst_hold_1'):
                r, data = run_case(c,name,inputs[name],tau,gains,keep=True)
                sensitivity.append(r)
                if tau == .12:
                    save(args.output/(name+'_'+c.name+'.json'),data)
    after = protected_hashes()
    assert before == after
    save(args.output/'PROTECTED_AFTER.json', after)
    summary = dict(simulation_only=True, physical_car_started=False, firmware_changed=False,
        original_unchanged=True, input_provenance=provenance, case_count=11,
        profiles=len(SPEEDS)*len(ACCELS)*len(RPMS), reused_comparisons=reused,
        reused_source=str(args.extend_from) if args.extend_from else None,
        comparison_count=len(results), ranked=ranked, selected_near_best_by_rpm=selected,
        sensitivity=sensitivity, safety_deadlines_unchanged=True, angular_limits_unchanged=[.1,.4],
        selection_rule='all 11 cases pass; 8 smooth/real primary cases equal-case moving RMS; burst velocity steps/jitter are separate stress cases; smallest speed/accel within 5% of minimum in each RPM tier',
        hardware_qualification=False, model_identified=False)
    save(args.output/'RESULT.json',summary)
    print(json.dumps(dict(selected=selected,original_unchanged=True),ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
