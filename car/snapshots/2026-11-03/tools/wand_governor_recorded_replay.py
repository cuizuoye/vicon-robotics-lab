#!/usr/bin/env python3
"""Replay recorded ACTIVE car/Sign poses into the PC follower. No ROS/socket.

No new vehicle dynamics are simulated. This is not hardware acceptance or a
test of network recovery; original faulted real trial remains unchanged.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

from mecanum_wand_control.core import Pose, WandSession
from mecanum_wand_hardware.commissioning import Runtime


def rows(path):
    with path.open() as stream:
        for line in stream:
            yield json.loads(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    epochs = {}
    for row in rows(args.record):
        if row['topic'] == 'supervisor_status' and row['value']['state'] == 'ACTIVE':
            p, t = row['value'], row['received_steady']
            entry = epochs.setdefault(p['epoch'], dict(start=t, end=t, anchors=p['anchors']))
            entry['end'] = t
    epoch, trial = max(epochs.items(), key=lambda pair: pair[1]['end']-pair[1]['start'])
    c = Runtime('shadow', str(args.profile)).config
    assert c.goal_reference_policy == 'governed'
    session = WandSession(c.follower_config(), c.project_goal)
    for source in ('car', 'wand'):
        session.observe(source, Pose(**trial['anchors'][source+'_origin']), trial['start'])
    assert session.engage(trial['start'])[0]
    count, max_error, max_reference_step_speed, max_command = 0, 0., 0., 0.
    previous_goal, previous_tick = session.goal, trial['start']
    for row in rows(args.record):
        t, topic, value = row['received_steady'], row['topic'], row['value']
        if t < trial['start']: continue
        if t > trial['end']: break
        if topic in ('car_pose', 'wand_pose'):
            session.observe(topic[:-5], Pose(**{k: value[k] for k in ('x', 'y', 'z', 'yaw')}), t)
        if topic != 'supervisor_status' or value.get('epoch') != epoch or t <= previous_tick:
            continue
        out = session.tick(t)
        assert session.state == 'ACTIVE', session.reason
        car = session.samples['car'][0]
        max_error = max(max_error, math.hypot(session.goal.x-car.x, session.goal.y-car.y))
        max_reference_step_speed = max(max_reference_step_speed,
            math.hypot(session.goal.x-previous_goal.x, session.goal.y-previous_goal.y)/(t-previous_tick))
        max_command = max(max_command, math.hypot(out.vx, out.vy))
        previous_goal, previous_tick = session.goal, t
        count += 1
    assert count > 50 and max_error < c.max_tracking_error_m
    assert max_command <= c.max_speed_mps+1e-9
    assert max_reference_step_speed <= c.max_speed_mps+1e-7
    assert math.hypot(session.raw_goal.x-car.x, session.raw_goal.y-car.y) > .65
    report = dict(passed=True, verification='RECORDED POSES ONLY / NO PLANT, ROS OR HARDWARE',
        limitation='Does not prove future hardware motion or eliminate physical feedback interruptions',
        source=str(args.record.resolve()), epoch=epoch, samples=count, duration_s=trial['end']-trial['start'],
        retained_tracking_error_limit_m=c.max_tracking_error_m,
        max_reference_tracking_error_m=max_error, max_reference_step_speed_mps=max_reference_step_speed,
        max_command_speed_mps=max_command, final_car=asdict(car),
        raw_goal=asdict(session.raw_goal), mapped_goal=asdict(session.mapped_goal),
        instantaneous_reference=asdict(session.goal))
    with args.output.open('x') as stream: json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
