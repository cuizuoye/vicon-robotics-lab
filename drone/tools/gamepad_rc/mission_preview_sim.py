#!/usr/bin/env python3
"""Bounded in-memory kinematic mock of mission preview, not PX4/Gazebo or flight."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.mission_preview import MissionPreview, Observation, State


def simulate(scenario):
    mission = MissionPreview(height=.5, hover_seconds=3.)
    position = [2.3, -1.4, .02]  # Deliberately nonzero world origin.
    origin_z = position[2]
    o = Observation(0., tuple(position), yaw=.7)
    assert mission.start(o, confirmed=True)
    armed, landed = False, True
    rows = []
    for i in range(1, 601):
        t, dt = i*.1, .1
        previous_position = tuple(position)
        if mission.state == State.ASCEND:
            armed, landed = True, False  # Mock plant only; no encoded arming command.
            position[2] = min(mission.target[2], position[2] + .2*dt)
        elif mission.state in (State.LAND, State.ABORT_LAND, State.LAND_TIMEOUT):
            position[2] = max(origin_z, position[2] - .15*dt)
            if position[2] <= origin_z+1e-6:
                landed, armed = True, False
        velocity = tuple((b-a)/dt for a,b in zip(previous_position, position))
        o = Observation(t, tuple(position), velocity=velocity, yaw=.7, armed=armed, landed=landed)
        if scenario == 'pose_loss' and t >= 4:
            o = replace(o, pose_age_s=1.)
        if scenario == 'home_stop' and 4 <= t < 4.2:
            o = replace(o, home_pressed=True)
        if scenario == 'climb_timeout' and mission.state == State.ASCEND:
            position[2] = origin_z
            o = replace(o, position=tuple(position))
        result = mission.tick(o)
        rows.append({'t_s': round(t, 2), 'position_world_enu_m': list(position), **result})
        if mission.state == State.LANDED:
            break
        if mission.state == State.STOP_LATCHED and t > 5:
            break  # HOME release cannot resume; mock motors/physics are not an emergency-stop test.
    expected = State.STOP_LATCHED if scenario == 'home_stop' else State.LANDED
    return {'scenario': scenario, 'passed': mission.state == expected,
            'scope': 'Pure kinematic mock; synthetic status/arming/landing observations, not actual command ACKs, PX4 SITL or flight',
            'transitions': mission.transitions, 'last_result': mission.result(), 'samples': rows,
            'actual_sent_to_fc': 0, 'flight_ready': False}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    cases = [simulate(s) for s in ('nominal', 'pose_loss', 'home_stop', 'climb_timeout')]
    result = {'passed': all(c['passed'] for c in cases), 'cases': cases,
              'no_serial_network_ros_or_mavlink_opened': True, 'actual_sent_to_fc': 0, 'flight_ready': False}
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps({'passed': result['passed'], 'cases': [{k:c[k] for k in ('scenario','passed','transitions','last_result')} for c in cases]}, indent=2))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
