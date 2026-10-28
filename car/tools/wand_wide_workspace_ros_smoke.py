#!/usr/bin/env python3
"""Wide-input / bounded-car ROS + loopback MC-Link test; NEVER real transport."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

import rclpy
from mecanum_wand_control.core import Pose
from mecanum_wand_control.offline_suite import Plant
from mecanum_wand_control.safety_core import ZERO
from mecanum_wand_control.safety_ros_smoke import audit_domain_zero
from mecanum_wand_hardware.acceptance_smoke import McuEmulator, Probe
from mecanum_wand_hardware.commissioning import Runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(passed=False, verification='ACTUAL ROS / DOMAIN93 FAKE VICON / LOOPBACK MC-LINK; NO REAL CAR')
    process = probe = mcu = None
    try:
        result['domain0_before'] = audit_domain_zero()
        runtime = Runtime('emulator', str(args.profile.resolve()))
        assert runtime.ip == '127.0.0.1' and runtime.source_domain == 93
        assert runtime.config.goal_boundary_policy == 'clamp'
        os.environ.update(ROS_DOMAIN_ID='94', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        rclpy.init(domain_id=94)
        mcu = McuEmulator()
        mcu.plant = Plant(runtime.initial_pose)
        probe = Probe(mcu)
        probe.car_z = runtime.config.ground_z_m
        with (args.output/'launch.log').open('x') as log:
            process = subprocess.Popen(['ros2', 'launch', 'mecanum_wand_hardware',
                'wand_sign_acceptance.launch.py', 'mode:=emulator', 'gui:=false',
                'profile:='+str(args.profile.resolve())], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            probe.arm()
            origin, hand = mcu.plant.pose, probe.wand
            for i in range(1, 41):
                probe.wand = Pose(hand.x+.8*i/40, hand.y, .1+.02*i)
                probe.pump(.025)
                assert probe.status['state'] == 'ACTIVE', probe.status
            probe.wait(lambda: probe.status.get('goal_limited'))
            probe.pump(.5)
            goal = Pose(**probe.status['goal'])
            assert runtime.config.inside(goal)
            raw = probe.status['raw_goal']
            assert raw['x']-origin.x > 1.5
            assert not runtime.config.inside(Pose(**raw))
            bounded_displacement = goal.x-origin.x
            probe.wait(lambda: abs(mcu.plant.pose.x-goal.x) < .03, 9.)
            saturated_position = mcu.plant.pose
            probe.pump(.5)
            assert probe.status['state'] == 'ACTIVE' and not probe.gate['fault']
            for i in range(1, 41):
                probe.wand = Pose(hand.x+.8*(1-i/40), hand.y, 1.2)
                probe.pump(.025)
            probe.wait(lambda: math.dist((mcu.plant.pose.x, mcu.plant.pose.y),
                                        (origin.x, origin.y)) < .025, 10.)
            assert not probe.status.get('goal_limited')
            return_error = math.dist((mcu.plant.pose.x, mcu.plant.pose.y), (origin.x, origin.y))
            probe.wand = Pose(hand.x+.05, hand.y, 1.2)
            probe.wait(lambda: math.hypot(mcu.command.vx, mcu.command.vy) > .025, 3.)
            pre_release_speed = math.hypot(mcu.command.vx, mcu.command.vy)
            at = time.monotonic()
            probe.held = False
            probe.wait(lambda: mcu.command == ZERO and not mcu.permit, .6)
            release_latency = time.monotonic()-at
            probe.wait(lambda: probe.status.get('state') == 'PAUSED' and probe.status.get('stop_confirmed'))
            probe.held = True
            probe.pump(.3)
            assert mcu.command == ZERO and not mcu.permit
            probe.arm()
            rebase_position = mcu.plant.pose
            probe.pump(.15)
            assert math.dist((mcu.plant.pose.x, mcu.plant.pose.y),
                             (rebase_position.x, rebase_position.y)) < .001
            probe.wand = Pose(probe.wand.x+.08, probe.wand.y, .2)
            probe.wait(lambda: math.hypot(mcu.command.vx, mcu.command.vy) > .025, 3.)
            probe.wand_on = False
            probe.wait(lambda: probe.status.get('state') == 'FAULT' and probe.status.get('stop_confirmed'), 4.)
            probe.wand_on = True
            probe.pump(.5)
            assert mcu.command == ZERO and not mcu.permit and probe.status['state'] == 'FAULT'
            probe.recover()
            assert probe.status['state'] == 'PAUSED' and not mcu.permit
            assert not mcu.errors, mcu.errors
            result.update(passed=True, checks=['0.8m wand -> 1.6m raw goal; car target bounded without fault',
                'continued held enable at boundary stays ACTIVE, not locked',
                'height changes do not command car z', 'absolute wand origin return restores car origin',
                'moving release disables MCU permit', 're-hold alone cannot resume',
                'new arm rebases without old-goal jump', 'Sign loss stops and restoration cannot resume',
                'explicit acknowledgement returns PAUSED; no automatic motion'],
                raw_goal_displacement_m=1.6, bounded_goal_dx_m=bounded_displacement,
                saturated_car_xy=[saturated_position.x, saturated_position.y],
                return_error_m=return_error, pre_release_wire_speed_mps=pre_release_speed,
                moving_release_to_disabled_wire_s=release_latency,
                max_wire_speed_mps=max(math.hypot(r[2], r[3]) for r in mcu.received),
                protocol_errors=mcu.errors, endpoint='127.0.0.1:54600/54601',
                profile_sha256=runtime.profile_hash, events=probe.events,
                final_state=probe.status['state'], stop_confirmed=probe.status['stop_confirmed'])
        result['domain0_after'] = audit_domain_zero()
    except Exception as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc())
        if probe: result.update(supervisor=probe.status, gate=probe.gate,
                                transport=probe.transport, events=probe.events)
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if probe: probe.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if mcu: mcu.sock.close()
    (args.output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
