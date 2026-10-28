#!/usr/bin/env python3
"""Three arm/move/release cycles with actual ROS nodes and slow LOOPBACK MCU.

NO real Sign, car socket, lease, firmware, or network configuration is touched.
Domain93 synthetic Vicon; domain94 reviewed transport forced to 127.0.0.1.
"""
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
from std_msgs.msg import String
from mecanum_wand_control.core import Pose
from mecanum_wand_control.offline_suite import Plant
from mecanum_wand_control.safety_core import ZERO
from mecanum_wand_control.safety_nodes import LATEST
from mecanum_wand_control.safety_ros_smoke import audit_domain_zero
from mecanum_wand_hardware.acceptance_smoke import McuEmulator, Probe
from mecanum_wand_hardware.commissioning import Runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(passed=False, verification='ACTUAL ROS / 10HZ LOOPBACK MCU / NO REAL CAR')
    process = probe = mcu = None
    feedback, cycles = [], []
    try:
        result['domain0_before'] = audit_domain_zero()
        runtime = Runtime('emulator', str(args.profile.resolve()))
        assert runtime.ip == '127.0.0.1' and runtime.source_domain == 93
        os.environ.update(ROS_DOMAIN_ID='94', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        rclpy.init(domain_id=94)
        mcu = McuEmulator(feedback_period=.10)
        mcu.plant = Plant(runtime.initial_pose)
        probe = Probe(mcu)
        probe.car_z = runtime.config.ground_z_m
        probe.create_subscription(String, 'plant_feedback',
            lambda m: feedback.append((time.monotonic(), json.loads(m.data))), LATEST)
        with (args.output/'launch.log').open('x') as log:
            process = subprocess.Popen(['ros2', 'launch', 'mecanum_wand_hardware',
                'wand_sign_acceptance.launch.py', 'mode:=emulator', 'gui:=false',
                'profile:='+str(args.profile.resolve())], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            for cycle in range(3):
                probe.held = False
                probe.wait(lambda: probe.status.get('ready_to_start') and probe.status.get('stop_confirmed'), 12.)
                prior_controls = len(mcu.controls)
                prior_hellos = len(mcu.hellos)
                armed_at = time.monotonic()
                probe.call('start')
                probe.wait(lambda: probe.status.get('state') == 'ARMED' and probe.status.get('gate_ready'), 6.)
                ready_at = time.monotonic()
                assert ready_at-armed_at >= .60
                assert not mcu.permit and mcu.command == ZERO
                assert all(not row[2] for row in mcu.controls[prior_controls:]), 'MCU enable before held key'
                assert len(mcu.hellos) == prior_hellos, 'prepare unexpectedly re-HELLOed'
                session = mcu.session
                at = time.monotonic()
                probe.held = True
                probe.wait(lambda: probe.status.get('state') == 'ACTIVE' and mcu.permit, 2.)
                active_at = time.monotonic()
                probe.wand = Pose(probe.wand.x+.04, probe.wand.y, 1.)
                probe.wait(lambda: math.hypot(mcu.command.vx, mcu.command.vy) > .025, 3.)
                probe.pump(.8)
                assert probe.status['state'] == 'ACTIVE', probe.status
                assert mcu.session == session and len(mcu.hellos) == prior_hellos
                live = [(receipt, f) for receipt, f in feedback if receipt >= at]
                assert len(live) >= 6
                ages = [receipt-f['issued_steady'] for receipt, f in live]
                gaps = [b[1]['issued_steady']-a[1]['issued_steady'] for a, b in zip(live, live[1:])]
                assert max(ages) <= .20 and max(gaps) <= .20, (ages, gaps)
                probe.held = False
                released = time.monotonic()
                probe.wait(lambda: not mcu.permit and mcu.command == ZERO, .6)
                latency = time.monotonic()-released
                probe.wait(lambda: probe.status.get('state') == 'PAUSED' and probe.status.get('stop_confirmed'), 5.)
                probe.held = True
                probe.pump(.25)
                assert not mcu.permit and mcu.command == ZERO, 're-hold automatically resumed'
                cycles.append(dict(cycle=cycle+1, preparation_s=ready_at-armed_at,
                    held_to_real_emulator_permit_s=active_at-at, session_id=session,
                    release_to_disabled_s=latency, max_feedback_age_s=max(ages),
                    max_feedback_sample_gap_s=max(gaps), healthy_budget_resets=mcu.rearms))
            assert mcu.rearms == 2 and not mcu.errors, (mcu.rearms, mcu.errors)
            result.update(passed=True, cycles=cycles, hello_count=len(mcu.hellos),
                healthy_budget_resets=mcu.rearms, protocol_errors=mcu.errors,
                max_wire_speed_mps=max(math.hypot(r[2], r[3]) for r in mcu.received),
                checks=['600ms genuine disabled baseline before ARMED ready; no enable before held key',
                    'held key reuses prepared MCU session; no HELLO or feedback cache reset',
                    '10Hz feedback remains within original 200ms deadline',
                    'three independent arm/move/release cycles without MCU power cycle',
                    'two healthy run-budget resets only while ARMED/disabled',
                    'released key disables permit; re-hold alone cannot resume'],
                events=probe.events, final_state=probe.status['state'],
                stop_confirmed=probe.status['stop_confirmed'], endpoint='127.0.0.1:54600/54601',
                profile_sha256=runtime.profile_hash)
        result['domain0_after'] = audit_domain_zero()
    except Exception as error:
        result.update(error=repr(error), traceback=traceback.format_exc(), cycles=cycles)
        if probe: result.update(supervisor=probe.status, gate=probe.gate,
                                transport=probe.transport, events=probe.events)
        if mcu: result.update(protocol_errors=mcu.errors, hellos=mcu.hellos, controls=mcu.controls)
    finally:
        if probe: probe.held = False
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
