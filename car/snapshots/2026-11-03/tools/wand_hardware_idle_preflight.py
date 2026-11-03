#!/usr/bin/env python3
"""Powered MC-Link admission with released leases only; NEVER enables motion.

Uses the reviewed hardware entry, sends HELLO/disabled zero, reads real MCU
diagnostics/feedback, then closes its own launch. Default: no services at all.
Optional --prepare-only checks ARMED readiness and cancels with the lease
permanently released: no ACTIVE state, no held-enable, no synthesized Sign,
no firmware/network mutation. Healthy used-budget reset may occur in ARMED.
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
from std_msgs.msg import String, Float32MultiArray, Float32, UInt32
from std_srvs.srv import Trigger
from mecanum_wand_control.operator_console import OperatorNode
from mecanum_wand_control.operator_model import OperatorModel
from mecanum_wand_control.safety_nodes import LATEST
from mecanum_wand_control.safety_ros_smoke import audit_domain_zero
from mecanum_wand_hardware.commissioning import Runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--approved-sha256', required=True)
    parser.add_argument('--hardware-consent', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--prepare-only', action='store_true',
        help='After idle admission, prepare with permanently released lease, verify ready, then cancel; NEVER enables')
    args = parser.parse_args()
    if not 6 <= args.seconds <= 30: raise ValueError('idle sample duration must be 6..30s')
    runtime = Runtime('hardware', str(args.profile.resolve()), args.approved_sha256, args.hardware_consent)
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(passed=False, verification='POWERED REAL MCU / RELEASED ENABLE ONLY / NO MOTION REQUEST')
    process = node = None
    values, counts, diags = {}, {}, []
    try:
        result['domain0_before'] = audit_domain_zero()
        if any(result['domain0_before'].values()): raise RuntimeError('legacy motion publishers present')
        os.environ.update(ROS_DOMAIN_ID='94', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        rclpy.init(domain_id=94)
        model = OperatorModel()
        node = OperatorNode(model, runtime)
        assert node.hand_pub is None
        def receive(name, value):
            if name == 'battery_v' and not math.isfinite(value): value = None
            counts[name] = counts.get(name, 0)+1
            values[name] = dict(received_steady=time.monotonic(), value=value)
            if name == 'diagnostics':
                diags.append(value)
                if value.get('permitted') or any(value[k] for k in ('compares', 'raw_duty', 'target_cps', 'body_milli') if isinstance(value.get(k), (int, float))):
                    raise RuntimeError('MCU unexpectedly permitted during idle-only check')
                if any(any(value[k]) for k in ('compares', 'raw_duty', 'target_cps', 'body_milli')):
                    raise RuntimeError('MCU reports nonzero requested outputs during idle-only check')
        status_rows = []
        node.create_subscription(String, 'supervisor_status',
            lambda m: status_rows.append(json.loads(m.data)), LATEST)
        for topic, name in (('transport_status', 'transport'), ('plant_feedback', 'feedback'),
                            ('/car/mc_link_diagnostics', 'diagnostics')):
            node.create_subscription(String, topic,
                lambda msg, name=name: receive(name, json.loads(msg.data)), LATEST)
        node.create_subscription(Float32MultiArray, '/car/wheel_rpm', lambda m: receive('wheel_rpm', list(m.data)), LATEST)
        node.create_subscription(Float32, '/car/battery_voltage', lambda m: receive('battery_v', m.data), LATEST)
        node.create_subscription(UInt32, '/car/motor_fault_bits', lambda m: receive('fault_bits', m.data), LATEST)
        with (args.output/'launch.log').open('x') as log:
            process = subprocess.Popen(['ros2', 'launch', 'mecanum_wand_hardware',
                'wand_sign_acceptance.launch.py', 'mode:=hardware', 'gui:=false',
                'profile:='+str(args.profile.resolve()), 'approved_sha256:='+args.approved_sha256,
                'hardware_consent:='+args.hardware_consent], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            end = time.monotonic()+args.seconds
            while time.monotonic()<end:
                node.pump()
                assert not model.held and not model.key_down
                if model.status: assert model.status['state'] == 'STANDBY', model.status
                if process.poll() is not None: raise RuntimeError('hardware entry exited')
                time.sleep(.005)
            now = time.monotonic()
            for name in ('diagnostics', 'transport', 'feedback', 'fault_bits', 'wheel_rpm'):
                assert name in values and 0 <= now-values[name]['received_steady'] <= .20, ('missing/stale', name, values.get(name))
            assert len(diags) >= 20 and diags[-1]['tag'] == runtime.motion_tag
            assert diags[-1]['ticks'] != diags[0]['ticks'], 'MCU clock did not progress'
            assert values['fault_bits']['value'] == 0
            assert max(abs(v) for v in values['wheel_rpm']['value']) < .5
            assert model.fresh(now) and model.gate.get('output_zero')
            assert model.status.get('stop_confirmed') and model.status.get('ready_to_start')
            assert not node.input_hint(), node.input_hint()
            feedback = values['feedback']['value']
            assert feedback['output_zero'] and not feedback['enabled'] and feedback['fault_bits'] == 0
            assert values['transport']['value']['fresh'] and not values['transport']['value']['fault']
            if args.prepare_only:
                def wait_for(predicate, timeout):
                    until = time.monotonic()+timeout
                    while time.monotonic() < until:
                        node.pump()
                        assert not model.held and not model.key_down
                        assert model.status.get('state') not in ('ACTIVE', 'FAULT', 'STOP_UNCONFIRMED'), model.status
                        if process.poll() is not None: raise RuntimeError('hardware entry exited')
                        if predicate(): return
                        time.sleep(.005)
                    raise AssertionError(('released-lease preparation timeout', model.status, values.get('transport')))
                def service(name):
                    client = node.service_clients[name]
                    wait_for(client.service_is_ready, 3.)
                    future = client.call_async(Trigger.Request())
                    wait_for(future.done, 3.)
                    assert future.result().success, future.result().message
                before = time.monotonic()
                service('start')
                wait_for(lambda: model.status.get('state') == 'ARMED' and model.status.get('gate_ready'), 6.)
                prepared_status = dict(model.status)
                preparation_s = time.monotonic()-before
                assert preparation_s >= .60
                hold_until = time.monotonic()+.6
                wait_for(lambda: time.monotonic() >= hold_until, 1.)
                assert model.status['state'] == 'ARMED' and model.status['gate_ready']
                assert values['transport']['value']['prepared_epoch'] == model.status['epoch']
                assert not values['transport']['value']['motion_gate_requested']
                service('stop')
                wait_for(lambda: model.status.get('state') == 'PAUSED' and model.status.get('stop_confirmed'), 5.)
                assert not any(row.get('state') == 'ACTIVE' for row in status_rows)
                result.update(released_lease_prepare_verified=True, preparation_s=preparation_s,
                    armed_status=prepared_status, cancelled_status=dict(model.status))
            result.update(passed=True, mcu_diagnostic_tag=diags[-1]['tag'], diagnostic_frames=len(diags),
                host_wire_timeout_ms=600, supervisor=model.status, gate=model.gate,
                measurements=values, counts=counts, car_pose=vars(node.car), wand_pose=vars(node.raw_wand),
                profile_sha256=runtime.profile_hash, no_prepare_or_enable_requested=not args.prepare_only,
                no_held_or_enable_requested=True,
                note='Identity is live diagnostic/protocol identity, not a full Flash hash; wheel feedback is not a physical braking test')
        result['domain0_after'] = audit_domain_zero()
    except Exception as error:
        result.update(error=repr(error), traceback=traceback.format_exc(), measurements=values, counts=counts)
        if node: result.update(supervisor=node.model.status, gate=node.model.gate)
    finally:
        if 'status_rows' in locals():
            (args.output/'supervisor_samples.json').write_text(json.dumps(status_rows, ensure_ascii=False, indent=2)+'\n')
        if node: node.close_node()
        if rclpy.ok(): rclpy.shutdown()
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
    (args.output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
