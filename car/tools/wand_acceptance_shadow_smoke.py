#!/usr/bin/env python3
"""Launch reviewed Sign/software-car entry and actual operator node, NO motion.

Real Sign may be absent. No synthetic Sign, real UDP transport, start service,
or held-enable is created. Only this test's launch is stopped on exit.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from mecanum_wand_control.operator_model import OperatorModel
from mecanum_wand_control.operator_console import OperatorNode
from mecanum_wand_control.safety_ros_smoke import audit_domain_zero
from mecanum_wand_hardware.commissioning import Runtime, NAMESPACE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result, process, node = dict(passed=False), None, None
    try:
        result['domain0_before'] = audit_domain_zero()
        os.environ['ROS_DOMAIN_ID'], os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = '94', 'LOCALHOST'
        runtime = Runtime('shadow', str(args.profile.resolve()))
        rclpy.init(domain_id=94)
        model = OperatorModel()
        node = OperatorNode(model, runtime)
        assert node.hand_pub is None
        with (args.output/'launch.log').open('x') as log:
            process = subprocess.Popen(['ros2', 'launch', 'mecanum_wand_hardware',
                'wand_sign_acceptance.launch.py', 'mode:=shadow', 'gui:=false',
                'profile:='+str(args.profile.resolve())], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            until = time.monotonic()+6.
            while time.monotonic() < until:
                node.pump()
                if model.status:
                    assert model.status['state'] == 'STANDBY'
                assert not model.held
                time.sleep(.005)
            assert process.poll() is None and model.fresh(time.monotonic())
            assert model.gate.get('output_zero') and node.car is not None
            publishers = node.get_publishers_info_by_topic(NAMESPACE+'/wand_pose')
            assert [p.node_name for p in publishers] == ['wand_sign_source']
            assert not any(name == 'wand_hardware_bridge' for name, ns in node.get_node_names_and_namespaces())
            result.update(passed=True, state=model.status['state'], output_zero=model.gate['output_zero'],
                verification='REAL DOMAIN0 INPUT / REVIEWED SHADOW PROFILE / ACTUAL OPERATOR NODE; NO HARDWARE',
                real_sign_visible=node.raw_wand is not None, input_hint=node.input_hint(),
                car_pose=vars(node.car), profile_sha256=runtime.profile_hash,
                operator_never_publishes_sign=True, hardware_transport_present=False)
        result['domain0_after'] = audit_domain_zero()
    except Exception as error:
        result.update(passed=False, error=repr(error))
    finally:
        if node: node.close_node()
        if rclpy.ok(): rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
    (args.output/'summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
