import ast
import math
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped

from mecanum_wand_control.live_bridge import (
    LiveInputs, OUTPUT_NAMES, SIM_DOMAIN, SOURCE_DOMAIN, input_fresh, relay_message,
)


def source_message():
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 10
    msg.pose.position.x = .3
    msg.pose.position.y = -.2
    msg.pose.orientation.w = 1.0
    return msg


def test_bridge_copies_pose_without_restamping_or_modifying_the_real_source():
    inputs = LiveInputs()
    msg = source_message()
    assert inputs.update('wand', msg, 10_001_000_000, 1.0)
    sample = inputs.snapshot()[0]['wand']
    relayed = relay_message(sample)
    assert relayed.header.stamp == msg.header.stamp
    assert relayed.pose == msg.pose
    assert relayed.header.frame_id == 'wand_sim_map'
    assert msg.header.frame_id == 'map'
    assert input_fresh(sample, 1.1, 10_101_000_000)
    assert not input_fresh(sample, 1.21, 10_211_000_000)


def test_republishing_old_input_cannot_renew_freshness():
    inputs = LiveInputs()
    msg = source_message()
    inputs.update('wand', msg, 10_001_000_000, 1.0)
    assert not inputs.update('wand', msg, 10_150_000_000, 1.15)
    sample = inputs.snapshot()[0]['wand']
    assert sample['received_s'] == 1.0
    assert not input_fresh(sample, 1.21, 10_211_000_000)


@pytest.mark.parametrize('error', ['quaternion', 'stale', 'frame'])
def test_invalid_real_source_is_removed(error):
    inputs = LiveInputs()
    msg = source_message()
    inputs.update('wand', msg, 10_001_000_000, 1)
    if error == 'quaternion':
        msg.pose.orientation.w = 0.0
    elif error == 'stale':
        msg.header.stamp.sec = 8
    else:
        msg.header.frame_id = 'other'
    assert not inputs.update('wand', msg, 10_010_000_000, 1.1)
    latest, _, rejected = inputs.snapshot()
    assert 'wand' not in latest and rejected['wand'] == 1


def test_real_car_heading_uses_existing_offset_but_wand_heading_is_not_retargeted():
    inputs = LiveInputs()
    msg = source_message()
    inputs.update('car', msg, 10_001_000_000, 1)
    inputs.update('wand', msg, 10_001_000_000, 1)
    latest, _, _ = inputs.snapshot()
    assert latest['car']['pose'].yaw == pytest.approx(3*math.pi/4)
    assert latest['wand']['pose'].yaw == 0.0


def test_live_launch_and_source_node_cannot_create_actuator_commands():
    assert SOURCE_DOMAIN == 0 and SIM_DOMAIN == 99
    assert set(OUTPUT_NAMES) == {'wand_pose', 'disturbance', 'source_status', 'source_markers'}
    root = Path(__file__).resolve().parents[1]
    source = (root/'mecanum_wand_control/live_bridge.py').read_text()
    tree = ast.parse(source)
    subscriber = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SourceSubscriber')
    assert 'create_publisher' not in ast.unparse(subscriber)
    assert not any(s in source for s in ('/car/cmd_vel', 'socket', 'serial.Serial', 'cmd_vel_udp_bridge'))
    launch = ast.parse((root/'launch/sign_live_sim.launch.py').read_text())
    executables = []
    for call in ast.walk(launch):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == 'Node':
            args = {kw.arg: kw.value for kw in call.keywords}
            executables.append(ast.literal_eval(args['executable']))
            assert ast.literal_eval(args['namespace']) == 'wand_sim'
    assert set(executables) == {'wand_sim_car', 'wand_sim_controller', 'sign_to_simulation', 'rviz2'}
    assert 'IncludeLaunchDescription' not in ast.unparse(launch)
