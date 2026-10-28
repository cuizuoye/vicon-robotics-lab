import ast
from dataclasses import asdict
import json
import math
from pathlib import Path

import pytest
from rclpy.serialization import serialize_message

from mecanum_wand_control.core import Pose, Velocity
from mecanum_wand_control.safety_core import Frame, Lease
from mecanum_wand_control.safety_nodes import decode_frame, decode_lease, decode_pose, pose_message


def test_envelopes_roundtrip():
    lease = Lease('operator', 1, 10., 1000., False)
    frame = Frame('supervisor', 2, 10., 1000., 'epoch', 'ACTIVE', True,
                  Velocity(.02, .03, .1), Pose(0., .2, .1))
    assert decode_lease(json.dumps(asdict(lease))) == lease
    assert decode_frame(json.dumps(asdict(frame))) == frame


@pytest.mark.parametrize('text', ['{"held":true,"held":false}', '[]', 'null',
    '{"issued_steady":NaN}', '{"issued_steady":Infinity}', ' '*5000])
def test_malformed_or_ambiguous_leases_fail_closed(text):
    with pytest.raises((ValueError, TypeError, KeyError)):
        decode_lease(text)


def test_raw_pose_keeps_height_roll_and_pitch():
    from builtin_interfaces.msg import Time
    pose = Pose(1, 2, .5, .2)
    msg = pose_message(pose, Time(sec=1000), roll=.3, pitch=-.4)
    parsed, stamp, roll, pitch = decode_pose(msg)
    assert (parsed.x, parsed.y, parsed.z, parsed.yaw, stamp, roll, pitch) == pytest.approx(
        (1, 2, .5, .2, 1000., .3, -.4))
    assert serialize_message(msg)
    msg.pose.orientation.w = math.nan
    with pytest.raises(ValueError): decode_pose(msg)


def test_launch_is_new_isolated_domain_with_exactly_three_safe_nodes():
    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root/'launch/wand_safe_sim.launch.py').read_text())
    executables, env = [], {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name): continue
        if call.func.id == 'Node':
            kw = {v.arg: v.value for v in call.keywords}
            executables.append(ast.literal_eval(kw['executable']))
            assert ast.literal_eval(kw['namespace']) == 'wand_safe_sim'
        if call.func.id == 'SetEnvironmentVariable':
            env[ast.literal_eval(call.args[0])] = ast.literal_eval(call.args[1])
    assert env['ROS_DOMAIN_ID'] == '96'
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert set(executables) == {'wand_safe_plant', 'wand_safe_supervisor', 'wand_safe_gate'}
    assert 'IncludeLaunchDescription' not in ast.unparse(tree)


