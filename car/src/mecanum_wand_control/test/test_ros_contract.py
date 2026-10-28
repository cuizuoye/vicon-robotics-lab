import ast
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import serialize_message
from mecanum_wand_control.core import Pose, WandSession
from mecanum_wand_control.ros_nodes import FRAME, WandSimController, parse_pose, pose_message


def message():
    msg = PoseStamped()
    msg.header.frame_id = FRAME
    msg.header.stamp.sec = 10
    msg.pose.orientation.w = 1.0
    return msg


@pytest.mark.parametrize('change', ['frame', 'old', 'future', 'quaternion', 'nan'])
def test_adapter_rejects_bad_source_data(change):
    msg = message()
    if change == 'frame':
        msg.header.frame_id = 'other_world'
    elif change == 'old':
        msg.header.stamp.sec = 9
    elif change == 'future':
        msg.header.stamp.sec = 11
    elif change == 'quaternion':
        msg.pose.orientation.w = 0.0
    else:
        msg.pose.position.x = float('nan')
    with pytest.raises(ValueError):
        parse_pose(msg, 10.01)


def test_pose_message_roundtrip():
    p = Pose(1, 2, 3, .5)
    stamp = message().header.stamp
    actual, source_stamp = parse_pose(pose_message(p, stamp), 10.01)
    assert (actual.x, actual.y, actual.z, actual.yaw) == pytest.approx((1, 2, 3, .5))
    assert source_stamp == 10


def test_small_source_clock_skew_cannot_create_a_future_freshness_stamp():
    actual, source_stamp = parse_pose(message(), 9.999)
    assert actual.valid()
    assert source_stamp == 9.999


def test_ros_wire_serialization_including_markers_and_integer_coordinates():
    stamp = message().header.stamp
    assert serialize_message(pose_message(Pose(0, 0, 1), stamp))
    session = WandSession()
    session.observe('car', Pose(0, 0), 1.0)
    session.observe('wand', Pose(1, 2, 1), 1.0)
    session.engage(1.0)
    messages = []
    fake = SimpleNamespace(
        session=session, trace=deque(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp)),
        markers_pub=SimpleNamespace(publish=messages.append))
    WandSimController.publish_markers(fake)
    assert serialize_message(messages[0])


def test_simulation_launch_has_only_explicit_simulation_nodes():
    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root/'launch/wand_sim.launch.py').read_text())
    executables = []
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == 'Node':
            args = {kw.arg: kw.value for kw in call.keywords}
            executables.append(ast.literal_eval(args['executable']))
            assert ast.literal_eval(args['namespace']) == 'wand_sim'
    assert set(executables) == {'wand_sim_car', 'wand_sim_controller', 'wand_demo', 'rviz2'}
    assert 'IncludeLaunchDescription' not in ast.unparse(tree)
    # Modules are not allowed to acquire a physical transport as a side effect.
    for path in (root/'mecanum_wand_control').glob('*.py'):
        imports = [n.module or '' for n in ast.walk(ast.parse(path.read_text()))
                   if isinstance(n, ast.ImportFrom)]
        assert not any('cmd_vel_udp_bridge' in value for value in imports)
