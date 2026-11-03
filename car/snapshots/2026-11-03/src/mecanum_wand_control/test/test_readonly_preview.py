import ast
from collections import deque
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import serialize_message

from mecanum_wand_control.core import Pose
from mecanum_wand_control.preview_model import ReadonlyMapping, StreamQuality
from mecanum_wand_control.readonly_preview import OUTPUTS, SignReadonlyPreview, parse_input


def ready(yaw=math.pi/3):
    model = ReadonlyMapping()
    model.receive('car', Pose(1, -2, .1, yaw), 1.0)
    model.receive('wand', Pose(-1, 3, 1.2), 1.0)
    assert model.capture(1.0)[0]
    return model


@pytest.mark.parametrize('yaw', [0.0, math.pi/2, math.pi/3, -1.5])
def test_live_mapping_uses_fixed_start_heading_and_exact_double_displacement(yaw):
    model = ready(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    # Move in small increments, as a real 330 Hz tracked wand would.
    for step in range(1, 11):
        f, l, t = .05*step, .025*step, 1+.01*step
        model.receive('wand', Pose(-1+c*f-s*l, 3+s*f+c*l, 5, 2), t)
        model.receive('car', Pose(1, -2, .1, yaw+.2), t)
    target = model.target(1.1)
    assert target is not None
    assert (target.x, target.y) == pytest.approx((1+c-s*.5, -2+s+c*.5))
    assert target.yaw == yaw
    for step in range(1, 11):
        f, l, t = .5-.05*step, .25-.025*step, 1.1+.01*step
        model.receive('wand', Pose(-1+c*f-s*l, 3+s*f+c*l, 1.2), t)
        model.receive('car', Pose(1, -2, .1, yaw), t)
    assert model.target(t) == model.mapping.car_origin


def test_no_target_without_fresh_pair_and_no_auto_resume_after_loss():
    model = ReadonlyMapping()
    assert not model.capture(1)[0]
    model = ready()
    assert model.target(1.21) is None
    assert model.state == 'RECAPTURE_REQUIRED'
    model.receive('wand', Pose(4, 5), 1.3)
    model.receive('car', Pose(2, 3), 1.3)
    assert model.target(1.3) is None
    assert model.capture(1.3)[0]
    assert model.target(1.3) == Pose(2, 3)


def test_invalid_jump_and_pause_hide_preview_until_explicit_recapture():
    model = ready()
    model.receive('wand', Pose(5, 5), 1.01)
    assert model.target(1.01) is None
    assert 'jump' in model.reason
    assert model.capture(1.01)[0]
    model.invalidate('wand', 'occluded')
    assert model.target(1.01) is None
    model.receive('wand', Pose(5, 5), 1.02)
    assert model.capture(1.02)[0]
    model.pause()
    assert model.target(1.02) is None
    assert model.state == 'PAUSED'


def test_quality_reports_repeatability_and_rejects_duplicate_timestamps():
    q = StreamQuality()
    assert q.accept(Pose(1, 2, 3), 100, 1.0, .001)
    assert q.accept(Pose(1.002, 2, 3), 200, 1.01, .002)
    assert not q.accept(Pose(8, 8, 8), 200, 1.02, .003)
    report = q.snapshot()
    assert report['count'] == 2 and report['duplicate_or_reordered'] == 1
    assert report['hz'] == pytest.approx(100)
    assert report['xyz_std_m'] == pytest.approx([.001, 0, 0])
    assert report['stationary_candidate']


@pytest.mark.parametrize('error', ['frame', 'old', 'future', 'nan', 'quaternion'])
def test_live_source_validation(error):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 10
    msg.pose.orientation.w = 1.0
    if error == 'frame':
        msg.header.frame_id = 'wrong_frame'
    elif error == 'old':
        msg.header.stamp.sec = 9
    elif error == 'future':
        msg.header.stamp.sec = 11
    elif error == 'nan':
        msg.pose.position.x = float('nan')
    else:
        msg.pose.orientation.w = 0.0
    with pytest.raises(ValueError):
        parse_input(msg, 10_010_000_000, 'map')


def test_preview_markers_serialize_real_points_and_target():
    stamp = PoseStamped().header.stamp
    model = ready()
    captured = []
    fake = SimpleNamespace(frame='map', model=model, trace=deque([(1, 2)]),
        marker_points={i: (Pose(i, 0, 1), 1.0) for i in range(1, 6)},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp)),
        pubs={'markers': SimpleNamespace(publish=captured.append)})
    fake.pose_message = lambda pose, t: SignReadonlyPreview.pose_message(fake, pose, t)
    SignReadonlyPreview.publish_markers(fake, model.target(1.0), 1.0)
    assert serialize_message(captured[-1])


def test_readonly_contract_has_no_command_publishers_or_hardware_nodes():
    assert set(OUTPUTS) == {'target_pose', 'anchor_pose', 'markers', 'status'}
    root = Path(__file__).resolve().parents[1]
    source = (root/'mecanum_wand_control/readonly_preview.py').read_text()
    assert 'Twist' not in source and 'create_client' not in source
    assert not any(term in source for term in ('cmd_vel', 'control_enable', 'clear_faults', 'socket'))
    tree = ast.parse((root/'launch/wand_preview.launch.py').read_text())
    executables = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'Node':
            args = {kw.arg: kw.value for kw in node.keywords}
            executables.append(ast.literal_eval(args['executable']))
            assert ast.literal_eval(args['namespace']) == 'wand_preview'
    assert set(executables) == {'sign_readonly_preview', 'rviz2'}
    assert 'IncludeLaunchDescription' not in ast.unparse(tree)
