import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from mecanum_vicon_control.car_state_bridge import (
    CarStateBridge, pose_is_fresh, validate_input_pose,
)
from mecanum_vicon_control.math_utils import quaternion_from_yaw


def pose():
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 1
    msg.pose.orientation.w = 1.0
    return msg


def test_valid_pose_and_scaled_quaternion():
    msg = pose()
    q = quaternion_from_yaw(0.7)
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = [2*v for v in q]
    assert validate_input_pose(msg, 'map', 1_100_000_000, 0.2) == pytest.approx(0.7)


@pytest.mark.parametrize('frame', ['', 'vicon', 'odom'])
def test_wrong_frame_not_relabelled(frame):
    msg = pose()
    msg.header.frame_id = frame
    with pytest.raises(ValueError, match='坐标系'):
        validate_input_pose(msg, 'map', 1_000_000_000, 0.2)


@pytest.mark.parametrize('part,field', [('position', 'x'), ('position', 'y'), ('position', 'z'),
                                      ('orientation', 'x'), ('orientation', 'y'),
                                      ('orientation', 'z'), ('orientation', 'w')])
@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_pose_rejected(part, field, value):
    msg = pose()
    setattr(getattr(msg.pose, part), field, value)
    with pytest.raises(ValueError, match='NaN/Inf'):
        validate_input_pose(msg, 'map', 1_000_000_000, 0.2)


def test_zero_quaternion_rejected():
    msg = pose()
    msg.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match='零长度'):
        validate_input_pose(msg, 'map', 1_000_000_000, 0.2)


@pytest.mark.parametrize('stamp_ns', [0, 700_000_000, 1_060_000_000])
def test_stale_zero_future_stamp_rejected(stamp_ns):
    msg = pose()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    with pytest.raises(ValueError, match='时间戳'):
        validate_input_pose(msg, 'map', 1_000_000_000, 0.2)


@pytest.mark.parametrize('last_ns,now_ns,expected', [
    (None, 1_000_000_000, False), (1_000_000_000, 1_100_000_000, True),
    (1_000_000_000, 1_200_000_000, True), (1_000_000_000, 1_210_000_000, False),
    (1_000_000_000, 900_000_000, False),
])
def test_freshness(last_ns, now_ns, expected):
    assert pose_is_fresh(last_ns, now_ns, 0.2) == expected


def test_timeout_publishes_invalid_without_pose_or_command():
    published = []
    node = SimpleNamespace(input_valid=True, last_valid_ns=1_000_000_000, pose_timeout_s=0.2,
                           valid_pub=SimpleNamespace(publish=lambda msg: published.append(msg.data)),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_300_000_000)))
    CarStateBridge.check_timeout(node)
    assert published == [False]


def test_reject_immediately_invalidates_even_after_recent_good_sample():
    published = []
    node = SimpleNamespace(input_valid=True, valid_pub=SimpleNamespace(publish=lambda msg: published.append(msg.data)),
                           get_logger=lambda: SimpleNamespace(warn=lambda *args, **kwargs: None))
    CarStateBridge.reject(node, 'invalid')
    assert node.input_valid is False
    assert published == [False]
