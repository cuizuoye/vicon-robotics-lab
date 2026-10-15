import math

import pytest
from geometry_msgs.msg import PoseStamped

from mecanum_vicon_control.math_utils import quaternion_from_yaw, yaw_from_quaternion
from mecanum_vicon_control.rviz_goal_bridge import sanitize_planar_goal


def make_goal(frame='map', x=1.2, y=-0.4, yaw=0.7):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 3.0
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def test_sanitize_planar_goal_keeps_xy_yaw_and_flattens_z():
    result = sanitize_planar_goal(make_goal(), 'map')
    assert result.header.frame_id == 'map'
    assert result.pose.position.x == pytest.approx(1.2)
    assert result.pose.position.y == pytest.approx(-0.4)
    assert result.pose.position.z == 0.0
    assert yaw_from_quaternion(
        result.pose.orientation.x,
        result.pose.orientation.y,
        result.pose.orientation.z,
        result.pose.orientation.w,
    ) == pytest.approx(0.7)


def test_sanitize_planar_goal_accepts_empty_frame_as_world_frame():
    result = sanitize_planar_goal(make_goal(frame=''), 'map')
    assert result.header.frame_id == 'map'


def test_sanitize_planar_goal_rejects_wrong_frame():
    with pytest.raises(ValueError, match='does not match'):
        sanitize_planar_goal(make_goal(frame='odom'), 'map')


def test_sanitize_planar_goal_rejects_invalid_numbers_and_quaternion():
    non_finite = make_goal(x=math.nan)
    with pytest.raises(ValueError, match='non-finite'):
        sanitize_planar_goal(non_finite, 'map')

    zero_quaternion = make_goal()
    zero_quaternion.pose.orientation.x = 0.0
    zero_quaternion.pose.orientation.y = 0.0
    zero_quaternion.pose.orientation.z = 0.0
    zero_quaternion.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match='zero length'):
        sanitize_planar_goal(zero_quaternion, 'map')

