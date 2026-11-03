from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from mecanum_vicon_control.recovery_visualizer import recovery_markers


def pose(x, y, frame='map'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def test_recovery_markers_show_projection_target_vector_and_state():
    markers = recovery_markers(
        pose(0.4, 0.2), pose(0.4, 0.0), pose(0.6, 0.0),
        'RECOVERY', Time(), 'map')
    assert len(markers.markers) == 4
    assert markers.markers[2].points[0].y == 0.2
    assert markers.markers[2].points[1].y == 0.0
    assert markers.markers[2].color.r == 1.0
    assert markers.markers[3].text == 'RECOVERY'


def test_recovery_markers_reject_mixed_frames():
    try:
        recovery_markers(
            pose(0, 0), pose(0, 0, 'odom'), pose(0, 0),
            'TRACKING', Time(), 'map')
    except ValueError as error:
        assert 'frames' in str(error)
    else:
        raise AssertionError('mixed marker frames accepted')
