"""刚体转换、报文与过期保护；不连接飞控/Vicon硬件。"""

import math
from pathlib import Path
import sys

import numpy as np
import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uav_coord_bridge.mavlink_vision import (  # noqa: E402
    PoseGate, odometry_message, pose_covariance, transform_rigid_pose,
)


def test_arbitrary_heading_world_conversion_not_claimed_north_aligned():
    position, q = transform_rigid_pose([1, 2, 3], [0, 0, 0, 1])
    assert position == [1, -2, -3]
    assert q == [1, 0, 0, 0]


def test_positive_flu_yaw_becomes_negative_frd_yaw():
    _, q = transform_rigid_pose([0, 0, 0], [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    assert np.allclose(q, [math.cos(math.pi / 4), 0, 0, -math.sin(math.pi / 4)])


def test_mount_translation_is_rotated_exactly_once():
    position, _ = transform_rigid_pose([0, 0, 1], [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)],
                                      [0.1, 0, 0])
    assert np.allclose(position, [0, -0.1, -1])


def test_mount_rotation_is_compensated():
    s = math.sqrt(0.5)
    _, q = transform_rigid_pose([0, 0, 0], [0, 0, s, s], body_orientation_in_rigid=[0, 0, -s, s])
    assert np.allclose(q, [1, 0, 0, 0])


@pytest.mark.parametrize('q', [[0, 0, 0, 0], [0, 0, 0, 2], [0, math.nan, 0, 1]])
def test_invalid_quaternion_rejected(q):
    with pytest.raises(ValueError):
        transform_rigid_pose([0, 0, 0], q)


def test_mavlink_v2_odometry_roundtrip_and_unknown_velocity():
    enc = m.MAVLink(None, srcSystem=250, srcComponent=197)
    msg = odometry_message(enc, 123456789, [1, -2, -3], [1, 0, 0, 0])
    packet = msg.pack(enc)
    assert packet[0] == 0xfd
    decoded = m.MAVLink(None).parse_char(packet)
    assert decoded.frame_id == m.MAV_FRAME_LOCAL_FRD
    assert decoded.child_frame_id == m.MAV_FRAME_BODY_FRD
    assert decoded.estimator_type == m.MAV_ESTIMATOR_TYPE_VISION
    assert decoded.quality == 0
    assert all(math.isnan(x) for x in (decoded.vx, decoded.vy, decoded.vz,
                                     decoded.rollspeed, decoded.pitchspeed, decoded.yawspeed))
    assert math.isnan(decoded.velocity_covariance[0])
    assert decoded.time_usec == 123456789


def sample(gate, stamp=1_000_000_000, now=None, receipt=1_000_000_000, frame='map', position=(0, 0, 1)):
    return gate.ingest(stamp, stamp if now is None else now, receipt, frame, position, [0, 0, 0, 1])


def test_new_sample_only_no_timer_restamp_or_replay():
    gate = PoseGate()
    assert sample(gate)
    assert gate.take_new(1_010_000_000, 1_010_000_000).stamp_ns == 1_000_000_000
    assert gate.take_new(1_020_000_000, 1_020_000_000) is None


@pytest.mark.parametrize('stamp,now,frame', [
    (0, 1, 'map'), (1_000_000_000, 1_160_000_000, 'map'),
    (1_010_000_000, 1_000_000_000, 'map'), (1_000_000_000, 1_000_000_000, 'other'),
])
def test_zero_stale_future_and_wrong_frame_rejected(stamp, now, frame):
    gate = PoseGate()
    assert not sample(gate, stamp=stamp, now=now, frame=frame)
    assert gate.take_new(now, 1_000_000_000) is None


def test_duplicates_and_backwards_stamps_are_rejected():
    gate = PoseGate()
    assert sample(gate)
    assert not sample(gate)
    assert not sample(gate, stamp=999_000_000)
    assert gate.accepted == 1 and gate.rejected == 2


def test_pose_expiring_in_buffer_is_never_encoded():
    gate = PoseGate()
    assert sample(gate)
    assert gate.take_new(1_160_000_000, 1_160_000_000) is None
    assert gate.expired == 1
    assert not gate.fresh(1_160_000_000, 1_160_000_000)


def test_clock_rollback_is_not_treated_as_fresh():
    gate = PoseGate()
    assert sample(gate)
    assert not gate.fresh(999_000_000, 1_010_000_000)
    assert gate.take_new(999_000_000, 1_010_000_000) is None


def test_jump_and_nan_position_rejected():
    gate = PoseGate()
    assert sample(gate)
    assert not sample(gate, stamp=1_010_000_000, position=(2, 0, 1))
    assert not sample(gate, stamp=1_020_000_000, position=(math.nan, 0, 1))


def test_covariance_upper_triangle_indices():
    cov = pose_covariance(0.02, 0.1)
    assert len(cov) == 21
    assert all(cov[i] == 0.02**2 for i in (0, 6, 11))
    assert all(cov[i] == 0.1**2 for i in (15, 18, 20))
