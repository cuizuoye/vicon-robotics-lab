import copy
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uav_coord_bridge.world_reference import (
    body_pose_in_map, displacement_from_map, displacement_to_map, freeze_reference,
    observe_in_map, reference_axes, waypoint_in_map,
)


def q(yaw):
    return [0, 0, math.sin(yaw/2), math.cos(yaw/2)]


def test_map_position_and_origin_preserved_with_independent_ground_height():
    ref = freeze_reference(q(math.pi/2), [8,9,.05])
    value = observe_in_map(ref, [1,2,.05], q(math.pi/2))
    assert value['body_reference_position_map_m'] == [1,2,.05]
    assert value['height_above_initial_ground_reference_m'] == 0
    assert not ref['ground_reference_xy_used_as_motion_origin']
    assert not ref['north_alignment_verified'] and not ref['flight_ready']


def test_fixed_forward_left_up_and_inverse():
    ref=freeze_reference(q(math.pi/2), [8,9,.05])
    assert np.allclose(displacement_to_map(ref, [1,0,0]), [0,1,0])
    assert np.allclose(displacement_to_map(ref, [0,1,0]), [-1,0,0])
    assert displacement_to_map(ref, [0,0,1]) == [0,0,1]
    assert np.allclose(displacement_from_map(ref, displacement_to_map(ref,[1,-2,3])), [1,-2,3])
    assert np.linalg.det(reference_axes(ref)) == pytest.approx(1)


def test_body_rotation_does_not_change_frozen_axes_or_map_waypoint():
    ref=freeze_reference(q(0), [8,9,.05]); original=copy.deepcopy(ref)
    obs=observe_in_map(ref,[1,2,1.05],q(math.pi/2))
    assert obs['yaw_relative_to_fixed_forward_rad'] == pytest.approx(math.pi/2)
    assert obs['height_above_initial_ground_reference_m'] == pytest.approx(1)
    assert ref == original
    assert waypoint_in_map(ref,[1,2,3],[1,0,0]) == [2,2,3]
    with pytest.raises(TypeError): waypoint_in_map(ref, [1,0,0])


def test_yaw_wrap_and_quaternion_sign():
    ref=freeze_reference(q(math.radians(179)),[0,0,0])
    assert observe_in_map(ref,[0,0,0],q(math.radians(-179)))['yaw_relative_to_fixed_forward_rad'] == pytest.approx(math.radians(2))
    assert freeze_reference(q(.2),[0,0,0]) == freeze_reference([-v for v in q(.2)],[0,0,0])


def test_existing_mount_compensation_exactly_once():
    p, orientation = body_pose_in_map([1,2,3],q(math.pi/2),[.1,0,0],q(-math.pi/2))
    assert np.allclose(p,[1,2.1,3])
    assert np.allclose(orientation,[0,0,0,1])


@pytest.mark.parametrize('bad', [[0,0,0,0], [0,0,0,2], [0,math.nan,0,1]])
def test_invalid_quaternion_rejected(bad):
    with pytest.raises(ValueError): freeze_reference(bad,[0,0,0])


def test_vertical_nose_rejected():
    with pytest.raises(ValueError): freeze_reference([0,math.sqrt(.5),0,math.sqrt(.5)],[0,0,0])


@pytest.mark.parametrize('change', ['left', 'up', 'frame', 'height'])
def test_corrupt_reference_rejected(change):
    ref=freeze_reference(q(0),[0,0,0])
    if change=='left': ref['left_map_unit']=[0,-1,0]
    if change=='up': ref['up_map_unit']=[0,0,-1]
    if change=='frame': ref['position_frame']='NED'
    if change=='height': ref['ground_reference_z_map_m']=math.nan
    with pytest.raises(ValueError): observe_in_map(ref,[0,0,0],q(0))


@pytest.mark.parametrize('bad', [[math.nan,0,0], [1,2], [0,math.inf,0]])
def test_invalid_position_rejected(bad):
    ref=freeze_reference(q(0),[0,0,0])
    with pytest.raises(ValueError): displacement_to_map(ref,bad)
    with pytest.raises(ValueError): observe_in_map(ref,bad,q(0))
