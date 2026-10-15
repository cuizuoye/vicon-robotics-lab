import copy
import math
from pathlib import Path
import sys

import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from uav_coord_bridge.target_mapping import (make_alignment,map_target_to_controller,
    controller_target_to_map,fixed_direction_delta,height_target_map,yaw_target_controller)
from uav_coord_bridge.world_reference import freeze_reference


def reference(yaw=0):
    return freeze_reference([0,0,math.sin(yaw/2),math.cos(yaw/2)],[8,9,.05])


def test_height_half_meter_does_not_mean_controller_z_minus_half():
    align=make_alignment([1,2,.05],[5,6,-25.81],0,0)
    target=height_target_map(reference(),[1,2],.5)
    assert target==[1,2,.55]
    assert map_target_to_controller(align,target)==pytest.approx([5,6,-26.31])
    assert not align['live_dispatch_permitted'] and not align['flight_ready']


def test_ground_height_is_absolute_and_not_relative_to_an_elevated_anchor():
    align=make_alignment([1,2,.3],[5,6,-25.81],0,0)
    target=height_target_map(reference(),[1,2],.5)
    assert map_target_to_controller(align,target)[2]==pytest.approx(-26.06)


@pytest.mark.parametrize('map_yaw,fc_yaw',[(0,0),(math.pi/2,0),(-3.1,3.1),(.4,-.8)])
def test_yaw_alignment_roundtrip_and_fixed_front_not_body_following(map_yaw,fc_yaw):
    align=make_alignment([1,2,3],[4,5,6],map_yaw,fc_yaw)
    ref=reference(map_yaw)
    delta=fixed_direction_delta(align,ref,[.2,0,0])
    assert delta==pytest.approx([.2*math.cos(fc_yaw),.2*math.sin(fc_yaw),0])
    assert fixed_direction_delta(align,ref,[0,0,.2])==pytest.approx([0,0,-.2])
    target=[-2,4,1]
    assert controller_target_to_map(align,map_target_to_controller(align,target))==pytest.approx(target)
    assert yaw_target_controller(align,map_yaw)==pytest.approx(fc_yaw)
    assert np.linalg.det(align['rotation_controller_from_map'])==pytest.approx(1)


@pytest.mark.parametrize('case',['nan_heading','nan_origin','wrong_schema','bad_rotation','wrong_up'])
def test_corrupt_mapping_rejected(case):
    if case=='nan_heading':
        with pytest.raises(ValueError): make_alignment([0,0,0],[0,0,0],math.nan,0)
        return
    if case=='nan_origin':
        with pytest.raises(ValueError): make_alignment([math.nan,0,0],[0,0,0],0,0)
        return
    align=copy.deepcopy(make_alignment([0,0,0],[0,0,0],0,0))
    if case=='wrong_schema': align['schema']='NED'
    if case=='bad_rotation': align['rotation_controller_from_map'][0][0]=2
    if case=='wrong_up': align['rotation_controller_from_map']=np.eye(3).tolist()
    with pytest.raises(ValueError): map_target_to_controller(align,[0,0,0])


@pytest.mark.parametrize('height',[-1,math.inf,math.nan])
def test_invalid_height_and_explicit_xy_required(height):
    with pytest.raises(ValueError): height_target_map(reference(),[1,2],height)
    with pytest.raises(ValueError): height_target_map(reference(),[1],.5)
