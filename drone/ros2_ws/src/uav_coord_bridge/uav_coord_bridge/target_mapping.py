"""map目标到控制器局部坐标的只读算术；不是NED标定或飞行发送接口。"""

import math
import numpy as np
from .mavlink_vision import F, finite_vector
from .world_reference import reference_axes, displacement_to_map


def make_alignment(map_anchor_m,controller_anchor_m,map_body_yaw_rad,controller_body_yaw_rad):
    if not all(math.isfinite(v) for v in (map_body_yaw_rad,controller_body_yaw_rad)):
        raise ValueError('invalid_heading')
    angle=controller_body_yaw_rad+map_body_yaw_rad
    c,s=math.cos(angle),math.sin(angle)
    rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])@F
    return {'schema':'map_controller_alignment_preview_v1',
            'map_anchor_m':finite_vector(map_anchor_m,3).tolist(),
            'controller_anchor_m':finite_vector(controller_anchor_m,3).tolist(),
            'rotation_controller_from_map':rotation.tolist(),
            'north_alignment_verified':False,'flight_ready':False,'actual_sent_to_fc':0,
            'live_dispatch_permitted':False,
            'invalidate_on':'FC reboot, EKF switch, position/heading reset, Tracker calibration or mount changes'}


def checked_alignment(alignment):
    if alignment.get('schema')!='map_controller_alignment_preview_v1':
        raise ValueError('invalid_alignment_schema')
    rotation=np.asarray(alignment['rotation_controller_from_map'],dtype=float)
    if (rotation.shape!=(3,3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8)
            or not math.isclose(float(np.linalg.det(rotation)),1,abs_tol=1e-8)
            or not np.allclose(rotation@[0,0,1],[0,0,-1],atol=1e-8)):
        raise ValueError('invalid_map_up_to_controller_down_rotation')
    return rotation,finite_vector(alignment['map_anchor_m'],3),finite_vector(alignment['controller_anchor_m'],3)


def map_target_to_controller(alignment,target_map_m):
    rotation,origin,controller=checked_alignment(alignment)
    return (controller+rotation@(finite_vector(target_map_m,3)-origin)).tolist()


def controller_target_to_map(alignment,target_controller_m):
    rotation,origin,controller=checked_alignment(alignment)
    return (origin+rotation.T@(finite_vector(target_controller_m,3)-controller)).tolist()


def fixed_direction_delta(alignment,reference,forward_left_up_m):
    rotation,_,_=checked_alignment(alignment)
    return (rotation@np.asarray(displacement_to_map(reference,forward_left_up_m))).tolist()


def height_target_map(reference,explicit_xy_map_m,height_m):
    reference_axes(reference)
    xy=finite_vector(explicit_xy_map_m,2)
    if not math.isfinite(height_m) or height_m<0:
        raise ValueError('invalid_ground_relative_height')
    # 相对地面高度→map绝对Z，之后才能映射到控制器局部Z；不直接写z=-height。
    return [*xy.tolist(),reference['ground_reference_z_map_m']+height_m]


def yaw_target_controller(alignment,map_forward_yaw_rad):
    if not math.isfinite(map_forward_yaw_rad): raise ValueError('invalid_target_heading')
    rotation,_,_=checked_alignment(alignment)
    forward=rotation@np.array([math.cos(map_forward_yaw_rad),math.sin(map_forward_yaw_rad),0])
    return math.atan2(forward[1],forward[0])
