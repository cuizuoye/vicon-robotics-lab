"""保留Vicon世界位置，仅冻结方向向量和独立地面高度；无ROS/飞控IO。"""

import math

import numpy as np

from .frames import matrix_to_quaternion, quaternion_to_matrix
from .mavlink_vision import F, checked_quaternion, finite_vector, transform_rigid_pose


def body_pose_in_map(position, quaternion, offset, mounting):
    """沿用已有外参补偿，返回板中心候选在原map的位置/机体FLU姿态。"""
    p, q = transform_rigid_pose(position, quaternion, offset, mounting)
    w, x, y, z = q
    rotation = F @ quaternion_to_matrix([x, y, z, w]) @ F
    return (F @ np.asarray(p)).tolist(), matrix_to_quaternion(rotation)


def freeze_reference(body_quaternion_xyzw, ground_reference_map_m):
    rotation = quaternion_to_matrix(checked_quaternion(body_quaternion_xyzw))
    forward = rotation[:, 0].copy()
    forward[2] = 0
    norm = float(np.linalg.norm(forward))
    if norm < 0.1:
        raise ValueError('nose_too_vertical_for_horizontal_reference')
    forward /= norm
    up = np.array([0., 0., 1.])
    left = np.cross(up, forward)
    ground = finite_vector(ground_reference_map_m, 3)
    return {
        'schema': 'mircoair_world_reference_v1',
        'position_frame': 'map',
        'world_origin_and_axes_preserved': True,
        'forward_map_unit': forward.tolist(),
        'left_map_unit': left.tolist(),
        'up_map_unit': up.tolist(),
        'fixed_forward_yaw_map_rad': math.atan2(forward[1], forward[0]),
        # 只取Z为高度基准，不把历史起点的XY作为新运动原点。
        'ground_reference_map_m': ground.tolist(),
        'ground_reference_z_map_m': float(ground[2]),
        'ground_reference_xy_used_as_motion_origin': False,
        'north_alignment_verified': False,
        'imu_origin_verified': False,
        'flight_ready': False,
        'actual_sent_to_fc': 0,
    }


def reference_axes(reference):
    if reference.get('schema') != 'mircoair_world_reference_v1' or reference.get('position_frame') != 'map':
        raise ValueError('unexpected_reference_schema_or_frame')
    axes = np.column_stack([finite_vector(reference[key], 3) for key in
                            ('forward_map_unit', 'left_map_unit', 'up_map_unit')])
    if (not np.allclose(axes.T @ axes, np.eye(3), atol=1e-8)
            or not math.isclose(float(np.linalg.det(axes)), 1., abs_tol=1e-8)
            or not np.allclose(axes[:, 2], [0, 0, 1], atol=1e-8)):
        raise ValueError('reference_not_horizontal_right_handed_orthonormal')
    z = reference['ground_reference_z_map_m']
    if not math.isfinite(z):
        raise ValueError('invalid_ground_height')
    return axes


def displacement_to_map(reference, forward_left_up_m):
    """相对位移→map位移。不是map绝对位置，更不是NED坐标。"""
    return (reference_axes(reference) @ finite_vector(forward_left_up_m, 3)).tolist()


def displacement_from_map(reference, displacement_map_m):
    return (reference_axes(reference).T @ finite_vector(displacement_map_m, 3)).tolist()


def waypoint_in_map(reference, start_map_m, forward_left_up_m):
    """必须显式提供任务起点；不默认使用人员操作区或历史地面XY。"""
    return (finite_vector(start_map_m, 3) + np.asarray(
        displacement_to_map(reference, forward_left_up_m))).tolist()


def observe_in_map(reference, body_position_map_m, body_quaternion_xyzw):
    axes = reference_axes(reference)
    p = finite_vector(body_position_map_m, 3)
    rotation = quaternion_to_matrix(checked_quaternion(body_quaternion_xyzw))
    current_forward = rotation[:, 0]
    if np.linalg.norm(current_forward[:2]) < 0.1:
        raise ValueError('nose_too_vertical_for_horizontal_reference')
    yaw = math.atan2(current_forward[1], current_forward[0])
    fixed_yaw = math.atan2(axes[1, 0], axes[0, 0])
    return {
        'position_frame': 'map',
        'body_reference_position_map_m': p.tolist(),
        'height_above_initial_ground_reference_m': float(p[2] - reference['ground_reference_z_map_m']),
        'yaw_relative_to_fixed_forward_rad': math.atan2(math.sin(yaw-fixed_yaw), math.cos(yaw-fixed_yaw)),
        'not_obstacle_clearance_measurement': True,
        'actual_sent_to_fc': 0,
        'flight_ready': False,
    }
