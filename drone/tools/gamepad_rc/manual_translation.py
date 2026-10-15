"""断动力人工平移的静止端点比较；无串口、无控制命令。"""

import math
import numpy as np

from uav_coord_bridge.frames import quaternion_to_matrix
from uav_coord_bridge.mavlink_vision import checked_quaternion, finite_vector


def rotation(q_wxyz):
    w, x, y, z = checked_quaternion(q_wxyz)
    return quaternion_to_matrix([x, y, z, w])


def quaternion_angle(a, b):
    a, b = checked_quaternion(a), checked_quaternion(b)
    return math.degrees(2 * math.acos(float(np.clip(abs(a @ b), 0, 1))))


def mean_quaternion(quaternions):
    qs = np.asarray([checked_quaternion(q) for q in quaternions])
    qs[qs @ qs[0] < 0] *= -1
    return checked_quaternion(np.median(qs, axis=0)).tolist()


def endpoint(rows):
    """只接受至少5个不同、有效、跨度>=0.8s的静止端点观察。"""
    if len(rows) < 5 or rows[-1]['elapsed_s'] - rows[0]['elapsed_s'] < .8:
        return None
    try:
        if any(not r['local']['xy_valid'] or not 0 <= r['attitude_age_ms'] <= 300 for r in rows):
            return None
        if any(b['local']['timestamp_sample'] <= a['local']['timestamp_sample']
               or b['sent']['source_stamp_ns'] <= a['sent']['source_stamp_ns'] for a,b in zip(rows, rows[1:])):
            return None
        ev = np.asarray([finite_vector(r['sent']['position'], 3) for r in rows])
        fc = np.asarray([finite_vector([r['local'][n] for n in ('x','y','z')], 3) for r in rows])
        evq = mean_quaternion([r['sent']['q'] for r in rows])
        fcq = mean_quaternion([r['fc_q'] for r in rows])
    except ValueError:
        return None
    ev_center, fc_center = np.median(ev, axis=0), np.median(fc, axis=0)
    ev_span = float(np.max(np.linalg.norm(ev - ev_center, axis=1)))
    fc_span = float(np.max(np.linalg.norm(fc[:,:2] - fc_center[:2], axis=1)))
    rotation_span = max(quaternion_angle(r['sent']['q'], evq) for r in rows)
    if ev_span > .01 or fc_span > .03 or rotation_span > 2:
        return None
    return {'rows': len(rows), 'elapsed_start_s': rows[0]['elapsed_s'], 'elapsed_end_s': rows[-1]['elapsed_s'],
            'ev_position_frd_m': ev_center.tolist(), 'fc_position_m': fc_center.tolist(),
            'ev_q_wxyz': evq, 'fc_q_wxyz': fcq, 'ev_stationary_spread_m': ev_span,
            'fc_horizontal_stationary_spread_m': fc_span, 'ev_rotation_spread_deg': rotation_span}


def assess_translation(start, end):
    if not start or not end:
        return {'verified': False, 'reason': 'missing_stationary_endpoints'}
    ev_delta = np.asarray(end['ev_position_frd_m']) - start['ev_position_frd_m']
    fc_delta = np.asarray(end['fc_position_m']) - start['fc_position_m']
    # LOCAL_FRD不是相同航向：按起点双方机体姿态将EV世界位移转到EKF世界。
    alignment = rotation(start['fc_q_wxyz']) @ rotation(start['ev_q_wxyz']).T
    expected = alignment @ ev_delta
    expected_length, fc_length = float(np.linalg.norm(expected[:2])), float(np.linalg.norm(fc_delta[:2]))
    ev_length = float(np.linalg.norm(ev_delta[:2]))
    orientation_change = quaternion_angle(start['ev_q_wxyz'], end['ev_q_wxyz'])
    angle = None
    if expected_length > .01 and fc_length > .01:
        angle = math.degrees(math.acos(float(np.clip(expected[:2] @ fc_delta[:2] / (expected_length*fc_length), -1, 1))))
    scale_error = abs(fc_length-expected_length)/expected_length if expected_length > .01 else None
    vector_error = float(np.linalg.norm(fc_delta[:2]-expected[:2]))
    forward = rotation(start['ev_q_wxyz'])[:2, 0]
    forward_angle = None
    if ev_length > .01 and np.linalg.norm(forward) > .1:
        forward_angle = math.degrees(math.acos(float(np.clip(ev_delta[:2] @ forward / (ev_length*np.linalg.norm(forward)), -1, 1))))
    verified = bool(.15 <= ev_length <= .5 and abs(ev_delta[2]) <= .04 and orientation_change <= 3
                    and angle is not None and angle <= 10 and scale_error <= .1 and vector_error <= .04)
    return {'verified': verified, 'scope': 'stationary-endpoint displacement consistency, NOT independent accuracy or flight validation',
            'ev_delta_frd_m': ev_delta.tolist(), 'ev_delta_map_m': [ev_delta[0], -ev_delta[1], -ev_delta[2]],
            'expected_fc_delta_m': expected.tolist(), 'observed_fc_delta_m': fc_delta.tolist(),
            'ev_horizontal_distance_m': ev_length, 'fc_horizontal_distance_m': fc_length,
            'horizontal_direction_difference_deg': angle, 'relative_length_difference': scale_error,
            'horizontal_vector_difference_m': vector_error, 'body_rotation_change_deg': orientation_change,
            'movement_vs_body_forward_deg': forward_angle,
            'forward_motion_candidate_consistent': forward_angle is not None and forward_angle <= 15,
            'independent_metrology_accuracy_verified': False, 'vertical_fusion_verified': False}


def validate_control(data):
    if (not isinstance(data, dict) or set(data) != {'sequence', 'command'}
            or type(data['sequence']) is not int or not 1 <= data['sequence'] <= 10
            or data['command'] not in ('finish', 'cancel')):
        raise ValueError('Manual control accepts only bounded finish/cancel observations')
    return data
