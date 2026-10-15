"""断动力人工升降：只比较静止端点的有符号高度增量，不控制飞机。"""

import math
import numpy as np

from manual_translation import endpoint as horizontal_endpoint, rotation, quaternion_angle


def endpoint(rows):
    if not rows or any(r.get('local', {}).get('z_valid') is not True for r in rows):
        return None
    result = horizontal_endpoint(rows)
    if not result:
        return None
    fc = np.asarray([[r['local'][n] for n in ('x', 'y', 'z')] for r in rows], dtype=float)
    center = np.asarray(result['fc_position_m'])
    spread = float(np.max(np.linalg.norm(fc - center, axis=1)))
    if spread > .03:
        return None
    result['fc_3d_stationary_spread_m'] = spread
    result['fc_vertical_stationary_spread_m'] = float(np.max(np.abs(fc[:, 2] - center[2])))
    return result


def assess_vertical(start, end):
    if not start or not end:
        return {'verified': False, 'reason': 'missing_stationary_endpoints'}
    ev_delta = np.asarray(end['ev_position_frd_m']) - start['ev_position_frd_m']
    fc_delta = np.asarray(end['fc_position_m']) - start['fc_position_m']
    # 同样先对齐世界系，不将不同航向/倾角的世界系直接逐分量比较。
    alignment = rotation(start['fc_q_wxyz']) @ rotation(start['ev_q_wxyz']).T
    expected = alignment @ ev_delta
    ev_up, fc_up, expected_up = -float(ev_delta[2]), -float(fc_delta[2]), -float(expected[2])
    error = abs(fc_up - expected_up)
    scale = error / abs(expected_up) if abs(expected_up) >= .01 else None
    sideways = float(np.linalg.norm(ev_delta[:2]))
    angle = quaternion_angle(start['ev_q_wxyz'], end['ev_q_wxyz'])
    g0 = rotation(start['ev_q_wxyz']).T @ np.array([0, 0, 1])
    g1 = rotation(end['ev_q_wxyz']).T @ np.array([0, 0, 1])
    tilt = math.degrees(math.acos(float(np.clip(g0 @ g1, -1, 1))))
    # 小幅手动转向仅记录警告；独立限制倾斜与侧移，避免未知杆臂变成高度误差。
    checks = {'net_vicon_lift_8_to_40_cm': .08 <= ev_up <= .4,
              'expected_ekf_lift_8_to_40_cm': .08 <= expected_up <= .4,
              'ekf_lift_same_positive_direction': fc_up > 0,
              'horizontal_drift_at_most_8_cm': sideways <= .08,
              'tilt_axis_change_at_most_5_deg': tilt <= 5,
              'vertical_increment_error_at_most_5_cm': error <= .05,
              'vertical_scale_error_at_most_20_percent': scale is not None and scale <= .2}
    return {'verified': all(checks.values()), 'checks': checks,
            'scope': 'Signed vertical endpoint increment consistency; NOT independent accuracy, ground zero or flight validation',
            'ev_delta_frd_m': ev_delta.tolist(), 'expected_fc_delta_m': expected.tolist(),
            'observed_fc_delta_m': fc_delta.tolist(), 'vicon_net_lift_m': ev_up,
            'expected_ekf_net_lift_m': expected_up, 'observed_ekf_net_lift_m': fc_up,
            'vertical_increment_difference_m': error, 'relative_vertical_increment_difference': scale,
            'vicon_horizontal_drift_m': sideways, 'body_rotation_change_deg': angle,
            'tilt_axis_change_deg': tilt, 'body_rotation_warning_only': angle > 3,
            'absolute_height_offset_removed_by_endpoint_difference': True,
            'independent_accuracy_verified': False, 'ground_zero_verified': False,
            'vision_height_reference_verified': False, 'flight_ready': False}
