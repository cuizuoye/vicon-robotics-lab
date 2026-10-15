#!/usr/bin/env python3
"""从两次真实ROS只读采集估计机头方向；不接串口、不修改运行中的预览。"""

import argparse
import datetime
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import matrix_to_quaternion, quaternion_to_matrix  # noqa: E402
from uav_coord_bridge.mavlink_vision import checked_quaternion, finite_vector  # noqa: E402


def endpoint(record):
    """只读采集通过且结束时新鲜、静止，才用最后一帧作端点；不伪造曝光时间。"""
    if record.get('source_checks_passed') is not True or record.get('frame_ids') != ['map']:
        raise ValueError('invalid_source_capture')
    if record.get('pose_topic') != '/vicon/MircoAir/MircoAir':
        raise ValueError('wrong_rigid_body')
    timing = record['source_timing']
    if timing['samples'] < 100 or not math.isfinite(timing['rate_hz']) or timing['rate_hz'] < 30:
        raise ValueError('insufficient_live_samples')
    gap = timing['maximum_arrival_gap_ms']
    if not math.isfinite(gap) or not 0 <= gap <= 150:
        raise ValueError('capture_continuity_exceeds_150_ms')
    std = finite_vector(record['position_std_mm'], 3)
    if np.any(std < 0) or np.any(std > 3):
        raise ValueError('endpoint_not_stationary')
    pose = record['last_pose']
    captured = datetime.datetime.fromisoformat(record['captured_at_utc'])
    if captured.utcoffset() is None:
        raise ValueError('capture_timezone_missing')
    age = captured.timestamp() - pose['stamp_ns'] / 1e9
    if not 0 <= age <= 0.15:
        raise ValueError('stale_endpoint')
    return (finite_vector(pose['position_m'], 3),
            checked_quaternion(pose['quaternion_xyzw']), pose['stamp_ns'])


def solve_heading(start, end, forward_horizontal_confirmed=False):
    if not forward_horizontal_confirmed:
        raise ValueError('manual_forward_and_level_confirmation_required')
    p0, q0, t0 = endpoint(start)
    p1, q1, t1 = endpoint(end)
    if not 0 < (t1 - t0) / 1e9 <= 1800:
        raise ValueError('endpoint_order_or_session_mismatch')
    delta = p1 - p0
    distance = float(np.linalg.norm(delta[:2]))
    if not 0.15 <= distance <= 0.8 or abs(delta[2]) > 0.05:
        raise ValueError('translation_outside_calibration_bounds')
    rotation_change = math.degrees(2 * math.acos(float(np.clip(abs(q0 @ q1), 0, 1))))
    if rotation_change > 5:
        raise ValueError('aircraft_turned_more_than_5_degrees')

    # 平移只观测前向；机体z向上来自用户确认水平的约束，不是IMU滚俯测量。
    x = np.array([delta[0] / distance, delta[1] / distance, 0.0])
    z = np.array([0.0, 0.0, 1.0])
    body_in_world = np.column_stack((x, np.cross(z, x), z))
    if q0 @ q1 < 0:
        q1 = -q1
    mean_q = (q0 + q1) / np.linalg.norm(q0 + q1)
    body_in_rigid = quaternion_to_matrix(mean_q).T @ body_in_world
    mount_q = matrix_to_quaternion(body_in_rigid)
    heading_world = math.degrees(math.atan2(x[1], x[0]))
    heading_rigid = math.degrees(math.atan2(body_in_rigid[1, 0], body_in_rigid[0, 0]))
    warnings = []
    if rotation_change > 2:
        warnings.append('Endpoint orientation changed >2 degrees: coarse candidate, repeat before deployment')
    if not 0.2 <= distance <= 0.3:
        warnings.append('Observed distance differs from requested 20..30 cm')
    return {
        'scope': 'Offline mounting-direction candidate from real ROS captures; no flight-controller send',
        'manual_forward_horizontal_confirmed': True,
        'endpoint_position_source': 'last valid pose in each stationary capture; not an averaged pose',
        'start_stamp_ns': t0, 'end_stamp_ns': t1,
        'delta_map_m': delta.tolist(), 'horizontal_distance_m': distance,
        'endpoint_rotation_change_deg': rotation_change,
        'forward_heading_in_map_deg': heading_world,
        'forward_heading_in_rigid_xy_deg': heading_rigid,
        'body_orientation_in_rigid_xyzw_candidate': mount_q,
        'body_origin_in_rigid': None,
        'origin_offset_verified': False,
        'orientation_candidate_quality': 'coarse' if rotation_change > 2 else 'initial_estimate',
        'extrinsics_verified': False, 'north_alignment_verified': False,
        'time_sync_verified': False, 'ready_to_send_to_fc': False,
        'actual_sent_to_fc': 0, 'warnings': warnings,
    }


def compare_candidates(first, second, tolerance_deg=5.0):
    """同一安装的两次机头估计应一致；单次动作质量通过不代表标定可采纳。"""
    if not math.isfinite(tolerance_deg) or not 0 < tolerance_deg <= 15:
        raise ValueError('invalid_consistency_tolerance')
    q0 = checked_quaternion(first['body_orientation_in_rigid_xyzw_candidate'])
    q1 = checked_quaternion(second['body_orientation_in_rigid_xyzw_candidate'])
    difference = math.degrees(2 * math.acos(float(np.clip(abs(q0 @ q1), 0, 1))))
    consistent = difference <= tolerance_deg
    return {'mounting_candidate_difference_deg': difference,
            'consistency_tolerance_deg': tolerance_deg,
            'mounting_candidates_consistent': consistent,
            'automatic_adoption_allowed': False,
            'extrinsics_verified': False, 'ready_to_send_to_fc': False,
            'action': 'Confirm physical nose and movement labels before adoption' if not consistent
                      else 'Still confirm mounting origin and independent body-axis observations'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', type=Path, required=True)
    parser.add_argument('--end', type=Path, required=True)
    parser.add_argument('--forward-horizontal-confirmed', action='store_true')
    parser.add_argument('--compare-with', type=Path,
                        help='Optional previous candidate; inconsistency is never auto-resolved')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to overwrite calibration evidence')
    with args.start.open() as stream:
        start = json.load(stream)
    with args.end.open() as stream:
        end = json.load(stream)
    result = solve_heading(start, end, args.forward_horizontal_confirmed)
    if args.compare_with is not None:
        with args.compare_with.open() as stream:
            previous = json.load(stream)
        result['repeat_consistency'] = compare_candidates(previous, result)
    result.update({'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                   'source_files': [str(args.start.resolve()), str(args.end.resolve())]})
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
