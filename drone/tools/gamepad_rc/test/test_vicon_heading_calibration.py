"""离线标定数学与输入闸门，不连接真实Vicon/飞控。"""

import copy
import datetime
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vicon_heading_calibration import compare_candidates, solve_heading  # noqa: E402
from uav_coord_bridge.frames import quaternion_to_matrix  # noqa: E402
from uav_coord_bridge.mavlink_vision import transform_rigid_pose  # noqa: E402


def snapshot(p, yaw=0, seconds=0):
    stamp = 1_700_000_000_000_000_000 + seconds * 10**9
    return {
        'source_checks_passed': True, 'frame_ids': ['map'],
        'pose_topic': '/vicon/MircoAir/MircoAir',
        'source_timing': {'samples': 1650, 'rate_hz': 330, 'maximum_arrival_gap_ms': 5},
        'position_std_mm': [0.03, 0.03, 0.05],
        'captured_at_utc': datetime.datetime.fromtimestamp(
            stamp / 1e9 + 0.001, datetime.timezone.utc).isoformat(),
        'last_pose': {'stamp_ns': stamp, 'position_m': p,
                      'quaternion_xyzw': [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]},
    }


def pair(heading=0, rigid_yaw=0):
    return (snapshot([0, 0, 1], rigid_yaw),
            snapshot([0.25 * math.cos(heading), 0.25 * math.sin(heading), 1], rigid_yaw, 30))


def test_mounting_direction_not_assumed_rigid_x():
    heading, rigid_yaw = -0.5, 0.8
    start, end = pair(heading, rigid_yaw)
    result = solve_heading(start, end, True)
    mount = result['body_orientation_in_rigid_xyzw_candidate']
    forward = quaternion_to_matrix(start['last_pose']['quaternion_xyzw']) @ quaternion_to_matrix(mount)[:, 0]
    assert np.allclose(forward, [math.cos(heading), math.sin(heading), 0])
    _, q = transform_rigid_pose([0, 0, 1], start['last_pose']['quaternion_xyzw'],
                                body_orientation_in_rigid=mount)
    assert np.allclose(q, [math.cos(heading / 2), 0, 0, -math.sin(heading / 2)])


def test_unknown_origin_not_silently_zeroed_or_full_extrinsics_verified():
    result = solve_heading(*pair(), True)
    assert result['body_origin_in_rigid'] is None
    assert result['extrinsics_verified'] is result['ready_to_send_to_fc'] is False
    assert result['actual_sent_to_fc'] == 0


def test_antipodal_quaternion_has_no_spurious_rotation():
    start, end = pair()
    end['last_pose']['quaternion_xyzw'] = [0, 0, 0, -1]
    assert solve_heading(start, end, True)['endpoint_rotation_change_deg'] == 0


def test_user_movement_confirmation_is_required():
    with pytest.raises(ValueError, match='confirmation_required'):
        solve_heading(*pair())


def test_small_turn_is_coarse_not_precise_calibration():
    start, end = pair()
    end = snapshot(end['last_pose']['position_m'], math.radians(4.8), 30)
    result = solve_heading(start, end, True)
    assert result['orientation_candidate_quality'] == 'coarse'
    assert result['warnings']


@pytest.mark.parametrize('distance', [0, 0.05, 0.81])
def test_too_small_or_large_translation_rejected(distance):
    start, end = pair()
    end['last_pose']['position_m'][0] = distance
    with pytest.raises(ValueError, match='translation_outside'):
        solve_heading(start, end, True)


def test_height_change_and_large_rotation_rejected():
    start, end = pair()
    end['last_pose']['position_m'][2] += 0.06
    with pytest.raises(ValueError, match='translation_outside'):
        solve_heading(start, end, True)
    end = snapshot([0.25, 0, 1], math.radians(6), 30)
    with pytest.raises(ValueError, match='turned_more'):
        solve_heading(start, end, True)


@pytest.mark.parametrize('bad', ['quality', 'frame', 'topic', 'moving', 'age', 'order', 'nan'])
def test_bad_endpoint_cannot_be_adopted(bad):
    start, end = pair()
    end = copy.deepcopy(end)
    if bad == 'quality':
        end['source_checks_passed'] = False
    elif bad == 'frame':
        end['frame_ids'] = ['other']
    elif bad == 'topic':
        end['pose_topic'] = '/vicon/car01/car01'
    elif bad == 'moving':
        end['position_std_mm'][0] = 10
    elif bad == 'age':
        end['last_pose']['stamp_ns'] -= 10**9
    elif bad == 'order':
        end = start
    else:
        end['last_pose']['position_m'][0] = math.nan
    with pytest.raises(ValueError):
        solve_heading(start, end, True)


def test_perpendicular_movements_cannot_be_adopted_as_same_nose_direction():
    first = solve_heading(*pair(), True)
    second = solve_heading(*pair(math.pi / 2), True)
    result = compare_candidates(first, second)
    assert result['mounting_candidate_difference_deg'] == pytest.approx(90)
    assert not result['mounting_candidates_consistent']
    assert not result['automatic_adoption_allowed']


def test_consistency_never_substitutes_for_origin_and_full_extrinsics():
    first = solve_heading(*pair(), True)
    result = compare_candidates(first, first)
    assert result['mounting_candidates_consistent']
    assert result['extrinsics_verified'] is result['ready_to_send_to_fc'] is False


@pytest.mark.parametrize('limit', [0, 16, float('nan')])
def test_invalid_consistency_limit_rejected(limit):
    first = solve_heading(*pair(), True)
    with pytest.raises(ValueError):
        compare_candidates(first, first, limit)


@pytest.mark.parametrize('gap', [151, 337, float('nan'), -1])
def test_stable_last_pose_does_not_hide_interrupted_capture(gap):
    start, end = pair()
    end['source_timing']['maximum_arrival_gap_ms'] = gap
    with pytest.raises(ValueError, match='capture_continuity'):
        solve_heading(start, end, True)
