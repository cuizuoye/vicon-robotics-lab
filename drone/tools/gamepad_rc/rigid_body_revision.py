"""Offline same-body frame migration and an explicit live-injection revision gate.

Physical forward is inherited, not redefined. Four fixed marker positions and
the operator-described motion select a PROVISIONAL registration. This is not
an IMU calibration, dynamic pose validation, or permission for flight.
"""
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import quaternion_to_matrix, matrix_to_quaternion
from uav_coord_bridge.mavlink_vision import checked_quaternion, finite_vector

REGISTRY = Path(__file__).resolve().parent / 'known_incidents/mircoair_rigid_body_revision.json'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def solve_registration(old_markers_mm, new_markers_mm, old_mount_xyzw, new_world_xyzw,
                       *, only_old_marker_four_moved_confirmed=False):
    if not only_old_marker_four_moved_confirmed:
        raise ValueError('Operator confirmation of exactly one moved physical marker required')
    old, new = np.asarray(old_markers_mm, dtype=float), np.asarray(new_markers_mm, dtype=float)
    if old.shape != (5, 3) or new.shape != (5, 3) or not np.isfinite([old, new]).all():
        raise ValueError('Two finite five-marker constellations required')
    mount = quaternion_to_matrix(checked_quaternion(old_mount_xyzw))
    world = quaternion_to_matrix(checked_quaternion(new_world_xyzw))
    fixed = [0, 1, 2, 4]  # old physical elevated marker was named MircoAir4.
    a = old[fixed]
    if np.linalg.svd(a - a.mean(0), compute_uv=False)[1] < 1:
        raise ValueError('Non-collinear fixed marker geometry required')
    all_rows, plausible = [], []
    for permutation in itertools.permutations(range(5), 4):
        b = new[list(permutation)]
        u, _, vt = np.linalg.svd((a - a.mean(0)).T @ (b - b.mean(0)))
        handedness = np.eye(3)
        handedness[2, 2] = np.linalg.det(vt.T @ u.T)
        rotation = vt.T @ handedness @ u.T
        translation = b.mean(0) - rotation @ a.mean(0)
        errors = np.linalg.norm(a @ rotation.T + translation - b, axis=1)
        moved = next(i for i in range(5) if i not in permutation)
        motion_flu = mount.T @ (rotation.T @ (new[moved] - translation) - old[3])
        up_dot = float((world @ rotation @ mount)[2, 2])
        row = {'old_fixed_names': [f'MircoAir{i+1}' for i in fixed],
               'new_matching_names': [f'MircoAir{i+1}' for i in permutation],
               'new_moved_marker_name_candidate': f'MircoAir{moved+1}',
               'rotation_old_rigid_to_new_rigid': rotation.tolist(),
               'translation_old_rigid_to_new_rigid_mm': translation.tolist(),
               'fixed_marker_fit_rms_mm': float(np.sqrt(np.mean(errors**2))),
               'fixed_marker_fit_max_mm': float(errors.max()),
               'moved_marker_forward_left_up_mm': motion_flu.tolist(),
               'body_up_world_up_dot': up_dot}
        all_rows.append(row)
        # Broad, labelled operator-approximation bounds, NOT precision tolerances.
        if (row['fixed_marker_fit_rms_mm'] <= 3 and up_dot > .95
                and 10 <= motion_flu[0] <= 70 and abs(motion_flu[1]) <= 15
                and -50 <= motion_flu[2] <= -10):
            plausible.append(row)
    if len(plausible) != 1:
        raise ValueError('No unique registration consistent with fixed points, upright body and reported forward/downward relocation')
    return {**plausible[0], 'hypotheses_evaluated': len(all_rows),
            'operator_motion_bounds_forward_left_up_mm': [[10, 70], [-15, 15], [-50, -10]],
            'operator_approximation_used_to_disambiguate': True,
            'registration_quality': 'provisional_geometric_transfer',
            'independently_verified': False, 'automatic_live_adoption_allowed': False}


def rebase(registration, old_heading, old_board, revision):
    rotation = np.asarray(registration['rotation_old_rigid_to_new_rigid'], dtype=float)
    translation = finite_vector(registration['translation_old_rigid_to_new_rigid_mm'], 3) / 1000
    mounting = quaternion_to_matrix(checked_quaternion(old_heading['body_orientation_in_rigid_xyzw_candidate']))
    center = finite_vector(old_board['board_center_in_rigid_m_candidate'], 3)
    new_mounting = rotation @ mounting
    heading = {
        'scope': 'Offline new-rigid-frame candidate; inherits physical nose, not a new forward movement calibration',
        'rigid_body_revision_id': revision,
        'body_orientation_in_rigid_xyzw_candidate': matrix_to_quaternion(new_mounting),
        'forward_heading_in_rigid_xy_deg': math.degrees(math.atan2(new_mounting[1, 0], new_mounting[0, 0])),
        'forward_heading_in_map_deg': None,
        'manual_forward_horizontal_confirmed': old_heading.get('manual_forward_horizontal_confirmed') is True,
        'physical_forward_definition_inherited': True,
        'new_manual_forward_translation_performed': False,
        'orientation_candidate_quality': 'provisional_geometric_transfer',
        'source_files': [], 'actual_sent_to_fc': 0,
        'body_origin_in_rigid': None, 'origin_offset_verified': False,
        'north_alignment_verified': False, 'time_sync_verified': False,
        'ready_to_send_to_fc': False, 'extrinsics_verified': False,
        'warnings': ['Rebuilt body: transferred candidate requires locked/static validation before live adoption']}
    # Do not retain old-layout marker height/name assumptions in the new record.
    board = {'scope': 'Transferred approximate board geometric center; NOT calibrated IMU center',
        'rigid_body_revision_id': revision,
        'target_reference': old_board['target_reference'],
        'board_center_in_rigid_m_candidate': (rotation @ center + translation).tolist(),
        'measurement_quality': 'old approximate board center plus ~millimeter rigid-registration uncertainty; not quantified',
        'old_elevated_marker_height_assumption_reused': False,
        'new_moved_marker_name_candidate': registration['new_moved_marker_name_candidate'],
        'marker_name': None, 'body_origin_in_rigid': None,
        'imu_offset_verified': False, 'extrinsics_verified': False,
        'ready_to_send_to_fc': False, 'actual_sent_to_fc': 0}
    return heading, board


def inspect_current_calibration(heading_path, board_path, *, registry_path=REGISTRY):
    """Verify version/hashes, WITHOUT granting a live-injection permission."""
    registry = json.loads(Path(registry_path).read_text())
    revision = registry['active_revision_id']
    heading = json.loads(Path(heading_path).read_text())
    board = json.loads(Path(board_path).read_text())
    if any(doc.get('rigid_body_revision_id') != revision for doc in (heading, board)):
        raise ValueError('MircoAir was rebuilt: old rigid-body extrinsics are invalid; use current revision candidates')
    if (digest(heading_path) != registry['heading_candidate_sha256']
            or digest(board_path) != registry['board_candidate_sha256']):
        raise ValueError('Current revision candidate hash mismatch')
    return registry


def require_current_calibration(heading_path, board_path, *, registry_path=REGISTRY):
    """Live senders fail closed; inspection alone never grants permission.

    This supplements existing incident/physical/one-use guards; it never clears
    them. Read-only and restore-only paths remain usable.
    """
    registry = inspect_current_calibration(heading_path, board_path, registry_path=registry_path)
    if registry.get('static_validation_completed') is not True:
        raise ValueError('Current rigid-body transfer is provisional; locked/static validation not completed; no live pose injection')
    return registry
