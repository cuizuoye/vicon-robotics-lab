#!/usr/bin/env python3
"""从只读采集冻结方向；保留map坐标，无串口、ROS发布或飞控命令。"""

import argparse
import datetime
import hashlib
import json
from pathlib import Path

from uav_coord_bridge.mavlink_vision import F, finite_vector
from uav_coord_bridge.world_reference import body_pose_in_map, freeze_reference, observe_in_map


def prepare(capture, ground, heading, board):
    if capture.get('source_checks_passed') is not True or capture.get('frame_ids') != ['map']:
        raise ValueError('validated_map_capture_required')
    gap = capture['source_timing']['maximum_arrival_gap_ms']
    age = capture['ros_header_age_ms']['maximum']
    spread = finite_vector(capture['position_std_mm'], 3)
    if gap is None or not 0 <= gap <= 150 or age is None or not 0 <= age <= 150 or min(spread) < 0 or max(spread) > 10:
        raise ValueError('capture_not_continuous_fresh_and_stationary')
    pose = capture['last_pose']
    p, q = body_pose_in_map(pose['position_m'], pose['quaternion_xyzw'],
                            board['board_center_in_rigid_m_candidate'],
                            heading['body_orientation_in_rigid_xyzw_candidate'])
    ground_map = (F @ finite_vector(ground['ev_position_frd_m'], 3)).tolist()
    reference = freeze_reference(q, ground_map)
    reference.update({
        'scope': 'Offline geometry from validated live ROS capture; no FC send or flight target',
        'user_choice': 'Preserve Vicon map origin/axes; freeze present nose direction; retain initial ground height zero',
        'heading_frozen_at_utc': capture['captured_at_utc'],
        'heading_frozen_source_stamp_ns': pose['stamp_ns'],
        'orientation_source': 'last valid pose in stationary capture; mounting direction candidate applied',
        'body_reference': board['target_reference'],
        'extrinsics_verified': False,
        'current_observation': observe_in_map(reference, p, q),
        'task_origin_map_m': None,
        'flight_waypoints_generated': False,
        'safety_boundary_verified': False,
        'validity': 'Invalidate after Tracker world recalibration, rigid-body redefinition or marker/mount changes; not an arming gate',
    })
    return reference


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for key in ('capture','ground','heading','board','output'):
        ap.add_argument('--'+key, type=Path, required=True)
    args=ap.parse_args()
    if args.output.exists(): ap.error('Use a new output file; do not replace existing reference')
    inputs={name:getattr(args,name).resolve() for name in ('capture','ground','heading','board')}
    try:
        documents={name:json.loads(path.read_text()) for name,path in inputs.items()}
        result=prepare(**documents)
        result['prepared_at_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        result['source_files']={name:{'path':str(path), 'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
                                for name,path in inputs.items()}
        with args.output.open('x') as stream:
            json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
            stream.write('\n')
        print(json.dumps(result,ensure_ascii=False,indent=2))
    except (ValueError, KeyError, TypeError, OSError) as exc:
        ap.error(str(exc))


if __name__=='__main__': main()
