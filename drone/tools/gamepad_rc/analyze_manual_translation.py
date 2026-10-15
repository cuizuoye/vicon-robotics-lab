#!/usr/bin/env python3
"""只读分析已结束人工台架；不改原证据，不接串口，不把迟到确认改成现场确认。"""

import argparse
import datetime
import json
from pathlib import Path
import sys

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.mavlink_vision import transform_rigid_pose
from manual_translation import endpoint, assess_translation, quaternion_angle, rotation


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for name in ('record','current-pose','heading','board','output','plot'):
        ap.add_argument('--'+name, type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists() or args.plot.exists():
        ap.error('New analysis/plot required; original evidence stays unchanged')
    data = json.loads(args.record.read_text())
    current = json.loads(args.current_pose.read_text())
    heading = json.loads(args.heading.read_text())
    board = json.loads(args.board.read_text())
    start = data['manual_start']
    candidate = endpoint(data['manual_pairs'][-5:])
    if not start or not candidate or not current['source_checks_passed']:
        ap.error('Missing valid stationary start/tail or current live source')
    p, q = transform_rigid_pose(current['last_pose']['position_m'], current['last_pose']['quaternion_xyzw'],
                                board['board_center_in_rigid_m_candidate'],
                                heading['body_orientation_in_rigid_xyzw_candidate'])
    current_distance = float(np.linalg.norm(np.asarray(p)-candidate['ev_position_frd_m']))
    current_angle = quaternion_angle(q,candidate['ev_q_wxyz'])
    same_tail = current_distance <= .005 and current_angle <= 1
    comparison = assess_translation(start,candidate)
    rows = data['manual_pairs']
    ev_delta = np.asarray([r['sent']['position'] for r in rows])-start['ev_position_frd_m']
    fc_delta = np.asarray([[r['local'][n] for n in ('x','y','z')] for r in rows])-start['fc_position_m']
    ev_forward = ev_delta @ rotation(start['ev_q_wxyz'])[:,0]
    fc_forward = fc_delta[:,:2] @ rotation(start['fc_q_wxyz'])[:2,0]
    transform = rotation(start['fc_q_wxyz']) @ rotation(start['ev_q_wxyz']).T
    vector_errors = np.linalg.norm((ev_delta @ transform.T)[:,:2]-fc_delta[:,:2],axis=1)
    times = np.asarray([r['elapsed_s'] for r in rows])
    control_path = args.record.parent / 'manual_control.json'
    ended = datetime.datetime.fromisoformat(data['captured_at_utc']).timestamp()
    late_control = control_path.exists() and control_path.stat().st_mtime > ended
    report = {
        'scope':'Posthoc interpretation of recorded forward-then-reverse unpowered motion; NOT independent metrology',
        'record_source':str(args.record.resolve()), 'current_pose_source':str(args.current_pose.resolve()),
        'operator_report':'Small forward movement followed by reverse movement due to USB cable length',
        'interactive_finish_received':data['manual_finish_requested_at_s'] is not None,
        'control_file_created_after_bench_ended':late_control,
        'original_record_modified':False, 'endpoint_provenance':'Last five valid stationary pairs BEFORE automatic restore',
        'recovered_endpoint_candidate':candidate, 'current_pose_matches_tail':same_tail,
        'current_pose_to_tail_distance_m':current_distance, 'current_pose_to_tail_rotation_deg':current_angle,
        'endpoint_comparison':comparison,
        'strict_single_forward_translation_verified':comparison['verified'] and comparison['forward_motion_candidate_consistent'] and same_tail and not late_control,
        'horizontal_fusion_observed_during_motion':data['horizontal_fusion_verified'],
        'source_forward_peak_m':float(max(ev_forward)), 'source_reverse_peak_m':float(-min(ev_forward)),
        'fc_forward_peak_m':float(max(fc_forward)), 'fc_reverse_peak_m':float(-min(fc_forward)),
        'signed_motion_correlation':float(np.corrcoef(ev_forward,fc_forward)[0,1]),
        'horizontal_comparison_p95_m':float(np.percentile(vector_errors,95)),
        'maximum_horizontal_comparison_m':float(max(vector_errors)),
        'comparison_errors_are_not_independent_vicon_accuracy':True,
        'dynamic_comparison_uses_arrival_pairs_not_exposure_synchronized_ground_truth':True,
        'clock_max_observed_estimated_residual_us':max(abs(r['clock']['observed_offset']-r['clock']['estimated_offset']) for r in data['fusion_snapshots']),
        'rollback_verified':data['rollback_verified'], 'ev_ctrl_at_end':data['parameters_after']['EKF2_EV_CTRL'],
        'flight_ready':False,
    }
    with args.output.open('x') as f:
        json.dump(report,f,indent=2,ensure_ascii=False,allow_nan=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax = plt.subplots(figsize=(8,3.8),layout='constrained')
    ax.plot(times-start['elapsed_end_s'],ev_forward,label='Vicon board-center projection',linewidth=2)
    ax.plot(times-start['elapsed_end_s'],fc_forward,label='PX4 EKF horizontal projection',linewidth=1.3,alpha=.85)
    ax.set(xlabel='Time since stationary start (s)',ylabel='Relative forward displacement (m)',
           title='Unpowered forward/reverse observation (not independent accuracy)')
    ax.grid(alpha=.25); ax.legend(); fig.savefig(args.plot,dpi=150); plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k not in ('recovered_endpoint_candidate',)},indent=2,ensure_ascii=False))
    return 0 if same_tail and data['rollback_verified'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
