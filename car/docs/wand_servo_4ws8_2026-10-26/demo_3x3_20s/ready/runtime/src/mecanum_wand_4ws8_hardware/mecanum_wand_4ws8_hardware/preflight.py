"""Read Vicon/graph only: no transport instance, hardware service or UDP write."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from mecanum_wand_4ws8_control.safety_nodes import decode_pose
from .commissioning import Runtime, CONSENT
from .nodes import RawSource


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=5.)
    parser.add_argument('--car-samples', type=Path,
                        help='optional read-only pose capture for deterministic noise-estimator regression')
    args = parser.parse_args()
    if not 3 <= args.seconds <= 30: raise ValueError('read-only sample duration must be 3..30s')
    context = Context()
    rclpy.init(context=context, domain_id=0)
    node = RawSource(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    samples, seen = {'car': [], 'wand': []}, {}
    try:
        until = time.monotonic()+args.seconds
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
            for source, msg in node.samples.items():
                pose, stamp, roll, pitch = decode_pose(msg, 'map')
                if seen.get(source) == stamp: continue
                seen[source] = stamp
                samples[source].append(dict(x=pose.x, y=pose.y, z=pose.z, yaw=pose.yaw,
                                            roll=roll, pitch=pitch, source_ros=stamp,
                                            received_steady=time.monotonic()))
        report = dict(verification='READ-ONLY VICON/GRAPH; NO HARDWARE TRANSPORT',
                      source_counts={k: len(v) for k, v in samples.items()},
                      conflicts=node.audit(), rejected=node.rejected,
                      blockers=[], current_poses={}, calibration_candidates={})
        for source, rows in samples.items():
            if len(rows) < 10: report['blockers'].append(source.upper()+'_UNAVAILABLE_OR_TOO_FEW_SAMPLES')
            else:
                report['current_poses'][source] = rows[-1]
                if source == 'car':
                    report['calibration_candidates'] = dict(
                        ground_z_mean_m=statistics.mean(r['z'] for r in rows),
                        ground_z_std_m=statistics.pstdev(r['z'] for r in rows),
                        car_xy_range_m=max(math.dist((r['x'], r['y']), (rows[0]['x'], rows[0]['y'])) for r in rows),
                        note='Candidate only: confirm the installed rigid body is fixed and car is on the ground; no auto approval')
        if report['conflicts']: report['blockers'].append('LEGACY_OR_DUPLICATE_SOURCE_CONFLICT')
        report['hardware_authorized'] = False
        with args.output.open('x') as stream: json.dump(report, stream, ensure_ascii=False, indent=2)
        if args.car_samples:
            with args.car_samples.open('x') as stream: json.dump(samples['car'], stream, indent=2)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def review_main():
    parser = argparse.ArgumentParser(description='Validate a user-reviewed profile WITHOUT opening any transport')
    parser.add_argument('profile', type=Path)
    args = parser.parse_args()
    digest = hashlib.sha256(args.profile.read_bytes()).hexdigest()
    Runtime('hardware', str(args.profile), digest, CONSENT)  # Validation only, no Node/socket.
    print('PROFILE_VALIDATED_ONLY; NOT PHYSICAL ACCEPTANCE; NO SOCKET OPENED')
    print('approved_sha256:='+digest)
