#!/usr/bin/env python3
"""只读五个marker与刚体原点的几何关系；不能据此推断IMU位置。"""

import argparse
from collections import OrderedDict
import datetime
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import quaternion_to_matrix  # noqa: E402
from uav_coord_bridge.mavlink_vision import checked_quaternion, finite_vector  # noqa: E402


def relative_geometry(position, quaternion, markers, mounting):
    """坐标均为map；输出刚体系及候选FLU轴，后者原点仍是刚体原点。"""
    p = finite_vector(position, 3)
    rotation = quaternion_to_matrix(checked_quaternion(quaternion))
    mount = quaternion_to_matrix(checked_quaternion(mounting))
    points = np.asarray(markers, dtype=float)
    if points.shape != (5, 3) or not np.isfinite(points).all():
        raise ValueError('five_finite_markers_required')
    rigid = (points - p) @ rotation
    return rigid, rigid @ mount


def square_board_center_candidate(marker_rigid_mm, mounting, side_mm, height_mm):
    """仅假设球心在机头角正上方：目标是板几何中心，不冒充IMU外参。"""
    if not all(math.isfinite(v) and v > 0 for v in (side_mm, height_mm)):
        raise ValueError('positive_finite_dimensions_required')
    point = finite_vector(marker_rigid_mm, 3)
    rotation = quaternion_to_matrix(checked_quaternion(mounting))
    marker_to_center_flu = np.array([-side_mm / math.sqrt(2), 0, -height_mm])
    center_rigid = point + rotation @ marker_to_center_flu
    return {'target_reference': 'flight_controller_board_geometric_center_on_upper_surface_plane',
            'assumption': 'fifth_marker_center_directly_above_forward_facing_board_corner',
            'measurement_quality': 'approximate_user_ruler_dimensions; uncertainty_not_quantified',
            'board_side_mm': side_mm, 'marker_center_height_above_board_mm': height_mm,
            'marker_to_board_center_flu_mm': marker_to_center_flu.tolist(),
            'board_center_in_rigid_m_candidate': (center_rigid / 1000).tolist(),
            'board_center_forward_left_up_mm_candidate': (rotation.T @ center_rigid).tolist(),
            'body_origin_in_rigid': None, 'imu_offset_verified': False,
            'extrinsics_verified': False, 'ready_to_send_to_fc': False, 'actual_sent_to_fc': 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--heading-candidate', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or not 5 <= args.seconds <= 30:
        parser.error('New output and 5..30 seconds required')
    candidate = json.loads(args.heading_candidate.read_text())
    if candidate.get('manual_forward_horizontal_confirmed') is not True:
        parser.error('Confirmed mounting-direction candidate required')
    mounting = checked_quaternion(candidate['body_orientation_in_rigid_xyzw_candidate'])

    # ROS仅在main中导入：纯数学测试不需要ROS，更不接飞控串口。
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, PointStamped

    class Observe(Node):
        def __init__(self):
            super().__init__('vicon_origin_readonly_observer')
            self.pending = OrderedDict()
            self.rows, self.positions, self.times, self.ages = [], [], [], []
            self.last_stamp = 0
            self.rejected = 0
            self.message_counts = [0] * 6
            self.frames = set()
            qos = QoSProfile(depth=60, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.kept = [self.create_subscription(PoseStamped, '/vicon/MircoAir/MircoAir',
                                                 lambda m: self.ingest(0, m), qos)]
            for i in range(1, 6):
                self.kept.append(self.create_subscription(
                    PointStamped, f'/vicon/MircoAir/markers/MircoAir{i}',
                    lambda m, index=i: self.ingest(index, m), qos))

        def ingest(self, index, message):
            self.message_counts[index] += 1
            self.frames.add(message.header.frame_id)
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e6
            if message.header.frame_id != 'map' or stamp <= self.last_stamp or not 0 <= age <= 150:
                self.rejected += 1
                return
            group = self.pending.setdefault(stamp, {})
            group[index] = message
            while len(self.pending) > 120:
                self.pending.popitem(last=False)
            if len(group) != 6:
                return
            pose = group[0].pose
            position = [pose.position.x, pose.position.y, pose.position.z]
            q = pose.orientation
            points = [[group[i].point.x, group[i].point.y, group[i].point.z]
                      for i in range(1, 6)]
            try:
                rigid, flu = relative_geometry(position, [q.x, q.y, q.z, q.w], points, mounting)
            except ValueError:
                self.rejected += 1
                del self.pending[stamp]
                return
            self.rows.append((rigid, flu))
            self.positions.append(position)
            self.times.append(time.monotonic())
            self.ages.append(age)
            self.last_stamp = stamp
            del self.pending[stamp]

        def result(self):
            count = len(self.rows)
            gap = float(max(np.diff(self.times)) * 1000) if count > 1 else None
            rate = (count - 1) / (self.times[-1] - self.times[0]) if count > 1 else None
            position_std = (np.std(self.positions, axis=0) * 1000).tolist() if count else None
            rigid = np.asarray([r[0] for r in self.rows])
            flu = np.asarray([r[1] for r in self.rows])
            geometry_std = float(np.max(np.std(rigid, axis=0)) * 1000) if count else None
            checks = {'at_least_100_complete_groups': count >= 100,
                      'map_frame_only': self.frames == {'map'},
                      'rate_at_least_30_hz': rate is not None and rate >= 30,
                      'gap_at_most_150_ms': gap is not None and gap <= 150,
                      'stationary_within_3_mm': position_std is not None and max(position_std) <= 3,
                      'rigid_geometry_std_within_3_mm': geometry_std is not None and geometry_std <= 3,
                      'no_invalid_or_out_of_order_messages': self.rejected == 0,
                      'latest_complete_group_fresh': bool(self.times) and time.monotonic() - self.times[-1] <= .15}
            markers = []
            if count:
                for i in range(5):
                    markers.append({'name': f'MircoAir{i+1}',
                                    'rigid_xyz_mm': (rigid.mean(axis=0)[i] * 1000).tolist(),
                                    'candidate_forward_left_up_mm': (flu.mean(axis=0)[i] * 1000).tolist()})
            centroid = rigid.mean(axis=(0, 1)) * 1000 if count else None
            return {'scope': 'ROS-only geometry observer; no serial, FC or Tracker write',
                    'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    'heading_candidate': str(args.heading_candidate.resolve()),
                    'user_layout': 'Four corners plus fifth marker ~1cm aft of front pair midpoint; IMU unknown',
                    'matching': 'Exact ROS publication header stamp, NOT camera exposure/time synchronization proof',
                    'complete_groups': count, 'message_counts_pose_then_markers': self.message_counts,
                    'rate_hz': rate, 'maximum_arrival_gap_ms': gap,
                    'position_std_mm': position_std, 'maximum_marker_relative_std_mm': geometry_std,
                    'geometry_caveat': 'Near-constant relative geometry may be model-derived; not independent raw-marker precision evidence',
                    'checks': checks, 'checks_passed': all(checks.values()), 'rejected_messages': self.rejected,
                    'markers': markers, 'centroid_minus_rigid_origin_mm': centroid.tolist() if count else None,
                    'origin_to_marker_centroid_distance_mm': float(np.linalg.norm(centroid)) if count else None,
                    'candidate_axes_origin': 'rigid origin, NOT IMU',
                    'body_origin_in_rigid': None, 'origin_offset_verified': False,
                    'extrinsics_verified': False, 'ready_to_send_to_fc': False, 'actual_sent_to_fc': 0}

    rclpy.init()
    node = Observe()
    try:
        until = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        result = node.result()
        encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
        with args.output.open('x') as stream:
            stream.write(encoded + '\n')
        print(encoded)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['checks_passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
