#!/usr/bin/env python3
"""只订阅真实Vicon与位姿预览，统计频率和质量；不连接飞控、不发布数据。"""

import argparse
import datetime
import json
import math
from pathlib import Path
import statistics
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, Int64, String


def timing(times):
    intervals_ms = [(b - a) * 1000 for a, b in zip(times, times[1:])]
    return {'samples': len(times),
            'rate_hz': (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else None,
            'maximum_arrival_gap_ms': max(intervals_ms) if intervals_ms else None}


class Capture(Node):
    def __init__(self, pose_topic):
        super().__init__('vicon_readonly_capture')
        self.pose_topic = pose_topic
        self.pose_times, self.preview_times = [], []
        self.stamps, self.ages_ms, self.positions, self.norms = [], [], [], []
        self.frame_ids = set()
        self.frames, self.latencies = [], []
        self.bad_pose_count = 0
        self.preview_last = self.last_status = self.status_time = None
        self.last_pose = None
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.subscriptions_kept = [
            self.create_subscription(PoseStamped, pose_topic, self.on_pose, qos),
            self.create_subscription(Int64, '/vicon/frame_number', self.on_frame, qos),
            self.create_subscription(Float64, '/vicon/latency', self.on_latency, qos),
            self.create_subscription(Odometry, '/uav/mavlink_vision_preview', self.on_preview, qos),
            self.create_subscription(String, '/uav/mavlink_vision_status', self.on_status, qos),
        ]

    def on_pose(self, message):
        self.pose_times.append(time.monotonic())
        self.frame_ids.add(message.header.frame_id)
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        self.stamps.append(stamp)
        self.ages_ms.append((self.get_clock().now().nanoseconds - stamp) / 1e6)
        p, q = message.pose.position, message.pose.orientation
        xyz, xyzw = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
        norm = math.sqrt(sum(v * v for v in xyzw))
        if not all(math.isfinite(v) for v in xyz + xyzw) or not 0.95 <= norm <= 1.05:
            self.bad_pose_count += 1
            return
        self.positions.append(xyz)
        self.norms.append(norm)
        self.last_pose = {'stamp_ns': stamp, 'position_m': xyz, 'quaternion_xyzw': xyzw}

    def on_frame(self, message):
        self.frames.append(message.data)

    def on_latency(self, message):
        if math.isfinite(message.data) and message.data >= 0:
            self.latencies.append(message.data * 1000)

    def on_preview(self, message):
        self.preview_times.append(time.monotonic())
        p = message.pose.pose.position
        self.preview_last = {'frame_id': message.header.frame_id,
                             'position_m': [p.x, p.y, p.z]}

    def on_status(self, message):
        self.last_status = json.loads(message.data)
        self.status_time = time.monotonic()

    def result(self):
        duplicate = sum(a == b for a, b in zip(self.stamps, self.stamps[1:]))
        backwards = sum(b < a for a, b in zip(self.stamps, self.stamps[1:]))
        checks = {
            'live_rigid_body_received': len(self.pose_times) >= 20,
            'expected_map_frame': self.frame_ids == {'map'},
            'finite_pose_and_valid_quaternion': bool(self.positions) and self.bad_pose_count == 0,
            'header_stamps_increase': len(self.stamps) >= 2 and duplicate == backwards == 0,
            'vicon_frame_number_advances': bool(self.frames) and self.frames[-1] > self.frames[0],
        }
        fresh_status = self.status_time is not None and time.monotonic() - self.status_time < 2
        preview_checks = {
            'preview_received': len(self.preview_times) >= 10,
            'fresh_status': fresh_status,
            'fresh_source_in_preview': bool(self.last_status) and self.last_status.get('input_fresh') is True,
            'flight_controller_send_count_zero': bool(self.last_status)
                and self.last_status.get('actual_sent_to_fc') == 0,
        }
        return {
            'scope': 'Live ROS2 rigid-body/preview subscription only; no flight-controller connection',
            'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'pose_topic': self.pose_topic, 'source_timing': timing(self.pose_times),
            'preview_timing': timing(self.preview_times), 'source_checks': checks,
            'source_checks_passed': all(checks.values()), 'preview_checks': preview_checks,
            'preview_checks_passed': all(preview_checks.values()),
            'frame_ids': sorted(self.frame_ids), 'invalid_pose_count': self.bad_pose_count,
            'duplicate_header_stamps': duplicate, 'backwards_header_stamps': backwards,
            'frame_numbers': {'samples': len(self.frames), 'unique': len(set(self.frames)),
                              'first': self.frames[0] if self.frames else None,
                              'last': self.frames[-1] if self.frames else None},
            'quaternion_norm_min_max': [min(self.norms), max(self.norms)] if self.norms else None,
            'position_std_mm': [statistics.pstdev(axis) * 1000 for axis in zip(*self.positions)]
                if self.positions else None,
            'ros_header_age_ms': {'median': statistics.median(self.ages_ms) if self.ages_ms else None,
                                  'maximum': max(self.ages_ms) if self.ages_ms else None,
                                  'scope': 'ROS publication-to-subscription age, NOT camera-to-estimator latency'},
            'sdk_reported_latency_ms': {'median': statistics.median(self.latencies) if self.latencies else None,
                                       'maximum': max(self.latencies) if self.latencies else None},
            'last_pose': self.last_pose, 'last_preview': self.preview_last,
            'last_preview_status': self.last_status,
        }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pose-topic', default='/vicon/MircoAir/MircoAir')
    ap.add_argument('--seconds', type=float, default=10)
    ap.add_argument('--require-preview', action='store_true')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if not 5 <= args.seconds <= 60 or args.output.exists():
        ap.error('Use 5..60 seconds and a new output file')
    rclpy.init(args=[])
    node = Capture(args.pose_topic)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
        result = node.result()
        result['duration_s'] = time.monotonic() - start
        result['passed'] = result['source_checks_passed'] and (
            result['preview_checks_passed'] if args.require_preview else True)
        with args.output.open('x') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
