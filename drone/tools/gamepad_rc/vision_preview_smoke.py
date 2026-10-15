#!/usr/bin/env python3
"""独立ROS域内的合成位姿运行测试，不打开串口，不发送飞控报文。"""

import argparse
import datetime
import json
import os
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from uav_coord_bridge.vicon_mavlink_preview import ViconMavlinkPreview


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output-dir', type=Path, required=True)
    args = ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '91':
        ap.error('Use process-local ROS_DOMAIN_ID=91; never inject test poses into the live Vicon domain')
    args.output_dir = args.output_dir.resolve()
    capture = args.output_dir / 'synthetic_vision_preview.json'
    report = args.output_dir / 'synthetic_ros_smoke.json'
    if not args.output_dir.is_dir() or capture.exists() or report.exists():
        ap.error('Use an existing output folder with new evidence filenames')
    rclpy.init(args=['--ros-args', '-p', 'pose_topic:=/test/mircoair_pose',
                     '-p', 'run_seconds:=5.0', '-p', f'output_file:={capture}'])
    preview = ViconMavlinkPreview()
    source = Node('synthetic_pose_source')
    publisher = source.create_publisher(PoseStamped, '/test/mircoair_pose', 5)
    observed = []
    subscription = source.create_subscription(
        Odometry, '/uav/mavlink_vision_preview', observed.append, 5)
    executor = SingleThreadedExecutor()
    executor.add_node(preview)
    executor.add_node(source)
    start = time.monotonic()
    next_sample = 0.5  # 留出DDS发现时间，不依赖真实Vicon网络。
    published = 0
    steady_count = None
    try:
        while not preview.done:
            elapsed = time.monotonic() - start
            if 0.5 <= elapsed < 2.5 and elapsed >= next_sample:
                message = PoseStamped()
                message.header.frame_id = 'map'
                message.header.stamp = source.get_clock().now().to_msg()
                message.pose.position.x = 0.1 * (elapsed - 0.5)
                message.pose.position.y = 0.2
                message.pose.position.z = 1.0
                message.pose.orientation.w = 1.0
                publisher.publish(message)
                published += 1
                next_sample = elapsed + 1 / 30
            executor.spin_once(timeout_sec=0.005)
            if elapsed >= 3.0 and steady_count is None:
                steady_count = preview.preview_count
        status = preview.status()
        checks = {
            'fresh_synthetic_samples_encoded': preview.preview_count >= 10,
            'ros_preview_received': len(observed) >= 5,
            'world_conversion_correct': bool(observed)
                and all(abs(msg.pose.pose.position.y + 0.2) < 1e-6
                        and abs(msg.pose.pose.position.z + 1.0) < 1e-6 for msg in observed),
            'stale_input_not_replayed': steady_count is not None
                and steady_count == preview.preview_count and not status['input_fresh'],
            'no_flight_controller_transmission': status['actual_sent_to_fc'] == 0,
            'not_claimed_as_mount_calibration': status['extrinsics_verified'] is False,
        }
        result = {
            'scope': 'Synthetic ROS2 runtime test, NOT live Vicon or flight-controller fusion',
            'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'ros_domain_id': 91, 'published_samples': published,
            'received_previews': len(observed), 'checks': checks,
            'passed': all(checks.values()), 'preview_status': status,
        }
        with report.open('x') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result['passed'] else 1
    finally:
        preview.save()
        executor.remove_node(preview)
        executor.remove_node(source)
        source.destroy_subscription(subscription)
        preview.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
