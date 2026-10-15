#!/usr/bin/env python3
"""MircoAir位姿→LOCAL_FRD MAVLink2预览；无串口、无网络飞控发送入口。"""

import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from pymavlink.dialects.v20 import common as m
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from uav_coord_bridge.mavlink_vision import PoseGate, odometry_message, transform_rigid_pose


class ViconMavlinkPreview(Node):
    def __init__(self):
        super().__init__('vicon_mavlink_preview')
        defaults = {
            'pose_topic': '/vicon/MircoAir/MircoAir', 'expected_frame': 'map',
            'preview_rate_hz': 30.0, 'max_sample_age_ms': 150.0,
            'body_origin_in_rigid': [0.0, 0.0, 0.0],
            'body_orientation_in_rigid_xyzw': [0.0, 0.0, 0.0, 1.0],
            'extrinsics_verified': False, 'run_seconds': 0.0, 'output_file': '',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        rate = self.config['preview_rate_hz']
        if not math.isfinite(rate) or not 1 <= rate <= 50:
            raise ValueError('preview_rate_hz must be 1..50')
        if not math.isfinite(self.config['run_seconds']) or not 0 <= self.config['run_seconds'] <= 300:
            raise ValueError('run_seconds must be 0..300; 0 means monitor until interrupted')
        self.output = Path(self.config['output_file']) if self.config['output_file'] else None
        if self.output is not None and self.output.exists():
            raise ValueError('Refusing to overwrite preview evidence')
        # 同样校验外参占位值，但占位值不会被宣称为安装标定结果。
        transform_rigid_pose([0, 0, 0], [0, 0, 0, 1], self.config['body_origin_in_rigid'],
                             self.config['body_orientation_in_rigid_xyzw'])
        self.gate = PoseGate(self.config['expected_frame'], self.config['max_sample_age_ms'])
        self.encoder = m.MAVLink(None, srcSystem=250, srcComponent=197)
        self.started = time.monotonic()
        self.preview_count = self.encoded_bytes = 0
        self.first_sample = self.last_sample = None
        self.preview_pub = self.create_publisher(Odometry, '/uav/mavlink_vision_preview', 5)
        self.status_pub = self.create_publisher(String, '/uav/mavlink_vision_status', 5)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(PoseStamped, self.config['pose_topic'], self.on_pose, qos)
        self.timer = self.create_timer(1 / rate, self.on_timer)
        self.status_timer = self.create_timer(1.0, self.on_status)
        self.done = False
        self.get_logger().info('预览模式：不向飞控发送数据，不改EKF参数，不解锁。安装外参默认尚未验证。')

    def on_pose(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        p, q = message.pose.position, message.pose.orientation
        self.gate.ingest(stamp, self.get_clock().now().nanoseconds, time.monotonic_ns(),
                         message.header.frame_id, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def on_timer(self):
        duration = self.config['run_seconds']
        if duration > 0 and time.monotonic() - self.started >= duration:
            self.done = True
            return
        sample = self.gate.take_new(self.get_clock().now().nanoseconds, time.monotonic_ns())
        if sample is None:
            return
        position, quaternion = transform_rigid_pose(
            sample.position, sample.quaternion, self.config['body_origin_in_rigid'],
            self.config['body_orientation_in_rigid_xyzw'])
        message = odometry_message(self.encoder, sample.stamp_ns // 1000, position, quaternion)
        packet = message.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq + 1) % 256
        self.preview_count += 1
        self.encoded_bytes += len(packet)
        record = {'stamp_ns': sample.stamp_ns, 'position_frd_m': position,
                  'quaternion_wxyz': quaternion, 'packet_bytes': len(packet)}
        if self.first_sample is None:
            self.first_sample = record
        self.last_sample = record
        preview = Odometry()
        preview.header.stamp.sec = sample.stamp_ns // 10**9
        preview.header.stamp.nanosec = sample.stamp_ns % 10**9
        preview.header.frame_id = 'mocap_local_frd_preview'
        preview.child_frame_id = 'aircraft_frd_preview'
        preview.pose.pose.position.x, preview.pose.pose.position.y, preview.pose.pose.position.z = position
        (preview.pose.pose.orientation.w, preview.pose.pose.orientation.x,
         preview.pose.pose.orientation.y, preview.pose.pose.orientation.z) = quaternion
        for index in (0, 7, 14):
            preview.pose.covariance[index] = 0.02**2
        for index in (21, 28, 35):
            preview.pose.covariance[index] = 0.1**2
        for vector in (preview.twist.twist.linear, preview.twist.twist.angular):
            vector.x = vector.y = vector.z = math.nan
        preview.twist.covariance[0] = math.nan
        self.preview_pub.publish(preview)

    def status(self):
        fresh = self.gate.fresh(self.get_clock().now().nanoseconds, time.monotonic_ns())
        reason = self.gate.reason
        if not fresh and reason == 'fresh':
            reason = 'stale_input'
        return {'scope': 'ROS2 and in-memory MAVLink encoding preview only; no flight-controller messages',
                'source_topic': self.config['pose_topic'], 'expected_frame': self.config['expected_frame'],
                'input_fresh': fresh,
                'accepted': self.gate.accepted, 'rejected': self.gate.rejected,
                'expired_before_encoding': self.gate.expired, 'reason': reason,
                'preview_count': self.preview_count, 'encoded_bytes': self.encoded_bytes,
                'actual_sent_to_fc': 0, 'extrinsics_verified': self.config['extrinsics_verified'],
                'timestamp_layer': 'source ROS header stamp; receiver uses arrival time, camera exposure time unverified',
                'first_sample': self.first_sample, 'last_sample': self.last_sample,
                'duration_s': round(time.monotonic() - self.started, 3)}

    def on_status(self):
        status = self.status()
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, allow_nan=False)
        self.status_pub.publish(message)
        self.get_logger().info(f"fresh={status['input_fresh']} accepted={status['accepted']} "
                               f"preview={status['preview_count']} rejected={status['rejected']} sent_to_fc=0")

    def save(self):
        if self.output is not None:
            with self.output.open('x') as stream:
                json.dump(self.status(), stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write('\n')


def main():
    rclpy.init()
    node = None
    try:
        node = ViconMavlinkPreview()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.save()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
