#!/usr/bin/env python3
"""Vicon(动捕) 位姿 -> PX4 EKF2 外部位姿 (px4_msgs/VehicleOdometry)。

数据流:
    Vicon -> (vicon_receiver) -> geometry_msgs/PoseStamped (+TwistStamped)
          -> 本节点(ENU/FLU -> NED/FRD)
          -> /fmu/in/vehicle_visual_odometry  (uXRCE-DDS -> PX4 EKF2)

关键结论（依据 PX4 v1.17 源码）:
1. EKF2 只订阅 uORB 话题 `vehicle_visual_odometry`（`vehicle_mocap_odometry`
   只被 LPE 使用），所以默认发布 /fmu/in/vehicle_visual_odometry。
2. PX4 反序列化 DDS 消息时对 timestamp / timestamp_sample 做:
       timestamp == 0 -> 用收到消息的时刻 (hrt_absolute_time)
       否则           -> timestamp - time_offset  (time_offset 来自 DDS timesync)
   DDS timesync 的远端时钟是 Agent 所在主机的 epoch 时间(纳秒)，因此这里用
   ROS 系统时间(epoch 微秒)最为自然；若 timesync 未收敛，PX4 会把时间戳
   钳到自身收到消息的时刻，是安全的回退。
3. PX4 的 /fmu/in/* 订阅使用 BEST_EFFORT + VOLATILE + KEEP_LAST，
   ROS 侧发布者也必须是 BEST_EFFORT，否则 QoS 不兼容收不到数据。
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import TimesyncStatus, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from uav_coord_bridge.frames import (
    enu_to_ned_vector,
    flu_to_frd_vector,
    quat_enu_flu_to_ned_frd,
    quat_flu_to_frd_local,
    quat_ros_to_px4,
)


def _px4_qos(depth: int = 5) -> QoSProfile:
    """与 PX4 uXRCE-DDS 客户端订阅一致的 QoS。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class ViconPoseToPx4Odometry(Node):

    def __init__(self):
        super().__init__('vicon_pose_to_px4_odometry')

        self.declare_parameter('pose_topic', '/vicon/drone/drone')
        self.declare_parameter('twist_topic', '')
        self.declare_parameter('odometry_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('input_frame', 'ENU')
        self.declare_parameter('timestamp_mode', 'ros_epoch')
        self.declare_parameter('publish_velocity', True)
        self.declare_parameter('publish_angular_velocity', False)
        self.declare_parameter('max_publish_rate_hz', 200.0)
        self.declare_parameter('position_variance', [0.0001, 0.0001, 0.0002])
        self.declare_parameter('orientation_variance', [0.0005, 0.0005, 0.001])
        self.declare_parameter('velocity_variance', [0.001, 0.001, 0.001])
        self.declare_parameter('timesync_topic', '/fmu/out/timesync_status')

        self.pose_topic = self.get_parameter('pose_topic').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.input_frame = self.get_parameter('input_frame').value.upper()
        self.timestamp_mode = self.get_parameter('timestamp_mode').value
        self.publish_velocity = self.get_parameter('publish_velocity').value
        self.publish_angular_velocity = self.get_parameter('publish_angular_velocity').value
        self.max_publish_rate_hz = float(self.get_parameter('max_publish_rate_hz').value)
        self.position_variance = list(self.get_parameter('position_variance').value)
        self.orientation_variance = list(self.get_parameter('orientation_variance').value)
        self.velocity_variance = list(self.get_parameter('velocity_variance').value)
        self.timesync_topic = self.get_parameter('timesync_topic').value

        if self.input_frame not in ('ENU', 'FRD'):
            raise ValueError(f"input_frame 只能是 'ENU' 或 'FRD'，收到 {self.input_frame}")
        if self.timestamp_mode not in ('ros_epoch', 'arrival'):
            raise ValueError(
                f"timestamp_mode 只能是 'ros_epoch' 或 'arrival'，收到 {self.timestamp_mode}")

        self.last_twist = None
        self.timesync_ok = False
        self.last_publish_ns = 0

        self.odom_pub = self.create_publisher(VehicleOdometry, self.odometry_topic, _px4_qos(1))
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.on_pose, _px4_qos(10))

        self.twist_sub = None
        if self.publish_velocity and self.twist_topic:
            self.twist_sub = self.create_subscription(
                TwistStamped, self.twist_topic, self.on_twist, _px4_qos(10))

        self.timesync_sub = self.create_subscription(
            TimesyncStatus, self.timesync_topic, self.on_timesync, _px4_qos(5))

        self.get_logger().info(
            f'{self.pose_topic} ({self.input_frame}) -> {self.odometry_topic} (NED/FRD), '
            f'timestamp_mode={self.timestamp_mode}')

    def on_timesync(self, msg: TimesyncStatus):
        # estimated_offset 收敛后才会明显非 0；round_trip_time 为 0 表示还没握手
        self.timesync_ok = msg.round_trip_time > 0
        if not self.timesync_ok:
            self.get_logger().warn(
                'DDS 时间同步尚未收敛：PX4 将按“收到消息的时刻”处理时间戳 '
                '(需 UXRCE_DDS_SYNCT=1 且 Agent 已连接)',
                throttle_duration_sec=10.0)

    def on_twist(self, msg: TwistStamped):
        self.last_twist = msg

    def _timestamp_us(self, pose_msg) -> int:
        if self.timestamp_mode == 'arrival':
            # 置 0：PX4 侧用收到消息的时刻，完全不需要时间同步
            return 0
        # ROS 系统时间(epoch) 微秒 —— 与 DDS timesync 的远端时钟一致
        return int(self.get_clock().now().nanoseconds // 1000)

    def on_pose(self, pose_msg: PoseStamped):
        now_ns = self.get_clock().now().nanoseconds
        if self.max_publish_rate_hz > 0.0:
            min_interval_ns = int(1e9 / self.max_publish_rate_hz)
            if now_ns - self.last_publish_ns < min_interval_ns:
                return
        self.last_publish_ns = now_ns

        pos = pose_msg.pose.position
        ori = pose_msg.pose.orientation

        if self.input_frame == 'ENU':
            # Vicon 世界系 ENU + 机体系 FLU -> PX4 NED/FRD
            position = enu_to_ned_vector([pos.x, pos.y, pos.z])
            quat = quat_enu_flu_to_ned_frd([ori.x, ori.y, ori.z, ori.w])
            pose_frame = VehicleOdometry.POSE_FRAME_NED
            velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        else:
            # 直接用 Vicon 世界系作为 PX4 的局部 FRD 系（允许存在任意偏航偏置）
            position = flu_to_frd_vector([pos.x, pos.y, pos.z])
            quat = quat_flu_to_frd_local([ori.x, ori.y, ori.z, ori.w])
            pose_frame = VehicleOdometry.POSE_FRAME_FRD
            velocity_frame = VehicleOdometry.VELOCITY_FRAME_FRD

        msg = VehicleOdometry()
        msg.timestamp = self._timestamp_us(pose_msg)
        msg.timestamp_sample = msg.timestamp
        msg.pose_frame = pose_frame
        msg.position = [float(v) for v in position]
        # 内部/ROS 用 (x, y, z, w)，px4_msgs 用 (w, x, y, z)
        msg.q = [float(v) for v in quat_ros_to_px4(quat)]

        if not self.publish_velocity:
            msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            msg.velocity = [math.nan, math.nan, math.nan]
            msg.velocity_variance = [math.nan, math.nan, math.nan]
        elif self.last_twist is not None:
            lin = self.last_twist.twist.linear
            if self.input_frame == 'ENU':
                velocity = enu_to_ned_vector([lin.x, lin.y, lin.z])
            else:
                velocity = flu_to_frd_vector([lin.x, lin.y, lin.z])
            msg.velocity_frame = velocity_frame
            msg.velocity = [float(v) for v in velocity]
            msg.velocity_variance = [float(v) for v in self.velocity_variance]
        else:
            # 没有速度来源：留 NaN，EKF2 只用位置+姿态
            msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            msg.velocity = [math.nan, math.nan, math.nan]
            msg.velocity_variance = [math.nan, math.nan, math.nan]

        if self.publish_angular_velocity and self.last_twist is not None:
            ang = self.last_twist.twist.angular
            ang_frd = flu_to_frd_vector([ang.x, ang.y, ang.z])
            msg.angular_velocity = [float(v) for v in ang_frd]
        else:
            msg.angular_velocity = [math.nan, math.nan, math.nan]

        msg.position_variance = [float(v) for v in self.position_variance]
        msg.orientation_variance = [float(v) for v in self.orientation_variance]
        msg.reset_counter = 0
        msg.quality = 0

        self.odom_pub.publish(msg)

def main():
    rclpy.init()
    node = ViconPoseToPx4Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
