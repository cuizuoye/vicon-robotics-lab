#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from .math_utils import (
    quaternion_from_yaw,
    world_error_to_body,
    wrap_angle,
    yaw_from_quaternion,
)


def validate_input_pose(msg, world_frame, now_ns, timeout_s):
    """源已变换到world_frame才可接收；时间戳是接收器时间，不是曝光时间。"""
    if msg.header.frame_id != world_frame:
        raise ValueError('Vicon坐标系不匹配：先在接收器完成变换，不能只改frame_id')
    p, q = msg.pose.position, msg.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('拒绝包含NaN/Inf的Vicon位姿')
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('拒绝长度无效或零长度的Vicon四元数')
    stamp = msg.header.stamp
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('Vicon时间戳格式无效')
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    age_s = (now_ns - stamp_ns) * 1e-9
    if stamp_ns <= 0 or age_s > timeout_s or age_s < -0.05:
        raise ValueError('拒绝无效、过期或未来的Vicon时间戳')
    return yaw_from_quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)


def pose_is_fresh(last_valid_ns, now_ns, timeout_s):
    return last_valid_ns is not None and 0 <= now_ns - last_valid_ns <= timeout_s * 1e9


class CarStateBridge(Node):
    """校验刚体位姿，并把刚体原点转换到小车 base_link。"""

    def __init__(self):
        super().__init__('car_state_bridge')
        self.declare_parameter('input_pose_topic', '/vicon/car01/base')
        self.declare_parameter('output_pose_topic', '/car/pose')
        self.declare_parameter('output_odom_topic', '/car/odometry')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('base_offset_x_m', 0.0)
        self.declare_parameter('base_offset_y_m', 0.0)
        self.declare_parameter('base_offset_yaw_rad', 0.0)
        self.declare_parameter('max_jump_m', 0.5)
        self.declare_parameter('max_jump_yaw_rad', 1.2)
        self.declare_parameter('reacquire_after_s', 0.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('pose_timeout_s', 0.2)

        self.input_topic = str(self.get_parameter('input_pose_topic').value)
        self.output_topic = str(self.get_parameter('output_pose_topic').value)
        self.odom_topic = str(self.get_parameter('output_odom_topic').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.offset_x = float(self.get_parameter('base_offset_x_m').value)
        self.offset_y = float(self.get_parameter('base_offset_y_m').value)
        self.offset_yaw = float(self.get_parameter('base_offset_yaw_rad').value)
        self.max_jump_m = float(self.get_parameter('max_jump_m').value)
        self.max_jump_yaw = float(self.get_parameter('max_jump_yaw_rad').value)
        self.reacquire_after_s = float(self.get_parameter('reacquire_after_s').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.pose_timeout_s = float(self.get_parameter('pose_timeout_s').value)
        if not math.isfinite(self.pose_timeout_s) or self.pose_timeout_s <= 0:
            raise ValueError('pose_timeout_s must be finite and positive')

        self.last_state = None
        self.last_valid_ns = None
        self.input_valid = False
        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.valid_pub = self.create_publisher(Bool, '/car/vicon_valid', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # BEST_EFFORT订阅兼容两种发布可靠性；下游仍由本节点可靠发布。
        self.create_subscription(PoseStamped, self.input_topic, self.on_pose, qos_profile_sensor_data)
        self.create_timer(0.05, self.check_timeout)

        self.get_logger().info(f'刚体桥接: {self.input_topic} -> {self.output_topic}')

    def reject(self, reason: str):
        self.input_valid = False
        self.valid_pub.publish(Bool(data=False))
        self.get_logger().warn(reason, throttle_duration_sec=1.0)

    def check_timeout(self):
        now_ns = self.get_clock().now().nanoseconds
        valid = self.input_valid and pose_is_fresh(self.last_valid_ns, now_ns, self.pose_timeout_s)
        self.valid_pub.publish(Bool(data=valid))

    def on_pose(self, msg: PoseStamped):
        now_ns = self.get_clock().now().nanoseconds
        try:
            rigid_yaw = validate_input_pose(msg, self.world_frame, now_ns, self.pose_timeout_s)
        except ValueError as error:
            self.reject(str(error))
            return
        cos_yaw = math.cos(rigid_yaw)
        sin_yaw = math.sin(rigid_yaw)
        x = msg.pose.position.x + cos_yaw * self.offset_x - sin_yaw * self.offset_y
        y = msg.pose.position.y + sin_yaw * self.offset_x + cos_yaw * self.offset_y
        yaw = wrap_angle(rigid_yaw + self.offset_yaw)

        if self.last_state is not None:
            last_x, last_y, last_yaw, last_ns = self.last_state
            can_reacquire = (now_ns - last_ns) * 1e-9 >= self.reacquire_after_s
            if math.hypot(x - last_x, y - last_y) > self.max_jump_m:
                if not can_reacquire:
                    self.reject('拒绝超过 max_jump_m 的 Vicon 位姿跳变')
                    return
            if abs(wrap_angle(yaw - last_yaw)) > self.max_jump_yaw:
                if not can_reacquire:
                    self.reject('拒绝超过 max_jump_yaw_rad 的 Vicon 航向跳变')
                    return

        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        vx = vy = wz = 0.0
        if self.last_state is not None:
            last_x, last_y, last_yaw, last_ns = self.last_state
            dt = (now_ns - last_ns) * 1e-9
            if 0.001 < dt < 0.5:
                world_vx = (x - last_x) / dt
                world_vy = (y - last_y) / dt
                vx, vy = world_error_to_body(world_vx, world_vy, yaw)
                wz = wrap_angle(yaw - last_yaw) / dt
        self.last_state = (x, y, yaw, now_ns)
        self.last_valid_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.input_valid = True

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.pose_pub.publish(pose)
        self.odom_pub.publish(odom)
        self.valid_pub.publish(Bool(data=True))

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = CarStateBridge()
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
