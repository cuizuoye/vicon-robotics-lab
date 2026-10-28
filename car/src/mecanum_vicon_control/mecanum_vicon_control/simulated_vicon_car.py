#!/usr/bin/env python3
import math
import random

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from rclpy.node import Node

from .math_utils import quaternion_from_yaw, wrap_angle


class SimulatedViconCar(Node):
    """二维软件小车：把安全速度指令积分为类似 Vicon 的刚体位姿。"""

    def __init__(self):
        super().__init__('simulated_vicon_car')
        self.declare_parameter('cmd_topic', '/car/cmd_vel')
        self.declare_parameter('pose_topic', '/vicon/car01/base')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('command_timeout_s', 0.3)
        self.declare_parameter('velocity_time_constant_s', 0.12)
        self.declare_parameter('x_gain', 0.98)
        self.declare_parameter('y_gain', 0.88)
        self.declare_parameter('yaw_gain', 0.95)
        self.declare_parameter('position_noise_std_m', 0.0005)
        self.declare_parameter('disturbance_topic', '/car/sim_disturbance')

        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)
        self.tau = max(0.001, float(self.get_parameter('velocity_time_constant_s').value))
        self.x_gain = float(self.get_parameter('x_gain').value)
        self.y_gain = float(self.get_parameter('y_gain').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.noise_std = max(0.0, float(self.get_parameter('position_noise_std_m').value))

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.target = Twist()
        self.actual = Twist()
        self.last_cmd_ns = None
        self.last_tick_ns = self.get_clock().now().nanoseconds

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.create_subscription(
            Pose2D, str(self.get_parameter('disturbance_topic').value),
            self.on_disturbance, 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(
            f'软件小车: {self.cmd_topic} -> {self.pose_topic}，{self.rate_hz:.1f} Hz')

    def on_cmd(self, msg: Twist):
        self.target = msg
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def on_disturbance(self, msg: Pose2D):
        """Apply a world-frame pose impulse in simulation only."""
        if not all(math.isfinite(value) for value in (msg.x, msg.y, msg.theta)):
            self.get_logger().error('拒绝包含NaN/Inf的仿真扰动')
            return
        self.x += msg.x
        self.y += msg.y
        self.yaw = wrap_angle(self.yaw + msg.theta)
        self.get_logger().info(
            f'已施加仿真扰动: dx={msg.x:.3f}m, dy={msg.y:.3f}m, '
            f'dyaw={math.degrees(msg.theta):.1f}deg')

    def tick(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        dt = max(0.0, min(0.1, (now_ns - self.last_tick_ns) * 1e-9))
        self.last_tick_ns = now_ns

        cmd = self.target
        if self.last_cmd_ns is None or (now_ns - self.last_cmd_ns) * 1e-9 > self.command_timeout_s:
            cmd = Twist()

        alpha = 1.0 - math.exp(-dt / self.tau) if dt > 0.0 else 0.0
        self.actual.linear.x += alpha * (cmd.linear.x - self.actual.linear.x)
        self.actual.linear.y += alpha * (cmd.linear.y - self.actual.linear.y)
        self.actual.angular.z += alpha * (cmd.angular.z - self.actual.angular.z)

        vx = self.actual.linear.x * self.x_gain
        vy = self.actual.linear.y * self.y_gain
        wz = self.actual.angular.z * self.yaw_gain
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (cos_yaw * vx - sin_yaw * vy) * dt
        self.y += (sin_yaw * vx + cos_yaw * vy) * dt
        self.yaw = wrap_angle(self.yaw + wz * dt)

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x + random.gauss(0.0, self.noise_std)
        pose.pose.position.y = self.y + random.gauss(0.0, self.noise_std)
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = SimulatedViconCar()
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
