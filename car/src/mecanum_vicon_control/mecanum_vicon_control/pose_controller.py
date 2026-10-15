#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from .math_utils import clamp, slew, world_error_to_body, wrap_angle, yaw_from_quaternion


class PoseController(Node):
    """二维位置外环：Vicon 位姿与目标位姿转换为车体系 vx/vy/wz。"""

    def __init__(self):
        super().__init__('pose_controller')
        self.declare_parameter('pose_topic', '/car/pose')
        self.declare_parameter('goal_topic', '/car/goal_pose')
        self.declare_parameter('cmd_topic', '/car/cmd_vel_raw')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('kp_x', 0.9)
        self.declare_parameter('kp_y', 0.9)
        self.declare_parameter('kp_yaw', 1.5)
        self.declare_parameter('max_vx_mps', 0.30)
        self.declare_parameter('max_vy_mps', 0.30)
        self.declare_parameter('max_wz_radps', 0.8)
        self.declare_parameter('max_linear_accel_mps2', 0.45)
        self.declare_parameter('max_angular_accel_radps2', 1.2)
        self.declare_parameter('position_tolerance_m', 0.02)
        self.declare_parameter('yaw_tolerance_rad', 0.035)

        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.kp_x = float(self.get_parameter('kp_x').value)
        self.kp_y = float(self.get_parameter('kp_y').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.max_vx = float(self.get_parameter('max_vx_mps').value)
        self.max_vy = float(self.get_parameter('max_vy_mps').value)
        self.max_wz = float(self.get_parameter('max_wz_radps').value)
        self.max_linear_accel = float(self.get_parameter('max_linear_accel_mps2').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel_radps2').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance_m').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance_rad').value)

        pose_topic = str(self.get_parameter('pose_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.pose = None
        self.goal = None
        self.last_goal_key = None
        self.command = Twist()
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.reached_pub = self.create_publisher(Bool, '/car/goal_reached', 10)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        self.create_subscription(PoseStamped, goal_topic, self.on_goal, 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(f'位置控制器: {pose_topic} + {goal_topic} -> {cmd_topic}')

    def on_pose(self, msg: PoseStamped):
        self.pose = msg

    def on_goal(self, msg: PoseStamped):
        self.goal = msg
        goal_key = (
            round(msg.pose.position.x, 6), round(msg.pose.position.y, 6),
            round(msg.pose.orientation.z, 6), round(msg.pose.orientation.w, 6),
        )
        if goal_key != self.last_goal_key:
            self.last_goal_key = goal_key
            self.get_logger().info(
                f'收到目标: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}')

    def tick(self):
        if self.pose is None or self.goal is None:
            self.command = Twist()
            self.cmd_pub.publish(self.command)
            self.reached_pub.publish(Bool(data=False))
            return

        pose_yaw = yaw_from_quaternion(
            self.pose.pose.orientation.x, self.pose.pose.orientation.y,
            self.pose.pose.orientation.z, self.pose.pose.orientation.w)
        goal_yaw = yaw_from_quaternion(
            self.goal.pose.orientation.x, self.goal.pose.orientation.y,
            self.goal.pose.orientation.z, self.goal.pose.orientation.w)
        ex = self.goal.pose.position.x - self.pose.pose.position.x
        ey = self.goal.pose.position.y - self.pose.pose.position.y
        yaw_error = wrap_angle(goal_yaw - pose_yaw)
        body_ex, body_ey = world_error_to_body(ex, ey, pose_yaw)

        reached = math.hypot(ex, ey) <= self.position_tolerance and abs(yaw_error) <= self.yaw_tolerance
        target_vx = 0.0 if reached else clamp(self.kp_x * body_ex, -self.max_vx, self.max_vx)
        target_vy = 0.0 if reached else clamp(self.kp_y * body_ey, -self.max_vy, self.max_vy)
        target_wz = 0.0 if reached else clamp(self.kp_yaw * yaw_error, -self.max_wz, self.max_wz)

        dt = 1.0 / self.rate_hz
        self.command.linear.x = slew(
            self.command.linear.x, target_vx, self.max_linear_accel * dt)
        self.command.linear.y = slew(
            self.command.linear.y, target_vy, self.max_linear_accel * dt)
        self.command.angular.z = slew(
            self.command.angular.z, target_wz, self.max_angular_accel * dt)
        self.cmd_pub.publish(self.command)
        self.reached_pub.publish(Bool(data=reached))


def main():
    rclpy.init()
    node = PoseController()
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
