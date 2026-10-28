#!/usr/bin/env python3
"""Pre-started left-translation goal source for suspended commissioning.

This node never opens either motion gate.  Start it before the guarded run so
that no new DDS participant is created while the motors are enabled.  Once the
trajectory is active it replaces the generated goal with a body-left target,
then remains alive until the safety gate closes.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32


def body_left_goal(pose, distance_m):
    """Return a map-frame pose displaced along the pose's positive body Y."""
    q = pose.pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(distance_m) or distance_m <= 0.0 or norm < 1e-9:
        raise ValueError('left goal requires a positive distance and valid quaternion')
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    goal = PoseStamped()
    goal.header.frame_id = pose.header.frame_id
    goal.pose.position.x = pose.pose.position.x - distance_m * math.sin(yaw)
    goal.pose.position.y = pose.pose.position.y + distance_m * math.cos(yaw)
    goal.pose.position.z = pose.pose.position.z
    goal.pose.orientation = pose.pose.orientation
    return goal, yaw


class SuspendedLeftGoal(Node):
    def __init__(self):
        super().__init__('suspended_left_goal')
        self.declare_parameter('left_distance_m', 0.15)
        self.declare_parameter('activation_delay_s', 0.50)
        self.declare_parameter('startup_timeout_s', 20.0)
        self.distance = float(self.get_parameter('left_distance_m').value)
        self.delay = float(self.get_parameter('activation_delay_s').value)
        self.deadline = time.monotonic() + float(
            self.get_parameter('startup_timeout_s').value)
        if not 0.02 <= self.distance <= 0.30:
            raise ValueError('left_distance_m must be between 0.02 and 0.30')
        if not 0.20 <= self.delay <= 2.0:
            raise ValueError('activation_delay_s must be between 0.20 and 2.0')

        latest = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/car/goal_pose', latched)
        self.create_subscription(PoseStamped, '/car/pose', self.on_pose, latest)
        self.create_subscription(Bool, '/car/trajectory_active', self.on_active, latched)
        self.create_subscription(Bool, '/car/safety_enabled', self.on_safety, 10)
        self.create_subscription(UInt32, '/car/motor_fault_bits', self.on_fault, 10)
        self.create_timer(0.02, self.tick)

        self.pose = None
        self.pose_time = None
        self.active_since = None
        self.safety_seen = False
        self.safety = False
        self.fault = None
        self.goal = None
        self.goal_yaw = None
        self.published = 0

    def on_pose(self, msg):
        self.pose = msg
        self.pose_time = time.monotonic()

    def on_active(self, msg):
        if msg.data and self.active_since is None:
            self.active_since = time.monotonic()

    def on_safety(self, msg):
        self.safety = bool(msg.data)
        self.safety_seen |= self.safety

    def on_fault(self, msg):
        self.fault = int(msg.data)

    def finish(self, reason):
        print(json.dumps({
            'reason': reason,
            'goal_published': self.published > 0,
            'publish_count': self.published,
            'left_distance_m': self.distance,
            'goal_x': None if self.goal is None else self.goal.pose.position.x,
            'goal_y': None if self.goal is None else self.goal.pose.position.y,
            'anchor_yaw_rad': self.goal_yaw,
            'fault_bits': self.fault,
        }, ensure_ascii=False), flush=True)
        rclpy.shutdown()

    def tick(self):
        now = time.monotonic()
        if self.safety_seen and not self.safety:
            self.finish('safety gate closed')
            return
        if now >= self.deadline and not self.safety_seen:
            self.finish('startup timeout')
            return
        if self.fault not in (None, 0):
            self.finish(f'STM32 fault {self.fault:#x}')
            return
        if self.active_since is None or now - self.active_since < self.delay:
            return
        if self.pose is None or self.pose_time is None or now - self.pose_time > 0.20:
            return
        if self.goal is None:
            self.goal, self.goal_yaw = body_left_goal(self.pose, self.distance)
            self.get_logger().warn(
                f'发布车体左移{self.distance:.3f}m悬空目标；节点保持在线直到安全门关闭')
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(self.goal)
        self.published += 1


def main():
    rclpy.init()
    node = SuspendedLeftGoal()
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
