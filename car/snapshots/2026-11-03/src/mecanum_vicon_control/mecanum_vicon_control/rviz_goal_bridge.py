#!/usr/bin/env python3
import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty

from .math_utils import quaternion_from_yaw, yaw_from_quaternion


def sanitize_planar_goal(msg: PoseStamped, world_frame: str) -> PoseStamped:
    """Validate an RViz goal and project its orientation onto the ground plane."""
    source_frame = msg.header.frame_id.strip()
    if source_frame and source_frame != world_frame:
        raise ValueError(
            f'goal frame {source_frame!r} does not match world frame {world_frame!r}')

    values = (
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('goal contains a non-finite number')

    quaternion_norm = math.sqrt(
        msg.pose.orientation.x ** 2
        + msg.pose.orientation.y ** 2
        + msg.pose.orientation.z ** 2
        + msg.pose.orientation.w ** 2)
    if quaternion_norm < 1.0e-9:
        raise ValueError('goal quaternion has zero length')

    normalized = tuple(value / quaternion_norm for value in (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ))
    yaw = yaw_from_quaternion(*normalized)
    qx, qy, qz, qw = quaternion_from_yaw(yaw)

    result = copy.deepcopy(msg)
    result.header.frame_id = world_frame
    result.pose.position.z = 0.0
    result.pose.orientation.x = qx
    result.pose.orientation.y = qy
    result.pose.orientation.z = qz
    result.pose.orientation.w = qw
    return result


class RvizGoalBridge(Node):
    """Convert RViz 2D Goal Pose messages into the car's validated goal topic."""

    def __init__(self):
        super().__init__('rviz_goal_bridge')
        self.declare_parameter('input_goal_topic', '/goal_pose')
        self.declare_parameter('output_goal_topic', '/car/goal_pose')
        self.declare_parameter('world_frame', 'map')

        input_topic = str(self.get_parameter('input_goal_topic').value)
        output_topic = str(self.get_parameter('output_goal_topic').value)
        self.world_frame = str(self.get_parameter('world_frame').value)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.goal_pub = self.create_publisher(PoseStamped, output_topic, latched_qos)
        self.cancel_pub = self.create_publisher(Empty, '/car/trajectory_cancel', 10)
        self.create_subscription(PoseStamped, input_topic, self.on_goal, 10)
        self.get_logger().info(
            f'RViz目标桥接: {input_topic} -> {output_topic}, frame={self.world_frame}')

    def on_goal(self, msg: PoseStamped):
        try:
            goal = sanitize_planar_goal(msg, self.world_frame)
        except ValueError as error:
            self.get_logger().warning(f'拒绝RViz目标: {error}')
            return

        goal.header.stamp = self.get_clock().now().to_msg()
        self.cancel_pub.publish(Empty())
        self.goal_pub.publish(goal)
        yaw = yaw_from_quaternion(
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w,
        )
        self.get_logger().info(
            f'RViz目标: x={goal.pose.position.x:.3f}, '
            f'y={goal.pose.position.y:.3f}, yaw={yaw:.3f} rad')


def main():
    rclpy.init()
    node = RvizGoalBridge()
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
