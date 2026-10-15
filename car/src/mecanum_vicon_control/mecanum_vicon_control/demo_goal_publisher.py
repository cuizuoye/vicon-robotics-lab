#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .math_utils import quaternion_from_yaw


class DemoGoalPublisher(Node):
    def __init__(self):
        super().__init__('demo_goal_publisher')
        self.declare_parameter('goal_x_m', 1.0)
        self.declare_parameter('goal_y_m', 0.5)
        self.declare_parameter('goal_yaw_rad', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.publisher = self.create_publisher(PoseStamped, '/car/goal_pose', 10)
        self.timer = self.create_timer(0.5, self.publish_goal)

    def publish_goal(self):
        yaw = float(self.get_parameter('goal_yaw_rad').value)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.pose.position.x = float(self.get_parameter('goal_x_m').value)
        msg.pose.position.y = float(self.get_parameter('goal_y_m').value)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.publisher.publish(msg)
        # 只发布一次，避免后续覆盖 RViz 鼠标指定的目标。
        self.timer.cancel()
        self.get_logger().info(
            f'已发布一次演示目标: x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}')


def main():
    rclpy.init()
    node = DemoGoalPublisher()
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
