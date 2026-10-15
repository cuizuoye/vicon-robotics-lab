#!/usr/bin/env python3
import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from .math_utils import quaternion_from_yaw, wrap_angle, yaw_from_quaternion


class CarVisualizer(Node):
    """生成实际轨迹、目标直线预览和目标箭头，供 RViz2 显示。"""

    def __init__(self):
        super().__init__('car_visualizer')
        self.declare_parameter('pose_topic', '/car/pose')
        self.declare_parameter('goal_topic', '/car/goal_pose')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('max_actual_points', 5000)
        self.declare_parameter('min_path_step_m', 0.005)
        self.declare_parameter('min_path_step_yaw_rad', 0.01)
        self.declare_parameter('target_path_step_m', 0.02)

        pose_topic = str(self.get_parameter('pose_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.max_actual_points = int(self.get_parameter('max_actual_points').value)
        self.min_path_step_m = float(self.get_parameter('min_path_step_m').value)
        self.min_path_step_yaw = float(self.get_parameter('min_path_step_yaw_rad').value)
        self.target_path_step_m = max(0.001, float(self.get_parameter('target_path_step_m').value))

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.actual_pub = self.create_publisher(Path, '/car/path_actual', 10)
        self.target_pub = self.create_publisher(Path, '/car/path_target', transient_qos)
        self.marker_pub = self.create_publisher(Marker, '/car/goal_marker', transient_qos)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        self.create_subscription(PoseStamped, goal_topic, self.on_goal, 10)

        self.actual_path = Path()
        self.actual_path.header.frame_id = self.world_frame
        self.latest_pose = None
        self.goal = None
        self.goal_key = None
        self.target_built_for = None
        self.get_logger().info(
            f'RViz数据: {pose_topic} -> /car/path_actual, '
            f'{goal_topic} -> /car/path_target + /car/goal_marker')

    @staticmethod
    def pose_key(msg: PoseStamped):
        return (
            round(msg.pose.position.x, 6),
            round(msg.pose.position.y, 6),
            round(msg.pose.orientation.z, 6),
            round(msg.pose.orientation.w, 6),
        )

    def on_pose(self, msg: PoseStamped):
        self.latest_pose = copy.deepcopy(msg)
        append = not self.actual_path.poses
        if self.actual_path.poses:
            previous = self.actual_path.poses[-1]
            distance = math.hypot(
                msg.pose.position.x - previous.pose.position.x,
                msg.pose.position.y - previous.pose.position.y,
            )
            current_yaw = yaw_from_quaternion(
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w)
            previous_yaw = yaw_from_quaternion(
                previous.pose.orientation.x, previous.pose.orientation.y,
                previous.pose.orientation.z, previous.pose.orientation.w)
            append = distance >= self.min_path_step_m or abs(
                wrap_angle(current_yaw - previous_yaw)) >= self.min_path_step_yaw

        if append:
            point = copy.deepcopy(msg)
            point.header.frame_id = self.world_frame
            self.actual_path.poses.append(point)
            if len(self.actual_path.poses) > self.max_actual_points:
                self.actual_path.poses = self.actual_path.poses[-self.max_actual_points:]

        self.actual_path.header.stamp = self.get_clock().now().to_msg()
        self.actual_pub.publish(self.actual_path)
        if self.goal is not None and self.target_built_for != self.goal_key:
            self.publish_target_path()

    def on_goal(self, msg: PoseStamped):
        key = self.pose_key(msg)
        if key == self.goal_key:
            return
        self.goal = copy.deepcopy(msg)
        self.goal.header.frame_id = self.world_frame
        self.goal_key = key
        self.target_built_for = None
        self.publish_goal_marker()
        if self.latest_pose is not None:
            self.publish_target_path()

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = 'mecanum_goal'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(self.goal.pose)
        marker.scale.x = 0.20
        marker.scale.y = 0.045
        marker.scale.z = 0.045
        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.10
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.0).to_msg()
        self.marker_pub.publish(marker)

    def publish_target_path(self):
        start = self.latest_pose
        goal = self.goal
        distance = math.hypot(
            goal.pose.position.x - start.pose.position.x,
            goal.pose.position.y - start.pose.position.y,
        )
        count = max(2, int(math.ceil(distance / self.target_path_step_m)) + 1)
        start_yaw = yaw_from_quaternion(
            start.pose.orientation.x, start.pose.orientation.y,
            start.pose.orientation.z, start.pose.orientation.w)
        goal_yaw = yaw_from_quaternion(
            goal.pose.orientation.x, goal.pose.orientation.y,
            goal.pose.orientation.z, goal.pose.orientation.w)
        yaw_delta = wrap_angle(goal_yaw - start_yaw)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.world_frame
        for index in range(count):
            ratio = index / (count - 1)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = (
                start.pose.position.x
                + ratio * (goal.pose.position.x - start.pose.position.x))
            pose.pose.position.y = (
                start.pose.position.y
                + ratio * (goal.pose.position.y - start.pose.position.y))
            yaw = wrap_angle(start_yaw + ratio * yaw_delta)
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        # 该蓝线只是当前点到目标的显示参考，位置控制器仍直接跟踪目标位姿。
        self.target_pub.publish(path)
        self.target_built_for = self.goal_key


def main():
    rclpy.init()
    node = CarVisualizer()
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

