#!/usr/bin/env python3
"""PX4 (NED/FRD) 里程计 -> ROS 2 (ENU/FLU) 位姿 / 里程计 / TF。

订阅: /fmu/out/vehicle_odometry        (px4_msgs/msg/VehicleOdometry)
发布: <frame_prefix>/vehicle_odometry_enu  (nav_msgs/msg/Odometry)
      TF: map -> base_link

用途: 在 RViz2 里查看 PX4 估计器输出、和 Vicon/Gazebo 真值对照、录制数据集。
"""

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from uav_coord_bridge.frames import (
    ned_to_enu_vector,
    quat_ned_frd_to_enu_flu,
    quat_px4_to_ros,
)


class Px4OdometryToRos(Node):

    def __init__(self):
        super().__init__('px4_odometry_to_ros')

        self.declare_parameter('odometry_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('output_topic', '/px4/odometry_enu')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # PX4 侧 (px4_msgs) 使用 best-effort + volatile
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            VehicleOdometry, self.odometry_topic, self.on_odometry, px4_qos)
        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            f'订阅 {self.odometry_topic} (PX4 NED/FRD) -> 发布 {self.output_topic} (ROS ENU/FLU)')

    def on_odometry(self, msg: VehicleOdometry):
        # PX4: [N, E, D] -> ROS: [E, N, -U] -> [x, y, z]
        position = ned_to_enu_vector(msg.position)
        # msg.q 是 (w, x, y, z)，先转成 (x, y, z, w) 再做 NED/FRD -> ENU/FLU
        quat = quat_ned_frd_to_enu_flu(quat_px4_to_ros(msg.q))
        velocity = ned_to_enu_vector(msg.velocity)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.parent_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = velocity[0]
        odom.twist.twist.linear.y = velocity[1]
        odom.twist.twist.linear.z = velocity[2]
        self.pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.child_frame
            tf.transform.translation.x = position[0]
            tf.transform.translation.y = position[1]
            tf.transform.translation.z = position[2]
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = Px4OdometryToRos()
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
