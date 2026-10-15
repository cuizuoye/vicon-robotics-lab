#!/usr/bin/env python3
"""把 nav_msgs/Odometry 发布成 TF 和 PoseStamped。

典型用途: Gazebo (ros_gz_bridge) 的真值里程计 -> TF，让 RViz2 里能看到机体。
"""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):

    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic', '/gz/odometry')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'gz_base_link')
        self.declare_parameter('publish_pose', False)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_pose = self.get_parameter('publish_pose').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = None
        if self.publish_pose:
            self.pose_pub = self.create_publisher(PoseStamped, f'{self.odom_topic}/pose', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos)
        self.get_logger().info(f'订阅 {self.odom_topic} -> TF [{self.parent_frame} -> '
                               f'{self.child_frame}]')

    def on_odom(self, msg: Odometry):
        stamp = msg.header.stamp

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        if self.pose_pub is not None:
            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = self.parent_frame
            pose.pose = msg.pose.pose
            self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = OdomToTf()
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
