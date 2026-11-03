#!/usr/bin/env python3
"""RViz markers for path projection, recovery target and controller state."""

import copy

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def recovery_markers(pose, projection, target, state, stamp, world_frame='map'):
    if not all(msg.header.frame_id == world_frame for msg in (pose, projection, target)):
        raise ValueError('recovery marker frames must match world_frame')
    markers = MarkerArray()

    def base(marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = world_frame
        marker.ns = 'path_recovery'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=0.25).to_msg()
        marker.color.a = 1.0
        return marker

    projection_marker = base(0, Marker.SPHERE)
    projection_marker.pose = copy.deepcopy(projection.pose)
    projection_marker.scale.x = projection_marker.scale.y = projection_marker.scale.z = 0.08
    projection_marker.color.b = 1.0
    projection_marker.color.g = 0.45
    markers.markers.append(projection_marker)

    target_marker = base(1, Marker.SPHERE)
    target_marker.pose = copy.deepcopy(target.pose)
    target_marker.scale.x = target_marker.scale.y = target_marker.scale.z = 0.07
    target_marker.color.r = 1.0
    target_marker.color.g = 0.80
    markers.markers.append(target_marker)

    vector = base(2, Marker.ARROW)
    vector.scale.x = 0.018
    vector.scale.y = 0.045
    vector.scale.z = 0.06
    vector.points = [
        Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.05),
        Point(x=projection.pose.position.x, y=projection.pose.position.y, z=0.05),
    ]
    if state == 'RECOVERY':
        vector.color.r = 1.0
        vector.color.g = 0.10
    else:
        vector.color.g = 0.90
        vector.color.b = 0.25
    markers.markers.append(vector)

    label = base(3, Marker.TEXT_VIEW_FACING)
    label.pose = copy.deepcopy(pose.pose)
    label.pose.position.z = 0.30
    label.scale.z = 0.13
    label.text = state
    label.color.r = 1.0 if state in ('RECOVERY', 'HOLD', 'VICON_LOST') else 0.2
    label.color.g = 0.25 if state in ('HOLD', 'VICON_LOST') else 1.0
    label.color.b = 0.15
    markers.markers.append(label)
    return markers


class RecoveryVisualizer(Node):
    def __init__(self):
        super().__init__('recovery_visualizer')
        self.declare_parameter('world_frame', 'map')
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.pose = self.projection = self.target = None
        self.state = 'WAITING'
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            MarkerArray, '/car/recovery_markers', latched)
        self.create_subscription(PoseStamped, '/car/pose', self.on_pose, 10)
        self.create_subscription(
            PoseStamped, '/car/path_projection', self.on_projection, 10)
        self.create_subscription(
            PoseStamped, '/car/recovery_target', self.on_target, 10)
        self.create_subscription(String, '/car/recovery_state', self.on_state, 10)
        self.create_timer(0.05, self.tick)

    def on_pose(self, msg):
        self.pose = msg

    def on_projection(self, msg):
        self.projection = msg

    def on_target(self, msg):
        self.target = msg

    def on_state(self, msg):
        self.state = msg.data

    def tick(self):
        if None in (self.pose, self.projection, self.target):
            return
        try:
            markers = recovery_markers(
                self.pose, self.projection, self.target, self.state,
                self.get_clock().now().to_msg(), self.world_frame)
        except ValueError as error:
            self.get_logger().warn(str(error), throttle_duration_sec=1.0)
            return
        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = RecoveryVisualizer()
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
