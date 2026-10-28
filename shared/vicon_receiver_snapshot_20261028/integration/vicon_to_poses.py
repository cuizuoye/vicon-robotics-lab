#!/usr/bin/env python3
"""Vicon (ros2-vicon-receiver) -> Crazyswarm2 的 /poses 桥接节点。

背景：
    * 你现有的 `ros2-vicon-receiver` 用 Vicon DataStream SDK 直接连 Tracker，
      发布 `/vicon/<subject>/<segment>`（geometry_msgs/PoseStamped）+ TF；
    * 而 Crazyswarm2 的 server 只认 `motion_capture_tracking_interfaces/msg/NamedPoseArray`
      这一个话题 `/poses`（按位姿里的 name 匹配到机器人名）；
    * apt 版 `motion_capture_tracking` 没有编译 Vicon 客户端（只带了 OptiTrack 的 libNatNet），
      所以用这个节点把两者接起来即可，不必自己编译 libmotioncapture。

工作方式：
    1. 按 `topic_prefix` 自动发现所有 `geometry_msgs/msg/PoseStamped` 话题；
    2. 位姿名字默认取话题的最后一段（即 Vicon 刚体名，例如 /vicon/cf01/cf01 -> cf01）；
    3. 以 `publish_rate_hz` 的频率发布 `/poses`；超过 `timeout_s` 没更新的刚体会被剔除。

用法：
    # 终端 1: Vicon 接收端（你们已有的包）
    ros2 launch vicon_receiver client.launch.py hostname:=192.168.100.184 \
        topic_namespace:=vicon world_frame:=map vicon_frame:=vicon \
        map_xyz:='[0.0,0.0,0.0]' map_rpy:='[0.0,0.0,0.0]'

    # 终端 2: 本节点
    ros2 run uav_coord_bridge vicon_to_poses

    # 终端 3: Crazyswarm2（注意 mocap:=False，因为 /poses 由本节点提供）
    ros2 launch crazyflie launch.py backend:=cflib mocap:=False \
        crazyflies_yaml_file:=$(pwd)/crazyflie_config/crazyflies.yaml
"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def mocap_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def poses_qos(deadline_hz: float = 100.0, depth: int = 10) -> QoSProfile:
    """/poses 发布端 QoS —— 必须声明 DEADLINE，否则 Crazyswarm2 的订阅不兼容。

    Crazyswarm2 的 server 按 motion_capture.yaml 里的 `deadline: 100.0`(Hz) 订阅 /poses，
    DDS 要求发布端承诺的 deadline 不慢于订阅端要求（offered <= requested），
    否则会报 "offering incompatible QoS ... Last incompatible policy: DEADLINE" 并丢包。
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        deadline=Duration(nanoseconds=int(1e9 / deadline_hz)),
    )


class ViconToPoses(Node):

    def __init__(self):
        super().__init__('vicon_to_poses')

        self.declare_parameter('topic_prefix', '/vicon/')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('frame_id', 'mocap')
        self.declare_parameter('timeout_s', 0.5)
        self.declare_parameter('name_map', [''])   # 形如 "话题末段=机器人名"，一般留空

        self.prefix = self.get_parameter('topic_prefix').value
        self.rate = float(self.get_parameter('publish_rate_hz').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.timeout = float(self.get_parameter('timeout_s').value)
        self.name_map = [s for s in self.get_parameter('name_map').value if '=' in s]

        # 注意：deadline 必须与 motion_capture.yaml 里的一致（默认 100 Hz）
        self.pub = self.create_publisher(NamedPoseArray, '/poses', poses_qos(100.0))
        self.subs = {}          # topic -> subscription
        self.latest = {}        # name -> (stamp_ros_ns, Pose)

        self.create_timer(1.0, self.discover)
        if self.rate > 0:
            self.create_timer(1.0 / self.rate, self.publish)
        self.discover()
        self.get_logger().info(f'桥接 {self.prefix}* (PoseStamped) -> /poses，名字取话题最后一段')

    # ---------- 发现并订阅 ----------
    def discover(self):
        try:
            topics = self.get_topic_names_and_types()
        except Exception:
            return
        for topic, types in topics:
            if not topic.startswith(self.prefix) or topic in self.subs:
                continue
            if 'geometry_msgs/msg/PoseStamped' not in types:
                continue
            name = topic.rstrip('/').split('/')[-1]
            for mapping in self.name_map:
                src, dst = mapping.split('=', 1)
                if name == src:
                    name = dst
            self.subs[topic] = self.create_subscription(
                PoseStamped, topic, lambda msg, n=name: self.on_pose(msg, n), mocap_qos())
            self.get_logger().info(f'订阅 {topic}  -> 名字 "{name}"')

    def on_pose(self, msg: PoseStamped, name: str):
        self.latest[name] = (self.get_clock().now().nanoseconds, msg.pose)

    # ---------- 发布 ----------
    def publish(self):
        if not self.latest:
            return
        now_ns = self.get_clock().now().nanoseconds
        out = NamedPoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        for name, (stamp_ns, pose) in list(self.latest.items()):
            if (now_ns - stamp_ns) * 1e-9 > self.timeout:
                continue
            np = NamedPose()
            np.name = name
            np.pose = pose
            out.poses.append(np)
        if out.poses:
            self.pub.publish(out)


def main():
    rclpy.init()
    node = ViconToPoses()
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
