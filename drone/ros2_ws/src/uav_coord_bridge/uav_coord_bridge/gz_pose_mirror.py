#!/usr/bin/env python3
"""把真机（或任意 ROS 位姿源）的位姿"镜像"到本机 Gazebo 里的模型上。

用途: 真机在室内飞的时候，本机 Gazebo 里那个模型跟着动 —— 相当于一个
      可视化用的"数字孪生"，比只看 RViz 直观。

原理:
    订阅一个 ROS 位姿话题（默认 /px4/odometry_enu，ENU），按固定频率调用
    Gazebo 的 /world/<world>/set_pose 服务移动指定模型。服务需要通过
    ros_gz_bridge 桥出来：

        ros2 run ros_gz_bridge parameter_bridge \
          /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean
    （服务桥必须写满 4 段：ROS 服务名@ROS 类型@gz 请求类型@gz 响应类型，
      少写响应类型时 bridge 会直接报错退出）

坐标系:
    /px4/odometry_enu、/vicon/... 都是 ENU(东-北-上)，与 Gazebo 世界系一致，
    所以可以直接用。默认 relative_to_first=true：把收到第一条位姿的位置当作
    Gazebo 世界原点，视觉上无人机从原点起飞，不受真机实际绝对坐标影响。

用法:
    # 终端 1：起一个空的 Gazebo 世界
    gz sim -r ~/.simulation-gazebo/worlds/default.sdf
    # 终端 2：把 x500 模型放进去（不放也行，用 --entity 指定已有模型）
    ros2 run ros_gz_sim create -world default -name x500_0 \
        -file ~/.simulation-gazebo/models/x500/model.sdf
    # 终端 3：桥出 set_pose 服务
    ros2 run ros_gz_bridge parameter_bridge \
        /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean
    # 终端 4：镜像节点
    ros2 run uav_coord_bridge gz_pose_mirror
    ros2 run uav_coord_bridge gz_pose_mirror --ros-args \
        -p pose_topic:=/vicon/drone/drone -p pose_type:=pose_stamped
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.srv import SetEntityPose


def px4_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class GzPoseMirror(Node):

    def __init__(self):
        super().__init__('gz_pose_mirror')

        self.declare_parameter('pose_topic', '/px4/odometry_enu')
        self.declare_parameter('pose_type', 'odometry')      # odometry | pose_stamped
        self.declare_parameter('service_name', '/world/default/set_pose')
        self.declare_parameter('entity_name', 'x500_0')
        self.declare_parameter('entity_type', 2)             # 2 = MODEL
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('relative_to_first', True)
        self.declare_parameter('offset_xyz', [0.0, 0.0, 0.0])

        self.pose_topic = self.get_parameter('pose_topic').value
        self.pose_type = self.get_parameter('pose_type').value.lower()
        self.service_name = self.get_parameter('service_name').value
        self.entity_name = self.get_parameter('entity_name').value
        self.entity_type = int(self.get_parameter('entity_type').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.relative = bool(self.get_parameter('relative_to_first').value)
        self.offset = list(self.get_parameter('offset_xyz').value)

        self.latest = None          # 最近一次收到的位姿
        self.origin = None          # 第一帧位置（相对模式用）
        self.pose_count = 0
        self.call_pending = False
        self.fail_count = 0

        self.client = self.create_client(SetEntityPose, self.service_name)
        if self.pose_type == 'pose_stamped':
            self.create_subscription(PoseStamped, self.pose_topic, self.on_pose_stamped, px4_qos())
        elif self.pose_type == 'odometry':
            self.create_subscription(Odometry, self.pose_topic, self.on_odometry, px4_qos())
        else:
            raise ValueError("pose_type 只能是 odometry 或 pose_stamped")

        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f'镜像 {self.pose_topic} ({self.pose_type}) -> Gazebo 模型 {self.entity_name} '
            f'via {self.service_name}')
        self.get_logger().info('等待 Gazebo 的 set_pose 服务…（没起 Gazebo/bridge 时会一直等）')

    # ---------- 位姿输入 ----------
    def on_odometry(self, msg: Odometry):
        self.store(msg.pose.pose)

    def on_pose_stamped(self, msg: PoseStamped):
        self.store(msg.pose)

    def store(self, pose: Pose):
        self.pose_count += 1
        self.latest = pose
        if self.origin is None:
            self.origin = (pose.position.x, pose.position.y, pose.position.z)
            self.get_logger().info(
                f'收到第一帧位姿 ({self.origin[0]:.2f}, {self.origin[1]:.2f}, '
                f'{self.origin[2]:.2f})，作为镜像原点')

    # ---------- 输出 ----------
    def tick(self):
        if self.latest is None or self.call_pending:
            return
        if not self.client.service_is_ready():
            if self.fail_count % 20 == 0:
                self.get_logger().warn(f'服务 {self.service_name} 还不可用', throttle_duration_sec=5.0)
            self.fail_count += 1
            return

        p = self.latest.position
        if self.relative and self.origin is not None:
            x = p.x - self.origin[0]
            y = p.y - self.origin[1]
            z = p.z - self.origin[2]
        else:
            x, y, z = p.x, p.y, p.z

        req = SetEntityPose.Request()
        req.entity.name = self.entity_name
        req.entity.type = self.entity_type
        req.pose.position.x = float(x + self.offset[0])
        req.pose.position.y = float(y + self.offset[1])
        req.pose.position.z = float(z + self.offset[2])
        req.pose.orientation = self.latest.orientation

        self.call_pending = True
        future = self.client.call_async(req)
        future.add_done_callback(self.on_result)

    def on_result(self, future):
        self.call_pending = False
        try:
            ok = future.result().success
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f'set_pose 调用失败: {exc}', throttle_duration_sec=5.0)
            return
        if not ok:
            self.get_logger().warn('Gazebo 返回 success=false（实体名/世界名对不对？）',
                                   throttle_duration_sec=5.0)
        elif self.pose_count % 100 == 1:
            self.get_logger().info(f'已镜像 {self.pose_count} 帧位姿')


def main():
    rclpy.init()
    node = GzPoseMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
