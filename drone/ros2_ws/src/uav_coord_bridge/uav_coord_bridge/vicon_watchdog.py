#!/usr/bin/env python3
"""动捕数据看门狗：位姿超时就告警、Hold 或降落。

为什么需要:
    当 Vicon 位姿是通过"Vicon PC -> 本机 PC -> WiFi -> 机载电脑 -> 飞控"这种
    中转链路送到飞控时，WiFi 掉链、丢包或 Vicon 丢点都会让 EKF2 失去外部位姿。
    PX4 自己会在 EV 数据超时后降级（`COM_OF_LOSS_T`/EKF2 超时），但那是"事后"，
    本节点让超时处理更明确、更快，并且能立刻记录原因。

建议运行位置: **机载电脑**（订阅 Vicon 位姿话题，或订阅我们发给飞控的
    `/fmu/in/vehicle_visual_odometry`），因为它离飞控最近、判断最及时。

动作选项（`on_timeout`）:
    warn     只打印告警（默认，最安全，先观察）
    hold     发 DO_SET_MODE 切入 PX4 AUTO_LOITER（Hold/Loiter 悬停）
    land     发 VEHICLE_CMD_NAV_LAND（自动降落）

用法:
    ros2 run uav_coord_bridge vicon_watchdog
    ros2 run uav_coord_bridge vicon_watchdog --ros-args \
        -p pose_topic:=/vicon/drone/drone -p timeout_s:=0.5 -p on_timeout:=land
"""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_DO_SET_MODE = 176

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3


def px4_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class ViconWatchdog(Node):

    def __init__(self):
        super().__init__('vicon_watchdog')

        self.declare_parameter('pose_topic', '/vicon/drone/drone')
        self.declare_parameter('timeout_s', 0.5)
        self.declare_parameter('on_timeout', 'warn')
        self.declare_parameter('check_rate_hz', 20.0)
        self.declare_parameter('recover_after_s', 0.0)  # >0 时数据恢复该时长后解除告警

        self.pose_topic = self.get_parameter('pose_topic').value
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.on_timeout = self.get_parameter('on_timeout').value
        self.check_rate = float(self.get_parameter('check_rate_hz').value)
        self.recover_after_s = float(self.get_parameter('recover_after_s').value)

        if self.on_timeout not in ('warn', 'hold', 'land'):
            raise ValueError("on_timeout 只能是 warn / hold / land")

        self.last_pose_time = None
        self.last_pose_log = 0.0
        self.action_sent = False

        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos(1))
        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, px4_qos(10))
        self.create_timer(1.0 / self.check_rate, self.tick)

        self.get_logger().info(
            f'看门狗: 监测 {self.pose_topic}，超过 {self.timeout_s:.2f}s 无数据 -> {self.on_timeout}')

    def on_pose(self, _msg: PoseStamped):
        now = time.time()
        if self.action_sent and self.recover_after_s > 0.0:
            self.get_logger().info('动捕数据恢复（注意：飞行模式不会自动恢复）')
            self.action_sent = False
        self.last_pose_time = now

    def send_command(
            self, command: int, param1: float = 0.0,
            param2: float = 0.0, param3: float = 0.0):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.param3 = float(param3)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = 0
        self.command_pub.publish(cmd)

    def tick(self):
        now = time.time()
        if self.last_pose_time is None:
            if now - self.last_pose_log > 5.0:
                self.last_pose_log = now
                self.get_logger().warn(f'还没收到 {self.pose_topic} 的数据')
            return

        elapsed = now - self.last_pose_time
        if elapsed <= self.timeout_s:
            return

        if self.action_sent:
            return

        self.action_sent = True
        self.get_logger().error(
            f'动捕数据已中断 {elapsed:.2f}s（阈值 {self.timeout_s:.2f}s），执行动作: {self.on_timeout}')

        if self.on_timeout == 'hold':
            self.send_command(
                VEHICLE_CMD_DO_SET_MODE,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                PX4_CUSTOM_MAIN_MODE_AUTO,
                PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
            )
        elif self.on_timeout == 'land':
            self.send_command(VEHICLE_CMD_NAV_LAND)
        else:
            self.get_logger().warn('on_timeout=warn，仅告警不干预（建议先这样试跑）')


def main():
    rclpy.init()
    node = ViconWatchdog()
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
