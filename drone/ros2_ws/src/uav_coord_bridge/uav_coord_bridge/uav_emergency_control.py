#!/usr/bin/env python3
"""通过 ROS 2 服务向 PX4 发送 Hold、Land 或双确认的飞行终止命令。

飞行终止会让 PX4 停止姿态控制并把输出切到 failsafe 值，不能替代独立的
硬件急停链路。该功能默认关闭，且只在状态新鲜、飞行器已解锁时允许确认。
"""

import json
import time

import rclpy
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .emergency_logic import TerminationGate, ack_result_name


VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_DO_FLIGHTTERMINATION = 185
VEHICLE_CMD_DO_SET_MODE = 176

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3


def px4_qos(depth: int = 5) -> QoSProfile:
    """与 PX4 uXRCE-DDS 客户端匹配的 QoS。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class UavEmergencyControl(Node):
    """提供受保护的 PX4 紧急控制服务。"""

    def __init__(self):
        """创建 PX4 通信接口和 ROS 2 服务。"""
        super().__init__('uav_emergency_control')

        self.declare_parameter('allow_flight_termination', False)
        self.declare_parameter('confirmation_window_s', 5.0)
        self.declare_parameter('status_timeout_s', 1.5)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('source_system', 1)
        self.declare_parameter('source_component', 1)

        self.status_timeout_s = float(
            self.get_parameter('status_timeout_s').value)
        self.target_system = int(self.get_parameter('target_system').value)
        self.target_component = int(
            self.get_parameter('target_component').value)
        self.source_system = int(self.get_parameter('source_system').value)
        self.source_component = int(
            self.get_parameter('source_component').value)
        self.termination_gate = TerminationGate(
            enabled=bool(self.get_parameter('allow_flight_termination').value),
            window_s=float(self.get_parameter('confirmation_window_s').value),
        )

        self.vehicle_status = None
        self.vehicle_status_at = None
        self.pending_commands = {}

        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos(1))
        self.status_pub = self.create_publisher(
            String, '/uav/emergency/status', 10)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.on_vehicle_status, px4_qos(5))
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self.on_command_ack, px4_qos(10))

        self.create_service(Trigger, '/uav/emergency/hold', self.on_hold)
        self.create_service(Trigger, '/uav/emergency/land', self.on_land)
        self.create_service(
            Trigger, '/uav/emergency/prepare_termination',
            self.on_prepare_termination)
        self.create_service(
            Trigger, '/uav/emergency/confirm_termination',
            self.on_confirm_termination)
        self.create_service(
            Trigger, '/uav/emergency/cancel_termination',
            self.on_cancel_termination)

        termination_state = (
            '已启用（仍需双确认）'
            if self.termination_gate.enabled else '已禁用')
        self.get_logger().warn(
            f'急停控制节点已启动；飞行终止: {termination_state}。'
            'Hold/Land 命令不会关闭电机。')
        self.publish_event(
            'node_started', termination_enabled=self.termination_gate.enabled)

    def publish_event(self, event: str, **fields):
        """在固定状态话题输出 JSON，便于 rosbag 和上层监控记录。"""
        payload = {
            'event': event,
            'monotonic_s': round(time.monotonic(), 3),
            **fields,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(msg)

    def on_vehicle_status(self, msg: VehicleStatus):
        """保存最新飞行器状态及本机接收时刻。"""
        self.vehicle_status = msg
        self.vehicle_status_at = time.monotonic()

    def vehicle_is_armed_and_fresh(self):
        """只允许在状态新鲜且已解锁时进入飞行终止确认。"""
        if self.vehicle_status is None or self.vehicle_status_at is None:
            return False, '尚未收到 PX4 VehicleStatus'
        age = time.monotonic() - self.vehicle_status_at
        if age > self.status_timeout_s:
            return False, f'PX4 VehicleStatus 已过期（{age:.2f}s）'
        armed = self.vehicle_status.arming_state
        if armed != VehicleStatus.ARMING_STATE_ARMED:
            return False, '飞行器当前未解锁，无需执行飞行终止'
        return True, 'PX4 状态新鲜且飞行器已解锁'

    def send_command(
            self, command: int, param1: float = 0.0,
            param2: float = 0.0, param3: float = 0.0):
        """发布一条 PX4 VehicleCommand 并登记待处理 ACK。"""
        cmd = VehicleCommand()
        cmd.command = int(command)
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.param3 = float(param3)
        cmd.target_system = self.target_system
        cmd.target_component = self.target_component
        cmd.source_system = self.source_system
        cmd.source_component = self.source_component
        cmd.from_external = True
        cmd.timestamp = 0
        self.command_pub.publish(cmd)
        self.pending_commands[int(command)] = time.monotonic()
        self.publish_event(
            'command_sent', command=int(command), param1=float(param1),
            param2=float(param2), param3=float(param3))

    def on_command_ack(self, msg: VehicleCommandAck):
        """记录本节点所发命令对应的 PX4 回执。"""
        command = int(msg.command)
        if command not in self.pending_commands:
            return
        elapsed = time.monotonic() - self.pending_commands.pop(command)
        result = ack_result_name(msg.result)
        self.publish_event(
            'command_ack', command=command, result=result,
            result_code=int(msg.result), elapsed_s=round(elapsed, 3))
        message = (
            f'PX4 ACK: command={command}, result={result}, '
            f'elapsed={elapsed:.3f}s')
        # rclpy 会缓存同一调用点的日志严重级别，因此不能用动态绑定的 log 方法。
        if msg.result in (0, 5):
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    def on_hold(self, _request, response):
        """请求 PX4 切入 AUTO_LOITER。"""
        # PX4 多旋翼不接受 NAV_LOITER_UNLIM 作为即时外部命令；显式切 AUTO_LOITER。
        self.send_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
        )
        response.success = True
        response.message = (
            'Hold 命令已发送；请以 /uav/emergency/status 的 PX4 ACK 为准')
        return response

    def on_land(self, _request, response):
        """请求 PX4 执行 AUTO_LAND。"""
        self.send_command(VEHICLE_CMD_NAV_LAND)
        response.success = True
        response.message = (
            'Land 命令已发送；请以 /uav/emergency/status 的 PX4 ACK 为准')
        return response

    def on_prepare_termination(self, _request, response):
        """创建一个短时有效的飞行终止确认窗口。"""
        if not self.termination_gate.enabled:
            response.success = False
            response.message = '飞行终止功能未启用'
            self.publish_event(
                'termination_prepare_rejected', reason=response.message)
            return response

        safe, reason = self.vehicle_is_armed_and_fresh()
        if not safe:
            response.success = False
            response.message = reason
            self.publish_event('termination_prepare_rejected', reason=reason)
            return response
        response.success, response.message = self.termination_gate.prepare(
            time.monotonic())
        event = (
            'termination_prepared'
            if response.success else 'termination_prepare_rejected')
        self.publish_event(event, reason=response.message)
        if response.success:
            self.get_logger().error(
                f'{response.message}；确认后将执行不可恢复的飞行终止，'
                'PX4 需重启才能清除')
        return response

    def on_confirm_termination(self, _request, response):
        """消费确认窗口并在复核状态后发送飞行终止。"""
        confirmed, reason = self.termination_gate.confirm(time.monotonic())
        if not confirmed:
            response.success = False
            response.message = reason
            self.publish_event('termination_confirm_rejected', reason=reason)
            return response

        safe, status_reason = self.vehicle_is_armed_and_fresh()
        if not safe:
            response.success = False
            response.message = status_reason
            self.publish_event(
                'termination_confirm_rejected', reason=status_reason)
            return response

        self.get_logger().fatal(
            '已确认飞行终止：发送 VEHICLE_CMD_DO_FLIGHTTERMINATION')
        self.send_command(VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0)
        response.success = True
        response.message = '飞行终止命令已发送；请立即检查 PX4 ACK 和现场状态'
        return response

    def on_cancel_termination(self, _request, response):
        """取消尚未消费的飞行终止确认窗口。"""
        response.success, response.message = self.termination_gate.cancel()
        self.publish_event('termination_cancelled', reason=response.message)
        return response


def main():
    """运行急停控制节点。"""
    rclpy.init()
    node = UavEmergencyControl()
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
