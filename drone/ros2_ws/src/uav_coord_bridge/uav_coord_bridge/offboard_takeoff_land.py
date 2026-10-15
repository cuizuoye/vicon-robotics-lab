#!/usr/bin/env python3
"""ROS 2 -> PX4 起飞 / 悬停 / 降落 演示（offboard 位置控制）。

流程:
    1. 持续发布 OffboardControlMode(position) + TrajectorySetpoint
    2. 等 pre_flight_checks_pass 通过后解锁（失败时每秒重试并打印原因）
    3. 切 OFFBOARD 模式
    4. 按位置设定点爬升到 takeoff_height，悬停 hold_seconds
    5. 发 VEHICLE_CMD_NAV_LAND 让 PX4 自动降落，等自动上锁后退出

坐标系: TrajectorySetpoint 用 PX4 的 NED
    x = 北, y = 东, z = -高度（z 取负值表示"向上飞"）

前置条件:
    * PX4 SITL 已运行、Agent 已运行（scripts/41_run_ros2_bridge.sh）
    * pre_flight_checks_pass 必须为 true，否则解锁一定被拒：
      开 QGC（端口 18570）或 python3 scripts/91_fake_gcs_heartbeat.py --port 18570

用法:
    ros2 run uav_coord_bridge offboard_takeoff_land
    ros2 run uav_coord_bridge offboard_takeoff_land --ros-args \
        -p takeoff_height:=2.0 -p hold_seconds:=10.0
"""

import sys
import time

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


VEHICLE_CMD_DO_SET_MODE = 176            # param1=1(自定义), param2=6(OFFBOARD)
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400   # param1=1 解锁, 0 上锁
VEHICLE_CMD_NAV_LAND = 21

CUSTOM_MODE_OFFBOARD = 6


def px4_qos(depth: int = 5) -> QoSProfile:
    """与 PX4 uXRCE-DDS 客户端一致的 QoS（best effort + volatile）。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class OffboardTakeoffLand(Node):

    def __init__(self):
        super().__init__('offboard_takeoff_land')

        self.declare_parameter('takeoff_height', 3.0)   # [m] 向上为正
        self.declare_parameter('hold_seconds', 6.0)     # 悬停时长
        self.declare_parameter('climb_timeout', 40.0)   # 爬升超时 [s]
        self.declare_parameter('land_timeout', 60.0)    # 降落+上锁超时 [s]
        self.declare_parameter('yaw', 0.0)              # [rad] 目标偏航(NED)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.height = float(self.get_parameter('takeoff_height').value)
        self.hold_seconds = float(self.get_parameter('hold_seconds').value)
        self.climb_timeout = float(self.get_parameter('climb_timeout').value)
        self.land_timeout = float(self.get_parameter('land_timeout').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos(1))
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos(1))
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos(1))

        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, px4_qos(5))
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_local_position, px4_qos(5))

        self.status = None
        self.position = None
        self.last_command_ns = 0
        self.last_log_ns = 0
        self.target = [0.0, 0.0, -self.height]
        self.land_sent_at = 0.0
        self.land_timeout_reported = False
        self.state = 'WAIT_PREFLIGHT'
        self.state_since = time.time()

        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f'目标: 起飞到 {self.height:.1f} m，悬停 {self.hold_seconds:.1f} s，然后自动降落')

    # ---------------- 订阅回调 ----------------
    def on_status(self, msg: VehicleStatus):
        self.status = msg

    def on_local_position(self, msg: VehicleLocalPosition):
        self.position = msg

    # ---------------- 发送工具 ----------------
    def publish_offboard_stream(self):
        """offboard 模式必须持续收到 setpoint，否则 PX4 会退出 offboard。"""
        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = 0
        self.offboard_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = [float(self.target[0]), float(self.target[1]), float(self.target[2])]
        sp.yaw = float(self.yaw)
        sp.timestamp = 0
        self.setpoint_pub.publish(sp)

    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = 0
        self.command_pub.publish(cmd)

    def seconds_in_state(self) -> float:
        return time.time() - self.state_since

    def set_state(self, state: str):
        self.state = state
        self.state_since = time.time()

    def every(self, period_s: float) -> bool:
        """限频辅助: 距上次触发超过 period_s 返回 True 并更新时间戳。"""
        now = self.get_clock().now().nanoseconds
        if (now - self.last_log_ns) > period_s * 1e9:
            self.last_log_ns = now
            return True
        return False

    # ---------------- 主状态机 ----------------
    def tick(self):
        if self.status is None:
            # 启动后给 6 秒宽限期：Agent 刚连上时第一条状态可能还没到
            if self.seconds_in_state() > 6.0 and self.every(5.0):
                self.get_logger().error(
                    '收不到 /fmu/out/vehicle_status_v1 —— Agent 没启动，或 PX4 没连上')
            return

        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        nav = self.status.nav_state

        if self.state == 'WAIT_PREFLIGHT':
            self.publish_offboard_stream()
            if armed:
                self.set_state('SET_OFFBOARD')
                return
            if (self.get_clock().now().nanoseconds - self.last_command_ns) > 1e9:
                self.last_command_ns = self.get_clock().now().nanoseconds
                if not self.status.pre_flight_checks_pass:
                    hint = '（没有地面站/遥控器连接：开 QGC 或跑 scripts/91_fake_gcs_heartbeat.py）' \
                        if self.status.gcs_connection_lost else ''
                    self.get_logger().warn(f'自检未通过，暂不能解锁 {hint}')
                else:
                    self.get_logger().info('自检通过，发送解锁命令')
                self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        elif self.state == 'SET_OFFBOARD':
            self.publish_offboard_stream()
            if nav == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info('已进入 OFFBOARD 模式，开始爬升')
                self.set_state('CLIMB')
                return
            if (self.get_clock().now().nanoseconds - self.last_command_ns) > 1e9:
                self.last_command_ns = self.get_clock().now().nanoseconds
                self.send_command(VEHICLE_CMD_DO_SET_MODE, 1.0, CUSTOM_MODE_OFFBOARD)

        elif self.state == 'CLIMB':
            if self.position is not None:
                # 目标点保持在本机正上方，避免水平漂移
                self.target = [self.position.x, self.position.y, -self.height]
            self.publish_offboard_stream()
            if self.position is not None:
                altitude = -self.position.z
                if abs(altitude - self.height) < 0.3:
                    self.get_logger().info(f'到达 {altitude:.2f} m，开始悬停')
                    self.set_state('HOLD')
                    return
            if self.seconds_in_state() > self.climb_timeout:
                self.get_logger().error('爬升超时，退出')
                self.shutdown(1)
            elif self.every(2.0) and self.position is not None:
                self.get_logger().info(f'爬升中… 当前高度 {-self.position.z:.2f} m')

        elif self.state == 'HOLD':
            self.publish_offboard_stream()
            if self.position is not None and self.every(2.0):
                remain = max(self.hold_seconds - self.seconds_in_state(), 0.0)
                self.get_logger().info(f'悬停中… 高度 {-self.position.z:.2f} m，还剩 {remain:.1f} s')
            if self.seconds_in_state() > self.hold_seconds:
                self.get_logger().info('悬停结束，发送降落命令 (AUTO_LAND)')
                self.send_command(VEHICLE_CMD_NAV_LAND)
                self.land_sent_at = time.time()
                self.set_state('LAND')

        elif self.state == 'LAND':
            # AUTO_LAND 由 PX4 自己控制，不需要再发 setpoint
            if self.position is not None and (time.time() - self.land_sent_at) > 2.0 \
                    and self.every(2.0):
                self.get_logger().info(f'降落中… 高度 {-self.position.z:.2f} m')
            if not armed:
                self.get_logger().info('已降落到地面并自动上锁，流程结束')
                self.shutdown(0)
            elif (time.time() - self.land_sent_at) > self.land_timeout:
                if not self.land_timeout_reported:
                    self.land_timeout_reported = True
                    self.get_logger().error(
                        '降落超时：再次发送 AUTO_LAND 并继续监视；'
                        '为避免空中停桨，本节点不会自动强制上锁')
                    self.send_command(VEHICLE_CMD_NAV_LAND)

    def shutdown(self, code: int):
        raise SystemExit(code)


def main():
    rclpy.init()
    node = OffboardTakeoffLand()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code or 0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
