#!/usr/bin/env python3
"""Safe ROS 2 Twist to MC-Link UDP bridge for the ESP32-S3 transport board."""

import math
import json
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import (
    Bool,
    Float32,
    Float32MultiArray,
    Int32MultiArray,
    String,
    UInt16,
    UInt32,
)

from .mc_link_session import McLinkSession, SessionMismatchError
from .readonly_link_status import decode_readonly_status
from .motion_link_status import decode_motion_status
from .udp_socket_qos import configure_low_latency_qos
from .transport_protocol import (
    MessageType,
    ProtocolError,
    unpack_wheel_telemetry,
)


class CmdVelUdpBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_udp_bridge')
        self.declare_parameter('cmd_topic', '/car/cmd_vel')
        self.declare_parameter('enable_topic', '/car/safety_enabled')
        self.declare_parameter('estop_topic', '/car/estop')
        self.declare_parameter('control_enable_topic', '/car/control_enable')
        self.declare_parameter('clear_faults_topic', '/car/clear_faults')
        self.declare_parameter('esp_ip', '')
        self.declare_parameter('network_interface', '')
        self.declare_parameter('udp_low_latency_qos', False)
        self.declare_parameter('readonly_mode', False)
        self.declare_parameter('bounded_motion_mode', False)
        self.declare_parameter('command_port', 46000)
        self.declare_parameter('telemetry_port', 46001)
        self.declare_parameter('send_rate_hz', 50.0)
        self.declare_parameter('command_timeout_s', 0.20)
        self.declare_parameter('wire_timeout_ms', 150)
        self.declare_parameter('hello_retry_s', 0.30)
        self.declare_parameter('link_receive_timeout_s', 0.50)
        self.declare_parameter('command_progress_timeout_s', 1.00)
        self.declare_parameter('max_vx_mps', 0.30)
        self.declare_parameter('max_vy_mps', 0.30)
        self.declare_parameter('max_wz_radps', 0.80)

        self.esp_ip = str(self.get_parameter('esp_ip').value).strip()
        self.network_interface = str(
            self.get_parameter('network_interface').value).strip()
        self.readonly_mode = bool(self.get_parameter('readonly_mode').value)
        self.bounded_motion_mode = bool(self.get_parameter('bounded_motion_mode').value)
        self.udp_low_latency_qos = bool(self.get_parameter('udp_low_latency_qos').value)
        self.socket_qos = dict(enabled=False)
        self.command_port = int(self.get_parameter('command_port').value)
        self.telemetry_port = int(self.get_parameter('telemetry_port').value)
        self.send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.command_timeout_s = float(
            self.get_parameter('command_timeout_s').value)
        self.wire_timeout_ms = int(self.get_parameter('wire_timeout_ms').value)
        self.hello_retry_s = float(self.get_parameter('hello_retry_s').value)
        self.link_receive_timeout_s = float(
            self.get_parameter('link_receive_timeout_s').value)
        self.command_progress_timeout_s = float(
            self.get_parameter('command_progress_timeout_s').value)
        self.max_vx_mps = float(self.get_parameter('max_vx_mps').value)
        self.max_vy_mps = float(self.get_parameter('max_vy_mps').value)
        self.max_wz_radps = float(self.get_parameter('max_wz_radps').value)

        if self.send_rate_hz <= 0.0:
            raise ValueError('send_rate_hz must be positive')
        if not 20 <= self.wire_timeout_ms <= 2000:
            raise ValueError('wire_timeout_ms must be between 20 and 2000')
        if self.bounded_motion_mode and (self.readonly_mode or self.wire_timeout_ms != 150
                or self.send_rate_hz != 50.0 or not 0 < self.max_vx_mps <= .05
                or not 0 < self.max_vy_mps <= .05 or not 0 < self.max_wz_radps <= .20):
            raise ValueError('bounded motion requires RW, 50Hz, 150ms and <=0.05m/s,0.2rad/s')

        cmd_topic = str(self.get_parameter('cmd_topic').value)
        enable_topic = str(self.get_parameter('enable_topic').value)
        estop_topic = str(self.get_parameter('estop_topic').value)
        control_enable_topic = str(
            self.get_parameter('control_enable_topic').value)
        clear_faults_topic = str(self.get_parameter('clear_faults_topic').value)
        self.latest_cmd = Twist()
        self.last_cmd_ns = None
        self.safety_enabled = False
        self.session = McLinkSession()
        self.last_hello_monotonic = -math.inf
        self.last_receive_monotonic = None
        self.last_command_progress_monotonic = None
        self.last_telemetry_command_sequence = None
        self.last_reported_link = None
        self.pending_control_action = None
        self.pending_control_repeats = 0
        self.local_stop_latched = False
        self.control_enable_reset_seen = False
        self.last_mcu_fault_bits = None
        self.last_wheel_telemetry_monotonic = None
        self.last_motion_status = None
        self.last_motion_status_monotonic = None
        self.enable_prime_started = None
        self.enable_prime_complete = False

        self.link_pub = self.create_publisher(Bool, '/car/mc_link_connected', 10)
        self.status_pub = self.create_publisher(String, '/car/mc_link_status', 10)
        self.wheel_rpm_pub = self.create_publisher(
            Float32MultiArray, '/car/wheel_rpm', 10)
        self.encoder_pub = self.create_publisher(
            Int32MultiArray, '/car/encoder_counts', 10)
        self.battery_pub = self.create_publisher(
            Float32, '/car/battery_voltage', 10)
        self.fault_pub = self.create_publisher(UInt32, '/car/motor_fault_bits', 10)
        self.state_flags_pub = self.create_publisher(
            UInt16, '/car/motor_state_flags', 10)
        self.last_sequence_pub = self.create_publisher(
            UInt32, '/car/last_motor_command_sequence', 10)
        self.diagnostics_pub = self.create_publisher(
            String, '/car/mc_link_diagnostics', 10)
        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_subscription(Bool, enable_topic, self.on_enable, 10)
        self.create_subscription(Bool, estop_topic, self.on_estop, 10)
        self.create_subscription(
            Bool, control_enable_topic, self.on_control_enable, 10)
        self.create_subscription(Bool, clear_faults_topic, self.on_clear_faults, 10)

        self.sock = None
        if self.esp_ip:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if self.udp_low_latency_qos:
                try:
                    self.socket_qos = configure_low_latency_qos(self.sock)
                except (OSError, RuntimeError):
                    self.sock.close()
                    self.sock = None
                    raise
            if self.network_interface:
                self.sock.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_BINDTODEVICE,
                    self.network_interface.encode() + b'\0')
            self.sock.bind(('0.0.0.0', self.telemetry_port))
            self.sock.setblocking(False)
            self.destination = (self.esp_ip, self.command_port)
            self.get_logger().info(
                f'MC-Link UDP目标 {self.esp_ip}:{self.command_port}，'
                f'本机监听 {self.telemetry_port}，接口 '
                f'{self.network_interface or "系统默认"}；启动状态仅允许零速')
            if self.udp_low_latency_qos:
                self.get_logger().info(
                    '本进程UDP优先级已读回：IP_TOS=184，SO_PRIORITY=6；'
                    '不改变系统网络，不能代替停机保护')
        else:
            self.destination = None
            self.get_logger().warn(
                'esp_ip为空：节点只做ROS侧监视，不会打开UDP或发送数据')

        self.create_timer(1.0 / self.send_rate_hz, self.tick)

    @staticmethod
    def monotonic_ms() -> int:
        return int(time.monotonic() * 1000.0) & 0xFFFFFFFF

    @staticmethod
    def clamp_finite(value: float, limit: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    def on_cmd(self, msg: Twist):
        self.latest_cmd = msg
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def on_enable(self, msg: Bool):
        self.safety_enabled = (bool(msg.data) and not self.readonly_mode
                               and not self.local_stop_latched)

    def send_immediate(self, frame: bytes, label: str) -> bool:
        if self.sock is None or not self.session.connected:
            self.get_logger().warn(f'{label}未发送：MC-Link尚未连接')
            return False
        try:
            self.sock.sendto(frame, self.destination)
        except OSError as error:
            self.get_logger().error(f'{label}发送失败: {error}')
            self.latch_local_stop('控制帧发送失败')
            self.session.connected = False
            return False
        return True

    def force_local_disabled(self):
        self.safety_enabled = False
        self.latest_cmd = Twist()
        self.last_cmd_ns = None

    def latch_local_stop(self, reason: str):
        """Re-handshake restores communication, never old motion authority."""
        if not self.local_stop_latched:
            self.control_enable_reset_seen = False
            self.get_logger().error(f'本机停机锁存: {reason}；重新握手不会恢复运动')
        self.local_stop_latched = True
        self.force_local_disabled()
        if self.pending_control_action == 'enable':
            self.pending_control_action = None
            self.pending_control_repeats = 0

    def queue_control_action(self, action: str, repeats: int = 5):
        """Prioritize a low-rate state command over the 50 Hz velocity stream.

        UDP may reorder adjacent datagrams. Repeating with new sequence numbers
        while suppressing CMD_VEL ensures a safety command cannot be made stale
        by the next periodic velocity frame.
        """
        self.pending_control_action = action
        self.pending_control_repeats = repeats
        self.enable_prime_started = None
        self.enable_prime_complete = False
        self.last_motion_status = None
        self.last_motion_status_monotonic = None
        if self.sock is None:
            self.get_logger().warn(f'{action}已在ROS侧锁定，但esp_ip为空')
            return

        # ESP32 deliberately locks a session after its UDP watchdog expires,
        # while UART telemetry may still arrive. Start a fresh session so a
        # safety command cannot be rejected by that hidden locked state.
        self.session.connected = False
        self.last_receive_monotonic = None
        self.last_command_progress_monotonic = None
        self.last_telemetry_command_sequence = None
        self.last_hello_monotonic = -math.inf
        try:
            self.send_hello_if_due()
        except OSError as error:
            self.get_logger().error(f'{action}重握手发送失败: {error}')
            self.latch_local_stop('控制请求重握手发送失败')

    def send_pending_control_action(self) -> bool:
        if self.pending_control_action is None or self.pending_control_repeats <= 0:
            return False
        if not self.session.connected:
            return True

        action = self.pending_control_action
        if action == 'enable' and (self.local_stop_latched or self.readonly_mode):
            self.pending_control_action = None
            self.pending_control_repeats = 0
            return True
        if action == 'enable' and self.bounded_motion_mode and not self.enable_prime_complete:
            now = time.monotonic()
            if self.enable_prime_started is None:
                self.enable_prime_started = now
            if now - self.enable_prime_started > 2.0:
                self.latch_local_stop('使能前零速基线或4WM1静止反馈超时')
                return True
            status = self.last_motion_status
            fresh = (self.last_motion_status_monotonic is not None and
                     now - self.last_motion_status_monotonic <= .30)
            settled = (status is not None and status['tag'] == '4WM1' and
                       status['stationary_ms'] >= 500 and not status['permitted'] and
                       not any(status['compares'] + status['raw_duty'] + status['target_cps']))
            if (now - self.enable_prime_started < .60 or not fresh or not settled
                    or self.last_mcu_fault_bits != 0):
                # New HELLO resets zero_seen. Establish a fresh disabled-zero
                # lease before CONTROL_ENABLE, without accepting old velocity.
                frame = self.session.cmd_vel(0., 0., 0., 150, False, self.monotonic_ms())
                self.send_immediate(frame, 'ZERO_BASELINE')
                return True
            self.enable_prime_complete = True
        if action == 'estop':
            frame = self.session.estop(self.monotonic_ms())
            label = 'ESTOP'
        elif action == 'clear_faults':
            frame = self.session.control_enable(False, True, self.monotonic_ms())
            label = 'CLEAR_FAULTS'
        elif action == 'enable':
            frame = self.session.control_enable(True, False, self.monotonic_ms())
            label = 'CONTROL_ENABLE'
        else:
            frame = self.session.control_enable(False, False, self.monotonic_ms())
            label = 'CONTROL_DISABLE'

        if self.send_immediate(frame, label):
            self.pending_control_repeats -= 1
            if self.pending_control_repeats == 0:
                self.pending_control_action = None
        return True

    def on_estop(self, msg: Bool):
        if not msg.data:
            return
        self.latch_local_stop('收到急停请求')
        self.queue_control_action('estop')
        self.get_logger().error('正在连续下发ESTOP；必须另行清故障且重新使能')

    def on_control_enable(self, msg: Bool):
        if self.readonly_mode and msg.data:
            self.force_local_disabled()
            self.get_logger().warn('通信只读模式拒绝使能请求')
            return
        if not msg.data:
            self.force_local_disabled()
            if self.local_stop_latched:
                self.control_enable_reset_seen = True
        elif self.local_stop_latched:
            fresh_healthy_feedback = (
                self.session.connected and self.last_mcu_fault_bits == 0
                and self.last_wheel_telemetry_monotonic is not None
                and time.monotonic() - self.last_wheel_telemetry_monotonic
                    <= self.link_receive_timeout_s)
            if not self.control_enable_reset_seen or not fresh_healthy_feedback:
                self.get_logger().warn(
                    '停机锁存拒绝使能：需新鲜无故障反馈，并先显式失能再重新使能')
                return
            self.local_stop_latched = False
            self.control_enable_reset_seen = False
            self.force_local_disabled()
        self.queue_control_action('enable' if msg.data else 'disable')
        state = '请求使能' if msg.data else '失能'
        self.get_logger().info(
            f'正在连续下发控制{state}；实际运动仍受safety_enabled和STM32故障门控')

    def on_clear_faults(self, msg: Bool):
        if self.readonly_mode:
            self.get_logger().warn('通信只读模式不下发清故障请求')
            return
        if not msg.data:
            return
        self.force_local_disabled()
        self.queue_control_action('clear_faults')
        self.get_logger().info('正在失能状态连续请求清除可恢复锁存故障')

    def cmd_is_fresh(self) -> bool:
        if self.last_cmd_ns is None:
            return False
        age_s = (self.get_clock().now().nanoseconds - self.last_cmd_ns) * 1e-9
        return age_s <= self.command_timeout_s

    def drain_receive(self):
        if self.sock is None:
            return
        while True:
            try:
                data, sender = self.sock.recvfrom(512)
            except BlockingIOError:
                return
            if sender[0] != self.esp_ip:
                continue
            try:
                frame = self.session.accept(data)
            except SessionMismatchError:
                # UDP may deliver telemetry queued just before a re-handshake.
                continue
            except ProtocolError as error:
                self.get_logger().warn(f'丢弃无效MC-Link回包: {error}')
                continue
            self.last_receive_monotonic = time.monotonic()
            if frame.message_type == MessageType.HELLO_ACK:
                self.get_logger().info(
                    f'完整链路HELLO_ACK通过，session=0x{self.session.session_id:08x}')
            elif frame.message_type == MessageType.WHEEL_TELEMETRY:
                try:
                    telemetry = unpack_wheel_telemetry(frame.payload)
                except ProtocolError as error:
                    self.get_logger().warn(f'丢弃无效轮速遥测: {error}')
                    continue
                self.last_mcu_fault_bits = telemetry.fault_bits
                self.last_wheel_telemetry_monotonic = time.monotonic()
                if telemetry.fault_bits:
                    self.latch_local_stop(f'STM32故障位={telemetry.fault_bits:#x}')
                self.wheel_rpm_pub.publish(Float32MultiArray(
                    data=list(telemetry.wheel_rpm)))
                self.encoder_pub.publish(Int32MultiArray(
                    data=list(telemetry.encoder_counts)))
                # 未安装/启用电池ADC时，0字段表示未知，不显示成实测0V。
                self.battery_pub.publish(Float32(
                    data=telemetry.battery_mv / 1000.0 if telemetry.battery_mv else math.nan))
                self.fault_pub.publish(UInt32(data=telemetry.fault_bits))
                self.state_flags_pub.publish(UInt16(data=telemetry.state_flags))
                self.last_sequence_pub.publish(UInt32(
                    data=telemetry.last_command_sequence))
                if telemetry.last_command_sequence != self.last_telemetry_command_sequence:
                    self.last_telemetry_command_sequence = (
                        telemetry.last_command_sequence)
                    self.last_command_progress_monotonic = time.monotonic()
            elif frame.message_type == MessageType.LINK_STATUS and self.readonly_mode:
                try:
                    status = decode_readonly_status(frame.payload)
                    self.diagnostics_pub.publish(String(data=json.dumps(status)))
                except ProtocolError as error:
                    self.latch_local_stop('只读诊断校验失败')
                    self.get_logger().error(f'只读诊断未通过: {error}')
            elif frame.message_type == MessageType.LINK_STATUS and self.bounded_motion_mode:
                try:
                    self.last_motion_status = decode_motion_status(frame.payload)
                    self.last_motion_status_monotonic = time.monotonic()
                    self.diagnostics_pub.publish(String(data=json.dumps(self.last_motion_status)))
                except ProtocolError as error:
                    self.latch_local_stop('受限运动固件身份或诊断不匹配')
                    self.get_logger().error(f'4WM1诊断未通过: {error}')

    def send_hello_if_due(self):
        now = time.monotonic()
        if now - self.last_hello_monotonic < self.hello_retry_s:
            return
        self.last_hello_monotonic = now
        self.last_mcu_fault_bits = None
        self.last_wheel_telemetry_monotonic = None
        frame = self.session.new_hello(self.monotonic_ms())
        self.sock.sendto(frame, self.destination)

    def send_command(self):
        enabled = (not self.readonly_mode and not self.local_stop_latched
                   and self.safety_enabled and self.cmd_is_fresh())
        if enabled:
            vx = self.clamp_finite(self.latest_cmd.linear.x, self.max_vx_mps)
            vy = self.clamp_finite(self.latest_cmd.linear.y, self.max_vy_mps)
            wz = self.clamp_finite(self.latest_cmd.angular.z, self.max_wz_radps)
        else:
            vx = vy = wz = 0.0
        frame = self.session.cmd_vel(
            vx, vy, wz, self.wire_timeout_ms, enabled, self.monotonic_ms())
        self.sock.sendto(frame, self.destination)

    def publish_link_state(self):
        connected = self.sock is not None and self.session.connected
        self.link_pub.publish(Bool(data=connected))
        status = 'CONNECTED_ZERO' if connected else 'WAITING_HELLO_ACK'
        if connected and self.local_stop_latched:
            status = 'CONNECTED_STOP_LATCHED'
        elif connected and not self.readonly_mode and self.safety_enabled and self.cmd_is_fresh():
            status = 'CONNECTED_ENABLED'
        if self.sock is None:
            status = 'NO_ESP_IP'
        self.status_pub.publish(String(data=status))
        if connected != self.last_reported_link:
            self.last_reported_link = connected
            if connected:
                self.get_logger().info(f'MC-Link状态: {status}')
            else:
                self.get_logger().warn(f'MC-Link状态: {status}')

    def tick(self):
        if self.sock is None:
            self.publish_link_state()
            return
        self.drain_receive()
        if (self.session.connected
                and self.last_receive_monotonic is not None
                and time.monotonic() - self.last_receive_monotonic
                > self.link_receive_timeout_s):
            self.get_logger().error('MC-Link遥测超时，停止旧会话并重新握手')
            self.latch_local_stop('遥测超时')
            self.session.connected = False
            self.last_receive_monotonic = None
            self.last_command_progress_monotonic = None
            self.last_telemetry_command_sequence = None
            self.last_hello_monotonic = -math.inf
        elif (self.session.connected
                and self.last_command_progress_monotonic is not None
                and time.monotonic() - self.last_command_progress_monotonic
                > self.command_progress_timeout_s):
            self.get_logger().error(
                'STM32最后命令序号停止推进，ESP会话可能已超时锁定；重新握手')
            self.latch_local_stop('STM32命令序号停止推进')
            self.session.connected = False
            self.last_receive_monotonic = None
            self.last_command_progress_monotonic = None
            self.last_telemetry_command_sequence = None
            self.last_hello_monotonic = -math.inf
        try:
            if self.session.connected:
                if not self.send_pending_control_action():
                    self.send_command()
            else:
                self.send_hello_if_due()
        except OSError as error:
            self.get_logger().error(f'UDP发送失败: {error}')
            self.latch_local_stop('UDP发送失败')
            self.session.connected = False
        self.publish_link_state()

    def destroy_node(self):
        if self.sock is not None:
            if self.session.connected:
                for _ in range(3):
                    try:
                        self.sock.sendto(
                            self.session.cmd_vel(
                                0.0, 0.0, 0.0, self.wire_timeout_ms,
                                False, self.monotonic_ms()),
                            self.destination)
                    except OSError:
                        break
            self.sock.close()
            self.sock = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelUdpBridge()
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
