"""Dedicated Sign-only transport constructor/decoder extension.

The legacy bridge and rectangle modules remain byte-for-byte unchanged.
Only __init__ and drain_receive are a reviewed snapshot of legacy bridge
SHA256 a41794191204c120003ced241ef5a291f4611f6e1b200718082d2425d2a193ff;
all other transport/timeout/control methods are inherited unchanged.
No runtime monkey-patching, fake identity, or global allowlist mutation.
"""
import json
import math
import socket
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray, String, UInt16, UInt32
from mecanum_vicon_control.cmd_vel_udp_bridge import (
    CmdVelUdpBridge as LegacyBridge, recoverable_idle_expiry)
from mecanum_vicon_control.mc_link_session import SessionMismatchError
from .stage0_session import Stage0Session as McLinkSession
from mecanum_vicon_control.readonly_link_status import decode_readonly_status
from mecanum_vicon_control.transport_protocol import MessageType, ProtocolError, unpack_wheel_telemetry
from mecanum_vicon_control.udp_socket_qos import configure_low_latency_qos
from .continuous_status import MOTION_TAGS, MOTION_LINEAR_LIMITS, MOTION_WIRE_TIMEOUTS, decode_motion_status


class ContinuousCmdVelUdpBridge(LegacyBridge):
    def __init__(self):
        Node.__init__(self, 'cmd_vel_udp_bridge')
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
        self.declare_parameter('required_motion_tag', '')
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
        # The installed chassis motor frame is rotated 180 degrees from the
        # Vicon/ROS body frame: firmware +x/+y drive toward physical rear/right.
        # Keep REP-103 semantics everywhere above this transport boundary and
        # rotate translations by pi on the wire; yaw is unchanged by that
        # frame rotation.  Default false keeps non-car/legacy launches intact.
        self.declare_parameter('invert_body_xy', False)

        self.esp_ip = str(self.get_parameter('esp_ip').value).strip()
        self.network_interface = str(
            self.get_parameter('network_interface').value).strip()
        self.readonly_mode = bool(self.get_parameter('readonly_mode').value)
        self.bounded_motion_mode = bool(self.get_parameter('bounded_motion_mode').value)
        self.required_motion_tag = str(
            self.get_parameter('required_motion_tag').value).strip()
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
        self.invert_body_xy = bool(self.get_parameter('invert_body_xy').value)

        if self.send_rate_hz <= 0.0:
            raise ValueError('send_rate_hz must be positive')
        if not 20 <= self.wire_timeout_ms <= 2000:
            raise ValueError('wire_timeout_ms must be between 20 and 2000')
        if self.required_motion_tag and self.required_motion_tag not in MOTION_TAGS:
            raise ValueError('required_motion_tag is not a known bounded-motion image')
        bounded_linear_limit = MOTION_LINEAR_LIMITS.get(self.required_motion_tag, 0.05)
        expected_wire_timeout = MOTION_WIRE_TIMEOUTS.get(self.required_motion_tag, 150)
        if self.bounded_motion_mode and (self.readonly_mode
                or self.wire_timeout_ms != expected_wire_timeout
                or self.send_rate_hz != 50.0
                or not 0 < self.max_vx_mps <= bounded_linear_limit
                or not 0 < self.max_vy_mps <= bounded_linear_limit
                or not 0 < self.max_wz_radps <= 2.):
            raise ValueError(
                'bounded motion requires RW, 50Hz, the profile-specific lease '
                'and limits compatible '
                'with the exact required_motion_tag')

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
        self.last_mcu_state_flags = None
        self.last_wheel_rpm = None
        self.last_wheel_telemetry_monotonic = None
        self.last_motion_status = None
        self.last_motion_status_monotonic = None
        self.last_tx_mode = None
        self.enable_prime_started = None
        self.enable_prime_complete = False
        self.enable_command_complete = False
        self.motion_gate_requested = False

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
                self.last_mcu_state_flags = telemetry.state_flags
                self.last_wheel_rpm = tuple(telemetry.wheel_rpm)
                self.last_wheel_telemetry_monotonic = time.monotonic()
                if telemetry.fault_bits:
                    if self.bounded_motion_mode and recoverable_idle_expiry(telemetry):
                        self.get_logger().warn(
                            'STM32未使能零输出会话空闲过期；自动建立新会话，不锁存停机')
                        self.session.connected = False
                        self.last_receive_monotonic = None
                        self.last_command_progress_monotonic = None
                        self.last_telemetry_command_sequence = None
                        self.last_hello_monotonic = -math.inf
                    else:
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
                    status = decode_motion_status(frame.payload)
                    if (self.required_motion_tag
                            and status['tag'] != self.required_motion_tag):
                        raise ProtocolError(
                            f"expected {self.required_motion_tag}, got {status['tag']}")
                    self.last_motion_status = status
                    self.last_motion_status_monotonic = time.monotonic()
                    self.diagnostics_pub.publish(String(data=json.dumps(self.last_motion_status)))
                except ProtocolError as error:
                    self.latch_local_stop('受限运动固件身份或诊断不匹配')
                    self.get_logger().error(f'受限运动固件诊断未通过: {error}')


