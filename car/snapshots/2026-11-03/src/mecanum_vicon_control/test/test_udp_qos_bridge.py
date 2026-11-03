"""Local socket startup only: no executor spin, HELLO or motion frames."""
import rclpy
import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mecanum_vicon_control.cmd_vel_udp_bridge import CmdVelUdpBridge


@pytest.mark.parametrize('enabled', [False, True])
def test_socket_marking_does_not_enable_readonly_bridge(enabled):
    rclpy.init(args=['--ros-args', '-p', 'esp_ip:=127.0.0.1',
        '-p', 'telemetry_port:=0', '-p', 'readonly_mode:=true',
        '-p', f'udp_low_latency_qos:={str(enabled).lower()}'])
    node = None
    try:
        node = CmdVelUdpBridge()
        assert node.socket_qos['enabled'] == enabled
        if enabled:
            assert node.socket_qos['ip_tos'] == 184
            assert node.socket_qos['socket_priority'] == 6
        node.on_enable(Bool(data=True))
        node.on_control_enable(Bool(data=True))
        node.on_clear_faults(Bool(data=True))
        command = Twist()
        command.linear.x = 1.0
        node.on_cmd(command)
        assert not node.safety_enabled
        assert node.pending_control_action is None
        assert not node.session.connected
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
