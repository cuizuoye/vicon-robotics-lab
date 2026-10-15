import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from mecanum_vicon_control.cmd_vel_udp_bridge import CmdVelUdpBridge
from mecanum_vicon_control.transport_protocol import (
    FrameFlags, MessageType, WheelTelemetry, decode_frame, encode_frame,
    pack_wheel_telemetry, unpack_cmd_vel,
)


class SocketStub:
    def __init__(self):
        self.frames = []
        self.incoming = []
        self.fail_send = False

    def sendto(self, data, _destination):
        if self.fail_send:
            raise OSError('injected send failure')
        self.frames.append(data)
        return len(data)

    def recvfrom(self, _size):
        if not self.incoming:
            raise BlockingIOError
        return self.incoming.pop(0), ('192.0.2.1', 46000)

    def close(self):
        pass


@pytest.fixture
def node():
    rclpy.init(args=[])
    n = CmdVelUdpBridge()  # empty esp_ip: never opens a real socket
    n.esp_ip = '192.0.2.1'
    n.destination = (n.esp_ip, 46000)
    n.sock = SocketStub()
    n.session.session_id = 7
    n.session.connected = True
    try:
        yield n
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def feedback(n, fault):
    payload = pack_wheel_telemetry(WheelTelemetry(
        sample_time_ms=100, wheel_rpm=(0., 0., 0., 0.),
        encoder_counts=(0, 0, 0, 0), battery_mv=0,
        state_flags=125 if fault else 109, fault_bits=fault,
        last_command_sequence=10))
    n.sock.incoming.append(encode_frame(MessageType.WHEEL_TELEMETRY, 10,
        n.session.session_id, 100, payload,
        FrameFlags.FAULT if fault else FrameFlags.NONE))
    n.drain_receive()


def velocity(n):
    n.on_enable(Bool(data=True))
    msg = Twist()
    msg.linear.x = .2
    n.on_cmd(msg)


def test_mcu_fault_cancels_old_enable_and_handshake_cannot_rearm(node):
    velocity(node)
    node.pending_control_action = 'enable'
    node.pending_control_repeats = 5
    feedback(node, 1)
    assert node.local_stop_latched and not node.safety_enabled
    assert node.last_cmd_ns is None and node.pending_control_action is None
    node.send_hello_if_due()
    hello = decode_frame(node.sock.frames[-1])
    node.sock.incoming.append(encode_frame(MessageType.HELLO_ACK,
        hello.sequence, hello.session_id, 100, flags=FrameFlags.RESPONSE))
    node.drain_receive()
    feedback(node, 0)
    velocity(node)  # even fresh commands + a repeated watchdog True cannot rearm
    assert node.local_stop_latched and not node.safety_enabled
    node.safety_enabled = True  # independent send guard, even if misassigned
    node.send_command()
    frame = decode_frame(node.sock.frames[-1])
    command = unpack_cmd_vel(frame.payload)
    assert frame.flags == 0
    assert command.vx_mps == command.vy_mps == command.wz_radps == 0


def test_rearm_requires_explicit_disable_then_enable_and_fresh_feedback(node):
    node.latch_local_stop('test')
    feedback(node, 0)
    node.on_control_enable(Bool(data=True))
    assert node.local_stop_latched and node.pending_control_action is None
    node.on_control_enable(Bool(data=False))
    assert node.control_enable_reset_seen and node.local_stop_latched
    node.session.connected = True
    feedback(node, 0)
    node.last_wheel_telemetry_monotonic = time.monotonic() - 10
    node.on_control_enable(Bool(data=True))
    assert node.local_stop_latched
    feedback(node, 0)
    node.on_control_enable(Bool(data=True))
    assert not node.local_stop_latched
    assert node.pending_control_action == 'enable'
    assert not node.safety_enabled and node.last_cmd_ns is None


def test_clear_fault_request_does_not_clear_host_stop_latch(node):
    node.latch_local_stop('test')
    node.on_clear_faults(Bool(data=True))
    assert node.local_stop_latched and node.pending_control_action == 'clear_faults'


@pytest.mark.parametrize('receive,progress', [(0., 99.9), (99.9, 0.)])
def test_receive_or_progress_timeout_latches_before_auto_handshake(
        node, monkeypatch, receive, progress):
    velocity(node)
    node.last_receive_monotonic = receive
    node.last_command_progress_monotonic = progress
    monkeypatch.setattr(time, 'monotonic', lambda: 100.)
    node.tick()
    assert node.local_stop_latched and not node.safety_enabled
    assert node.last_cmd_ns is None
    assert decode_frame(node.sock.frames[-1]).message_type == MessageType.HELLO


def test_send_failure_latches_and_forced_pending_enable_is_blocked(node):
    velocity(node)
    node.sock.fail_send = True
    node.tick()
    assert node.local_stop_latched and not node.safety_enabled
    node.sock.fail_send = False
    node.session.connected = True
    node.pending_control_action = 'enable'
    node.pending_control_repeats = 5
    before = len(node.sock.frames)
    assert node.send_pending_control_action()
    assert node.pending_control_action is None and len(node.sock.frames) == before


def test_readonly_mode_cannot_clear_stop_latch_by_enable(node):
    node.readonly_mode = True
    node.latch_local_stop('test')
    node.control_enable_reset_seen = True
    feedback(node, 0)
    node.on_control_enable(Bool(data=True))
    assert node.local_stop_latched and node.pending_control_action is None
