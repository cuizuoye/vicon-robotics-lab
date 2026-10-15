import pytest

from mecanum_vicon_control.mc_link_session import (
    McLinkSession,
    SessionMismatchError,
)
from mecanum_vicon_control.transport_protocol import (
    FrameFlags,
    MessageType,
    ProtocolError,
    decode_frame,
    encode_frame,
    unpack_control_enable,
    unpack_cmd_vel,
)


def test_requires_matching_hello_ack_before_commands():
    session = McLinkSession(lambda: 0x12345678)
    hello = decode_frame(session.new_hello(10))
    assert hello.message_type == MessageType.HELLO
    assert hello.session_id == 0x12345678
    assert not session.connected
    with pytest.raises(ProtocolError, match='HELLO_ACK'):
        session.cmd_vel(0.0, 0.0, 0.0, 150, False, 11)

    wrong = encode_frame(
        MessageType.HELLO_ACK, hello.sequence, 0x87654321, 12,
        flags=FrameFlags.RESPONSE)
    with pytest.raises(SessionMismatchError, match='another session'):
        session.accept(wrong)

    ack = encode_frame(
        MessageType.HELLO_ACK, hello.sequence, hello.session_id, 13,
        flags=FrameFlags.RESPONSE)
    session.accept(ack)
    assert session.connected


def test_disabled_command_is_zero_and_has_no_enabled_flag():
    session = McLinkSession(lambda: 7)
    hello = decode_frame(session.new_hello(1))
    session.accept(encode_frame(
        MessageType.HELLO_ACK, hello.sequence, hello.session_id, 2,
        flags=FrameFlags.RESPONSE))

    frame = decode_frame(session.cmd_vel(0.0, 0.0, 0.0, 150, False, 3))
    command = unpack_cmd_vel(frame.payload)
    assert frame.flags == FrameFlags.NONE
    assert command.vx_mps == command.vy_mps == command.wz_radps == 0.0


def test_enabled_command_sets_flag_and_preserves_velocity():
    session = McLinkSession(lambda: 9)
    hello = decode_frame(session.new_hello(1))
    session.accept(encode_frame(
        MessageType.HELLO_ACK, hello.sequence, hello.session_id, 2,
        flags=FrameFlags.RESPONSE))

    frame = decode_frame(session.cmd_vel(0.2, -0.1, 0.5, 150, True, 3))
    command = unpack_cmd_vel(frame.payload)
    assert frame.flags == FrameFlags.ENABLED
    assert command.vx_mps == pytest.approx(0.2)
    assert command.vy_mps == pytest.approx(-0.1)
    assert command.wz_radps == pytest.approx(0.5)


def test_low_frequency_control_and_estop_frames():
    session = McLinkSession(lambda: 11)
    hello = decode_frame(session.new_hello(1))
    session.accept(encode_frame(
        MessageType.HELLO_ACK, hello.sequence, hello.session_id, 2,
        flags=FrameFlags.RESPONSE))

    control = decode_frame(session.control_enable(False, True, 3))
    request = unpack_control_enable(control.payload)
    assert control.message_type == MessageType.CONTROL_ENABLE
    assert control.flags == FrameFlags.ACK_REQUIRED
    assert not request.enabled
    assert request.clear_latched_faults

    estop = decode_frame(session.estop(4))
    assert estop.message_type == MessageType.ESTOP
    assert estop.flags == FrameFlags.ACK_REQUIRED
    assert estop.payload == b''
    assert estop.sequence == control.sequence + 1
