import pytest

from mecanum_vicon_control.transport_protocol import (
    FrameFlags,
    MessageType,
    ProtocolError,
    WheelTelemetry,
    cobs_decode,
    cobs_encode,
    decode_frame,
    decode_uart_packet,
    encode_frame,
    encode_uart_packet,
    pack_cmd_vel,
    pack_control_enable,
    pack_wheel_telemetry,
    sequence_is_newer,
    unpack_cmd_vel,
    unpack_control_enable,
    unpack_wheel_telemetry,
)


def test_cmd_vel_frame_round_trip():
    payload = pack_cmd_vel(0.25, -0.125, 0.8, timeout_ms=150)
    raw = encode_frame(
        MessageType.CMD_VEL,
        sequence=42,
        session_id=0x12345678,
        sender_time_ms=987654,
        payload=payload,
        flags=FrameFlags.ACK_REQUIRED | FrameFlags.ENABLED,
    )
    assert raw.hex() == (
        '4d43011009002a0000007856341206120f000a00'
        'fa0083ff200396000000835a4e9f')
    assert encode_uart_packet(raw).hex() == (
        '064d43011009022a0101087856341206120f020a02'
        'fa0683ff200396010105835a4e9f00')
    frame = decode_frame(raw)
    command = unpack_cmd_vel(frame.payload)
    assert frame.message_type == MessageType.CMD_VEL
    assert frame.sequence == 42
    assert frame.session_id == 0x12345678
    assert frame.flags == FrameFlags.ACK_REQUIRED | FrameFlags.ENABLED
    assert command.vx_mps == pytest.approx(0.25)
    assert command.vy_mps == pytest.approx(-0.125)
    assert command.wz_radps == pytest.approx(0.8)
    assert command.timeout_ms == 150


def test_crc_detects_corruption():
    raw = bytearray(encode_frame(MessageType.HEARTBEAT, 1, 2, 3, b'abc'))
    raw[-5] ^= 0x40
    with pytest.raises(ProtocolError, match='CRC32'):
        decode_frame(raw)


def test_control_enable_round_trip_and_reserved_validation():
    command = unpack_control_enable(pack_control_enable(True, False))
    assert command.enabled
    assert not command.clear_latched_faults

    clear = unpack_control_enable(pack_control_enable(False, True))
    assert not clear.enabled
    assert clear.clear_latched_faults

    with pytest.raises(ProtocolError, match='wrong size'):
        unpack_control_enable(b'\x00')
    with pytest.raises(ProtocolError, match='invalid fields'):
        unpack_control_enable(b'\x02\x00\x00\x00')


@pytest.mark.parametrize('data', [
    b'', b'\x00', b'a\x00b', bytes(range(1, 255)), b'\x00' * 300,
])
def test_cobs_round_trip(data):
    assert cobs_decode(cobs_encode(data)) == data


def test_uart_packet_round_trip():
    raw = encode_frame(MessageType.ESTOP, 9, 8, 7)
    packet = encode_uart_packet(raw)
    assert packet.endswith(b'\x00')
    assert b'\x00' not in packet[:-1]
    assert decode_uart_packet(packet) == raw


def test_wheel_telemetry_round_trip():
    original = WheelTelemetry(
        sample_time_ms=1234,
        wheel_rpm=(12.3, -4.5, 6.7, -8.9),
        encoder_counts=(100, -200, 300, -400),
        battery_mv=11980,
        state_flags=3,
        fault_bits=0x20,
        last_command_sequence=77,
    )
    decoded = unpack_wheel_telemetry(pack_wheel_telemetry(original))
    assert decoded == original


def test_sequence_comparison_handles_wraparound():
    assert sequence_is_newer(11, 10)
    assert not sequence_is_newer(10, 10)
    assert sequence_is_newer(0, 0xFFFFFFFF)
    assert not sequence_is_newer(0xFFFFFFFF, 0)


def test_invalid_velocity_and_timeout_are_rejected():
    with pytest.raises(ProtocolError, match='not finite'):
        pack_cmd_vel(float('nan'), 0.0, 0.0)
    with pytest.raises(ProtocolError, match='timeout_ms'):
        pack_cmd_vel(0.0, 0.0, 0.0, timeout_ms=5)
