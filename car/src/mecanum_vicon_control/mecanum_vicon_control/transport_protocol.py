"""MC-Link v1 binary protocol shared by ROS 2, ESP32-S3 and STM32F407."""

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import math
import struct
import zlib


MAGIC = b'MC'
PROTOCOL_VERSION = 1
MAX_PAYLOAD_SIZE = 256
HEADER_STRUCT = struct.Struct('<2sBBHIIIH')
CRC_STRUCT = struct.Struct('<I')
CMD_VEL_STRUCT = struct.Struct('<hhhHBB')
CONTROL_ENABLE_STRUCT = struct.Struct('<BBH')
WHEEL_TELEMETRY_STRUCT = struct.Struct('<IhhhhiiiiHHII')


class ProtocolError(ValueError):
    pass


class MessageType(IntEnum):
    HELLO = 0x01
    HELLO_ACK = 0x02
    HEARTBEAT = 0x03
    CMD_VEL = 0x10
    CONTROL_ENABLE = 0x11
    ESTOP = 0x12
    ACK = 0x20
    WHEEL_TELEMETRY = 0x30
    LINK_STATUS = 0x31


class FrameFlags(IntFlag):
    NONE = 0
    ACK_REQUIRED = 1 << 0
    RESPONSE = 1 << 1
    FAULT = 1 << 2
    ENABLED = 1 << 3


class ControlMode(IntEnum):
    BODY_VELOCITY = 0


@dataclass(frozen=True)
class Frame:
    message_type: int
    flags: int
    sequence: int
    session_id: int
    sender_time_ms: int
    payload: bytes


@dataclass(frozen=True)
class CmdVel:
    vx_mps: float
    vy_mps: float
    wz_radps: float
    timeout_ms: int
    control_mode: int


@dataclass(frozen=True)
class ControlEnable:
    enabled: bool
    clear_latched_faults: bool


@dataclass(frozen=True)
class WheelTelemetry:
    sample_time_ms: int
    wheel_rpm: tuple[float, float, float, float]
    encoder_counts: tuple[int, int, int, int]
    battery_mv: int
    state_flags: int
    fault_bits: int
    last_command_sequence: int


def _check_u32(value: int, name: str):
    if not 0 <= int(value) <= 0xFFFFFFFF:
        raise ProtocolError(f'{name} is outside uint32 range')


def encode_frame(
        message_type: int,
        sequence: int,
        session_id: int,
        sender_time_ms: int,
        payload: bytes = b'',
        flags: int = 0,
) -> bytes:
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ProtocolError('payload is too large')
    if not 0 <= int(message_type) <= 0xFF:
        raise ProtocolError('message_type is outside uint8 range')
    if not 0 <= int(flags) <= 0xFFFF:
        raise ProtocolError('flags is outside uint16 range')
    _check_u32(sequence, 'sequence')
    _check_u32(session_id, 'session_id')
    _check_u32(sender_time_ms, 'sender_time_ms')

    header = HEADER_STRUCT.pack(
        MAGIC,
        PROTOCOL_VERSION,
        int(message_type),
        int(flags),
        int(sequence),
        int(session_id),
        int(sender_time_ms),
        len(payload),
    )
    protected = header + payload
    return protected + CRC_STRUCT.pack(zlib.crc32(protected) & 0xFFFFFFFF)


def decode_frame(data: bytes) -> Frame:
    data = bytes(data)
    minimum_size = HEADER_STRUCT.size + CRC_STRUCT.size
    if len(data) < minimum_size:
        raise ProtocolError('frame is shorter than the fixed header and CRC')

    (magic, version, message_type, flags, sequence, session_id,
     sender_time_ms, payload_length) = HEADER_STRUCT.unpack_from(data)
    if magic != MAGIC:
        raise ProtocolError('invalid frame magic')
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f'unsupported protocol version: {version}')
    if payload_length > MAX_PAYLOAD_SIZE:
        raise ProtocolError('declared payload is too large')
    expected_size = HEADER_STRUCT.size + payload_length + CRC_STRUCT.size
    if len(data) != expected_size:
        raise ProtocolError(
            f'frame length mismatch: received {len(data)}, expected {expected_size}')

    protected = data[:-CRC_STRUCT.size]
    received_crc, = CRC_STRUCT.unpack_from(data, len(protected))
    calculated_crc = zlib.crc32(protected) & 0xFFFFFFFF
    if received_crc != calculated_crc:
        raise ProtocolError('CRC32 mismatch')

    payload = data[HEADER_STRUCT.size:HEADER_STRUCT.size + payload_length]
    return Frame(
        message_type=message_type,
        flags=flags,
        sequence=sequence,
        session_id=session_id,
        sender_time_ms=sender_time_ms,
        payload=payload,
    )


def _to_milli_int16(value: float, name: str) -> int:
    if not math.isfinite(value):
        raise ProtocolError(f'{name} is not finite')
    quantized = int(round(value * 1000.0))
    if not -32768 <= quantized <= 32767:
        raise ProtocolError(f'{name} exceeds the int16 wire range')
    return quantized


def pack_cmd_vel(
        vx_mps: float,
        vy_mps: float,
        wz_radps: float,
        timeout_ms: int = 150,
        control_mode: int = ControlMode.BODY_VELOCITY,
) -> bytes:
    if not 20 <= int(timeout_ms) <= 2000:
        raise ProtocolError('timeout_ms must be between 20 and 2000')
    if not 0 <= int(control_mode) <= 0xFF:
        raise ProtocolError('control_mode is outside uint8 range')
    return CMD_VEL_STRUCT.pack(
        _to_milli_int16(vx_mps, 'vx_mps'),
        _to_milli_int16(vy_mps, 'vy_mps'),
        _to_milli_int16(wz_radps, 'wz_radps'),
        int(timeout_ms),
        int(control_mode),
        0,
    )


def unpack_cmd_vel(payload: bytes) -> CmdVel:
    if len(payload) != CMD_VEL_STRUCT.size:
        raise ProtocolError('CMD_VEL payload has the wrong size')
    vx, vy, wz, timeout_ms, control_mode, _reserved = CMD_VEL_STRUCT.unpack(payload)
    return CmdVel(
        vx_mps=vx / 1000.0,
        vy_mps=vy / 1000.0,
        wz_radps=wz / 1000.0,
        timeout_ms=timeout_ms,
        control_mode=control_mode,
    )


def pack_control_enable(
        enabled: bool,
        clear_latched_faults: bool = False,
) -> bytes:
    """Pack the four-byte CONTROL_ENABLE v1 payload.

    Clearing is deliberately independent from enabling so callers can clear
    recoverable faults while the actuator path remains disabled.
    """
    return CONTROL_ENABLE_STRUCT.pack(
        1 if enabled else 0,
        1 if clear_latched_faults else 0,
        0,
    )


def unpack_control_enable(payload: bytes) -> ControlEnable:
    if len(payload) != CONTROL_ENABLE_STRUCT.size:
        raise ProtocolError('CONTROL_ENABLE payload has the wrong size')
    enabled, clear_latched_faults, reserved = CONTROL_ENABLE_STRUCT.unpack(payload)
    if reserved != 0 or enabled > 1 or clear_latched_faults > 1:
        raise ProtocolError('CONTROL_ENABLE payload contains invalid fields')
    return ControlEnable(
        enabled=bool(enabled),
        clear_latched_faults=bool(clear_latched_faults),
    )


def pack_wheel_telemetry(telemetry: WheelTelemetry) -> bytes:
    if len(telemetry.wheel_rpm) != 4 or len(telemetry.encoder_counts) != 4:
        raise ProtocolError('wheel telemetry requires exactly four wheels')
    rpm_x10 = [int(round(value * 10.0)) for value in telemetry.wheel_rpm]
    return WHEEL_TELEMETRY_STRUCT.pack(
        telemetry.sample_time_ms,
        *rpm_x10,
        *telemetry.encoder_counts,
        telemetry.battery_mv,
        telemetry.state_flags,
        telemetry.fault_bits,
        telemetry.last_command_sequence,
    )


def unpack_wheel_telemetry(payload: bytes) -> WheelTelemetry:
    if len(payload) != WHEEL_TELEMETRY_STRUCT.size:
        raise ProtocolError('WHEEL_TELEMETRY payload has the wrong size')
    values = WHEEL_TELEMETRY_STRUCT.unpack(payload)
    return WheelTelemetry(
        sample_time_ms=values[0],
        wheel_rpm=tuple(value / 10.0 for value in values[1:5]),
        encoder_counts=tuple(values[5:9]),
        battery_mv=values[9],
        state_flags=values[10],
        fault_bits=values[11],
        last_command_sequence=values[12],
    )


def sequence_is_newer(sequence: int, previous: int) -> bool:
    """Compare wrapping uint32 sequence numbers (RFC 1982 half-range rule)."""
    difference = (int(sequence) - int(previous)) & 0xFFFFFFFF
    return 0 < difference < 0x80000000


def cobs_encode(data: bytes) -> bytes:
    """COBS encode without the trailing zero delimiter."""
    output = bytearray([0])
    code_index = 0
    code = 1
    for value in bytes(data):
        if value == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(value)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    """COBS decode data that does not contain the trailing zero delimiter."""
    data = bytes(data)
    if not data or 0 in data:
        raise ProtocolError('invalid COBS packet')
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        index += 1
        end = index + code - 1
        if end > len(data):
            raise ProtocolError('truncated COBS packet')
        output.extend(data[index:end])
        index = end
        if code < 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


def encode_uart_packet(frame: bytes) -> bytes:
    return cobs_encode(frame) + b'\x00'


def decode_uart_packet(packet: bytes) -> bytes:
    packet = bytes(packet)
    if not packet.endswith(b'\x00') or packet == b'\x00':
        raise ProtocolError('UART packet must end with one zero delimiter')
    return cobs_decode(packet[:-1])
