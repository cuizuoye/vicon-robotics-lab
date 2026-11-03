"""Stateful MC-Link session builder kept independent from ROS for testing."""

from collections.abc import Callable
import secrets

from .transport_protocol import (
    Frame,
    FrameFlags,
    MessageType,
    ProtocolError,
    decode_frame,
    encode_frame,
    pack_control_enable,
    pack_cmd_vel,
)


class SessionMismatchError(ProtocolError):
    """A valid delayed frame from an older or foreign MC-Link session."""


class McLinkSession:
    def __init__(self, session_id_factory: Callable[[], int] | None = None):
        self._session_id_factory = session_id_factory or self._random_nonzero_u32
        self.session_id = 0
        self.sequence = 0
        self.pending_hello_sequence = 0
        self.connected = False

    @staticmethod
    def _random_nonzero_u32() -> int:
        value = 0
        while value == 0:
            value = secrets.randbits(32)
        return value

    def new_hello(self, sender_time_ms: int) -> bytes:
        self.session_id = int(self._session_id_factory()) & 0xFFFFFFFF
        if self.session_id == 0:
            raise ProtocolError('session_id_factory returned zero')
        self.sequence = 1
        self.pending_hello_sequence = self.sequence
        self.connected = False
        return encode_frame(
            MessageType.HELLO,
            sequence=self.sequence,
            session_id=self.session_id,
            sender_time_ms=sender_time_ms & 0xFFFFFFFF,
        )

    def accept(self, data: bytes) -> Frame:
        frame = decode_frame(data)
        if frame.session_id != self.session_id:
            raise SessionMismatchError('frame belongs to another session')
        if (frame.message_type == MessageType.HELLO_ACK
                and frame.sequence == self.pending_hello_sequence
                and frame.flags & FrameFlags.RESPONSE):
            self.connected = True
        return frame

    def cmd_vel(
            self,
            vx_mps: float,
            vy_mps: float,
            wz_radps: float,
            timeout_ms: int,
            enabled: bool,
            sender_time_ms: int,
    ) -> bytes:
        if not self.connected:
            raise ProtocolError('HELLO_ACK is required before CMD_VEL')
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        return encode_frame(
            MessageType.CMD_VEL,
            sequence=self.sequence,
            session_id=self.session_id,
            sender_time_ms=sender_time_ms & 0xFFFFFFFF,
            payload=pack_cmd_vel(vx_mps, vy_mps, wz_radps, timeout_ms),
            flags=FrameFlags.ENABLED if enabled else FrameFlags.NONE,
        )

    def control_enable(
            self,
            enabled: bool,
            clear_latched_faults: bool,
            sender_time_ms: int,
    ) -> bytes:
        if not self.connected:
            raise ProtocolError('HELLO_ACK is required before CONTROL_ENABLE')
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        return encode_frame(
            MessageType.CONTROL_ENABLE,
            sequence=self.sequence,
            session_id=self.session_id,
            sender_time_ms=sender_time_ms & 0xFFFFFFFF,
            payload=pack_control_enable(enabled, clear_latched_faults),
            flags=FrameFlags.ACK_REQUIRED,
        )

    def estop(self, sender_time_ms: int) -> bytes:
        if not self.connected:
            raise ProtocolError('HELLO_ACK is required before ESTOP')
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        return encode_frame(
            MessageType.ESTOP,
            sequence=self.sequence,
            session_id=self.session_id,
            sender_time_ms=sender_time_ms & 0xFFFFFFFF,
            flags=FrameFlags.ACK_REQUIRED,
        )
