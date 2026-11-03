"""Session-bound clock proof; no I/O except the explicitly wrapped socket.

Anchor to the MCU HELLO_ACK and the PC HELLO source time, NOT the first
possibly queued telemetry receipt.  A <=40 ms full round trip bounds that
unsynchronised anchor conservatively.  This is not measured one-way latency.
"""
import json
import math
import os
import struct
import time

from mecanum_vicon_control.transport_protocol import (
    MessageType, FrameFlags, ProtocolError, decode_frame, unpack_wheel_telemetry,
    unpack_cmd_vel)
from .commissioning import McuSampleClock
from .stage0_contract import decode_motion_status, disabled_motion_status_ready


class HandshakeSampleClock(McuSampleClock):
    MAX_HELLO_RTT_S = .040
    MIN_SAMPLES = 5
    MIN_SPAN_MS = 400

    def __init__(self):
        super().__init__()
        self.session_id = self.hello_sequence = 0
        self.source_at = self.sent_at = self.ack_at = self.rtt_s = None
        self.qualified = self.calibrated = self.frozen = False
        self.fault = self.notice = ''
        self.counts, self.first_ticks = {}, {}
        self.native_seen = set()
        self.stationary_ready = False

    def begin(self, session_id, sequence, sender_ms, now):
        if self.frozen or self.fault:
            raise OSError('startup handshake cannot reset arm intent or native faults')
        elapsed = (int(now * 1000) - sender_ms) & 0xffffffff
        if not session_id or elapsed > 10:
            raise OSError('HELLO was already delayed before socket submission')
        super().__init__()
        self.session_id, self.hello_sequence = session_id, sequence
        self.source_at, self.sent_at = (int(now * 1000) - elapsed) / 1000., now
        self.ack_at = self.rtt_s = None
        self.qualified = self.calibrated = False
        self.counts, self.first_ticks, self.native_seen = {}, {}, set()
        self.stationary_ready = False
        self.notice = 'WAIT_HELLO_RTT'

    def acknowledge(self, frame, now):
        if (self.qualified or frame.session_id != self.session_id
                or frame.sequence != self.hello_sequence
                or frame.flags != int(FrameFlags.RESPONSE) or frame.payload
                or self.source_at is None):
            return False
        self.rtt_s = now - self.source_at
        if not math.isfinite(self.rtt_s) or not 0 <= self.rtt_s <= self.MAX_HELLO_RTT_S:
            self.notice = 'HELLO_RTT_EXCEEDED'
            return False
        # Earliest possible processing instant, not a later receipt timestamp.
        self.latest_tick, self.local_baseline = frame.sender_time_ms, self.source_at
        self.qualified, self.ack_at, self.notice = True, now, 'WAIT_CLOCK_WINDOW'
        return True

    def note_native(self, frame):
        if frame.flags & ~int(FrameFlags.FAULT | FrameFlags.ENABLED):
            raise ProtocolError('native telemetry flags differ from firmware publish contract')
        if frame.message_type == MessageType.WHEEL_TELEMETRY:
            t = unpack_wheel_telemetry(frame.payload)
            if t.sample_time_ms != frame.sender_time_ms:
                raise ProtocolError('MCU telemetry/header timestamp mismatch')
            if t.fault_bits:
                self.fault = 'NATIVE_FAULT_' + hex(t.fault_bits)
            if bool(frame.flags & int(FrameFlags.FAULT)) != bool(t.fault_bits):
                raise ProtocolError('native fault header/payload mismatch')
            if bool(frame.flags & int(FrameFlags.ENABLED)) != bool(t.state_flags & 2):
                raise ProtocolError('native enabled header/payload mismatch')
            zero = (not any(t.wheel_rpm) and t.state_flags & 8
                    and not t.state_flags & (2 | 256))
        else:
            d = decode_motion_status(frame.payload)
            self.stationary_ready = disabled_motion_status_ready(d)
            zero = (not d['permitted']
                    and all(not any(d[k]) for k in ('compares','raw_duty','target_cps',
                        'body_milli','effective_cps','granted_direction')) and not d['watchdog_reset']
                    and not d['fault_wheel'] and not d['first_feedback_fault']['reason'])
        if frame.flags & int(FrameFlags.FAULT):
            self.fault = self.fault or 'NATIVE_FAULT_FLAG'
        if not self.frozen and (not zero or frame.flags & int(FrameFlags.ENABLED)):
            self.fault = self.fault or 'STARTUP_NOT_DISABLED_STATIONARY'
        self.native_seen.add(frame.message_type)

    def observe(self, kind, tick, now):
        if not self.qualified or self.fault or not math.isfinite(now):
            return None
        if type(tick) is not int or not 0 <= tick <= 0xffffffff:
            return None
        previous = self.by_kind.get(kind)
        if previous is not None and not 0 < ((tick-previous) & 0xffffffff) < 0x80000000:
            return None
        delta = (tick-self.latest_tick) & 0xffffffff
        if delta >= 0x80000000:
            return None
        result = self.local_baseline+(self.elapsed_ms+delta)/1000.
        if not -.02 <= now-result <= .20:
            return None
        # Never refine/rebase an ACK-bounded anchor on telemetry receipts,
        # including repeated small future innovations. Real clock jumps and
        # an accumulating queue must remain visible to the unchanged bounds.
        self.latest_tick, self.elapsed_ms = tick, self.elapsed_ms+delta
        self.by_kind[kind], self.accepted_at[kind] = tick, result
        if not self.calibrated:
            if previous is not None and ((tick - previous) & 0xffffffff) > 200:
                self.counts, self.first_ticks = {}, {}
            self.first_ticks.setdefault(kind, tick)
            self.counts[kind] = self.counts.get(kind, 0) + 1
            kinds = (MessageType.WHEEL_TELEMETRY, MessageType.LINK_STATUS)
            self.calibrated = all(k in self.native_seen and self.counts.get(k, 0) >= self.MIN_SAMPLES
                and ((self.by_kind[k] - self.first_ticks[k]) & 0xffffffff) >= self.MIN_SPAN_MS
                for k in kinds)
        return result

    def ready(self, now):
        return bool(self.qualified and self.calibrated and (self.frozen or self.stationary_ready)
                    and not self.fault and all(
            k in self.accepted_at and 0 <= now - self.accepted_at[k] <= .20
            for k in (MessageType.WHEEL_TELEMETRY, MessageType.LINK_STATUS)))

    def freeze(self, now):
        if not self.ready(now):
            raise OSError('cannot arm before a fresh same-session clock proof')
        self.frozen = True

    def snapshot(self, now):
        phase = (self.fault or ('READY' if self.ready(now) else
            'WAIT_FRESH_SAMPLES' if self.calibrated else
            'WAIT_CLOCK_WINDOW' if self.qualified else self.notice or 'WAIT_HELLO'))
        return dict(phase=phase, ready=self.ready(now), qualified=self.qualified,
                    session_id=self.session_id, hello_rtt_s=self.rtt_s,
                    max_hello_rtt_s=self.MAX_HELLO_RTT_S,
                    counts={MessageType(k).name: v for k, v in self.counts.items()},
                    frozen=self.frozen, fault=self.fault,
                    anchor='MCU_ACK_PC_HELLO_SOURCE_EARLIEST')


class StartupProgressSocket:
    """Reject unsuitable HELLO_ACKs before the legacy session can connect.

    Old/backlogged frames do not renew freshness.  No source time is rewritten,
    no enabled frame is allowed through a startup-only proof, no fault is cleared.
    """
    MAX_HELLOS = 3

    def __init__(self, sock, ip, session_id, trace_path=''):
        self.socket, self.ip, self.session_id = sock, ip, session_id
        self.clock = HandshakeSampleClock()
        self.hello_attempts = self.command_count = 0
        self.last_command_at = None
        self.trace = (os.fdopen(os.open(trace_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w')
                      if trace_path else None)

    def __getattr__(self, key):
        return getattr(self.socket, key)

    def close(self):
        try:
            return self.socket.close()
        finally:
            if self.trace:
                self.trace.close()

    def sendto(self, data, destination):
        f = decode_frame(data)
        now = time.monotonic()
        if f.message_type == MessageType.HELLO:
            if self.hello_attempts >= self.MAX_HELLOS:
                raise OSError('bounded startup HELLO attempts exhausted')
            self.clock.begin(f.session_id, f.sequence, f.sender_time_ms, now)
            self.hello_attempts += 1
            self.command_count, self.last_command_at = 0, None
        elif f.message_type != MessageType.ESTOP:
            if f.session_id != self.clock.session_id or not self.clock.qualified or self.clock.fault:
                raise OSError('unqualified/faulted startup session cannot send control')
            if f.message_type == MessageType.CMD_VEL:
                v = unpack_cmd_vel(f.payload)
                moving = bool(f.flags & int(FrameFlags.ENABLED) or v.vx_mps or v.vy_mps or v.wz_radps)
                if moving and not (self.clock.ready(now) and self.clock.frozen):
                    raise OSError('motion requires frozen fresh startup proof')
                self.command_count += 1
                self.last_command_at = now
            elif f.message_type == MessageType.CONTROL_ENABLE:
                if len(f.payload) != 4:
                    raise OSError('control payload length')
                enabled, clear, reserved = struct.unpack('<BBH', f.payload)
                if enabled > 1 or clear > 1 or reserved not in (0, 0x7804):
                    raise OSError('control differs from fixed Sign stage4')
                if not self.clock.ready(now) or ((enabled or clear) and not self.clock.frozen):
                    raise OSError('control requires fresh startup proof')
        return self.socket.sendto(data, destination)

    def recvfrom(self, size):
        while True:
            data, sender = self.socket.recvfrom(size)
            if sender[0] != self.ip:
                continue
            try:
                f = decode_frame(data)
                if f.session_id != self.session_id():
                    continue
                now = time.monotonic()
                if f.message_type == MessageType.HELLO_ACK:
                    if not self.clock.acknowledge(f, now):
                        continue
                elif f.message_type in (MessageType.WHEEL_TELEMETRY, MessageType.LINK_STATUS):
                    self.clock.note_native(f)
                    before = dict(latest_tick=self.clock.latest_tick, local_baseline=self.clock.local_baseline,
                                  elapsed_ms=self.clock.elapsed_ms, previous_kind_tick=self.clock.by_kind.get(f.message_type))
                    accepted = self.clock.observe(f.message_type, f.sender_time_ms, now)
                    if self.trace:
                        self.trace.write(json.dumps(dict(received_steady=now, kind=MessageType(f.message_type).name,
                            tick=f.sender_time_ms, sequence=f.sequence, session=f.session_id,
                            accepted_sample_steady=accepted, clock_before=before, payload_hex=f.payload.hex(),
                            startup_clock=self.clock.snapshot(now)))+'\n')
                        self.trace.flush()
                    if accepted is None and not self.clock.fault:
                        continue
            except ProtocolError:
                # Keep malformed native data fail-closed; never use it as proof.
                self.clock.fault = 'NATIVE_DECODE_INVALID'
                continue
            return data, sender
