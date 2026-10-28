"""Startup-only handshake discipline; runtime safety remains independent."""
import math
import time

from .continuous_bridge import ContinuousCmdVelUdpBridge
from .startup_idle import may_retry_unarmed_startup


class StartupBridgeMixin:
    def __init__(self):
        self._startup_connected_session = None
        self._startup_reconnect_checked = None
        self._startup_blocked = ''
        super().__init__()

    def _clear_session_evidence(self):
        self.reset_preparation()
        self.stage0_selection_session = self.stage0_selection_at = None
        self.last_motion_status = self.last_motion_status_monotonic = None
        self.last_wheel_rpm = self.last_wheel_telemetry_monotonic = None
        self.last_mcu_fault_bits = self.last_mcu_state_flags = None
        self.last_receive_monotonic = self.last_command_progress_monotonic = None
        self.last_telemetry_command_sequence = self.last_feedback_identity = None
        self.last_armed_epoch = self.enabled_epoch = self.rearm_epoch = ''
        self.permit_observed_epoch = ''

    def _block_startup(self, reason):
        self._startup_blocked = reason
        self.sock.clock.fault = self.sock.clock.fault or reason
        self.policy.revoke(reason)
        # The base latch itself sends no packet and never clears anything.
        ContinuousCmdVelUdpBridge.latch_local_stop(self, reason)

    def send_hello_if_due(self):
        now = time.monotonic()
        clock = self.sock.clock
        if self._startup_blocked or clock.fault:
            self._block_startup(self._startup_blocked or clock.fault)
            return
        if self.user_arm_seen or clock.frozen or self.policy.epoch:
            self._block_startup('RECONNECT_AFTER_ARM_REQUIRES_NEW_MANUAL_SESSION')
            return
        if self.local_stop_latched or self.policy.fault or self.policy.hard_fault:
            self._block_startup('STARTUP_CANNOT_RESET_LATCHED_FAULT')
            return
        sid = self.session.session_id
        if sid and self._startup_connected_session == sid and self._startup_reconnect_checked != sid:
            if self.sock.command_count and not may_retry_unarmed_startup(self, '遥测超时', now):
                self._block_startup('RECONNECT_WITHOUT_DISABLED_ZERO_PROOF')
                return
            self._startup_reconnect_checked = sid
            # No new HELLO before any old 600ms lease has expired.  Merely
            # receiving a late packet never revives old prepared evidence.
            if self.sock.last_command_at is not None:
                self.reconnect_after = max(getattr(self, 'reconnect_after', -math.inf),
                                           self.sock.last_command_at + .65)
            self._clear_session_evidence()
        if now < getattr(self, 'reconnect_after', -math.inf):
            return
        if now - self.last_hello_monotonic < self.hello_retry_s:
            return
        if self.sock.hello_attempts >= self.sock.MAX_HELLOS:
            self._block_startup('STARTUP_HELLO_RTT_OR_LINK_NOT_QUALIFIED')
            return
        previous = self.session.session_id
        super().send_hello_if_due()
        if self.session.session_id != previous:
            self._clear_session_evidence()

    def feedback_is_fresh(self, now):
        return bool(self.sock.clock.session_id == self.session.session_id
                    and self.sock.clock.ready(now) and self._legacy_feedback_is_fresh(now))

    def safety_tick(self):
        now = time.monotonic()
        clock = self.sock.clock
        if clock.fault and not self._startup_blocked:
            self._block_startup(clock.fault)
        if self.session.connected:
            self._startup_connected_session = self.session.session_id
        if self.user_arm_seen and not clock.frozen and not self._startup_blocked:
            try:
                clock.freeze(now)
            except OSError:
                self._block_startup('ARM_WITHOUT_STARTUP_CLOCK_PROOF')
        return self._legacy_safety_tick()

    def tick(self):
        if self._startup_blocked or self.sock.clock.fault:
            # Continue receiving native diagnostics for evidence, never attempt
            # a new session, a clear, or even a new zero stream into a bad anchor.
            self.drain_receive()
            self.safety_tick()
            self.publish_link_state()
            return
        return self._legacy_tick()
