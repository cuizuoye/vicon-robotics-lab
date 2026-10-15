"""Input-supervised OFFLINE preview; no device, transport or flight adapter.

An observed HOME press/release before explicit start checks response only. It
does not clear a real FC output lock. Idle evdev silence is not a sleep signal.
The caller must report disconnects, device replacement and input queue loss.
"""
from dataclasses import replace
import math

from .mission_preview import MissionPreview


class GuardedPreview:
    def __init__(self, mission=None, response_max_age_s=30.):
        if not math.isfinite(response_max_age_s) or response_max_age_s <= 0:
            raise ValueError('Finite positive response age required')
        self.mission = mission if mission is not None else MissionPreview()
        self.response_max_age_s = response_max_age_s
        self.device = None
        self.connected = False
        self.pressed = False
        self.response_at = None
        self.last_t = None
        self.fault = None
        self.started = False
        self.stop_latched = False

    def _clock(self, t):
        if not math.isfinite(t) or (self.last_t is not None and t < self.last_t):
            self.input_fault('Invalid/backwards input clock')
            return False
        self.last_t = t
        return True

    def input_fault(self, reason):
        # No clear/reconnect recovery API: review requires a new preview session.
        if self.fault is None:
            self.fault = reason
        self.connected = False
        self.response_at = None

    def observe_device(self, t, device_id, *, connected):
        if not self._clock(t) or self.fault is not None:
            return
        if not connected or not device_id:
            self.input_fault('Gamepad disconnected or unavailable')
        elif self.device is not None and device_id != self.device:
            self.input_fault('Gamepad identity changed')
        else:
            self.device, self.connected = device_id, True

    def home_event(self, t, value):
        if not self._clock(t) or self.fault is not None:
            return
        if not self.connected or value not in (0, 1, 2):
            self.input_fault('Invalid/unavailable gamepad input')
            return
        if value in (1, 2):
            self.pressed = True
            if self.started:
                self.stop_latched = True
        elif self.pressed:
            self.pressed = False
            if not self.started:
                self.response_at = t

    def _observation(self, o):
        self._clock(o.t)
        response_ok = (self.response_at is not None
                       and 0 <= o.t-self.response_at <= self.response_max_age_s)
        # The prestart age is not an idle-event timeout during a mission: no
        # events may simply mean no button changed, not that the pad is asleep.
        available = (self.connected and self.fault is None
                     and (self.started or (response_ok and not self.pressed)))
        return replace(o, emergency_available=bool(o.emergency_available and available),
                       home_pressed=bool(o.home_pressed or self.stop_latched))

    def start(self, o, *, confirmed=False):
        if self.started:
            return False
        accepted = self.mission.start(self._observation(o), confirmed=confirmed)
        self.started = accepted
        return accepted

    def tick(self, o):
        result = self.mission.tick(self._observation(o))
        return {**result, 'input_fault': self.fault,
                'gamepad_connection_observed': self.connected,
                'prestart_response_at_s': self.response_at,
                'sleep_detection_present': False,
                'live_input_adapter_present': False}
