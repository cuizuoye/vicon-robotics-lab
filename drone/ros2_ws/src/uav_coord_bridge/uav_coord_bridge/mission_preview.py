"""Transport-free takeoff/hover/land planning preview, in world ENU metres.

Intents are advisory strings only, not flight commands. No ROS, MAVLink, device,
network or arming interfaces. A real-flight adapter is deliberately absent.
"""
from dataclasses import dataclass
from enum import Enum
import math


class State(str, Enum):
    WAIT_START = 'WAIT_START'
    ASCEND = 'ASCEND'
    HOVER = 'HOVER'
    LAND = 'LAND'
    ABORT_LAND = 'ABORT_LAND'
    LANDED = 'LANDED'
    STOP_LATCHED = 'STOP_LATCHED'
    LAND_TIMEOUT = 'LAND_TIMEOUT'


@dataclass(frozen=True)
class Observation:
    t: float
    position: tuple[float, float, float]
    velocity: tuple[float, float, float] = (0., 0., 0.)
    yaw: float = 0.
    pose_age_s: float = 0.
    status_age_s: float = 0.
    position_valid: bool = True
    velocity_valid: bool = True
    health_ok: bool = True
    emergency_available: bool = True
    mapping_valid: bool = True
    armed: bool = False
    landed: bool = True
    home_pressed: bool = False


class MissionPreview:
    def __init__(self, height=.5, hover_seconds=5., climb_timeout=20., land_timeout=30.,
                 tolerance=.05, max_horizontal_excursion=.5, hover_timeout=30.):
        values = (height, hover_seconds, climb_timeout, land_timeout, tolerance, max_horizontal_excursion, hover_timeout)
        if not all(math.isfinite(v) and v > 0 for v in values) or tolerance >= height:
            raise ValueError('Finite positive configuration and tolerance smaller than height required')
        self.height, self.hover_seconds = height, hover_seconds
        self.climb_timeout, self.land_timeout = climb_timeout, land_timeout
        if hover_timeout <= hover_seconds:
            raise ValueError('Hover timeout must exceed stable hold duration')
        self.hover_timeout = hover_timeout
        self.tolerance, self.max_horizontal_excursion = tolerance, max_horizontal_excursion
        self.state = State.WAIT_START
        self.origin = self.target = self.yaw = None
        self.last_t = self.since = self.hover_since = None
        self.reason = 'Explicit start required'
        self.transitions = []

    @staticmethod
    def fresh(o):
        finite = all(math.isfinite(v) for v in (o.t, *o.position, *o.velocity, o.yaw, o.pose_age_s, o.status_age_s))
        return bool(finite and len(o.position) == len(o.velocity) == 3
                    and 0 <= o.pose_age_s <= .15 and 0 <= o.status_age_s <= .5
                    and o.position_valid and o.velocity_valid and o.health_ok
                    and o.emergency_available and o.mapping_valid)

    def change(self, state, o, reason):
        if state != self.state:
            self.transitions.append({'t_s': o.t, 'from': self.state.value, 'to': state.value, 'reason': reason})
            self.state, self.since = state, o.t
        self.reason = reason

    def start(self, o, *, confirmed=False):
        if self.state != State.WAIT_START:
            return False
        if o.home_pressed:
            self.change(State.STOP_LATCHED, o, 'HOME latched; no automatic clear')
            return False
        if not (confirmed and self.fresh(o) and not o.armed and o.landed):
            self.reason = 'Start refused: explicit confirmation and fresh valid disarmed landed input required'
            return False
        self.origin = tuple(o.position)
        self.target = (self.origin[0], self.origin[1], self.origin[2] + self.height)
        self.yaw = o.yaw
        self.last_t = o.t
        self.change(State.ASCEND, o, 'Offline start accepted; freeze world XY and initial yaw')
        return True

    def tick(self, o):
        if o.home_pressed:
            self.change(State.STOP_LATCHED, o, 'HOME latched; no automatic clear')
        if self.state in (State.STOP_LATCHED, State.LANDED, State.WAIT_START):
            return self.result()
        if not math.isfinite(o.t) or (self.last_t is not None and o.t < self.last_t):
            self.change(State.STOP_LATCHED, o, 'Invalid/backwards observation clock; operator review required')
            return self.result()
        self.last_t = o.t
        if self.state in (State.LAND, State.ABORT_LAND, State.LAND_TIMEOUT):
            # Do not infer landing from a timer or issue forced disarm, even after pose loss.
            if o.landed and not o.armed and 0 <= o.status_age_s <= .5:
                self.change(State.LANDED, o, 'Fresh landed AND disarmed observation')
            elif self.state != State.LAND_TIMEOUT and o.t-self.since > self.land_timeout:
                self.change(State.LAND_TIMEOUT, o, 'Landing timeout; keep advisory land request and alert, never force disarm')
            return self.result()
        if not self.fresh(o):
            self.change(State.ABORT_LAND, o, 'Stale/invalid pose, status, health, emergency availability or mapping')
            return self.result()
        if math.hypot(o.position[0]-self.origin[0], o.position[1]-self.origin[1]) > self.max_horizontal_excursion:
            self.change(State.ABORT_LAND, o, 'Horizontal excursion exceeds preview boundary')
            return self.result()
        if self.state == State.ASCEND:
            if o.t-self.since > self.climb_timeout:
                self.change(State.ABORT_LAND, o, 'Climb timeout')
            elif o.armed and math.dist(o.position, self.target) <= self.tolerance and math.dist(o.velocity, (0., 0., 0.)) <= .1:
                self.hover_since = o.t
                self.change(State.HOVER, o, 'Target and low-speed observation reached')
        elif self.state == State.HOVER:
            if o.t-self.since > self.hover_timeout:
                self.change(State.ABORT_LAND, o, 'Stable hover not achieved within bounded window')
            elif not o.armed:
                self.change(State.ABORT_LAND, o, 'Unexpected disarmed state during hover; never auto-rearm')
            elif math.dist(o.position, self.target) > self.tolerance or math.dist(o.velocity, (0., 0., 0.)) > .1:
                self.hover_since = None
            elif self.hover_since is None:
                self.hover_since = o.t
            elif o.t-self.hover_since >= self.hover_seconds:
                self.change(State.LAND, o, 'Continuous stable hover duration reached')
        return self.result()

    def result(self):
        intent = {
            State.WAIT_START: 'preview_wait', State.ASCEND: 'preview_ascend', State.HOVER: 'preview_hold',
            State.LAND: 'preview_land', State.ABORT_LAND: 'preview_land_and_alert',
            State.LAND_TIMEOUT: 'preview_land_and_alert', State.LANDED: 'preview_complete',
            State.STOP_LATCHED: 'preview_stop_latched',
        }[self.state]
        return {'state': self.state.value, 'intent': intent, 'target_world_enu_m': self.target,
                'origin_world_enu_m': self.origin, 'fixed_yaw_rad': self.yaw, 'reason': self.reason,
                'hardware_adapter_present': False, 'actual_sent_to_fc': 0, 'flight_ready': False}
