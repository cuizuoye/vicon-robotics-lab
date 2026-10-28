"""Offline-only wand supervisor and independent final gate, with injected clocks.

No ROS, socket, serial or firmware access. Monotonic deadlines and source ROS
timestamps are separate checks. Each ROS process owns its own Inputs instance.
"""

from dataclasses import asdict, dataclass
from .feedback_policy import FEEDBACK_TIMEOUT_S
from collections import deque
import math
import uuid

from mecanum_vicon_control.math_utils import wrap_angle
from .core import Config, Pose, Velocity
from .continuous_servo import ContinuousSession as WandSession


ZERO = Velocity()
STOP_STATES = ('STOPPING', 'STOP_UNCONFIRMED')


@dataclass(frozen=True)
class SafetyConfig:
    max_speed_mps: float = .05
    max_accel_mps2: float = .08
    max_yaw_rate_radps: float = .10
    max_yaw_accel_radps2: float = .40
    pose_timeout_s: float = .20
    lease_timeout_s: float = .15
    command_timeout_s: float = .10
    feedback_timeout_s: float = FEEDBACK_TIMEOUT_S
    ready_window_s: float = .60
    stationary_window_s: float = .50
    stop_confirmation_timeout_s: float = 1.50
    max_control_gap_s: float = .10
    max_clock_skew_s: float = .10
    future_tolerance_s: float = .005
    max_session_s: float = 30.0
    max_goal_radius_m: float = .50
    max_tracking_error_m: float = .65
    ground_z_m: float = .10
    lift_tolerance_m: float = .08
    tilt_limit_rad: float = .25
    car_jump_m: float = .20
    car_jump_yaw_rad: float = .50
    wand_jump_m: float = .25
    arena_x_min: float = -1.50
    arena_x_max: float = 1.50
    arena_y_min: float = -1.00
    arena_y_max: float = 1.00
    half_length_m: float = .18
    half_width_m: float = .15
    boundary_margin_m: float = .20
    goal_boundary_policy: str = 'fault'
    goal_reference_policy: str = 'direct'

    def __post_init__(self):
        numeric = {k: v for k, v in asdict(self).items()
                   if k not in ('goal_boundary_policy', 'goal_reference_policy')}
        if self.goal_boundary_policy not in ('fault', 'clamp'):
            raise ValueError('unknown goal boundary policy')
        if self.goal_reference_policy not in ('direct', 'governed'):
            raise ValueError('unknown goal reference policy')
        if not all(math.isfinite(v) for v in numeric.values()):
            raise ValueError('safety configuration must be finite')
        positive = [k for k in numeric if not k.startswith('arena_') and k != 'ground_z_m']
        if any(getattr(self, k) <= 0 for k in positive):
            raise ValueError('safety bounds must be positive')
        if not (self.max_speed_mps <= .20 and self.max_accel_mps2 <= .30
                and self.max_yaw_rate_radps <= .20 and self.max_yaw_accel_radps2 <= 1.0
                and self.max_session_s <= 140 and self.pose_timeout_s <= .20
                and self.command_timeout_s <= .15 and self.lease_timeout_s <= .30):
            raise ValueError('configuration exceeds offline commissioning hard bounds')
        if not (self.arena_x_min < self.arena_x_max and self.arena_y_min < self.arena_y_max):
            raise ValueError('invalid fixed arena')
        if not self.inside(Pose((self.arena_x_min+self.arena_x_max)/2,
                                (self.arena_y_min+self.arena_y_max)/2)):
            raise ValueError('arena too small for footprint and margin')

    def inside(self, pose):
        """Whole rotated footprint, not just center, in a fixed world rectangle."""
        c, s = abs(math.cos(pose.yaw)), abs(math.sin(pose.yaw))
        ex = c*self.half_length_m+s*self.half_width_m+self.boundary_margin_m
        ey = s*self.half_length_m+c*self.half_width_m+self.boundary_margin_m
        return (self.arena_x_min+ex <= pose.x <= self.arena_x_max-ex
                and self.arena_y_min+ey <= pose.y <= self.arena_y_max-ey)

    def center_bounds(self, yaw):
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        ex = c*self.half_length_m+s*self.half_width_m+self.boundary_margin_m
        ey = s*self.half_length_m+c*self.half_width_m+self.boundary_margin_m
        return (self.arena_x_min+ex, self.arena_x_max-ex,
                self.arena_y_min+ey, self.arena_y_max-ey)

    def project_goal(self, goal, origin, radius):
        # AABB projection first, then a convex interpolation toward the
        # in-bounds session origin for the independent target-radius limit.
        # Insets avoid floating-point roundoff landing outside the exact gate.
        xmin, xmax, ymin, ymax = self.center_bounds(origin.yaw)
        x = max(xmin+1e-8, min(xmax-1e-8, goal.x))
        y = max(ymin+1e-8, min(ymax-1e-8, goal.y))
        dx, dy = x-origin.x, y-origin.y
        distance = math.hypot(dx, dy)
        ratio = min(1., (radius-1e-8)/distance) if distance else 1.
        return Pose(origin.x+ratio*dx, origin.y+ratio*dy, origin.z, origin.yaw)

    def follower_config(self):
        return Config(scale=1., max_speed_mps=self.max_speed_mps,
                      max_accel_mps2=self.max_accel_mps2,
                      max_yaw_rate_radps=self.max_yaw_rate_radps,
                      max_yaw_accel_radps2=self.max_yaw_accel_radps2,
                      input_timeout_s=self.pose_timeout_s,
                      max_goal_radius_m=self.max_goal_radius_m,
                      governed_targets=self.goal_reference_policy == 'governed')


class ClockCheck:
    def __init__(self, config):
        self.c = config
        self.previous = None
        self.fault = ''
        self.idle_gap = False

    def tick(self, steady, ros, allow_idle_gap=False):
        self.idle_gap = False
        if self.fault:
            return self.fault
        if not all(math.isfinite(v) for v in (steady, ros)):
            self.fault = 'CLOCK_NONFINITE'
        elif self.previous is not None:
            ds, dr = steady-self.previous[0], ros-self.previous[1]
            if ds < 0 or dr < -self.c.future_tolerance_s:
                self.fault = 'CLOCK_BACKWARDS'
            elif ds > self.c.max_control_gap_s+1e-9:
                if allow_idle_gap:
                    self.idle_gap = True
                else:
                    self.fault = 'CONTROL_TIMER_GAP'
            elif abs(ds-dr) > self.c.max_clock_skew_s:
                self.fault = 'CLOCK_DISCONTINUITY'
        self.previous = (steady, ros)
        return self.fault


def fresh_envelope(issued_steady, issued_ros, steady, ros, timeout, tolerance=.005):
    return (all(math.isfinite(v) for v in (issued_steady, issued_ros, steady, ros))
            and 0 <= steady-issued_steady <= timeout+1e-9
            and -tolerance <= ros-issued_ros <= timeout+1e-9)


@dataclass(frozen=True)
class Lease:
    source: str
    sequence: int
    issued_steady: float
    issued_ros: float
    held: bool

    def valid(self):
        return (isinstance(self.source, str) and 0 < len(self.source) <= 80
                and type(self.sequence) is int and self.sequence >= 0
                and type(self.held) is bool)


@dataclass(frozen=True)
class Frame:
    source: str
    sequence: int
    issued_steady: float
    issued_ros: float
    epoch: str
    state: str
    enabled: bool
    command: Velocity
    goal: Pose | None
    hard_fault: bool = False


class Inputs:
    def __init__(self, config):
        self.c = config
        self.poses = {}
        self.pose_errors = {}
        self.lease = None
        self.lease_error = ''
        self.feedback = None
        self.feedback_error = ''
        self.actuator_fault_bits = 0
        self.car_speed = math.inf
        self.car_yaw_rate = math.inf
        self.mode = 'wand'
        self.graph_ok = True  # ROS adapter replaces this before any start.
        self.stationary_since = None
        self.car_motion_history = deque()
        self.event_sequence, self.event_reason = 0, ''
        self.events = []

    def event(self, reason):
        self.event_sequence += 1
        self.event_reason = reason
        self.events.append(reason)
        if len(self.events) > 64: self.events = ['INPUT_EVENT_OVERFLOW']

    def take_events(self):
        events, self.events = self.events, []
        return events

    def observe_pose(self, source, pose, source_ros, steady, ros, roll=0., pitch=0.):
        old = self.poses.get(source)
        error = ''
        if (source not in ('car', 'wand') or not pose.valid()
                or not all(math.isfinite(v) for v in (source_ros, roll, pitch))):
            error = 'POSE_INVALID'
        elif not -self.c.future_tolerance_s <= ros-source_ros <= self.c.pose_timeout_s:
            error = 'POSE_TIMESTAMP'
        elif old and source_ros <= old['source_ros']:
            # Replays do not refresh either the source or monotonic deadline.
            return False
        elif source == 'car' and (abs(pose.z-self.c.ground_z_m) > self.c.lift_tolerance_m
                or max(abs(roll), abs(pitch)) > self.c.tilt_limit_rad):
            error = 'CAR_LIFT_OR_TILT'
        elif old and (math.hypot(pose.x-old['pose'].x, pose.y-old['pose'].y)
                      > (self.c.car_jump_m if source == 'car' else self.c.wand_jump_m)
                      or (source == 'car' and abs(wrap_angle(pose.yaw-old['pose'].yaw))
                          > self.c.car_jump_yaw_rad)):
            error = source.upper()+'_POSE_JUMP'
        if error:
            self.event(error)
            self.pose_errors[source] = error
            self.poses.pop(source, None)
            if source == 'car':
                self.stationary_since = None
                self.car_motion_history.clear()
                self.car_speed = self.car_yaw_rate = math.inf
            return False
        if source == 'car':
            self.observe_car_motion(pose, source_ros)
        self.poses[source] = {'pose': pose, 'source_ros': min(source_ros, ros), 'received': steady}
        self.pose_errors.pop(source, None)
        return True

    def observe_car_motion(self, pose, source_ros):
        # This estimator is ONLY for confirming standstill. The safety path
        # still checks every raw pose for freshness/lift/tilt/jump/boundary.
        # Differentiating adjacent receipt times amplified Vicon yaw jitter
        # and ROS callback batching into false rotation. Use source time and
        # the configured 0.50s standstill window; include intermediate points so an out-and-
        # back excursion cannot disappear as it would with endpoint averaging.
        history = self.car_motion_history
        if history and source_ros-history[-1][0] > self.c.pose_timeout_s:
            history.clear()
            self.stationary_since = None
        history.append((source_ros, pose))
        window = self.c.stationary_window_s
        cutoff = source_ros-window
        while len(history)>2 and history[1][0] <= cutoff+1e-9:
            history.popleft()
        span = source_ros-history[0][0]
        if span < window-1e-9:
            self.car_speed = self.car_yaw_rate = math.inf
            return
        xs, ys, angles = [], [], []
        yaw_origin = history[0][1].yaw
        for _, sample in history:
            xs.append(sample.x)
            ys.append(sample.y)
            angles.append(wrap_angle(sample.yaw-yaw_origin))
        self.car_speed = math.hypot(max(xs)-min(xs), max(ys)-min(ys))/span
        self.car_yaw_rate = (max(angles)-min(angles))/span

    def observe_lease(self, lease, steady, ros):
        if (not lease.valid() or not fresh_envelope(lease.issued_steady, lease.issued_ros,
                steady, ros, self.c.lease_timeout_s, self.c.future_tolerance_s)):
            self.lease_error = 'OPERATOR_LEASE_INVALID'
            self.event(self.lease_error)
            self.lease = None
            return False
        if self.lease is not None:
            if lease.source != self.lease.source:
                self.lease_error = 'OPERATOR_OWNER_CHANGED'
                self.event(self.lease_error)
                self.lease = None
                return False
            if lease.sequence <= self.lease.sequence:
                return False
            if self.lease.held and not lease.held:
                self.event('OPERATOR_ENABLE_RELEASED')
        self.lease, self.lease_error = lease, ''
        return True

    def observe_feedback(self, sequence, issued_steady, issued_ros, steady, ros,
                         *, output_zero, enabled, speed, yaw_rate, fault_bits=0):
        valid = (type(sequence) is int and sequence >= 0 and type(output_zero) is bool
                 and type(enabled) is bool and type(fault_bits) is int and fault_bits >= 0
                 and all(math.isfinite(v) and v >= 0 for v in (speed, yaw_rate))
                 and fresh_envelope(issued_steady, issued_ros, steady, ros,
                                    self.c.feedback_timeout_s, self.c.future_tolerance_s))
        if not valid:
            self.event('FEEDBACK_INVALID')
            self.feedback, self.feedback_error = None, 'FEEDBACK_INVALID'
            return False
        if self.feedback and sequence <= self.feedback['sequence']:
            return False
        self.feedback = dict(sequence=sequence, issued_steady=issued_steady, issued_ros=issued_ros,
                             output_zero=output_zero, enabled=enabled, speed=speed,
                             yaw_rate=yaw_rate, fault_bits=fault_bits)
        self.actuator_fault_bits |= fault_bits
        self.feedback_error = ''
        return True

    def pose_fresh(self, source, steady, ros):
        p = self.poses.get(source)
        return bool(p and fresh_envelope(p['received'], p['source_ros'], steady, ros,
                                         self.c.pose_timeout_s, self.c.future_tolerance_s))

    def feedback_fresh(self, steady, ros):
        f = self.feedback
        return bool(f and fresh_envelope(f['issued_steady'], f['issued_ros'], steady, ros,
                                         self.c.feedback_timeout_s, self.c.future_tolerance_s))

    def lease_fresh(self, steady, ros):
        l = self.lease
        return bool(l and fresh_envelope(l.issued_steady, l.issued_ros, steady, ros,
                                         self.c.lease_timeout_s, self.c.future_tolerance_s))

    def health(self, steady, ros):
        if not self.graph_ok:
            return 'COMMAND_SOURCE_CONFLICT'
        if self.mode != 'wand':
            return 'CONTROL_MODE_CONFLICT'
        for source in ('car', 'wand'):
            if not self.pose_fresh(source, steady, ros):
                return self.pose_errors.get(source, source.upper()+'_POSE_TIMEOUT')
        if not self.c.inside(self.poses['car']['pose']):
            return 'CAR_OUTSIDE_FIXED_ARENA'
        if not self.feedback_fresh(steady, ros):
            return self.feedback_error or 'FEEDBACK_TIMEOUT'
        if self.actuator_fault_bits:
            return 'ACTUATOR_FAULT'
        if not self.lease_fresh(steady, ros):
            return self.lease_error or 'OPERATOR_LEASE_TIMEOUT'
        return ''

    def update_stationarity(self, steady, ros):
        f = self.feedback
        still = (self.pose_fresh('car', steady, ros) and self.feedback_fresh(steady, ros)
                 and f['output_zero'] and not f['enabled']
                 and f['speed'] <= .005 and f['yaw_rate'] <= .02
                 and self.car_speed <= .005 and self.car_yaw_rate <= .02)
        if not still:
            self.stationary_since = None
        elif self.stationary_since is None:
            self.stationary_since = steady
        return (self.stationary_since is not None
                and steady-self.stationary_since >= self.c.stationary_window_s-1e-9)


class Supervisor:
    def __init__(self, config=None):
        self.c = config or SafetyConfig()
        self.inputs = Inputs(self.c)
        self.clock = ClockCheck(self.c)
        self.follower = WandSession(self.c.follower_config(),
            self.c.project_goal if self.c.goal_boundary_policy == 'clamp' else None,
            reference_projector=getattr(self.c, 'project_reference', None))
        self.owner = uuid.uuid4().hex
        self.sequence = 0
        self.epoch = ''
        self.state, self.reason = 'STANDBY', 'explicit start required'
        self.fault_latched = self.hard_fault = False
        self.stop_confirmed = False
        self.stop_started = None
        self.ready_since = None
        self.active_started = None
        self.command = ZERO
        self.seen_input_event = 0
        self.gate_report = None
        self.recovery_deadline = None

    def observe_gate(self, *, steady, ros, epoch, fault, hard_fault):
        self.gate_report = dict(steady=steady, ros=ros, epoch=epoch, fault=fault,
                                hard_fault=hard_fault)

    def gate_fresh(self, steady, ros):
        g = self.gate_report
        return bool(g and fresh_envelope(g['steady'], g['ros'], steady, ros,
                                         self.c.command_timeout_s))

    def request_start(self, steady, ros):
        if self.state not in ('STANDBY', 'PAUSED') or self.fault_latched:
            return False, 'not stopped, or fault acknowledgement/restart required'
        healthy = not self.inputs.health(steady, ros)
        if not (healthy and self.inputs.lease and not self.inputs.lease.held
                and self.stop_confirmed and self.ready_since is not None
                and steady-self.ready_since >= self.c.ready_window_s-1e-9):
            return False, 'released fresh enable, stable poses and confirmed standstill required'
        self.state, self.reason = 'ARMED', 'waiting for a new held-enable edge'
        self.epoch, self.active_started = uuid.uuid4().hex, None
        self.recovery_deadline = None
        return True, 'armed; hold to capture current origins; no output yet'

    def request_stop(self, steady, reason='operator stop', fault=False, hard=False):
        self.recovery_deadline = None
        self.command = ZERO
        self.follower.pause(stop=True)
        self.epoch = ''
        self.fault_latched |= fault
        self.hard_fault |= hard
        if self.state not in STOP_STATES and self.state != 'FAULT':
            self.state, self.stop_started = 'STOPPING', steady
            self.stop_confirmed = False
            self.inputs.stationary_since = None
        self.reason = reason
        self.ready_since = None

    def acknowledge(self, steady, ros):
        if self.hard_fault:
            return False, 'hard fault/ESTOP requires a new offline process; never auto-clear'
        if (self.state != 'FAULT' or not self.stop_confirmed or self.inputs.health(steady, ros)
                or not self.inputs.lease or self.inputs.lease.held):
            return False, 'cause must be gone, enable released, and stop confirmed'
        self.fault_latched = False
        self.state, self.reason = 'PAUSED', 'acknowledged; explicit new start required'
        self.ready_since = None
        return True, self.reason

    def request_recover(self, steady, ros):
        """One explicit acknowledgement + reprepare, never a motion resume.

        Retain independent standstill/readiness windows and a new arm epoch.
        A renewed failure cancels the request; data return alone never arms.
        """
        if self.hard_fault or self.inputs.actuator_fault_bits:
            return False, 'hard/actuator fault cannot be recovered by this button'
        if (self.state not in ('FAULT', 'PAUSED') or not self.stop_confirmed
                or self.inputs.health(steady, ros) or not self.inputs.lease
                or self.inputs.lease.held or self.recovery_deadline is not None):
            return False, 'release enable, restore fresh feedback and confirm standstill first'
        if self.state == 'FAULT':
            ok, reason = self.acknowledge(steady, ros)
            if not ok:
                return ok, reason
        self.ready_since = None
        self.recovery_deadline = steady+5.
        self.reason = 'RECOVERY_WAIT_STABLE'
        return True, 'reprepare requested; remain released; new held edge still required'

    def tick(self, steady, ros):
        clock_fault = self.clock.tick(steady, ros, allow_idle_gap=self.state == 'STANDBY' and not self.epoch)
        if self.clock.idle_gap:
            self.ready_since = None
            self.inputs.stationary_since = None
        if clock_fault and not self.hard_fault:
            self.request_stop(steady, clock_fault, fault=True, hard=True)
        if (self.gate_fresh(steady, ros) and self.gate_report['hard_fault']
                and not self.hard_fault):
            self.request_stop(steady, 'FINAL_GATE: '+self.gate_report['fault'], fault=True, hard=True)
        stationary = self.inputs.update_stationarity(steady, ros)
        health = self.inputs.health(steady, ros)
        events = self.inputs.take_events()
        if self.recovery_deadline is not None and (
                health or (self.inputs.lease and self.inputs.lease.held) or events
                or steady >= self.recovery_deadline):
            self.recovery_deadline = None
            self.ready_since = None
            self.reason = 'RECOVERY_CANCELLED'
        if events:
            if self.state in ('ARMED', 'ACTIVE'):
                reason = next((e for e in events if e != 'OPERATOR_ENABLE_RELEASED'), events[0])
                self.request_stop(steady, reason, fault=reason != 'OPERATOR_ENABLE_RELEASED')
            self.seen_input_event = self.inputs.event_sequence
        if self.state in STOP_STATES:
            if stationary:
                self.stop_confirmed = True
                self.state = 'FAULT' if self.fault_latched else 'PAUSED'
            elif steady-self.stop_started >= self.c.stop_confirmation_timeout_s:
                self.state = 'STOP_UNCONFIRMED'
            return self.frame(steady, ros)
        if self.state in ('STANDBY', 'PAUSED', 'FAULT'):
            self.stop_confirmed = stationary
            ready = (not health and stationary and self.inputs.lease
                     and not self.inputs.lease.held and not self.fault_latched)
            if not ready:
                self.ready_since = None
            elif self.ready_since is None:
                self.ready_since = steady
            if (self.recovery_deadline is not None and self.ready_since is not None
                    and steady-self.ready_since >= self.c.ready_window_s-1e-9):
                self.request_start(steady, ros)
            return self.frame(steady, ros)
        if health:
            self.request_stop(steady, health, fault=True, hard=health == 'ACTUATOR_FAULT')
            return self.frame(steady, ros)
        if self.state == 'ARMED':
            if self.inputs.lease.held:
                if not (self.gate_fresh(steady, ros) and not self.gate_report['fault']
                        and self.gate_report['epoch'] == self.epoch):
                    self.request_stop(steady, 'ENABLE_BEFORE_GATE_READY', fault=True)
                elif not stationary:
                    self.request_stop(steady, 'NOT_STATIONARY_AT_ENGAGE', fault=True)
                else:
                    for source, value in self.inputs.poses.items():
                        self.follower.observe(source, value['pose'], value['received'], source_stamp=value['source_ros'])
                    # Source ROS age was independently checked by Inputs;
                    # control integration itself uses only steady time.
                    ok, reason = self.follower.engage(steady)
                    if ok:
                        self.state, self.reason, self.active_started = 'ACTIVE', '', steady
                    else:
                        self.request_stop(steady, reason, fault=True)
            return self.frame(steady, ros)
        if not self.gate_fresh(steady, ros):
            self.request_stop(steady, 'FINAL_GATE_STATUS_TIMEOUT', fault=True)
        elif self.gate_report['fault'] or self.gate_report['hard_fault']:
            self.request_stop(steady, 'FINAL_GATE: '+self.gate_report['fault'],
                              fault=True, hard=self.gate_report['hard_fault'])
        elif not self.inputs.lease.held:
            self.request_stop(steady, 'operator enable released')
        elif steady-self.active_started >= self.c.max_session_s:
            self.request_stop(steady, 'SESSION_TIME_LIMIT', fault=True)
        else:
            for source, value in self.inputs.poses.items():
                self.follower.observe(source, value['pose'], value['received'], source_stamp=value['source_ros'])
            self.command = self.follower.tick(steady)
            goal = self.follower.goal
            if self.follower.state != 'ACTIVE':
                self.request_stop(steady, self.follower.reason, fault=True)
            elif not self.c.inside(goal):
                self.request_stop(steady, 'GOAL_OUTSIDE_FIXED_ARENA', fault=True)
            elif math.hypot(goal.x-self.inputs.poses['car']['pose'].x,
                            goal.y-self.inputs.poses['car']['pose'].y) > self.c.max_tracking_error_m:
                self.request_stop(steady, 'TRACKING_ERROR_LIMIT', fault=True)
        return self.frame(steady, ros)

    def frame(self, steady, ros):
        self.sequence += 1
        enabled = self.state == 'ACTIVE' and not self.fault_latched
        return Frame(self.owner, self.sequence, steady, ros, self.epoch, self.state,
                     enabled, self.command if enabled else ZERO,
                     self.follower.goal if enabled else None, self.hard_fault)


class FinalGate:
    """Independent inputs, timing, lease, source pinning and fail-closed output."""
    def __init__(self, config=None):
        self.c = config or SafetyConfig()
        self.inputs = Inputs(self.c)
        self.clock = ClockCheck(self.c)
        self.frame = None
        self.owner, self.last_sequence = None, -1
        self.primed_epoch = ''
        self.blocked_epoch = ''
        self.fault, self.hard_fault = '', False
        self.output = ZERO
        self.last_applied_stamp = None
        self.seen_input_event = 0
        self.reference_sample = None

    def trip(self, reason, hard=False):
        if not self.hard_fault:
            self.fault = reason
        self.hard_fault |= hard
        if self.frame and self.frame.epoch:
            self.blocked_epoch = self.frame.epoch
        self.primed_epoch = ''
        self.output = ZERO

    def receive(self, frame, steady, ros):
        if (not frame.source or type(frame.sequence) is not int or frame.sequence < 0
                or type(frame.enabled) is not bool or type(frame.hard_fault) is not bool
                or not fresh_envelope(frame.issued_steady, frame.issued_ros, steady, ros,
                                       self.c.command_timeout_s, self.c.future_tolerance_s)):
            self.trip('COMMAND_ENVELOPE_INVALID')
            return False
        if self.owner is None:
            if frame.enabled:
                self.trip('NO_DISABLED_BASELINE')
                return False
            self.owner = frame.source
        if frame.source != self.owner:
            self.trip('COMMAND_OWNER_CHANGED')
            return False
        if frame.sequence <= self.last_sequence:
            return False
        self.last_sequence = frame.sequence
        self.frame = frame
        return True

    def tick(self, steady, ros):
        f = self.frame
        idle = not self.primed_epoch and (f is None or (not f.enabled and f.state == 'STANDBY'))
        timing = self.clock.tick(steady, ros, allow_idle_gap=idle)
        stationary = self.inputs.update_stationarity(steady, ros)
        if timing:
            self.trip(timing, hard=True)
        health = self.inputs.health(steady, ros)
        events = self.inputs.take_events()
        if events:
            if f and f.enabled:
                self.trip(next((e for e in events if e != 'OPERATOR_ENABLE_RELEASED'), events[0]))
            self.seen_input_event = self.inputs.event_sequence
        if self.inputs.actuator_fault_bits:
            self.trip('ACTUATOR_FAULT', hard=True)
        if not f or not fresh_envelope(f.issued_steady, f.issued_ros, steady, ros,
                self.c.command_timeout_s, self.c.future_tolerance_s):
            self.trip('COMMAND_TIMEOUT')
            return self.output
        if f.hard_fault:
            self.trip('SUPERVISOR_HARD_FAULT', hard=True)
        if not f.enabled:
            self.output = ZERO
            self.last_applied_stamp = None
            self.reference_sample = None
            # Only a new explicit arm epoch, at measured standstill and with
            # released live enable, can clear a recoverable gate latch.
            if (not self.hard_fault and f.state == 'ARMED' and f.epoch
                    and f.epoch != self.blocked_epoch and not health and stationary
                    and self.inputs.lease and not self.inputs.lease.held):
                self.primed_epoch, self.fault = f.epoch, ''
            return self.output
        if health:
            self.trip(health, hard=health == 'ACTUATOR_FAULT')
        elif not self.inputs.lease.held:
            self.trip('OPERATOR_ENABLE_RELEASED')
        elif (self.fault or self.hard_fault or f.state != 'ACTIVE'
                or f.epoch != self.primed_epoch or not f.epoch):
            self.output = ZERO
        elif not f.goal or not f.goal.valid() or not self.c.inside(f.goal):
            self.trip('GOAL_OUTSIDE_FIXED_ARENA')
        elif math.hypot(f.goal.x-self.inputs.poses['car']['pose'].x,
                        f.goal.y-self.inputs.poses['car']['pose'].y) > self.c.max_tracking_error_m:
            self.trip('TRACKING_ERROR_LIMIT')
        elif self.c.goal_reference_policy == 'governed' and not self.reference_valid(f):
            self.trip('FINAL_REFERENCE_STEP_LIMIT')
        else:
            values = (f.command.vx, f.command.vy, f.command.wz)
            if (not all(math.isfinite(v) for v in values)
                    or math.hypot(*values[:2]) > self.c.max_speed_mps+1e-9
                    or abs(values[2]) > self.c.max_yaw_rate_radps+1e-9):
                self.trip('FINAL_VECTOR_LIMIT')
            else:
                ds = (f.issued_steady-self.last_applied_stamp
                      if self.last_applied_stamp is not None else .02)
                dv = math.hypot(f.command.vx-self.output.vx, f.command.vy-self.output.vy)
                dw = abs(f.command.wz-self.output.wz)
                if (ds < 0 or dv > self.c.max_accel_mps2*ds+1e-7
                        or dw > self.c.max_yaw_accel_radps2*ds+1e-7):
                    self.trip('FINAL_ACCELERATION_LIMIT')
                else:
                    self.output, self.last_applied_stamp = f.command, f.issued_steady
                    self.reference_sample = (f.goal, f.issued_steady)
        return self.output

    def reference_valid(self, frame):
        if self.reference_sample is None:
            car = self.inputs.poses['car']['pose']
            return math.hypot(frame.goal.x-car.x, frame.goal.y-car.y) <= .02
        goal, stamp = self.reference_sample
        elapsed = frame.issued_steady-stamp
        return (elapsed >= 0 and math.hypot(frame.goal.x-goal.x, frame.goal.y-goal.y)
                <= self.c.max_speed_mps*elapsed+1e-7)
