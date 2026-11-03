"""SIM ONLY parameter experiment; no ROS/device/network initialization.

Composition preserves the pinned accepted config and every guard. Explicit
simulation constructors replace ONLY the historical commissioning-stage
restriction; all supervisor/pilot/final-gate methods remain in use. Higher RPM
is a hypothetical replacement drive, never an approval for the existing car.
"""
from dataclasses import asdict, dataclass, replace
import math
from pathlib import Path
import sys
from types import FunctionType

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0, str(ROOT/'candidates/wand_latency_fast_follow_20261029'))
from common import config as accepted_config, Pose, Velocity, ZERO, protected_hashes, save, digest
from catchup import FastCatchupSession
from benchmark import Harness as AcceptedHarness, Sample, synthetic_cases
from mecanum_wand_4ws8_control.core import Config
from mecanum_wand_4ws8_control.safety_core import Supervisor as SafetySupervisor, FinalGate as SafetyGate
from mecanum_wand_4ws8_control.pilot_core import PilotMonitor, PilotFinalGate
from mecanum_wand_4ws8_control.rotation_core import RotationSupervisor, RotationFinalGate
from mecanum_wand_4ws8_control.rotation_servo import (RotationSession, RotationInputs,
    wheel_rpm, ANGULAR_LEAD, MAX_ANGULAR_TRACKING_ERROR)

SPEEDS = (.50, .70, 1.00, 1.20, 1.50)
ACCELS = (.80, 1.20, 2.00, 3.00, 4.00, 6.00, 8.00, 12.00)
RPMS = (400., 800.)


@dataclass(frozen=True)
class FollowerLimits(Config):
    sim_max_rpm: float = 400.


@dataclass(frozen=True)
class Limits:
    max_speed_mps: float = .70
    max_accel_mps2: float = .80
    sim_max_rpm: float = 400.

    def __post_init__(self):
        for value, choices in ((self.max_speed_mps, SPEEDS), (self.max_accel_mps2, ACCELS),
                               (self.sim_max_rpm, RPMS)):
            if type(value) not in (int, float) or value not in choices:
                raise ValueError('enumerated SIM ONLY profile required')
        object.__setattr__(self, '_accepted', accepted_config())

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)
        return getattr(self._accepted, name)

    def follower_config(self):
        values = asdict(self._accepted.follower_config())
        values.update(max_speed_mps=self.max_speed_mps, max_accel_mps2=self.max_accel_mps2,
                      sim_max_rpm=self.sim_max_rpm)
        return FollowerLimits(**values)

    @property
    def name(self):
        return 'v%.2f_a%.2f_rpm%d' % (self.max_speed_mps, self.max_accel_mps2, self.sim_max_rpm)


def shape(desired, previous, dt, c):
    """Same coupled convex interpolation, explicit simulation RPM envelope."""
    if dt <= 0 or not all(math.isfinite(v) for v in (dt, desired.vx, desired.vy, desired.wz)):
        raise ValueError('finite command and positive control interval required')
    peak = max(abs(v) for v in wheel_rpm(desired))
    speed = math.hypot(desired.vx, desired.vy)
    ratio = min(1., c.sim_max_rpm/peak if peak else 1., c.max_speed_mps/speed if speed else 1.,
                c.max_yaw_rate_radps/abs(desired.wz) if desired.wz else 1.)
    target = Velocity(desired.vx*ratio, desired.vy*ratio, desired.wz*ratio)
    dx, dy, dw = target.vx-previous.vx, target.vy-previous.vy, target.wz-previous.wz
    delta = math.hypot(dx, dy)
    alpha = min(1., c.max_accel_mps2*dt/delta if delta else 1.,
                c.max_yaw_accel_radps2*dt/abs(dw) if dw else 1.)
    return Velocity(previous.vx+alpha*dx, previous.vy+alpha*dy, previous.wz+alpha*dw)


# Copy the EXISTING function's private globals, not its module globals. Its
# body/closure (including super()) are unchanged; only the final shaper is
# replaced. Original RotationSession.tick and coupled_shape remain untouched.
_tick = RotationSession.tick
_globals = dict(_tick.__globals__, coupled_shape=shape)
_simulation_tick = FunctionType(_tick.__code__, _globals, _tick.__name__, _tick.__defaults__, _tick.__closure__)


class Session(FastCatchupSession):
    tick = _simulation_tick


class Supervisor(RotationSupervisor):
    def __init__(self, c):
        if not isinstance(c, Limits):
            raise ValueError('simulation-only config required')
        SafetySupervisor.__init__(self, c)
        self.pilot = PilotMonitor(c)
        self.inputs = RotationInputs(c)
        self.follower = Session(c.follower_config(), c.project_goal, c.project_reference)


class Gate(RotationFinalGate):
    def __init__(self, c):
        if not isinstance(c, Limits):
            raise ValueError('simulation-only config required')
        SafetyGate.__init__(self, c)
        self.pilot = PilotMonitor(c)
        self.inputs = RotationInputs(c)
        self.angle_epoch = ''
        self.car_anchor = self.wand_anchor = None

    def trip(self, reason, hard=False):
        if self.fault == 'ROTATION_SELECTED_SOURCE_INVALID' and not hard:
            reason = self.fault
        return super().trip(reason, hard)

    def tick(self, steady, ros):
        # Original rotation gate checks, ONLY the literal 400 RPM becomes the
        # explicitly named SIM experiment cap. Pose/time/lease/reference checks
        # and neutral epoch capture remain in the unchanged parent methods.
        output = PilotFinalGate.tick(self, steady, ros)
        f = self.frame
        if f and f.enabled and f.state == 'ACTIVE' and not self.fault and not self.hard_fault:
            if self.angle_epoch != f.epoch:
                if self.angle_epoch or output != ZERO or f.command != ZERO:
                    self.trip('ROTATION_NEUTRAL_CAPTURE_OR_REUSE'); return ZERO
                self.angle_epoch = f.epoch
                self.car_anchor = (self.inputs.car_continuous, f.goal.yaw)
                self.wand_anchor = self.inputs.wand_continuous
            car0, yaw0 = self.car_anchor
            if abs(f.goal.yaw-(yaw0+self.inputs.car_continuous-car0)) > MAX_ANGULAR_TRACKING_ERROR:
                self.trip('ROTATION_TRACKING_ERROR'); return ZERO
            if max(abs(v) for v in wheel_rpm(f.command)) > self.c.sim_max_rpm+1e-6:
                self.trip('SIM_ROTATION_WHEEL_SPEED_LIMIT'); return ZERO
        return output


class Harness(AcceptedHarness):
    def __init__(self, limits=Limits(), lag=.12, gains=(.98, .88, .95), origin_yaw=0.):
        super().__init__(True, lag, gains, origin_yaw)
        self.c = limits
        self.sup = Supervisor(limits)
        self.gate = Gate(limits)
