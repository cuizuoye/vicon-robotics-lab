"""Additional finite-observation envelope, never a continuous-run approval.

Original Supervisor/FinalGate are called first and are not modified. Each
process owns a separate monitor, reading its own independently checked poses.
The bound is deliberately not a commissioning parameter or a ROS override.
"""
import math
from .core import Pose
from .safety_core import Supervisor, FinalGate, ZERO

POLICY = dict(stage=4, max_active_s=2.0, max_hand_xy_m=.15,
              max_goal_xy_m=.15, max_car_xy_m=.20, max_car_path_m=.25,
              max_lateral_m=.10, min_forward_m=-.04, max_heading_rad=.20,
              engagements_per_process=1)


class PilotMonitor:
    def __init__(self):
        self.epoch = ''
        self.car = self.wand = self.last = None
        self.started = None
        self.path = 0.
        self.used = False
        self.reason = ''

    def arm(self, epoch, inputs):
        if self.used:
            return 'PILOT_CONSUMED_RELAUNCH_REQUIRES_REVIEW'
        if self.epoch == epoch:
            return ''
        if not epoch or any(k not in inputs.poses for k in ('car', 'wand')):
            return 'PILOT_ANCHOR_MISSING'
        car, wand = (inputs.poses[k]['pose'] for k in ('car', 'wand'))
        if not car.valid() or not wand.valid():
            return 'PILOT_ANCHOR_INVALID'
        self.epoch, self.car, self.wand, self.last = epoch, car, wand, car
        self.path = 0.
        return ''

    def active(self, epoch, inputs, goal, steady):
        if not self.epoch or epoch != self.epoch or self.car is None:
            return 'PILOT_EPOCH_OR_ANCHOR'
        if self.reason:
            return self.reason
        if self.started is None:
            if self.used:
                return 'PILOT_CONSUMED_RELAUNCH_REQUIRES_REVIEW'
            self.used, self.started = True, steady
        if not math.isfinite(steady) or not 0 <= steady-self.started < POLICY['max_active_s']:
            return 'PILOT_TIME_LIMIT_2S'
        if any(k not in inputs.poses for k in ('car', 'wand')):
            return 'PILOT_POSE_MISSING'
        car, wand = (inputs.poses[k]['pose'] for k in ('car', 'wand'))
        if any(not isinstance(p, Pose) or not p.valid() for p in (car, wand, goal)):
            return 'PILOT_POSE_OR_GOAL_INVALID'
        d = lambda a,b: math.hypot(a.x-b.x, a.y-b.y)
        self.path += d(car, self.last)
        self.last = car
        dx, dy = car.x-self.car.x, car.y-self.car.y
        c, s = math.cos(self.car.yaw), math.sin(self.car.yaw)
        forward, left = c*dx+s*dy, -s*dx+c*dy
        dyaw = abs(math.atan2(math.sin(car.yaw-self.car.yaw), math.cos(car.yaw-self.car.yaw)))
        checks = ((d(wand,self.wand) > POLICY['max_hand_xy_m'], 'PILOT_HAND_LIMIT'),
                  (d(goal,self.car) > POLICY['max_goal_xy_m'], 'PILOT_GOAL_LIMIT'),
                  (d(car,self.car) > POLICY['max_car_xy_m'], 'PILOT_CAR_LIMIT'),
                  (self.path > POLICY['max_car_path_m'], 'PILOT_PATH_LIMIT'),
                  (abs(left) > POLICY['max_lateral_m'], 'PILOT_LATERAL_LIMIT'),
                  (forward < POLICY['min_forward_m'], 'PILOT_REVERSE_LIMIT'),
                  (dyaw > POLICY['max_heading_rad'], 'PILOT_HEADING_LIMIT'))
        return next((reason for failed,reason in checks if failed), '')


def validate_config(config):
    if config.stage != 4 or config.max_speed_mps != .70 or config.max_accel_mps2 != .80:
        raise ValueError('finite first-ground pilot is fixed to stage4 .70/.80')


class PilotSupervisor(Supervisor):
    def __init__(self, config):
        validate_config(config)
        super().__init__(config)
        self.pilot = PilotMonitor()

    def request_start(self, steady, ros):
        if self.pilot.used:
            return False, 'PILOT_CONSUMED: one engagement only; no automatic retry'
        return super().request_start(steady, ros)

    def request_recover(self, steady, ros):
        if self.pilot.used:
            return False, 'PILOT_CONSUMED: inspect trial before another launch'
        return super().request_recover(steady, ros)

    def tick(self, steady, ros):
        frame = super().tick(steady, ros)
        reason = ''
        if self.state == 'ARMED' and not self.inputs.health(steady, ros):
            reason = self.pilot.arm(self.epoch, self.inputs)
        elif self.state == 'ACTIVE':
            reason = self.pilot.active(self.epoch, self.inputs, frame.goal, steady)
        if reason:
            self.pilot.reason = reason
            self.request_stop(steady, reason)
            return self.frame(steady, ros)
        return frame


class PilotFinalGate(FinalGate):
    def __init__(self, config):
        validate_config(config)
        super().__init__(config)
        self.pilot = PilotMonitor()

    def tick(self, steady, ros):
        output = super().tick(steady, ros)
        f = self.frame
        reason = ''
        if f and f.state == 'ARMED' and self.primed_epoch == f.epoch:
            reason = self.pilot.arm(f.epoch, self.inputs)
        elif f and f.enabled and f.state == 'ACTIVE':
            reason = self.pilot.active(f.epoch, self.inputs, f.goal, steady)
        if reason:
            self.pilot.reason = reason
            self.trip(reason)  # Revoke signed permit too, not just the Twist.
            return ZERO
        return output
