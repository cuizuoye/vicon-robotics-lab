"""Finite ground observation: prepare is not origin capture.

The original continuous controller and independent FinalGate are unchanged.
The supervisor reuses the mapping's actual engagement poses. The final gate
captures its own fresh wand pose on the first valid, neutral ACTIVE frame;
it shares that frame's car reference and monotonic start time. Thus a rod
picked up while ARMED is not counted as movement during the observation.
No input return, repeated frame, or new arm can rebase a consumed trial.
"""
import math
from .core import Pose
from .safety_core import Supervisor, FinalGate, ZERO, fresh_envelope

from .demo_policy import POLICY, active_limits, metrics


def start_issue(config, inputs):
    value = inputs.poses.get('car')
    if not value or not value['pose'].valid():
        return 'PILOT_START_POSE_REQUIRED'
    car = value['pose']
    goal = config.project_goal(car, car, config.max_goal_radius_m)
    if math.hypot(goal.x-car.x, goal.y-car.y) > .0005:
        return 'PILOT_START_REQUIRES_INTERIOR: 起点在绿色目标区外，请保持松键，人工向区域内部挪几厘米；软件不会自动挪车'
    return ''


def validate_config(config):
    if config.stage != 4 or config.max_speed_mps != .70 or config.max_accel_mps2 != .80:
        raise ValueError('finite first-ground pilot is fixed to stage4 .70/.80')


class PilotMonitor:
    def __init__(self, config):
        self.config = config
        self.epoch = ''
        self.car = self.wand = self.last = None
        self.started = None
        self.path = 0.
        self.used = False
        self.reason = ''

    def arm(self, epoch, inputs):
        if self.used:
            return 'PILOT_CONSUMED_RELAUNCH_REQUIRES_REVIEW'
        issue = start_issue(self.config, inputs)
        if issue:
            return issue
        if not epoch or any(k not in inputs.poses for k in ('car', 'wand')):
            return 'PILOT_ANCHOR_MISSING'
        if any(not inputs.poses[k]['pose'].valid() for k in ('car', 'wand')):
            return 'PILOT_ANCHOR_INVALID'
        # Only bind the explicit arm epoch. Do NOT capture poses, consume the
        # one-shot budget, start the timer, or accumulate path while ARMED.
        if epoch != self.epoch:
            self.epoch, self.reason = epoch, ''
        return ''

    def begin(self, epoch, inputs, steady, ros, started, *, car_origin, wand_origin):
        if self.used or self.started is not None:
            return 'PILOT_CONSUMED_RELAUNCH_REQUIRES_REVIEW'
        if not self.epoch or epoch != self.epoch:
            return 'PILOT_EPOCH_OR_ANCHOR'
        if (not fresh_envelope(started, ros-(steady-started), steady, ros,
                               self.config.command_timeout_s, self.config.future_tolerance_s)
                or inputs.health(steady, ros) or not inputs.lease or not inputs.lease.held):
            return 'PILOT_CAPTURE_INPUTS_NOT_HEALTHY'
        issue = start_issue(self.config, inputs)
        if issue:
            return issue
        if any(not isinstance(p, Pose) or not p.valid() for p in (car_origin, wand_origin)):
            return 'PILOT_ANCHOR_INVALID'
        car = inputs.poses['car']['pose']
        # The car origin is the first zero-reference point. Keep the original
        # FinalGate's 2cm initial-reference check, and independently reject
        # inconsistent height/heading. No arbitrary supervisor rebase.
        dyaw = abs(math.atan2(math.sin(car_origin.yaw-car.yaw), math.cos(car_origin.yaw-car.yaw)))
        if (math.hypot(car_origin.x-car.x, car_origin.y-car.y) > .02
                or abs(car_origin.z-car.z) > self.config.lift_tolerance_m
                or dyaw > self.config.car_jump_yaw_rad):
            return 'PILOT_CAPTURE_CAR_REFERENCE_INVALID'
        self.car, self.wand, self.last = car_origin, wand_origin, car
        self.started, self.path, self.used = started, 0., True
        return ''

    def active(self, epoch, inputs, goal, steady):
        if not self.used or self.started is None or self.car is None or self.wand is None:
            return 'PILOT_CAPTURE_REQUIRED'
        if not self.epoch or epoch != self.epoch:
            return 'PILOT_EPOCH_OR_ANCHOR'
        if self.reason:
            return self.reason
        if not math.isfinite(steady) or not 0 <= steady-self.started < POLICY['max_active_s']:
            return 'PILOT_TIME_LIMIT_120S'
        if any(k not in inputs.poses for k in ('car', 'wand')):
            return 'PILOT_POSE_MISSING'
        car, wand = (inputs.poses[k]['pose'] for k in ('car', 'wand'))
        if any(not isinstance(p, Pose) or not p.valid() for p in (car, wand, goal)):
            return 'PILOT_POSE_OR_GOAL_INVALID'
        d = lambda a,b: math.hypot(a.x-b.x, a.y-b.y)
        self.path += d(car, self.last)
        self.last = car
        return active_limits(self, car, wand, goal)


class PilotSupervisor(Supervisor):
    def __init__(self, config):
        validate_config(config)
        super().__init__(config)
        self.pilot = PilotMonitor(config)

    def request_start(self, steady, ros):
        if self.pilot.used:
            return False, 'PILOT_CONSUMED: one engagement only; no automatic retry'
        issue = start_issue(self.c, self.inputs)
        if issue:
            return False, issue
        return super().request_start(steady, ros)

    def request_recover(self, steady, ros):
        if self.pilot.used:
            return False, 'PILOT_CONSUMED: inspect trial before another launch'
        return super().request_recover(steady, ros)

    def pilot_status(self, steady):
        return dict(used=self.pilot.used, max_active_s=POLICY['max_active_s'],
            remaining_s=max(0., POLICY['max_active_s']-(steady-self.pilot.started)) if self.pilot.started is not None else POLICY['max_active_s'],
            capture_phase='CAPTURED' if self.pilot.used else 'WAIT_HELD_EDGE',
            capture_started_steady=self.pilot.started,
            prepare_blocked=start_issue(self.c,self.inputs) if not self.pilot.used else 'PILOT_CONSUMED',
            **metrics(self.pilot, self.inputs))

    def tick(self, steady, ros):
        previous = self.state
        frame = super().tick(steady, ros)
        reason = ''
        if self.state == 'ARMED' and not self.inputs.health(steady, ros):
            reason = self.pilot.arm(self.epoch, self.inputs)
        elif self.state == 'ACTIVE':
            if not self.pilot.used:
                mapping = self.follower.mapping
                if previous != 'ARMED' or frame.command != ZERO or mapping is None or mapping.scale != 1.:
                    reason = 'PILOT_NEUTRAL_CAPTURE_REQUIRED'
                else:
                    reason = self.pilot.begin(self.epoch, self.inputs, steady, ros,
                        self.active_started, car_origin=mapping.car_origin, wand_origin=mapping.wand_origin)
            if not reason:
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
        self.pilot = PilotMonitor(config)

    def tick(self, steady, ros):
        output = super().tick(steady, ros)
        f = self.frame
        reason = ''
        if f and f.state == 'ARMED' and self.primed_epoch == f.epoch:
            reason = self.pilot.arm(f.epoch, self.inputs)
        elif f and f.enabled and f.state == 'ACTIVE' and not self.fault and not self.hard_fault:
            if not self.pilot.used:
                if f.command != ZERO or output != ZERO or self.primed_epoch != f.epoch:
                    reason = 'PILOT_NEUTRAL_CAPTURE_REQUIRED'
                else:
                    reason = self.pilot.begin(f.epoch, self.inputs, steady, ros,
                        f.issued_steady, car_origin=f.goal, wand_origin=self.inputs.poses['wand']['pose'])
            if not reason:
                reason = self.pilot.active(f.epoch, self.inputs, f.goal, steady)
        if reason:
            self.pilot.reason = reason
            self.trip(reason)  # Revokes the signed permit as well as velocity.
            return ZERO
        return output
