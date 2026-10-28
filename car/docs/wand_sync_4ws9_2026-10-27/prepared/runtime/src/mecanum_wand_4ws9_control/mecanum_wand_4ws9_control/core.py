"""ROS-independent relative-position mapping and bounded planar follower.

All positions are meters in one fixed world frame. The engage-time car yaw
defines forward/left; wand orientation never defines the command frame.
"""

from dataclasses import dataclass
import math

from mecanum_vicon_control.math_utils import clamp, wrap_angle, world_error_to_body


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0

    def valid(self):
        return all(math.isfinite(v) for v in (self.x, self.y, self.z, self.yaw))


@dataclass(frozen=True)
class Velocity:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class Config:
    scale: float = 1.0
    input_timeout_s: float = 0.20
    max_speed_mps: float = 0.15
    max_accel_mps2: float = 0.30
    max_wheel_rpm: float = 400.0
    max_yaw_rate_radps: float = 0.50
    max_yaw_accel_radps2: float = 1.0
    position_tolerance_m: float = 0.015
    yaw_tolerance_rad: float = 0.025
    filter_tau_s: float = 0.08
    wand_deadband_m: float = 0.002
    wand_jump_m: float = 0.25
    max_goal_radius_m: float = 2.0
    kp_position: float = 1.4
    kp_yaw: float = 1.5
    governed_targets: bool = False
    reference_lead_m: float = .20
    progress_timeout_s: float = 3.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'governed_targets':
                if type(value) is not bool:
                    raise ValueError('governed_targets must be boolean')
                continue
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
            if value == 0 and name not in ('filter_tau_s', 'wand_deadband_m'):
                raise ValueError(f'{name} must be positive')


@dataclass(frozen=True)
class RelativeMapping:
    car_origin: Pose
    wand_origin: Pose
    scale: float = 1.0

    def local_displacement(self, wand):
        return world_error_to_body(
            wand.x - self.wand_origin.x,
            wand.y - self.wand_origin.y,
            self.car_origin.yaw)

    def goal_from_local(self, forward, left):
        c, s = math.cos(self.car_origin.yaw), math.sin(self.car_origin.yaw)
        return Pose(
            self.car_origin.x + self.scale * (c * forward - s * left),
            self.car_origin.y + self.scale * (s * forward + c * left),
            self.car_origin.z, self.car_origin.yaw)

    def goal(self, wand):
        return self.goal_from_local(*self.local_displacement(wand))


def limit_vector(x, y, maximum):
    norm = math.hypot(x, y)
    factor = min(1.0, maximum / norm) if norm > 0 else 1.0
    return x * factor, y * factor


class WandSession:
    def __init__(self, config=None, goal_projector=None, reference_projector=None):
        self.config = config or Config()
        self.goal_projector = goal_projector
        self.reference_projector = reference_projector or goal_projector
        self.goal_limited = False
        self.raw_goal = None
        self.state = 'STANDBY'
        self.reason = 'not engaged'
        self.samples = {}
        self.mapping = None
        self.goal = None
        self.accepted_local = (0.0, 0.0)
        self.filtered_local = (0.0, 0.0)
        self.command = Velocity()
        self.last_tick = None
        self.mapped_goal = None
        self.reference_velocity = (0., 0.)
        self.reference_limited = False
        self.progress_origin = None
        self.progress_elapsed = 0.

    def invalidate(self, source, reason):
        self.samples.pop(source, None)
        if self.state == 'ACTIVE':
            self._fault(f'{source}: {reason}')

    def observe(self, source, pose, stamp):
        if source not in ('wand', 'car'):
            raise ValueError('source must be wand or car')
        if not pose.valid() or not math.isfinite(stamp):
            self.invalidate(source, 'invalid sample')
            return False
        old = self.samples.get(source)
        # 重复帧和倒序帧不能续期，防止旧位姿被当成新反馈。
        if old is not None and stamp <= old[1]:
            return False
        self.samples[source] = (pose, stamp)
        if source == 'wand' and old and self.state == 'ACTIVE':
            if math.hypot(pose.x - old[0].x, pose.y - old[0].y) > self.config.wand_jump_m:
                self._fault('wand position jump')
        return True

    def fresh(self, now):
        return all(
            source in self.samples
            and 0 <= now - self.samples[source][1] <= self.config.input_timeout_s
            for source in ('wand', 'car'))

    def engage(self, now):
        if not self.fresh(now):
            return False, 'fresh car and wand poses are required'
        # 每次启动/恢复均锚定实测位置，不沿用暂停前尚未到达的目标。
        self.mapping = RelativeMapping(
            self.samples['car'][0], self.samples['wand'][0], self.config.scale)
        self.goal = self.mapping.car_origin
        self.accepted_local = self.filtered_local = (0.0, 0.0)
        self.command = Velocity()
        self.last_tick = now
        self.goal_limited, self.raw_goal = False, self.goal
        self.mapped_goal = self.goal
        self.reference_velocity = (0., 0.)
        self.reference_limited = False
        self.progress_origin, self.progress_elapsed = self.goal, 0.
        self.state, self.reason = 'ACTIVE', ''
        return True, 'engaged; origins captured from current measured poses'

    def pause(self, stop=False):
        self.state = 'STANDBY' if stop else 'PAUSED'
        self.reason = 'operator stop' if stop else 'operator pause'
        self.command = Velocity()
        self.goal = None
        self.mapped_goal = None
        self.reference_velocity = (0., 0.)
        self.reference_limited = False
        self.progress_origin, self.progress_elapsed = None, 0.

    def _fault(self, reason):
        self.state, self.reason = 'FAULT', reason
        self.command = Velocity()
        self.goal = self.mapped_goal = None
        self.reference_velocity = (0., 0.)
        self.progress_origin, self.progress_elapsed = None, 0.

    def advance_reference(self, dt, car):
        """World-fixed slew reference, never reanchored to a displaced car.

        A distant 2x target is preserved. Holding the reference when the car
        lags cannot hide a large actual tracking error or indefinite no motion.
        Only the explicit engage operation captures a new mapping origin.
        """
        dx, dy = self.mapped_goal.x-self.goal.x, self.mapped_goal.y-self.goal.y
        distance = math.hypot(dx, dy)
        acceleration = .5*self.config.max_accel_mps2
        speed = min(self.config.max_speed_mps, math.sqrt(2*acceleration*distance))
        desired = limit_vector(dx, dy, speed)
        dv = limit_vector(desired[0]-self.reference_velocity[0],
                          desired[1]-self.reference_velocity[1], acceleration*dt)
        vx, vy = (self.reference_velocity[i]+dv[i] for i in range(2))
        sx, sy = vx*dt, vy*dt
        if distance <= math.hypot(sx, sy):
            sx, sy = dx, dy
            vx = vy = 0.
        candidate = Pose(self.goal.x+sx, self.goal.y+sy, self.goal.z, self.goal.yaw)
        if self.reference_projector:
            candidate = self.reference_projector(candidate, self.mapping.car_origin,
                                             self.config.max_goal_radius_m)
            vx, vy = ((candidate.x-self.goal.x)/dt, (candidate.y-self.goal.y)/dt) if dt else (0., 0.)
        self.reference_limited = math.hypot(candidate.x-car.x, candidate.y-car.y) > self.config.reference_lead_m
        if self.reference_limited:
            self.reference_velocity = (0., 0.)
        else:
            self.goal, self.reference_velocity = candidate, (vx, vy)

    def check_progress(self, dt, car):
        # Demand updates do NOT restart this timer. Fresh stationary poses
        # alone are insufficient evidence that a commanded car is progressing.
        if self.progress_origin is None or math.hypot(
                car.x-self.progress_origin.x, car.y-self.progress_origin.y) >= .01:
            self.progress_origin, self.progress_elapsed = car, 0.
        demand = (math.hypot(self.command.vx, self.command.vy) > .005
                  and math.hypot(self.mapped_goal.x-car.x, self.mapped_goal.y-car.y)
                  > 2*self.config.position_tolerance_m)
        if demand:
            self.progress_elapsed += dt
        elif math.hypot(self.command.vx, self.command.vy) <= .005:
            self.progress_origin, self.progress_elapsed = car, 0.
        if self.progress_elapsed >= self.config.progress_timeout_s:
            self._fault('CAR_PROGRESS_TIMEOUT')

    def tick(self, now):
        if self.state != 'ACTIVE':
            self.command = Velocity()
            return self.command
        if not math.isfinite(now) or self.last_tick is None or now < self.last_tick:
            self._fault('control clock moved backwards')
            return self.command
        if not self.fresh(now):
            ages = {source: round(now-self.samples[source][1], 6)
                    if source in self.samples else None for source in ('car', 'wand')}
            self._fault(f'car or wand pose timeout: source_age_s={ages}')
            return self.command
        dt = min(0.10, now - self.last_tick)
        self.last_tick = now
        raw = self.mapping.local_displacement(self.samples['wand'][0])
        self.raw_goal = self.mapping.goal_from_local(*raw)
        self.goal_limited = False
        if not self.raw_goal.valid():
            self._fault('nonfinite mapped target')
            return self.command
        if self.goal_projector is not None:
            # Explicit bounded-target mode only. Never clip/re-stamp the raw
            # wand observation: loss/jump/finite checks remain independent.
            target = self.goal_projector(self.raw_goal, self.mapping.car_origin,
                                         self.config.max_goal_radius_m)
            if not target.valid():
                self._fault('nonfinite projected target')
                return self.command
            self.goal_limited = math.hypot(target.x-self.raw_goal.x, target.y-self.raw_goal.y) > 1e-9
            bx, by = world_error_to_body(target.x-self.mapping.car_origin.x,
                                        target.y-self.mapping.car_origin.y, self.mapping.car_origin.yaw)
            raw = (bx/self.config.scale, by/self.config.scale)
        if math.hypot(*raw) * self.config.scale > self.config.max_goal_radius_m:
            self._fault('mapped target outside workspace')
            return self.command
        if math.hypot(*raw) <= self.config.wand_deadband_m:
            self.accepted_local = (0.0, 0.0)
        elif math.dist(raw, self.accepted_local) > self.config.wand_deadband_m:
            self.accepted_local = raw
        alpha = (1.0 if self.config.filter_tau_s == 0 else
                 -math.expm1(-dt / self.config.filter_tau_s))
        self.filtered_local = tuple(
            f + alpha * (r - f)
            for f, r in zip(self.filtered_local, self.accepted_local))
        self.mapped_goal = self.mapping.goal_from_local(*self.filtered_local)
        if self.goal_projector is not None:
            self.mapped_goal = self.goal_projector(self.mapped_goal, self.mapping.car_origin,
                                                   self.config.max_goal_radius_m)
        car = self.samples['car'][0]
        if self.config.governed_targets:
            self.advance_reference(dt, car)
        else:
            self.goal = self.mapped_goal
        ex, ey = self.goal.x - car.x, self.goal.y - car.y
        bx, by = world_error_to_body(ex, ey, car.yaw)
        feedforward = (world_error_to_body(*self.reference_velocity, car.yaw)
                       if self.config.governed_targets else (0., 0.))
        if math.hypot(ex, ey) <= self.config.position_tolerance_m and not any(feedforward):
            tx, ty = 0.0, 0.0
        else:
            tx, ty = self.translation_target(bx, by, feedforward)
        yaw_error = wrap_angle(self.goal.yaw - car.yaw)
        tw = (0.0 if abs(yaw_error) <= self.config.yaw_tolerance_rad else
              clamp(self.config.kp_yaw * yaw_error,
                    -self.config.max_yaw_rate_radps, self.config.max_yaw_rate_radps))
        # 对合速度、合加速度限幅，斜走不会变成单轴上限的sqrt(2)倍。
        dx, dy = limit_vector(
            tx - self.command.vx, ty - self.command.vy,
            self.config.max_accel_mps2 * dt)
        dw = clamp(tw - self.command.wz,
                   -self.config.max_yaw_accel_radps2 * dt,
                   self.config.max_yaw_accel_radps2 * dt)
        self.command = Velocity(
            self.command.vx + dx, self.command.vy + dy, self.command.wz + dw)
        if self.config.governed_targets:
            self.check_progress(dt, car)
        return self.command
