"""Causal XY servo; no ROS, clocks, threads, transport or hardware I/O.

Position is not low-pass filtered. Only the feed-forward velocity uses a
short source-timestamped regression. The inherited final vector acceleration
limiter is the ONLY normal-motion acceleration shaping stage. The world-fixed
reference retains the independently checked speed/lead/arena constraints.
"""
from collections import deque
import math

from .core import Pose, WandSession, limit_vector


VELOCITY_WINDOW_S = .040
VELOCITY_MAX_RECEIPT_AGE_S = .060
MIN_VELOCITY_SPAN_S = .005
VELOCITY_NOISE_SPAN_M = .0006
SERVO_POLICY = {
    'revision': 'continuous_xy_offline_v1',
    'position_filter_tau_s': 0.,
    'velocity_window_s': .040,
    'velocity_max_receipt_age_s': .060,
    'velocity_noise_span_m': .0006,
    'moving_position_gain_per_s': .20,
    'settling_position_gain_per_s': 1.4,
    'motion_hysteresis_s': .080,
    'normal_command_acceleration_limiters': 1,
    'source_pose_prediction': False,
}


class ContinuousSession(WandSession):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if self.config.filter_tau_s != 0:
            raise ValueError('continuous servo must not stack position low-pass filtering')
        self.velocity_samples = deque()
        self.source_stamps = {}
        self.velocity_receipt = -math.inf
        self.motion_until = -math.inf

    def reset_estimator(self):
        self.velocity_samples.clear()
        self.velocity_receipt = -math.inf
        self.motion_until = -math.inf

    def observe(self, source, pose, stamp, *, source_stamp=None):
        # Inputs owns raw timestamp/freshness/jump validation. Never let a
        # repeated source stamp refresh a receipt or seed another derivative.
        sample_stamp = stamp if source_stamp is None else source_stamp
        if not math.isfinite(sample_stamp):
            self.invalidate(source, 'nonfinite source timestamp')
            return False
        if sample_stamp <= self.source_stamps.get(source, -math.inf):
            return False
        accepted = super().observe(source, pose, stamp)
        if not accepted:
            return False
        self.source_stamps[source] = sample_stamp
        if source == 'wand' and self.state == 'ACTIVE':
            self.record_target(pose, sample_stamp, stamp)
        return True

    def record_target(self, wand, source_stamp, received):
        target = self.mapping.goal(wand)
        if self.goal_projector:
            target = self.goal_projector(target, self.mapping.car_origin,
                                         self.config.max_goal_radius_m)
        history = self.velocity_samples
        if history and source_stamp-history[-1][0] > VELOCITY_MAX_RECEIPT_AGE_S:
            history.clear()
        history.append((source_stamp, target.x, target.y))
        while len(history) > 2 and history[0][0] < source_stamp-VELOCITY_WINDOW_S-1e-9:
            history.popleft()
        # Hard memory bound even under a pathological high-rate source.
        while len(history) > 16:
            history.popleft()
        self.velocity_receipt = received

    def engage(self, now):
        result = super().engage(now)
        self.reset_estimator()
        if result[0]:
            self.record_target(self.samples['wand'][0], self.source_stamps['wand'],
                               self.samples['wand'][1])
        return result

    def pause(self, stop=False):
        super().pause(stop)
        self.reset_estimator()

    def _fault(self, reason):
        super()._fault(reason)
        self.reset_estimator()

    def target_velocity(self):
        # No position extrapolation and no source freshness renewal. If a
        # new source sample stops arriving, drop feed-forward before the
        # independent pose timeout stops the whole session.
        h = self.velocity_samples
        age = self.last_tick-self.velocity_receipt
        if age < 0 or age > VELOCITY_MAX_RECEIPT_AGE_S or len(h) < 2:
            return (0., 0.)
        if h[-1][0]-h[0][0] < MIN_VELOCITY_SPAN_S:
            return (0., 0.)
        if math.hypot(max(s[1] for s in h)-min(s[1] for s in h),
                      max(s[2] for s in h)-min(s[2] for s in h)) <= VELOCITY_NOISE_SPAN_M:
            return (0., 0.)
        # Subtract the first epoch before regression, avoiding cancellation
        # when source stamps are Unix seconds rather than relative times.
        ts = [s[0]-h[0][0] for s in h]
        mean = sum(ts)/len(ts)
        denominator = sum((t-mean)**2 for t in ts)
        if denominator <= 0:
            return (0., 0.)
        velocity = [sum((t-mean)*(s[k]-h[0][k]) for t,s in zip(ts,h))/denominator
                    for k in (1, 2)]
        return limit_vector(*velocity, self.config.max_speed_mps)

    def advance_reference(self, dt, car):
        # Rate limit is retained for the independent gate's world-reference
        # continuity check; it adds no second acceleration ramp. The actual
        # command is acceleration-limited once, in WandSession.tick().
        dx, dy = self.mapped_goal.x-self.goal.x, self.mapped_goal.y-self.goal.y
        sx, sy = limit_vector(dx, dy, self.config.max_speed_mps*dt)
        candidate = Pose(self.goal.x+sx, self.goal.y+sy, self.goal.z, self.goal.yaw)
        if self.reference_projector:
            candidate = self.reference_projector(candidate, self.mapping.car_origin,
                                                   self.config.max_goal_radius_m)
        self.reference_limited = math.hypot(candidate.x-car.x, candidate.y-car.y) > self.config.reference_lead_m
        if self.reference_limited:
            self.reference_velocity = (0., 0.)
        else:
            self.goal = candidate
            self.reference_velocity = self.target_velocity()

    def translation_target(self, bx, by, feedforward):
        # While the hand moves, prioritize its velocity over aggressive late
        # position catch-up. Otherwise the old 1.4/s gain plus feed-forward
        # creates excess speed just before a reversal, then the retained
        # .08m/s2 bound prevents stopping it quickly. At rest, restore normal
        # position convergence. Hysteresis avoids toggling at the zero crossing.
        if math.hypot(*feedforward) > .01:
            self.motion_until = self.last_tick+SERVO_POLICY['motion_hysteresis_s']
        # A held-back reference is a catch-up condition, not permission to
        # stay in weak moving feedback indefinitely. Restore normal position
        # gain here so repeated lead-limit hits cannot throttle a large goal.
        moving = not self.reference_limited and self.last_tick < self.motion_until
        gain = (SERVO_POLICY['moving_position_gain_per_s'] if moving
                else self.config.kp_position)
        return limit_vector(feedforward[0]+gain*bx, feedforward[1]+gain*by,
                            self.config.max_speed_mps)

