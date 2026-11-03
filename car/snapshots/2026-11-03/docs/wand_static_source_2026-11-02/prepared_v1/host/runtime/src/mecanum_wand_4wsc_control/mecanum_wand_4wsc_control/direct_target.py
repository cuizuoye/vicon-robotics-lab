"""Direct 1:1 source target; no spatial/rate/lead reference governor.

Used only by an independently generated, private software runtime. Physical
command limits are still applied ONCE by RotationSession.coupled_shape and
independently checked by FinalGate. A distant valid hand target is demand,
not by itself evidence of bad tracking. Source jumps and no-progress checks
remain distinct from that removed proxy-distance restriction.
"""
import math
from .core import Pose, limit_vector
from .pilot_core import PilotSupervisor, PilotFinalGate
from .rotation_servo import RotationInputs, RotationSession, wheel_rpm
from .safety_core import ZERO


class DirectSession(RotationSession):
    def advance_reference(self, dt, car):
        # This legacy method name is only an integration hook. There is no
        # second position, spatial lead radius, target slew or angle governor.
        self.goal = self.mapped_goal
        self.reference_velocity = self.target_velocity()
        self.reference_limited = self.yaw_limited = False

    def translation_target(self, bx, by, feedforward):
        self.desired_xy = limit_vector(feedforward[0]+self.config.kp_position*bx,
                                       feedforward[1]+self.config.kp_position*by,
                                       self.config.max_speed_mps)
        return self.desired_xy


class DirectSupervisor(PilotSupervisor):
    def __init__(self, c):
        if not getattr(c, 'direct_sign_target', False):
            raise ValueError('explicit direct-Sign software profile required')
        super().__init__(c)
        self.inputs = RotationInputs(c)
        self.follower = DirectSession(c.follower_config(), c.project_goal, c.project_reference)


class DirectFinalGate(PilotFinalGate):
    """Reconstruct target from the EXACT bound Sign packet, not plausible slew.

The inherited owner/sequence/epoch, clock, input, lease, feedback, standstill,
command speed and acceleration checks remain authoritative. BoundGateMixin
still requires the same raw-packet digest and car timestamp for every frame.
"""
    def __init__(self, c):
        if not getattr(c, 'direct_sign_target', False):
            raise ValueError('explicit direct-Sign software profile required')
        super().__init__(c)
        self.inputs = RotationInputs(c)
        self.direct_epoch = ''
        self.car_origin = self.wand_origin = self.wand_angle_origin = None

    def reference_valid(self, frame):
        if self.direct_epoch != frame.epoch:
            if self.direct_epoch or self.reference_sample is not None or frame.command != ZERO:
                return False
            car = self.inputs.poses['car']['pose']
            return (math.hypot(frame.goal.x-car.x, frame.goal.y-car.y) <= .02
                    and abs(frame.goal.z-car.z) <= self.c.lift_tolerance_m
                    and abs(math.atan2(math.sin(frame.goal.yaw-car.yaw),
                                       math.cos(frame.goal.yaw-car.yaw))) <= .025)
        car, wand = self.car_origin, self.wand_origin
        current = self.inputs.poses['wand']['pose']
        expected = Pose(car.x+current.x-wand.x, car.y+current.y-wand.y, car.z,
                        car.yaw+self.inputs.wand_continuous-self.wand_angle_origin)
        return all(abs(a-b) <= 1e-6 for a, b in zip(
            (frame.goal.x, frame.goal.y, frame.goal.z, frame.goal.yaw),
            (expected.x, expected.y, expected.z, expected.yaw)))

    def tick(self, steady, ros):
        output = super().tick(steady, ros)
        f = self.frame
        if f and f.enabled and f.state == 'ACTIVE' and not self.fault and not self.hard_fault:
            if self.direct_epoch != f.epoch:
                if self.direct_epoch or output != ZERO or f.command != ZERO:
                    self.trip('DIRECT_NEUTRAL_CAPTURE_OR_REUSE')
                    return ZERO
                self.direct_epoch = f.epoch
                self.car_origin = f.goal
                self.wand_origin = self.inputs.poses['wand']['pose']
                self.wand_angle_origin = self.inputs.wand_continuous
            if max(map(abs, wheel_rpm(f.command))) > self.c.limits.target_rpm+1e-6:
                self.trip('DIRECT_WHEEL_SPEED_LIMIT')
                return ZERO
        return output
