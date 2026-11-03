"""SIM ONLY catch-up candidate, not a hardware authority or launch interface.

Keep the old reference generation, 20cm lead check, source validation and final
coupled acceleration/wheel limiter. Change only the translation velocity target
when the old lead governor has already entered catch-up. No pose prediction.
"""
import math
from common import RotationSession
from mecanum_vicon_control.math_utils import world_error_to_body
from mecanum_wand_4ws8_control.core import limit_vector


class FastCatchupSession(RotationSession):
    exit_distance_m = .10
    catchup_gain_per_s = 3.5
    fast_feedforward_threshold_mps = .55

    def __init__(self,*args,**kwargs):
        self.catchup_active=False
        self.catchup_ticks=0
        self.moving_feedforward_ticks=0
        self.last_target=None
        self.still_since=None
        super().__init__(*args,**kwargs)

    def reset_estimator(self):
        super().reset_estimator()
        self.catchup_active=False
        self.last_target=None
        self.still_since=None

    def record_target(self,wand,source_stamp,received):
        super().record_target(wand,source_stamp,received)
        p=self.mapping.goal(wand)
        if self.last_target is not None:
            stamp,x,y=self.last_target
            dt=source_stamp-stamp
            quiet=dt>0 and math.hypot(p.x-x,p.y-y)/dt<=.03
            if quiet:
                if self.still_since is None:self.still_since=received
            else:
                self.still_since=None
                self.catchup_active=False
        self.last_target=(source_stamp,p.x,p.y)

    def translation_target(self,bx,by,feedforward):
        baseline=super().translation_target(bx,by,feedforward)
        car=self.samples['car'][0]
        dx,dy=self.mapped_goal.x-car.x,self.mapped_goal.y-car.y
        distance=math.hypot(dx,dy)
        rx,ry=self.goal.x-car.x,self.goal.y-car.y
        reference_distance=math.hypot(rx,ry)
        aligned=(reference_distance>.025 and distance>0 and
                 (rx*dx+ry*dy)/(reference_distance*distance)>.90)
        vx,vy=self.target_velocity()
        velocity=math.hypot(vx,vy)
        moving_away=(velocity>self.fast_feedforward_threshold_mps and distance>0 and
                     (vx*dx+vy*dy)/(velocity*distance)>.90)
        if self.reference_limited and aligned and moving_away:
            # Keep genuine fresh, co-directional velocity feed-forward instead
            # of treating the lead governor as a request to crawl at kp*20cm.
            # Do NOT use this when the hand is returning toward the car.
            fx,fy=world_error_to_body(vx,vy,car.yaw)
            self.desired_xy=limit_vector(fx+.20*bx,fy+.20*by,self.config.max_speed_mps)
            self.moving_feedforward_ticks+=1
            self.catchup_active=False
            return self.desired_xy
        quiet_fresh=(self.still_since is not None and self.last_tick-self.still_since>=.12
                     and 0<=self.last_tick-self.velocity_receipt<=.04)
        eligible=aligned and quiet_fresh
        # Do not chase a moving/reversing hand past its governed reference.
        # This mode accelerates RECOVERY after a fast gesture is held still.
        # Overspeed gestures remain physically impossible to mirror in time.
        if not eligible or distance<=self.exit_distance_m:
            self.catchup_active=False
        elif self.reference_limited and distance>self.config.reference_lead_m:
            self.catchup_active=True
        if not self.catchup_active:
            return baseline
        remaining=max(0.,distance-self.config.position_tolerance_m)
        # Bounds normal demand, not a claim of measured physical braking.
        speed=min(self.config.max_speed_mps,self.catchup_gain_per_s*remaining,
                  math.sqrt(2*self.config.max_accel_mps2*remaining))
        vx,vy=world_error_to_body(dx,dy,car.yaw)
        self.desired_xy=(vx*speed/distance,vy*speed/distance) if distance else (0.,0.)
        self.catchup_ticks+=1
        # RotationSession.tick applies the SAME coupled SI acceleration/RPM
        # limits once. Do not feed this directly to a motor or skip FinalGate.
        return self.desired_xy
