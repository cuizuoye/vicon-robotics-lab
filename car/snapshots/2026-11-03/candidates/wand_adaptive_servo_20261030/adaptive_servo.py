"""Project-specific continuous servo, independently implemented from equations.

References: ArduPilot AP_Math/control.cpp sqrt-shaped correction + velocity
feedforward; Nav2 velocity smoother's acceleration/deceleration separation.
This is NOT an imported flight controller, Ruckig Pro, or a stability proof.
Raw 1:1 target is never projected, slowed, predicted or moved ahead of Sign.
"""
from dataclasses import dataclass,replace
import math

if __package__:
    from .direct_base import DirectSession
    from .continuous_servo import ContinuousSession
    from .core import Velocity,limit_vector
    from .rotation_servo import coupled_shape
else:
    from mecanum_wand_4wsb_control.direct_target import DirectSession
    from mecanum_wand_4wsb_control.continuous_servo import ContinuousSession
    from mecanum_wand_4wsb_control.core import Velocity,limit_vector
    from mecanum_wand_4wsb_control.rotation_servo import coupled_shape


@dataclass(frozen=True)
class Policy:
    position_gain:float=4.0
    angle_gain:float=4.0
    far_position_gain:float=1.4
    far_angle_gain:float=2.0
    accel_floor:float=1.0
    yaw_accel_floor:float=1.5
    error_full_m:float=.30
    angle_full_rad:float=.50
    hand_full_mps:float=.70
    hand_full_radps:float=1.2
    reversal_response_s:float=.12
    def __post_init__(self):
        if not all(math.isfinite(v) and v>0 for v in vars(self).values()):raise ValueError('finite positive adaptive policy required')
        if self.accel_floor>4 or self.yaw_accel_floor>12:raise ValueError('adaptive floors exceed hard ceiling')


def smooth_level(value,full):
    x=max(0.,min(1.,value/full))
    return x*x*(3.-2.*x)


def correction(error,gain,accel,dt):
    """C1 linear-near / braking-distance-shaped-far correction speed.

    Transition distance a/k² makes both value and first derivative continuous.
    This is a demand calculation, not a second trajectory/reference generator.
    """
    if not all(math.isfinite(x) for x in (error,gain,accel,dt)) or gain<=0 or accel<=0 or dt<=0:
        raise ValueError('invalid correction input')
    d=abs(error);join=accel/(gain*gain)
    magnitude=(gain*d if d<=join else math.sqrt(2.*accel*(d-.5*join)))
    return math.copysign(min(magnitude,d/dt),error)


def acceleration_budget(error,hand_rate,relative_closing,previous_rate,desired_rate,
                        floor,ceiling,error_full,hand_full,response,stop_tolerance):
    if not all(math.isfinite(v) for v in (error,hand_rate,relative_closing,previous_rate,desired_rate,floor,ceiling)):
        raise ValueError('nonfinite acceleration schedule')
    level=max(smooth_level(abs(error),error_full),smooth_level(abs(hand_rate),hand_full))
    normal=floor+(ceiling-floor)*level
    # Nearness must NOT force gentle braking when that would overshoot. This
    # estimate uses commanded relative motion, not an invented fresh sensor.
    braking=(relative_closing*relative_closing/(2.*max(abs(error)-stop_tolerance,.005))
             if relative_closing>0 else 0.)
    reversing=(abs(previous_rate-desired_rate)/response if previous_rate*desired_rate<0 else 0.)
    return min(ceiling,max(floor,normal,braking,reversing))


class AdaptiveSession(DirectSession):
    def __init__(self,*args,policy=None,**kwargs):
        self.policy=policy or Policy();self.last_adaptive={}
        super().__init__(*args,**kwargs)

    def translation_target(self,bx,by,feedforward):
        d=math.hypot(bx,by)
        available=(1.-smooth_level(d,self.policy.error_full_m))*(1.-smooth_level(abs(self.goal.yaw-self.car_continuous),self.policy.angle_full_rad))
        gain=self.policy.far_position_gain+(self.policy.position_gain-self.policy.far_position_gain)*available
        rate=correction(d,gain,self.config.max_accel_mps2,self._adaptive_dt) if d else 0.
        self.desired_xy=limit_vector(feedforward[0]+(bx/d*rate if d else 0.),
                                     feedforward[1]+(by/d*rate if d else 0.),self.config.max_speed_mps)
        return self.desired_xy

    def tick(self,now):
        previous=self.command
        dt=now-self.last_tick if self.last_tick is not None else .01
        self._adaptive_dt=max(1e-9,min(.1,dt))
        self.desired_xy=(0.,0.)
        # Same pattern as accepted RotationSession: use base source/mapping,
        # freshness, progress and unshaped demand; discard provisional ramp.
        output=ContinuousSession.tick(self,now)
        if self.state!='ACTIVE':
            self.last_adaptive={};return output
        c,p=self.config,self.policy
        car=self.samples['car'][0];dx,dy=self.goal.x-car.x,self.goal.y-car.y
        cosine,sine=math.cos(car.yaw),math.sin(car.yaw)
        bx,by=cosine*dx+sine*dy,-sine*dx+cosine*dy
        fw=self.reference_velocity
        ff=(cosine*fw[0]+sine*fw[1],-sine*fw[0]+cosine*fw[1])
        error=math.hypot(bx,by)
        closing=((previous.vx-ff[0])*bx+(previous.vy-ff[1])*by)/error if error else 0.
        desired_speed=math.hypot(*self.desired_xy);old_speed=math.hypot(previous.vx,previous.vy)
        dot=previous.vx*self.desired_xy[0]+previous.vy*self.desired_xy[1]
        a=acceleration_budget(error,math.hypot(*ff),closing,old_speed,
            -desired_speed if dot<0 else desired_speed,p.accel_floor,c.max_accel_mps2,
            p.error_full_m,p.hand_full_mps,p.reversal_response_s,c.position_tolerance_m)
        angle=self.goal.yaw-self.car_continuous
        yaw_ff=self.target_yaw_velocity()
        available=(1.-smooth_level(abs(angle),p.angle_full_rad))*(1.-smooth_level(error,p.error_full_m))
        yaw_gain=p.far_angle_gain+(p.angle_gain-p.far_angle_gain)*available
        desired_yaw=(0. if abs(angle)<=c.yaw_tolerance_rad and abs(yaw_ff)<.005 else
            max(-c.max_yaw_rate_radps,min(c.max_yaw_rate_radps,
                yaw_ff+correction(angle,yaw_gain,c.max_yaw_accel_radps2,self._adaptive_dt))))
        angular_closing=math.copysign(1.,angle)*(previous.wz-yaw_ff) if angle else 0.
        aa=acceleration_budget(angle,yaw_ff,angular_closing,previous.wz,desired_yaw,
            p.yaw_accel_floor,c.max_yaw_accel_radps2,p.angle_full_rad,p.hand_full_radps,
            p.reversal_response_s,c.yaw_tolerance_rad)
        # ONE effective final limiter, not a limiter on the base's ramped
        # command. Common scaling preserves XY/yaw direction and wheel budget.
        self.command=coupled_shape(Velocity(*self.desired_xy,desired_yaw),previous,self._adaptive_dt,
                                   replace(c,max_accel_mps2=a,max_yaw_accel_radps2=aa))
        self.last_adaptive=dict(position_error_m=error,angle_error_rad=angle,
            hand_speed_mps=math.hypot(*ff),hand_yaw_radps=yaw_ff,accel_budget_mps2=a,
            yaw_accel_budget_radps2=aa,intermediate_reference=False,hard_ceiling_changed=False)
        if abs(self.command.wz)>.05 and abs(angle)>.05:
            if abs(self.car_continuous-self.yaw_progress_origin)>.025:
                self.yaw_progress_origin=self.car_continuous;self.yaw_progress_elapsed=0.
            else:self.yaw_progress_elapsed+=self._adaptive_dt
            if self.yaw_progress_elapsed>=3.:
                self._fault('ROTATION_NO_PROGRESS');return self.command
        else:self.yaw_progress_origin=self.car_continuous;self.yaw_progress_elapsed=0.
        return self.command
