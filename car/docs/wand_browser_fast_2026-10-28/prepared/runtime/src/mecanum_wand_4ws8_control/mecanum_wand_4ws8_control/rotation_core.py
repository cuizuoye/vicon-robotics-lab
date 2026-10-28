"""Hardware-envelope adapters for the pinned, separately ported rotation servo."""
from dataclasses import dataclass, replace
import math
from .core import Pose
from .staged_config import StagedSafetyConfig
from .pilot_core import PilotSupervisor as BaseSupervisor, PilotFinalGate as BaseGate
from .rotation_servo import (RotationSession, RotationInputs, wheel_rpm, ANGULAR_LEAD,
    MAX_ANGULAR_TRACKING_ERROR)
from .safety_core import ZERO


@dataclass(frozen=True)
class RotationSafetyConfig(StagedSafetyConfig):
    def project_goal(self,goal,origin,radius):
        p=super().project_goal(goal,origin,radius)
        return replace(p,yaw=goal.yaw)

    def project_reference(self,goal,origin,radius):
        g=self.geometry();x,y=g.local(goal.x,goal.y)
        xmin,xmax,ymin,ymax=g.center_bounds(goal.yaw)
        x=max(xmin+1e-8,min(xmax-1e-8,x));y=max(ymin+1e-8,min(ymax-1e-8,y))
        wx,wy=g.world(x,y);dx,dy=wx-origin.x,wy-origin.y;d=math.hypot(dx,dy)
        ratio=min(1.,(radius-1e-8)/d) if d else 1.
        return Pose(origin.x+ratio*dx,origin.y+ratio*dy,origin.z,goal.yaw)


class RotationSupervisor(BaseSupervisor):
    def __init__(self,c):
        super().__init__(c)
        self.inputs=RotationInputs(c)
        self.follower=RotationSession(c.follower_config(),c.project_goal,c.project_reference)

    def tick(self,steady,ros):
        frame=super().tick(steady,ros)
        if self.state=='ACTIVE' and abs(self.follower.goal.yaw-self.follower.car_continuous)>MAX_ANGULAR_TRACKING_ERROR:
            self.request_stop(steady,'ROTATION_TRACKING_ERROR',fault=True)
            return self.frame(steady,ros)
        return frame


class RotationFinalGate(BaseGate):
    def __init__(self,c):
        super().__init__(c);self.inputs=RotationInputs(c)
        self.angle_epoch='';self.car_anchor=self.wand_anchor=None

    def reference_valid(self,frame):
        if not super().reference_valid(frame):return False
        if self.reference_sample is None:
            return abs(math.atan2(math.sin(frame.goal.yaw-self.inputs.poses['car']['pose'].yaw),
                math.cos(frame.goal.yaw-self.inputs.poses['car']['pose'].yaw)))<=.025
        goal,stamp=self.reference_sample;dt=frame.issued_steady-stamp
        if dt<0 or abs(frame.goal.yaw-goal.yaw)>self.c.max_yaw_rate_radps*dt+1e-7:return False
        if self.car_anchor is None or self.wand_anchor is None:return False
        car0,yaw0=self.car_anchor
        intended=yaw0+self.inputs.wand_continuous-self.wand_anchor
        measured=yaw0+self.inputs.car_continuous-car0
        step=max(-self.c.max_yaw_rate_radps*dt,min(self.c.max_yaw_rate_radps*dt,intended-goal.yaw))
        candidate=goal.yaw+step
        expected=goal.yaw if abs(candidate-measured)>ANGULAR_LEAD else candidate
        return abs(frame.goal.yaw-expected)<=1e-6

    def tick(self,steady,ros):
        output=super().tick(steady,ros);f=self.frame
        if f and f.enabled and f.state=='ACTIVE' and not self.fault and not self.hard_fault:
            if self.angle_epoch!=f.epoch:
                if self.angle_epoch or output!=ZERO or f.command!=ZERO:
                    self.trip('ROTATION_NEUTRAL_CAPTURE_OR_REUSE');return ZERO
                self.angle_epoch=f.epoch
                self.car_anchor=(self.inputs.car_continuous,f.goal.yaw)
                self.wand_anchor=self.inputs.wand_continuous
            car0,yaw0=self.car_anchor
            if abs(f.goal.yaw-(yaw0+self.inputs.car_continuous-car0))>MAX_ANGULAR_TRACKING_ERROR:
                self.trip('ROTATION_TRACKING_ERROR');return ZERO
            if max(abs(v) for v in wheel_rpm(f.command))>400.+1e-6:
                self.trip('ROTATION_WHEEL_SPEED_LIMIT');return ZERO
        return output
