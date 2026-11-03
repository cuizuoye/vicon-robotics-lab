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


# Independent direct-target cores; same physical output and source guards.
from .direct_target import DirectSupervisor as RotationSupervisor, DirectFinalGate as RotationFinalGate
