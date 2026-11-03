"""Reuse the accepted virtual plant and whole-footprint safety loop."""
from dataclasses import replace
import math

from bootstrap import HARNESS
from rotation_core import RotationSupervisor,RotationFinalGate,Pose,Velocity,ZERO,wheel_rpm


class RotationHarness(HARNESS.Harness):
    def __init__(self,c,*,origin_local=(-.5,.05),lag_s=.12,feedforward=True):
        super().__init__(c,origin_local=origin_local,lag_s=lag_s)
        self.sup=RotationSupervisor(c,feedforward);self.gate=RotationFinalGate(c)
        self.hand_origin=Pose(10.,10.,1.,.4);self.hand=self.hand_origin
        self.car_wrap=False
    def set_gesture(self,forward,left,turn):
        self.set_hand(forward,left,yaw=self.hand_origin.yaw+turn)
    def step(self,*a,**kw):
        # Simulate the real Vicon +/-pi representation without changing the
        # physical plant angle or substituting command for pose feedback.
        if self.car_wrap:
            self.plant.pose=replace(self.plant.pose,yaw=math.atan2(math.sin(self.plant.pose.yaw),math.cos(self.plant.pose.yaw)))
        measured_yaw=self.plant.pose.yaw;measured_wz=self.plant.velocity.wz
        output=super().step(*a,**kw)
        row=self.rows[-1];f=self.sup.follower
        row.update(car_yaw=measured_yaw,plant_wz=measured_wz,goal_yaw=f.goal.yaw if f.goal else None,
            target_yaw=f.mapping.car_origin.yaw+f.mapping.turn if f.mapping else None,
            reference_wand_yaw=self.hand.yaw,wheel_rpm=list(wheel_rpm(output)))
        return output
