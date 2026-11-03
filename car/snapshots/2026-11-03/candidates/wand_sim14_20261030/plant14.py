"""Signed-output nominal mathematical plant. No C firmware or motor socket."""
import math
import time
from types import SimpleNamespace as NS
from profile14 import Policy
from rclpy.node import Node
from std_msgs.msg import String
from mecanum_wand_4wsb_control.core import Velocity
from mecanum_wand_4wsb_control.safety_core import ZERO
from mecanum_wand_4wsb_control.safety_nodes import LATEST
from mecanum_wand_4wsb_control.rotation_source import canonical
from mecanum_wand_4wsb_control.rotation_servo import wheel_rpm


class NominalPlant:
    def __init__(self):self.pose=[0.,0.,0.];self.velocity=ZERO
    def step(self,command,dt):
        if not 0<=dt<=.1:raise ValueError('bounded real-time software step required')
        alpha=1-math.exp(-dt/.12)
        old=self.velocity
        self.velocity=Velocity(*(a+alpha*(b-a) for a,b in zip((old.vx,old.vy,old.wz),(command.vx,command.vy,command.wz))))
        v=self.velocity;heading=self.pose[2]
        self.pose[0]+=(math.cos(heading)*v.vx-math.sin(heading)*v.vy)*dt
        self.pose[1]+=(math.sin(heading)*v.vx+math.cos(heading)*v.vy)*dt
        self.pose[2]+=v.wz*dt


class SoftwarePlant(Node):
    def __init__(self,rt,key):
        if not rt.software_only or rt.experiment!='SIM14_HYPOTHETICAL_NO_C_FIRMWARE':raise ValueError('software runtime only')
        super().__init__('safe_native_twin',namespace=rt.namespace,enable_rosout=False);rt.guard(self)
        self.runtime=rt;self.policy=Policy(key);self.model=NominalPlant();self.last=time.monotonic();self.sequence=0
        self.controller=NS(pose=self.model.pose,status=dict(raw_duty=[0]*4))
        self.metrics=dict(peak_command_mps=0.,peak_measured_mps=0.,peak_rpm=0.,nominal_model=True)
        self.feedback=self.create_publisher(String,'plant_feedback',LATEST)
        self.status=self.create_publisher(String,'transport_status',LATEST)
        self.create_subscription(String,'actuator_command',self.on_signed,LATEST)
        self.create_timer(.01,self.tick)
    def on_signed(self,msg):self.policy.receive(msg.data,time.monotonic(),self.get_clock().now().nanoseconds*1e-9)
    def tick(self):
        now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9;dt=min(.1,max(0.,now-self.last));self.last=now
        active=self.policy.active(now);command=self.policy.command if active else ZERO
        self.model.step(command,dt);v=self.model.velocity;self.sequence+=1
        m=self.metrics;m['peak_command_mps']=max(m['peak_command_mps'],math.hypot(command.vx,command.vy))
        m['peak_measured_mps']=max(m['peak_measured_mps'],math.hypot(v.vx,v.vy));m['peak_rpm']=max(m['peak_rpm'],max(map(abs,wheel_rpm(command))))
        self.feedback.publish(String(data=canonical(dict(sequence=self.sequence,issued_steady=now,issued_ros=ros,
            output_zero=command==ZERO,enabled=active,speed=math.hypot(v.vx,v.vy),yaw_rate=abs(v.wz),fault_bits=0))))
        self.status.publish(String(data=canonical(dict(issued_steady=now,issued_ros=ros,fresh=True,ready=not active,
            fault=self.policy.fault,local_stop=False,armed_epoch=self.policy.epoch,baseline_current=True,
            safety_enabled=active,motion_gate_requested=active,command_progress_age_s=0.,
            startup_clock=dict(ready=True,qualified=True,fault=''),software_nominal_model=True))))
    def close(self):self.policy.revoke('SOFTWARE_TEST_END')
