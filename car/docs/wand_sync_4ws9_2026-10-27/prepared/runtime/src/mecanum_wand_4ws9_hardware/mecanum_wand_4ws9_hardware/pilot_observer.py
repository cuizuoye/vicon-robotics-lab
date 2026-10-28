"""Read-only per-trial ROS evidence. No services, commands, transport or SDK."""
import json
import os
import time
from pathlib import Path
from dataclasses import asdict
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String
from .commissioning import NAMESPACE, runtime_from_environment
from mecanum_wand_4ws9_control.ros_nodes import run_node
from mecanum_wand_4ws9_control.safety_nodes import LATEST, SENSOR


class Observer(Node):
    def __init__(self):
        runtime = runtime_from_environment()
        super().__init__('wand_pilot_observer',namespace=NAMESPACE,enable_rosout=False)
        runtime.guard(self)
        self.file = os.fdopen(os.open(os.environ['WAND_PILOT_TRACE'],
                    os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w')
        for topic in ('supervisor_status','gate_status','source_status','transport_status',
                      'plant_feedback','operator_lease','candidate','actuator_command'):
            self.create_subscription(String,topic,lambda m,t=topic:self.save(t,m.data),LATEST)
        for topic in ('car_pose','wand_pose'):
            self.create_subscription(PoseStamped,topic,lambda m,t=topic:self.pose(t,m),SENSOR)
        self.create_subscription(Twist,'cmd_vel',self.command,LATEST)

    def save(self,topic,data):
        if isinstance(data,str):
            try: data=json.loads(data)
            except ValueError: pass
        self.file.write(json.dumps(dict(received_steady=time.monotonic(),topic=topic,data=data),allow_nan=False)+'\n')
        self.file.flush()

    def pose(self,topic,m):
        p,q=m.pose.position,m.pose.orientation
        self.save(topic,dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            frame=m.header.frame_id,x=p.x,y=p.y,z=p.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w))

    def command(self,m):
        self.save('cmd_vel',dict(vx=m.linear.x,vy=m.linear.y,wz=m.angular.z))

    def destroy_node(self):
        try:self.file.close()
        finally:return super().destroy_node()


def main():run_node(Observer)
