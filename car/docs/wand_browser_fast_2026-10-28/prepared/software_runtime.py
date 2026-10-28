"""Real reviewed ROS supervisor/gate; private SOFTWARE plant/source only."""
from dataclasses import replace
import json
import math
import os
import secrets
import time
from types import SimpleNamespace as NS


class SoftwareRig:
    def __init__(self,ready,channel):
        if os.environ.get('ROS_DOMAIN_ID')!='187' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
            raise RuntimeError('Software browser requires PRIVATE local-only DDS187')
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import PoseStamped,Twist
        from std_msgs.msg import String
        from mecanum_wand_4ws8_control.core import Pose
        from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig
        from mecanum_wand_4ws8_control.toggle_ui import ToggleOperatorModel
        from mecanum_wand_4ws8_control.rotation_source import definition
        from mecanum_wand_4ws8_control.bound_history import HISTORY_QOS
        from mecanum_wand_4ws8_control.safety_nodes import LATEST,SENSOR
        from mecanum_wand_4ws8_control.safety_core import ZERO
        from mecanum_wand_4ws8_control.offline_suite import Plant
        from mecanum_wand_4ws8_hardware.rotation_nodes import RotationSupervisorNode,RotationGateNode
        from browser_operator import BrowserOperatorNode
        os.environ['WAND_4WS8_CHANNEL_KEY']=secrets.token_hex(32)
        self.rclpy=rclpy;rclpy.init(args=[])
        c=TrackingSafetyConfig(**json.loads((ready/'runtime/src/mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())['safety'])
        self.config=c;namespace='/browser_software_'+secrets.token_hex(4)
        def guard(node):
            if node.get_namespace()!=namespace or node.context.get_domain_id()!=187:raise RuntimeError('Software DDS escaped')
        runtime=NS(namespace=namespace,world='map',config=c,mode='shadow',guard=guard,
            expected=dict(car_pose='software_browser_source',wand_pose='software_browser_source',
                rotation_packet='software_browser_source',plant_feedback='software_browser_source',
                operator_lease='wand_real_operator',candidate='safe_wand_supervisor',
                cmd_vel='safe_wand_gate',actuator_command='safe_wand_gate'))
        self.source=Node('software_browser_source',namespace=namespace,enable_rosout=False)
        self.operator=BrowserOperatorNode(ToggleOperatorModel(),runtime,channel)
        self.supervisor=RotationSupervisorNode(runtime);self.gate=RotationGateNode(runtime)
        self.executor=SingleThreadedExecutor()
        for node in (self.source,self.supervisor,self.gate):self.executor.add_node(node)
        self.pose_pubs={k:self.source.create_publisher(PoseStamped,k+'_pose',HISTORY_QOS if k=='car' else SENSOR)
            for k in ('car','wand')}
        self.pubs={k:self.source.create_publisher(String,k,HISTORY_QOS if k=='rotation_packet' else LATEST)
            for k in ('rotation_packet','plant_feedback','source_status','mode_claim','transport_status')}
        self.command=ZERO;self.last_fixture=time.monotonic();self.sequence=0
        self.plant=Plant(Pose(12.,-8.,c.ground_z_m,0.));self.definition=definition()
        self.input={'x':0.,'y':0.,'yaw':0.,'visible':True}
        self.requested=dict(self.input)
        self.source.create_subscription(Twist,'cmd_vel',self.on_command,LATEST)
        self.source.create_timer(.02,self.fixture)
        self.callback_stats={}
        for node in (self.source,self.supervisor,self.gate):
            for timer in node.timers:
                cb=timer.callback;key=node.get_name()+'.'+cb.__name__
                record=self.callback_stats.setdefault(key,dict(count=0,max_gap=0.,max_duration=0.,last=None))
                def measured(cb=cb,record=record):
                    now=time.monotonic()
                    if record['last'] is not None:record['max_gap']=max(record['max_gap'],now-record['last'])
                    record['last']=now;record['count']+=1
                    try:return cb()
                    finally:record['max_duration']=max(record['max_duration'],time.monotonic()-now)
                timer.callback=measured

    def on_command(self,msg):
        from mecanum_wand_4ws8_control.core import Velocity
        self.command=Velocity(msg.linear.x,msg.linear.y,msg.angular.z)

    def fixture(self):
        from std_msgs.msg import String
        from mecanum_wand_4ws8_control.safety_nodes import pose_message
        from mecanum_wand_4ws8_control.rotation_source import Packet,Raw,canonical
        from mecanum_wand_4ws8_control.rotation_geometry import REQUIRED,rotation
        from mecanum_wand_4ws8_control.safety_core import ZERO
        import numpy as np
        if self.pose_pubs['car'].get_subscription_count()<3 or self.pubs['rotation_packet'].get_subscription_count()<2:return
        now=time.monotonic();stamp=self.source.get_clock().now().to_msg();ros=stamp.sec+stamp.nanosec*1e-9
        dt=min(.10,max(0.,now-self.last_fixture));self.last_fixture=now;self.plant.step(self.command,dt);self.sequence+=1
        # Software input-device trajectory only. Sliders must not teleport a
        # five-marker body through a jump guard; NO real Sign filtering added.
        for key,rate in (('x',.60),('y',.60),('yaw',1.5)):
            error=self.requested[key]-self.input[key]
            self.input[key]+=max(-rate*dt,min(rate*dt,error))
        self.input['visible']=self.requested['visible']
        p=replace(self.plant.pose,yaw=math.atan2(math.sin(self.plant.pose.yaw),math.cos(self.plant.pose.yaw)))
        self.pose_pubs['car'].publish(pose_message(p,stamp,world='map'))
        yaw=.5+self.input['yaw'];q=(0.,0.,math.sin(yaw/2),math.cos(yaw/2));r=rotation(q)
        point=np.array([30.+self.input['x'],-20.+self.input['y'],1.2])
        raw=point-r@np.array(self.definition['local_points']['Sign5'])
        packet=Packet((Raw('sign',tuple(raw),q,ros,now),*
            (Raw(k,tuple(raw+r@np.array(self.definition['local_points'][k])),None,ros,now) for k in REQUIRED[1:])))
        if self.input['visible']:
            self.pubs['rotation_packet'].publish(String(data=canonical(packet.payload())))
            self.pose_pubs['wand'].publish(pose_message(packet.pose(now,ros),stamp,world='map'))
        def send(k,v):self.pubs[k].publish(String(data=json.dumps(v)))
        send('plant_feedback',dict(sequence=self.sequence,issued_steady=now,issued_ros=ros,
            output_zero=self.command==ZERO,enabled=self.command!=ZERO,
            speed=math.hypot(self.plant.velocity.vx,self.plant.velocity.vy),yaw_rate=abs(self.plant.velocity.wz),fault_bits=0))
        send('source_status',dict(issued_steady=now,issued_ros=ros,rejected={},conflicts=[]))
        self.pubs['mode_claim'].publish(String(data='wand'))
        send('transport_status',dict(issued_steady=now,issued_ros=ros,fresh=True,ready=True,
            startup_reset_ready=False,fault='',local_stop=False,baseline_current=True,
            safety_enabled=False,motion_gate_requested=False,command_progress_age_s=0.,
            startup_clock=dict(ready=True,qualified=True,fault='')))

    def setpoint(self,action):self.requested={k:action[k] for k in ('x','y','yaw','visible')}

    def step(self,now):
        start=time.monotonic()
        for _ in range(64):
            self.executor.spin_once(timeout_sec=0.)
            if time.monotonic()-start>.008:break
        self.operator.step(time.monotonic(),self.setpoint)

    def snapshot(self,now):return self.operator.snapshot(now)

    def close(self):
        from mecanum_wand_4ws8_control.safety_core import ZERO
        self.operator.model.end_trial(time.monotonic(),'Software browser shutdown')
        self.operator.request('stop');self.operator.heartbeat()
        start=time.monotonic();zero_seen=False
        while time.monotonic()-start<1.5:
            self.step(time.monotonic())
            v=self.plant.velocity
            zero_seen=(self.command==ZERO and self.gate.core.output==ZERO
                and math.hypot(v.vx,v.vy)<.003 and abs(v.wz)<.003)
            if zero_seen and time.monotonic()-start>.10:break
            time.sleep(.005)
        evidence=dict(gate_zero_observed=zero_seen,held=self.operator.model.held,
            zero_wait_s=time.monotonic()-start,physical_stop_verified=False)
        self.executor.shutdown();self.operator.close_node()
        for node in (self.gate,self.supervisor,self.source):node.destroy_node()
        self.rclpy.shutdown()
        return evidence
