"""Real ROS XY+yaw adapters; all actuator IO remains in the unchanged bridge.

Supervisor and final gate independently validate the SAME selected raw six-topic
Sign frame and original car timestamp. A candidate cannot replace its source by
claiming a plausible angle. Local DDS graph/source checks are not DDS encryption.
"""
from collections import OrderedDict,deque
from dataclasses import asdict
import copy
import json
import math
import time
from geometry_msgs.msg import PointStamped,PoseStamped
from std_msgs.msg import String
from .nodes import RawSource,SignSource,AcceptanceGate,AcceptanceSupervisor
from .commissioning import runtime_from_environment,WORLD
from mecanum_wand_4wsa_control.safety_nodes import (SupervisorNode,SENSOR,LATEST,
    SafetyNode,decode_frame,decode_pose,pose_message)
from mecanum_wand_4wsa_control.rotation_geometry import IDS,REQUIRED
from mecanum_wand_4wsa_control.rotation_source import (Packet,Raw,PacketCache,decode_packet,canonical)
from mecanum_wand_4wsa_control.safety_core import ZERO
from mecanum_wand_4wsa_control.ros_nodes import run_node


class RotationRawSource(RawSource):
    def __init__(self,context):
        from mecanum_wand_4wsa_control.rotation_receiver_guard import verify_existing
        self.provenance=verify_existing()
        super().__init__(context);self.frames={};self.packet=None
        for name in IDS:
            self.create_subscription(PointStamped,'/vicon/Sign/markers/'+name,
                lambda msg,name=name:self.marker(name,msg),SENSOR)

    def raw_sample(self,name,p,q,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
        if msg.header.frame_id!=WORLD or not 0<=msg.header.stamp.nanosec<1_000_000_000:
            raise ValueError('SIGN_RAW_FRAME')
        sample=Raw(name,tuple(p),q,stamp,now)
        self.frames.setdefault(stamp,{})[name]=sample
        while len(self.frames)>32:del self.frames[min(self.frames)]
        frame=self.frames[stamp]
        if all(k in frame for k in REQUIRED):
            packet=Packet(tuple(frame[k] for k in REQUIRED));packet.pose(now,ros)
            if self.packet is None or packet.stamp>self.packet.stamp:self.packet=packet

    def receive(self,source,msg):
        super().receive(source,msg)
        if source!='wand' or self.samples.get(source) is None:return
        try:
            p,q=msg.pose.position,msg.pose.orientation
            self.raw_sample('sign',(p.x,p.y,p.z),(q.x,q.y,q.z,q.w),msg)
        except (ValueError,TypeError,KeyError):self.reject('sign')

    def marker(self,name,msg):
        try:self.raw_sample(name,(msg.point.x,msg.point.y,msg.point.z),None,msg)
        except (ValueError,TypeError,KeyError):self.reject(name)

    def reject(self,name):
        self.packet=None;self.frames.clear()
        self.rejected[name]=self.rejected.get(name,0)+1

    def audit(self):
        conflicts=super().audit()
        for topic in ('/vicon/Sign/Sign','/vicon/car01/car01',
                      *('/vicon/Sign/markers/'+k for k in IDS)):
            pubs=self.get_publishers_info_by_topic(topic)
            if len(pubs)!=1 or pubs[0].node_name!='vicon_client' or pubs[0].node_namespace!='/':
                conflicts.append('source identity '+topic)
        return conflicts


class RotationSignSource(SignSource):
    raw_type=RotationRawSource
    def __init__(self):
        super().__init__()
        # Replace only the raw Sign-origin relay with Sign5/control-heading.
        self.wand_pub=self.pubs.pop('wand')
        self.packet_pub=self.create_publisher(String,'rotation_packet',LATEST)
        self.packet_stamp=None

    def tick(self):
        start=time.monotonic()
        for _ in range(128):
            self.source_executor.spin_once(timeout_sec=0.)
            if time.monotonic()-start>.006:break
        super().tick()
        p=self.raw.packet
        if self.conflicts or p is None or p.stamp==self.packet_stamp:return
        try:
            pose=p.pose(time.monotonic(),self.get_clock().now().nanoseconds*1e-9)
            stamp=self.raw.samples['wand'].header.stamp
            # Construct the header from the coherent packet, never from a newer
            # raw Sign or the relay clock. Both data paths carry the old stamp.
            stamp=copy.deepcopy(stamp);stamp.sec=int(p.stamp)
            stamp.nanosec=round((p.stamp-stamp.sec)*1e9)
            if stamp.nanosec==1_000_000_000:stamp.sec+=1;stamp.nanosec=0
            self.packet_pub.publish(String(data=canonical(p.payload())))
            self.wand_pub.publish(pose_message(pose,stamp,world=WORLD))
            self.packet_stamp=p.stamp
        except (ValueError,TypeError,KeyError):self.raw.reject('geometry')


class RotationSupervisorNode(SupervisorNode):
    def __init__(self,runtime=None):
        super().__init__(runtime or runtime_from_environment())
        self.packet_cache=PacketCache();self.selected=None
        self.create_subscription(String,'rotation_packet',self.on_packet,LATEST)

    def on_pose(self,source,msg):
        if source=='car':super().on_pose(source,msg)

    def on_packet(self,msg):
        now,ros=self.clocks()
        try:
            p=decode_packet(json.loads(msg.data),now,ros)
            if self.packet_cache.add(p,now,ros):
                self.core.inputs.observe_pose('wand',p.pose(now,ros),p.stamp,p.received,ros)
                self.selected=p
        except (ValueError,TypeError,KeyError,AttributeError):
            self.selected=None;self.core.inputs.poses.pop('wand',None)
            self.core.inputs.event('ROTATION_RAW_SOURCE_INVALID')

    def operator_gate_ready(self,steady,ros):
        return AcceptanceSupervisor.operator_gate_ready(self,steady,ros)

    def tick(self):
        # Base tick publishes exactly one candidate and the normal status. A
        # publishing adapter wraps it; no duplicate raw-frame publisher exists.
        super().tick()

    def create_publisher(self,msg_type,topic,*args,**kw):
        pub=super().create_publisher(msg_type,topic,*args,**kw)
        if topic!='candidate':return pub
        node=self
        class BoundPublisher:
            def publish(self,msg):
                f=json.loads(msg.data);p=node.selected
                car=node.core.inputs.poses.get('car')
                msg.data=canonical(dict(frame=f,sign_digest=p.digest if p else None,
                    sign_stamp=p.stamp if p else None,car_stamp=car['source_ros'] if car else None))
                pub.publish(msg)
        return BoundPublisher()


class RotationGateNode(AcceptanceGate):
    def __init__(self,runtime=None):
        # AcceptanceGate's only extra initialization is its unchanged authority
        # signer. Allow an explicit isolated software runtime for the rehearsal.
        if runtime is None:super().__init__()
        else:
            from mecanum_wand_4wsa_control.safety_nodes import GateNode
            GateNode.__init__(self,runtime)
        self.packet_cache=PacketCache();self.cars=OrderedDict();self.pending=deque()
        self.create_subscription(String,'rotation_packet',self.on_packet,LATEST)

    def on_pose(self,source,msg):
        if source!='car':return
        now,ros=self.clocks()
        try:
            pose,stamp,roll,pitch=decode_pose(msg,self.world)
            if not -.005<=ros-stamp<=.20:raise ValueError('CAR_TIMESTAMP')
            if self.cars and stamp<=next(reversed(self.cars)):return
            self.cars[stamp]=(pose,stamp,now,roll,pitch)
            while len(self.cars)>64:self.cars.popitem(last=False)
            self.car_pose_quality.observe(pose,stamp,now,ros,roll,pitch)
        except (ValueError,TypeError,AttributeError):
            self.core.inputs.poses.pop('car',None);self.core.inputs.event('CAR_DECODE_INVALID')
            self.car_pose_quality.reject('CAR_DECODE_INVALID',now)

    def on_packet(self,msg):
        now,ros=self.clocks()
        try:self.packet_cache.add(decode_packet(json.loads(msg.data),now,ros),now,ros)
        except (ValueError,TypeError,KeyError,AttributeError):
            self.core.inputs.poses.pop('wand',None);self.core.inputs.event('ROTATION_RAW_SOURCE_INVALID')

    def on_candidate(self,msg):
        try:
            v=json.loads(msg.data)
            if set(v)!={'frame','sign_digest','sign_stamp','car_stamp'}:raise ValueError('BOUND_SCHEMA')
            f=decode_frame(canonical(v['frame']));now,ros=self.clocks()
            if f.sequence<=self.core.last_sequence or (self.pending and f.sequence<=self.pending[-1][1].sequence):return
            # STOP is never delayed waiting for a sensor packet.
            if (not f.enabled and (f.state in ('STOPPING','STOP_UNCONFIRMED','PAUSED','FAULT')
                    or (f.state=='STANDBY' and (v['sign_digest'] is None or v['car_stamp'] is None)))):
                self.pending.clear();self.core.receive(f,now,ros);self.publish_output(ZERO);return
            if len(self.pending)>=4:raise ValueError('BOUND_QUEUE_OVERFLOW')
            self.pending.append((v,f,now))
        except (ValueError,TypeError,KeyError,AttributeError):
            self.pending.clear();self.core.trip('ROTATION_BOUND_COMMAND_INVALID');self.publish_output(ZERO)

    def process_pending(self):
        if not self.pending:return
        v,f,arrived=self.pending[0];now,ros=self.clocks()
        try:
            p=self.packet_cache.exact(v['sign_digest'],v['sign_stamp'],now,ros)
            car=self.cars.get(v['car_stamp'])
            if p is None or car is None:
                if now-arrived>.040:raise ValueError('SELECTED_SOURCE_MISSING')
                return
            pose,stamp,received,roll,pitch=car
            if not 0<=now-received<=.20 or not -.005<=ros-stamp<=.20:raise ValueError('SELECTED_CAR_STALE')
            if f.issued_steady-p.received<0 or f.issued_ros-p.stamp<-.005:raise ValueError('SOURCE_AFTER_COMMAND')
            self.core.inputs.observe_pose('car',pose,stamp,received,ros,roll,pitch)
            self.core.inputs.observe_pose('wand',p.pose(now,ros),p.stamp,p.received,ros)
            applied_car=self.core.inputs.poses.get('car')
            if not applied_car or applied_car['source_ros']!=min(stamp,ros) or applied_car['pose']!=pose:
                raise ValueError('SELECTED_CAR_NOT_APPLIED')
            stored=self.core.inputs.poses.get('wand')
            if not stored or stored['source_ros']!=min(p.stamp,ros) or stored['pose']!=p.pose(now,ros):
                raise ValueError('SELECTED_POSE_NOT_APPLIED')
            self.core.receive(f,now,ros);self.pending.popleft()
        except (ValueError,TypeError,KeyError):
            self.pending.clear();self.core.trip('ROTATION_SELECTED_SOURCE_INVALID');self.publish_output(ZERO)

    def tick(self):
        self.process_pending();super().tick()


def source_main():run_node(RotationSignSource)
def supervisor_main():run_node(RotationSupervisorNode)
def gate_main():run_node(RotationGateNode)
