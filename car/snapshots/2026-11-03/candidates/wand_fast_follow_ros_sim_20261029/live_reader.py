"""Domain0 subscriptions only; private SIM relay never writes into domain0."""
from collections import OrderedDict
import sys
import threading
import time

from bootstrap import ROOT
sys.path.insert(0,str(ROOT/'candidates/wand_rotation_preview'))
from receiver_guard import verify_existing, source_ok
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from mecanum_wand_4ws8_control.rotation_geometry import REQUIRED
from mecanum_wand_4ws8_control.rotation_source import Raw, Packet
from mecanum_wand_4ws8_control.safety_nodes import SENSOR


class RealReader:
    def __init__(self):
        self.provenance=verify_existing()
        self.context=Context();rclpy.init(args=[],context=self.context,domain_id=0)
        self.node=Node('fast_follow_sign_readonly',context=self.context,
            enable_rosout=False,start_parameter_services=False)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.lock=threading.Lock();self.packet=None;self.rejected=0;self.frames=OrderedDict()
        self.latest={};self.flags={};self.error='等待Sign及五点';self.received=0;self.closed=False
        self.topics={'sign':'/vicon/Sign/Sign',**{k:'/vicon/Sign/markers/'+k for k in REQUIRED[1:]}}
        for name,topic in self.topics.items():
            self.node.create_subscription(PoseStamped if name=='sign' else PointStamped,topic,
                lambda msg,name=name:self.receive(name,msg),SENSOR)
        self.node.create_timer(.25,self.audit)
        self.stop_event=threading.Event();self.thread=None

    def audit(self):
        flags={k:source_ok(self.node.get_publishers_info_by_topic(t)) for k,t in self.topics.items()}
        with self.lock:
            if all(self.flags.values()) and self.flags and not all(flags.values()):self.rejected+=1
            self.flags=flags
            if not all(flags.values()):self.packet=None;self.error='等待唯一vicon_client和Sign六路输入'

    def receive(self,name,msg):
        now=time.monotonic();ros=self.node.get_clock().now().nanoseconds*1e-9
        try:
            if msg.header.frame_id!='map':raise ValueError('RAW_FRAME')
            if not 0<=msg.header.stamp.nanosec<1_000_000_000:raise ValueError('RAW_STAMP')
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            p=msg.pose.position if name=='sign' else msg.point
            q=msg.pose.orientation if name=='sign' else None
            sample=Raw(name,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w) if q else None,stamp,now)
            with self.lock:
                old=self.latest.get(name)
                if old and stamp<=old.stamp:
                    if stamp==old.stamp and (sample.position,sample.quaternion)!=(old.position,old.quaternion):
                        raise ValueError('SAME_STAMP_CONFLICT')
                    return
                self.latest[name]=sample
                self.frames.setdefault(stamp,{})[name]=sample
                while len(self.frames)>32:self.frames.popitem(last=False)
                frame=self.frames[stamp]
                if all(k in frame for k in REQUIRED):
                    packet=Packet(tuple(frame[k] for k in REQUIRED));packet.pose(now,ros)
                    if self.packet is None or packet.stamp>self.packet.stamp:
                        self.packet=packet;self.received+=1;self.error=''
        except (ValueError,TypeError,AttributeError) as error:
            with self.lock:self.packet=None;self.frames.clear();self.rejected+=1;self.error=str(error)

    def __call__(self,now,ros):
        with self.lock:
            if not self.flags or not all(self.flags.values()):return None
            packet=self.packet
        if packet:
            try:packet.pose(now,ros);return packet
            except ValueError:return None
        return None

    def pump(self):self.executor.spin_once(timeout_sec=0.)

    def start(self):
        def run():
            while not self.stop_event.is_set():self.executor.spin_once(timeout_sec=.002)
        self.thread=threading.Thread(target=run,name='vicon-passive-domain0',daemon=True);self.thread.start()

    def status(self):
        with self.lock:return dict(flags=self.flags.copy(),rejected=self.rejected,
            complete_packets=self.received,error=self.error,domain0_command_publishers=[])

    def close(self):
        if self.closed:return
        self.closed=True;self.stop_event.set()
        if self.thread:self.thread.join(timeout=2.)
        self.executor.shutdown();self.node.destroy_node();self.context.shutdown()
