"""Subscriptions only. No UDP, services, command publishers, or SDK launch."""
import time

from receiver_guard import source_ok,verify_existing
from preview_core import IDS

TOPICS={'sign':'/vicon/Sign/Sign','car':'/vicon/car01/car01',
    **{k:f'/vicon/Sign/markers/{k}' for k in IDS}}


class LiveInput:
    def __init__(self,model):
        self.provenance=verify_existing()
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from geometry_msgs.msg import PoseStamped,PointStamped
        self.rclpy=rclpy;self.model=model;self.raw={};self.flags={};self.graph_at=-1e9
        rclpy.init(args=[])
        self.node=Node('sign_rotation_readonly',enable_rosout=False,start_parameter_services=False)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        for name,topic in TOPICS.items():
            self.node.create_subscription(PointStamped if name in IDS else PoseStamped,topic,
                lambda msg,name=name:self.receive(name,msg),qos)

    def receive(self,name,msg):
        if not self.flags.get(name,False):return
        p=msg.point if name in IDS else msg.pose.position
        q=None
        if name not in IDS:
            value=msg.pose.orientation;q=(value.x,value.y,value.z,value.w)
        ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        now=time.monotonic()
        self.model.receive(name,(p.x,p.y,p.z),q,ns*1e-9,now,msg.header.frame_id)
        self.raw[name]=dict(position=[p.x,p.y,p.z],quaternion=q,source_ns=ns,received=now)

    def pump(self):
        now=time.monotonic()
        if now-self.graph_at>.25:
            self.flags={k:source_ok(self.node.get_publishers_info_by_topic(t)) for k,t in TOPICS.items()}
            self.model.source_ok=all(self.flags.get(k,False) for k in ('sign',*IDS))
            self.graph_at=now
        start=time.monotonic()
        for _ in range(128):
            self.rclpy.spin_once(self.node,timeout_sec=0.)
            if time.monotonic()-start>.006:break
        self.model.update(time.monotonic(),self.ros_now())

    def ros_now(self):return self.node.get_clock().now().nanoseconds*1e-9

    def close(self):self.node.destroy_node();self.rclpy.shutdown()
