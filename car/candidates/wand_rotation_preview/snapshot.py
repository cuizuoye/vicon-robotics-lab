"""Bounded, subscriptions-only Sign geometry snapshot; no motor interfaces."""
import argparse
import json
import os
from pathlib import Path
import time

from receiver_guard import verify_existing,source_ok

TOPICS={'sign':'/vicon/Sign/Sign','car':'/vicon/car01/car01',
        **{f'Sign{i}':f'/vicon/Sign/markers/Sign{i}' for i in range(1,6)}}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=4.)
    args=parser.parse_args()
    if not 1<=args.seconds<=15:raise ValueError('snapshot duration must be 1..15 seconds')
    if args.output.exists():raise ValueError('refusing to overwrite an observation')
    provenance=verify_existing()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import PoseStamped,PointStamped
    rclpy.init(args=[])
    node=Node('sign_rotation_snapshot',enable_rosout=False,start_parameter_services=False)
    latest={};counts={k:0 for k in TOPICS};valid_sources={}
    def receive(name,msg):
        if not valid_sources.get(name,False):return
        p=msg.point if name.startswith('Sign') else msg.pose.position
        row=dict(frame=msg.header.frame_id,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            received=time.monotonic(),xyz=[p.x,p.y,p.z])
        if not name.startswith('Sign'):
            q=msg.pose.orientation;row['xyzw']=[q.x,q.y,q.z,q.w]
        latest[name]=row;counts[name]+=1
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    for name,topic in TOPICS.items():
        node.create_subscription(PointStamped if name.startswith('Sign') else PoseStamped,
            topic,lambda msg,name=name:receive(name,msg),qos)
    started=time.monotonic();graph_at=-1e9
    try:
        while time.monotonic()-started<args.seconds:
            if time.monotonic()-graph_at>.25:
                valid_sources={k:source_ok(node.get_publishers_info_by_topic(t)) for k,t in TOPICS.items()}
                graph_at=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.02)
        now=time.monotonic();ros=node.get_clock().now().nanoseconds*1e-9
        result=dict(kind='SUBSCRIPTIONS_ONLY_GEOMETRY_SNAPSHOT',provenance=provenance,
            counts=counts,source_ok=valid_sources,latest=latest,finished_steady=now,finished_ros=ros,
            actuator_outputs=False,hardware_started=False)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(counts=counts,source_ok=valid_sources,
            output=str(args.output),actuator_outputs=False)))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
