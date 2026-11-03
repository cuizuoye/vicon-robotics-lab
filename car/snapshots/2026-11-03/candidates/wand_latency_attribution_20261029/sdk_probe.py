"""Read existing Vicon SDK-total/frame topics; no SDK new connection/config."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0,str(ROOT/'candidates/wand_rotation_preview'))
from receiver_guard import verify_existing,source_ok


def stats(v):
    a=np.array(v)
    return dict(n=len(a),min=float(a.min()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max())) if len(a) else None


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);provenance=verify_existing()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from std_msgs.msg import Float64,Int64
    rclpy.init(args=[],domain_id=0)
    node=Node('wand_latency_sdk_readonly',enable_rosout=False,start_parameter_services=False)
    raw=[];frames=[];invalid=0;flags={};started=time.monotonic()
    def receive(kind,msg):
        global invalid
        topic='/vicon/'+kind
        if not source_ok(node.get_publishers_info_by_topic(topic)):return
        if kind=='latency':
            v=float(msg.data)
            if not math.isfinite(v) or v<0:invalid+=1;return
            raw.append(dict(received=time.monotonic(),sdk_total_s=v))
        else:frames.append(dict(received=time.monotonic(),frame=int(msg.data)))
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Float64,'/vicon/latency',lambda m:receive('latency',m),qos)
    node.create_subscription(Int64,'/vicon/frame_number',lambda m:receive('frame_number',m),qos)
    try:
        while time.monotonic()-started<10.:rclpy.spin_once(node,timeout_sec=.01)
        flags={k:source_ok(node.get_publishers_info_by_topic('/vicon/'+k)) for k in ('latency','frame_number')}
        result=dict(kind='CURRENT_STATIC_VICON_SDK_TOTAL_ONLY',duration_s=time.monotonic()-started,
            sdk_total_ms=stats([r['sdk_total_s']*1000 for r in raw]),invalid=invalid,
            zeros=sum(r['sdk_total_s']==0 for r in raw),frame_messages=len(frames),source_identity_ok=flags,
            not_same_time_as_previous_motion_session=True,not_capture_to_wheel_latency=True,
            no_parameter_writes=True,no_new_sdk_connection=True,command_publishers=[],provenance=provenance)
        (a.output/'RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        (a.output/'samples.json').write_text(json.dumps(dict(latency=raw,frames=frames))+'\n')
        print(json.dumps(result,ensure_ascii=False,indent=2))
    finally:node.destroy_node();rclpy.shutdown()
