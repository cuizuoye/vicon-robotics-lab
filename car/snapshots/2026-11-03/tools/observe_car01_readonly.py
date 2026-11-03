#!/usr/bin/env python3
"""Subscribe only: real car01, projected car pose, TF and path; no control publishers."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Bool, Int64
from tf2_msgs.msg import TFMessage

FORBIDDEN=('/car/cmd_vel','/car/cmd_vel_raw','/car/enable','/car/control_enable',
    '/car/estop','/car/clear_faults','/car/safety_enabled','/car/trajectory_enable')

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(v):return math.atan2(math.sin(v),math.cos(v))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=8)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--display-yaw-offset',type=float,default=2.356194490192345)
    p.add_argument('--expect-occlusion',action='store_true',help='wait for >=1s invalid then recovery; no source interruption')
    a=p.parse_args()
    if a.output.exists() or not 2<=a.seconds<=(180 if a.expect_occlusion else 30):
        p.error('new output and bounded duration required')
    rclpy.init();node=Node('car01_readonly_acceptance')
    rows=defaultdict(list);valid=[];frames=[];paths=[];parents=defaultdict(set);started=time.monotonic()
    edges=[];outages=[];false_start=None;seen_true=False
    def validity(m):
        nonlocal false_start,seen_true
        now=time.monotonic()-started
        if not valid or valid[-1]!=m.data:edges.append(dict(host_s=now,valid=m.data))
        valid.append(m.data)
        if m.data:
            seen_true=True
            if false_start is not None and now-false_start>=1.0:
                outages.append(dict(false_start_s=false_start,recovery_s=now,duration_s=now-false_start))
            false_start=None
        elif seen_true and false_start is None:false_start=now
    def pose(name,m):
        q=m.pose.orientation;pos=m.pose.position
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        rows[name].append(dict(host_s=time.monotonic()-started,stamp_ns=stamp,
            age_s=(node.get_clock().now().nanoseconds-stamp)*1e-9,
            frame=m.header.frame_id,xyz=[pos.x,pos.y,pos.z],yaw=yaw(q),
            finite=all(math.isfinite(v) for v in (pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w)),
            quaternion_norm=math.hypot(q.x,q.y,q.z,q.w)))
    def tf(m):
        for t in m.transforms:parents[t.child_frame_id].add(t.header.frame_id)
    node.create_subscription(PoseStamped,'/vicon/car01/car01',lambda m:pose('raw',m),qos_profile_sensor_data)
    node.create_subscription(PoseStamped,'/car/pose',lambda m:pose('car',m),10)
    node.create_subscription(Bool,'/car/vicon_valid',validity,10)
    node.create_subscription(Int64,'/vicon/frame_number',lambda m:frames.append(m.data),10)
    node.create_subscription(RosPath,'/car/path_actual',lambda m:paths.append(len(m.poses)),10)
    node.create_subscription(TFMessage,'/tf',tf,50)
    result=dict(source='REAL_VICON_readonly_not_closed_loop',hardware_commands_sent=False,
                display_yaw_offset_provisional_rad=a.display_yaw_offset,success=False)
    try:
        end=time.monotonic()+a.seconds
        print('READONLY_OBSERVER_READY; no commands, source receiver unchanged',flush=True)
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.005)
            if a.expect_occlusion and outages and time.monotonic()-started>outages[-1]['recovery_s']+1:
                break
        result['forbidden_publishers']={t:node.count_publishers(t) for t in FORBIDDEN}
        result['streams']={}
        for name in ('raw','car'):
            r=rows[name]
            if not r:continue
            duration=r[-1]['host_s']-r[0]['host_s']
            result['streams'][name]=dict(count=len(r),hz=(len(r)-1)/duration if duration else 0,
                age_max_s=max(x['age_s'] for x in r),
                stale_samples=sum(x['age_s']>.2 or x['age_s']<-.01 for x in r),
                invalid_samples=sum(not x['finite'] or x['frame']!='map' or
                                    not .99<x['quaternion_norm']<1.01 for x in r),
                max_gap_s=max((b['host_s']-x['host_s'] for x,b in zip(r,r[1:])),default=0),
                first=r[0],last=r[-1])
        raw={r['stamp_ns']:r for r in rows['raw']}
        matched=[(raw[r['stamp_ns']],r) for r in rows['car'] if r['stamp_ns'] in raw]
        result['matched_samples']=len(matched)
        if matched:
            result['max_xy_projection_error_m']=max(math.dist(x['xyz'][:2],b['xyz'][:2]) for x,b in matched)
            result['max_yaw_offset_error_rad']=max(abs(wrap(b['yaw']-x['yaw']-a.display_yaw_offset)) for x,b in matched)
        result['vicon_frames_progress']=len(set(frames))>5
        result['valid_true']=sum(valid);result['valid_false']=len(valid)-sum(valid)
        result['validity_edges']=edges;result['invalid_intervals_over_1s']=outages
        result['actual_path_points']=paths[-1] if paths else 0
        result['tf_parents']={k:sorted(v) for k,v in parents.items()}
        result['success']=(len(result['streams'])==2 and
            all(s['count']>10 and s['hz']>10 and not s['invalid_samples'] and not s['stale_samples']
                for s in result['streams'].values()) and bool(matched) and
            result['max_xy_projection_error_m']<1e-6 and result['max_yaw_offset_error_rad']<1e-6 and
            result['valid_true']>10 and
            (bool(outages) and valid[-1] if a.expect_occlusion else result['valid_false']==0) and
            result['vicon_frames_progress'] and
            not any(result['forbidden_publishers'].values()) and parents['base_footprint']=={'map'})
        if a.expect_occlusion:
            silent=bool(outages) and all(not any(o['false_start_s']+.1<r['host_s']<o['recovery_s']-.1
                for r in rows['car']) for o in outages)
            result['no_car_pose_while_invalid']=silent
            result['success'] &= silent
        result['samples']=dict(rows)
    finally:
        node.destroy_node();rclpy.shutdown()
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with a.output.open('x') as f:json.dump(result,f,indent=2)
        print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))
    return 0 if result['success'] else 1

if __name__=='__main__':raise SystemExit(main())
