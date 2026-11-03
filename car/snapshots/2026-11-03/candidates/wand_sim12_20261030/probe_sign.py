"""Read-only raw-topic/geometry diagnosis; no control ROS domain or output."""
import argparse
from collections import Counter,defaultdict
import json
import os
from pathlib import Path
import time
from profile12 import save,protected
from live_sign import ensure_receiver
from common import diagnostic as d

def run(out,seconds=10.):
    out.mkdir(parents=True,exist_ok=False);before=protected();receiver=node=executor=None;initialized=False
    # This diagnostic is its own process, never the control/UI interpreter.
    os.environ['ROS_DOMAIN_ID']='0';os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
    try:
        receiver=ensure_receiver(out)
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import Int64
        from mecanum_wand_4wsb_hardware.rotation_nodes import RotationRawSource
        from mecanum_wand_4wsb_control.safety_nodes import SENSOR
        counts=Counter();errors=Counter();frames=Counter();stats=defaultdict(dict)
        raw={};snapshots=[];last_snapshot=0.
        def observed(name,msg):
            counts[name]+=1;stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            stats[name]=dict(last_stamp=stamp,age_s=time.time()-stamp,frame=msg.header.frame_id)
        class Probe(RotationRawSource):
            def receive(self,source,msg):observed(source,msg);return super().receive(source,msg)
            def marker(self,name,msg):observed(name,msg);return super().marker(name,msg)
            def raw_sample(self,name,p,q,msg):
                nonlocal last_snapshot
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                raw[name]=dict(position=list(p),quaternion=list(q) if q is not None else None,stamp=stamp)
                if len(raw)==6 and all(v['stamp']==stamp for v in raw.values()) and stamp-last_snapshot>=.02:
                    snapshots.append(raw.copy());last_snapshot=stamp
                    if len(snapshots)>120:snapshots.pop(0)
                try:return super().raw_sample(name,p,q,msg)
                except Exception as error:errors[str(error)]+=1;raise
        context=Context();rclpy.init(context=context,domain_id=0,signal_handler_options=SignalHandlerOptions.NO);initialized=True
        node=Probe(context);executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        node.create_subscription(Int64,'/vicon/frame_number',lambda m:frames.update([m.data]),SENSOR)
        begin=time.monotonic();packets=set()
        while time.monotonic()-begin<seconds:
            executor.spin_once(timeout_sec=.002)
            if node.packet is not None:packets.add(node.packet.stamp)
        result=dict(raw_counts=dict(counts),raw_last=dict(stats),geometry_errors=dict(errors),rejected=node.rejected,
            complete_packets=len(packets),retained_raw_frame_count=len(node.frames),
            raw_frame_members=[list(frame.keys()) for frame in list(node.frames.values())[-3:]],
            graph_conflicts=node.audit(),frame_status_messages=sum(frames.values()),frame_status_unique=len(frames),hardware_output=False)
        save(out/'RAW_SNAPSHOTS.json',snapshots)
        if snapshots:
            import numpy as np
            from itertools import combinations
            from mecanum_wand_4wsb_control.rotation_source import definition
            from mecanum_wand_4wsb_control.rotation_geometry import IDS,rotation,identify_t
            expected=definition()['local_points'];actual=[]
            for frame in snapshots:
                r=rotation(frame['sign']['quaternion']);o=np.array(frame['sign']['position'])
                actual.append([(r.T@(np.array(frame[k]['position'])-o)).tolist() for k in IDS])
            local=np.median(np.array(actual),axis=0);points={k:local[i].tolist() for i,k in enumerate(IDS)}
            result['geometry_comparison']=dict(local_points_now=points,
                per_marker_error_m={k:float(np.linalg.norm(local[i]-np.array(expected[k]))) for i,k in enumerate(IDS)},
                max_pairwise_distance_error_m=max(abs(np.linalg.norm(np.array(points[a])-points[b])-
                    np.linalg.norm(np.array(expected[a])-expected[b])) for a,b in combinations(IDS,2)),
                local_std_max_m=float(np.std(np.array(actual),axis=0).max()),
                t_candidates=[c.summary() for c in identify_t(points)])
        save(out/'RESULT.json',result);print(json.dumps(result,ensure_ascii=False),flush=True)
    finally:
        if executor is not None:executor.shutdown()
        if node is not None:node.destroy_node()
        if initialized:context.shutdown()
        if receiver is not None:d.terminate_owned(receiver)
        save(out/'PROTECTED.json',dict(unchanged=before==protected(),hardware_output=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--seconds',type=float,default=10.)
    a=p.parse_args();run(a.output,a.seconds)
