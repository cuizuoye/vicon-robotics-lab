"""PX4 1.14.3紧凑位姿的纯数学：显式FC航向对齐、协方差、台架时间测量。"""

import math
import re
import sys
from pathlib import Path
import numpy as np
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import quaternion_to_matrix, matrix_to_quaternion
from uav_coord_bridge.mavlink_vision import checked_quaternion, finite_vector, pose_covariance

SERIES_COMMAND=b'listener vehicle_visual_odometry -n 12\n'


def visual_series(text):
    from usb_vision_policy import parse_visual_snapshot
    blocks=re.split(r'TOPIC:\s*vehicle_visual_odometry[^\n]*\n',text)[1:]
    rows=[parse_visual_snapshot('TOPIC: vehicle_visual_odometry\n'+block) for block in blocks]
    rows=[r for r in rows if r]
    return rows if len(rows)==12 else None


def series_timing(rows):
    stamps=[r['timestamp'] for r in rows]
    if len(stamps)<2 or any(b<=a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('Fresh distinct increasing publication timestamps required')
    return {'samples':len(stamps),'rate_hz':(len(stamps)-1)*1e6/(stamps[-1]-stamps[0]),
            'maximum_publication_gap_ms':max(b-a for a,b in zip(stamps,stamps[1:]))/1000,
            'scope':'Short internal publication window; not an all-packets receipt count or camera synchronization'}


def rotation(q_wxyz):
    w,x,y,z=checked_quaternion(q_wxyz)
    return quaternion_to_matrix([x,y,z,w])


def euler(q_wxyz):
    r=rotation(q_wxyz)
    pitch=math.asin(float(np.clip(-r[2,0],-1,1)))
    if abs(pitch)>math.radians(85):
        raise ValueError('Euler singularity proximity; no compact encoding')
    return [math.atan2(r[2,1],r[2,2]),pitch,math.atan2(r[1,0],r[0,0])]


def yaw_alignment(source_body_q,fc_body_q):
    """只对齐静止机身航向，不冒充地理真北或跨重启持久外参。"""
    sr,sp,sy=euler(source_body_q)
    fr,fp,fy=euler(fc_body_q)
    if max(abs(sr-fr),abs(sp-fp))>math.radians(5):
        raise ValueError('Source/FC tilt disagreement; no NED candidate')
    return math.atan2(math.sin(fy-sy),math.cos(fy-sy))


def aligned_pose(position,q_wxyz,alignment_yaw):
    if not math.isfinite(alignment_yaw):
        raise ValueError('Explicit finite yaw alignment required')
    c,s=math.cos(alignment_yaw),math.sin(alignment_yaw)
    a=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    p=(a@finite_vector(position,3)).tolist()
    x,y,z,w=matrix_to_quaternion(a@rotation(q_wxyz))
    return p,[w,x,y,z]


def message(enc,stamp_us,position,q_wxyz,alignment_yaw):
    if stamp_us<=0:
        raise ValueError('Positive source timestamp required')
    p,q=aligned_pose(position,q_wxyz,alignment_yaw)
    return enc.vision_position_estimate_encode(stamp_us,*p,*euler(q),pose_covariance(),0),p,q


def pose_allowed(msg):
    return (msg.get_type()=='VISION_POSITION_ESTIMATE' and msg.usec>0 and msg.reset_counter==0
            and all(math.isfinite(v) for v in (msg.x,msg.y,msg.z,msg.roll,msg.pitch,msg.yaw))
            and abs(msg.pitch)<=math.radians(85)
            and all(math.isfinite(v) for v in msg.covariance)
            and list(msg.covariance)==pose_covariance())


def match(snapshot,sent):
    if snapshot.get('pose_frame')!=1 or snapshot.get('reset_counter')!=0:
        return None
    q=checked_quaternion(snapshot['q'])
    for row in reversed(sent):
        angle=math.degrees(2*math.acos(float(np.clip(abs(q@np.asarray(row['q'])),0,1))))
        distance=float(np.linalg.norm(np.asarray(snapshot['position'])-row['position']))
        if distance<=.002 and angle<=.5:
            return {'source_stamp_ns':row['source_stamp_ns'],'position_error_m':distance,
                    'quaternion_difference_deg':angle}
    return None


def reference_usable(ref,pc_now_us):
    return bool(ref and 0<=pc_now_us-ref['remote_timestamp']<1_000_000
                and 0<=ref['round_trip_time']<10_000
                and abs(ref['observed_offset']-ref['estimated_offset'])<=2000)


def arrival_measurement(row,snapshot,before,after):
    """USB时钟参考仅地面诊断；不改TRS同步器，不把发布时间说成曝光时间。"""
    send=row['pc_sent_us']
    a,b=before['remote_timestamp'],after['remote_timestamp']
    if not a<=send<=b or b<=a or not match(snapshot,[row]):
        raise ValueError('No bracketing reference or matching single real pose')
    if any(not 0<=ref['round_trip_time']<10_000 or abs(ref['observed_offset']-ref['estimated_offset'])>2000
           for ref in (before,after)):
        raise ValueError('USB reference clock outside original timing gate')
    ratio=(send-a)/(b-a)
    offset=before['estimated_offset']+ratio*(after['estimated_offset']-before['estimated_offset'])
    sent_age=send-row['wire_time_usec']
    radio_delay=snapshot['timestamp']-offset-send
    if not 0<=sent_age<=150_000 or not 0<=radio_delay<=150_000:
        raise ValueError('Unexpected source age or measured transport delay')
    return {'age_at_send_ms':sent_age/1000,'send_to_fc_publication_ms':radio_delay/1000,
            'source_ros_publication_to_fc_ms':(radio_delay+sent_age)/1000,
            'fc_publication_minus_sample_us':snapshot['timestamp']-snapshot['timestamp_sample'],
            'interpolated_usb_reference_offset_us':offset,
            'camera_exposure_latency_measured':False,'wireless_clock_conversion_verified':False,
            'scope':'USB-referenced same-source single-packet bench observation, not flight synchronization or independent accuracy'}
