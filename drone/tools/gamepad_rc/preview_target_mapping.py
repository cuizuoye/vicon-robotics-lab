#!/usr/bin/env python3
"""历史联合台架的目标映射算术预览；无串口、发布、MAVLink编码或发送。"""

import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.mavlink_vision import F, checked_quaternion
from uav_coord_bridge.frames import yaw_from_quaternion
from uav_coord_bridge.target_mapping import (make_alignment,map_target_to_controller,
    fixed_direction_delta,height_target_map,yaw_target_controller)
import usb_combined_fusion_policy as policy


def preview(bench,reference):
    rows=bench['fusion_snapshots']
    if (bench.get('combined_aiding_verified') is not True or bench.get('error') is not None
            or bench['parameters_after'].get('EKF2_HGT_REF')!=0
            or not policy.fusion_evidence_ok(rows,bench['primary_estimator'])):
        raise ValueError('Verified joint fusion evidence required')
    paired=[]
    for row in rows:
        local=row['local']; offset=row['clock']['estimated_offset']
        source=min(bench['sent'],key=lambda s:abs(s['wire_time_usec']+offset-local['timestamp_sample']))
        difference=abs(source['wire_time_usec']+offset-local['timestamp_sample'])
        if difference>50_000: raise ValueError('No nearby timestamp-mapped real source sample')
        w,x,y,z=checked_quaternion(source['q'])
        map_yaw=-yaw_from_quaternion([x,y,z,w])
        map_position=(F@np.asarray(source['position'])).tolist()
        alignment=make_alignment(map_position,[local[k] for k in ('x','y','z')],map_yaw,local['heading'])
        paired.append({'timestamp_difference_us':difference,'source_stamp_ns':source['source_stamp_ns'],
                       'fc_timestamp_sample_us':local['timestamp_sample'],'alignment':alignment})
    alignment=paired[-1]['alignment']
    anchor=alignment['map_anchor_m']
    target=height_target_map(reference,anchor[:2],.5)
    return {'scope':'Historical bench arithmetic examples only; NOT a live target, flight mission or calibration',
            'source_bench_captured_at_utc':bench['captured_at_utc'],
            'pairing_scope':'Nearby timestamps using sampled TIMESYNC offset; stationary-only candidate, NOT exposure synchronization',
            'paired_samples':paired,'alignment_candidate':alignment,
            'arithmetic_examples_not_mission':{
                'recorded_anchor_map_m':anchor,
                'recorded_anchor_controller_m':alignment['controller_anchor_m'],
                'height_half_meter_map_m':target,
                'height_half_meter_controller_m':map_target_to_controller(alignment,target),
                'forward_20cm_controller_delta_m':fixed_direction_delta(alignment,reference,[.2,0,0]),
                'left_20cm_controller_delta_m':fixed_direction_delta(alignment,reference,[0,.2,0]),
                'up_20cm_controller_delta_m':fixed_direction_delta(alignment,reference,[0,0,.2]),
                'fixed_front_yaw_controller_rad':yaw_target_controller(alignment,reference['fixed_forward_yaw_map_rad'])},
            'height_reference_remains_baro':True,'task_origin_selected':False,
            'actual_sent_to_fc':0,'flight_ready':False,'live_dispatch_permitted':False,
            'fresh_mapping_and_reset_counters_required_before_any_future_live_control':True}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bench',type=Path,required=True)
    ap.add_argument('--reference',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    if args.output.exists(): ap.error('Use a new output; do not replace evidence')
    try:
        inputs={k:getattr(args,k).resolve() for k in ('bench','reference')}
        result=preview(*(json.loads(inputs[k].read_text()) for k in ('bench','reference')))
        result['prepared_at_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        result['inputs']={k:{'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for k,p in inputs.items()}
        with args.output.open('x') as stream:
            json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False); stream.write('\n')
        print(json.dumps(result,ensure_ascii=False,indent=2))
    except (ValueError,KeyError,TypeError,OSError) as exc: ap.error(str(exc))


if __name__=='__main__': main()
