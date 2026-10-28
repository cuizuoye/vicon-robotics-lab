#!/usr/bin/env python3
"""Read recorded real-Sign/software-car events; no ROS or hardware access.

Timing starts at the receiver's PoseStamped header, NOT camera exposure.
Position error uses the actual source samples selected by one control tick;
their timestamp skew is reported, with no shifted-curve alignment.
"""
import argparse
from bisect import bisect_left
from collections import defaultdict
import json
import math
from pathlib import Path
import re


def stats(values):
    if not values:return dict(count=0,median=None,p95=None,max=None)
    values=sorted(values)
    return dict(count=len(values),median=values[len(values)//2],
                p95=values[int(.95*(len(values)-1))],max=values[-1])


def pose_index(rows):
    values={r['value']['source_ros']:r['value'] for r in rows}
    stamps=sorted(values)
    def lookup(stamp):
        i=bisect_left(stamps,stamp)
        candidates=stamps[max(0,i-1):i+1]
        if not candidates:return None
        t=min(candidates,key=lambda x:abs(x-stamp))
        return values[t] if abs(t-stamp)<2e-6 else None
    return lookup


def analyze(path):
    topics=defaultdict(list);partial_lines=0
    with (path/'events.jsonl').open() as f:
        for line in f:
            try:r=json.loads(line)
            except json.JSONDecodeError:
                partial_lines+=1;continue  # Recorder may be flushing its final line.
            topics[r['topic']].append(r)
    snapshot=json.loads((path/'snapshot.json').read_text())
    statuses=[r['value'] for r in topics['supervisor_status']]
    active=[s for s in statuses if s['state']=='ACTIVE' and s.get('anchors') and s.get('raw_goal')]
    by_steady={s['issued_steady']:s for s in statuses}
    status_times=sorted(by_steady)
    ui=[]
    if (path/'console.log').exists():
        for line in (path/'console.log').read_text().splitlines():
            match=re.search(r'\[INFO\] \[([0-9.]+)\].*UI_STATE (\{.*\})',line)
            if match:ui.append((float(match.group(1)),json.loads(match.group(2))))
    car_lookup=pose_index(topics['car_pose']);wand_lookup=pose_index(topics['wand_pose'])
    errors=[];mapping_errors=[];skews=[];header_ages=[];xy=[]
    for s in active:
        ages=s.get('pose_source_ages_s',{})
        if not all(k in ages for k in ('car','wand')):continue
        car=car_lookup(s['issued_ros']-ages['car'])
        wand=wand_lookup(s['issued_ros']-ages['wand'])
        if car:
            g=s['mapped_goal']
            errors.append(math.hypot(g['x']-car['x'],g['y']-car['y']))
            skews.append(abs(ages['car']-ages['wand']))
            xy.append((car['x'],car['y']))
        if wand:
            a=s['anchors'];origin=a['car_origin'];hand=a['wand_origin']
            raw=s['raw_goal']
            expected=(origin['x']+wand['x']-hand['x'],origin['y']+wand['y']-hand['y'])
            mapping_errors.append(math.hypot(expected[0]-raw['x'],expected[1]-raw['y']))
        header_ages.append(ages['wand'])
    gate_delays=[];header_to_gate=[];seen_candidates=set()
    zeros=[]
    for r in topics['gate_timing']:
        g=r['value'];c=g['command'];zero=all(c[k]==0 for k in ('vx','vy','wz'))
        if zero:zeros.append(g['issued_steady'])
        s=by_steady.get(g.get('candidate_issued_steady'))
        identity=(g.get('candidate_sequence'),g.get('candidate_issued_steady'))
        if s and s['state']=='ACTIVE' and identity not in seen_candidates:
            seen_candidates.add(identity)
            gate_delays.append(g['issued_steady']-s['issued_steady'])
            age=s.get('pose_source_ages_s',{}).get('wand')
            if age is not None:
                header_to_gate.append(g['issued_ros']-(s['issued_ros']-age))
    zeros.sort();leases=[];held=False
    for r in topics['operator_lease']:
        l=r['value']
        if held and not l['held']:
            at=l['issued_steady'];i=bisect_left(zeros,at)
            zero_at=zeros[i] if i<len(zeros) else None
            preceding=next((v['value'] for v in reversed(topics['gate_timing'])
                if v['value']['issued_steady']<=at),None)
            moving_before=bool(preceding and any(preceding['command'][k]!=0 for k in ('vx','vy','wz')))
            nearby=[(t,v) for t,v in ui if abs(t-l['issued_ros'])<.06 and not v['held']]
            input_state=min(nearby,key=lambda pair:abs(pair[0]-l['issued_ros']))[1] if nearby else {}
            cause=('window_focus_loss' if input_state.get('focused') is False else
                   'key_release_observed_by_ui' if input_state.get('key_down') is False else
                   'revocation_or_unknown')
            still=next((f['value']['issued_steady'] for f in topics['plant_feedback']
                if f['value']['issued_steady']>=at and f['value']['output_zero']
                and not f['value']['enabled'] and f['value']['speed']<=.005),None)
            leases.append(dict(release_issued_steady=at,
                nonzero_command_before_release=moving_before,
                release_cause=cause,
                release_to_gate_zero_s=zero_at-at if zero_at is not None else None,
                release_to_sim_speed_below_005mps_s=still-at if still is not None else None))
        held=l['held']
    transitions=snapshot.get('state_transitions',[])
    gaps=[]
    for old,new in zip(topics['wand_pose'],topics['wand_pose'][1:]):
        gap=new['value']['source_ros']-old['value']['source_ros']
        if gap<=.20:continue
        returned=new['received_steady']
        arm=next((t['steady'] for t in transitions if t['steady']>=returned and t['state']=='ARMED'),math.inf)
        before_arm=[r for r in topics['gate_timing'] if returned<=r['received_steady']<arm]
        resumed=any(any(r['value']['command'][k]!=0 for k in ('vx','vy','wz')) for r in before_arm)
        timeout=next((t for t in transitions if old['received_steady']<=t['steady']<=returned
            and ('WAND' in t['reason'].upper() or 'wand pose timeout' in t['reason'])),None)
        i=bisect_left(zeros,old['received_steady'])
        # First zero may precede actual loss if the hand was at rest; do not
        # call this a measured fault-stop latency without a nonzero output.
        gaps.append(dict(source_gap_s=gap,return_received_steady=returned,
            wand_fault_observed=timeout is not None,fault=timeout,
            auto_nonzero_before_new_arm=resumed,new_arm_observed=math.isfinite(arm)))
    velocities=[];active_velocities=[];window=[]
    for r in topics['wand_pose']:
        v=r['value'];window.append(v)
        while len(window)>2 and window[1]['source_ros']<=v['source_ros']-.10:
            window.pop(0)
        dt=v['source_ros']-window[0]['source_ros']
        if dt>=.08:
            speed=math.hypot(v['x']-window[0]['x'],v['y']-window[0]['y'])/dt
            velocities.append(speed)
            i=bisect_left(status_times,r['received_steady'])-1
            if i>=0 and by_steady[status_times[i]]['state']=='ACTIVE':active_velocities.append(speed)
    report=dict(scope='REAL Sign -> SOFTWARE CAR ONLY; no physical car or MCU',
        hardware_started=False,physical_camera_to_wheel_latency_measured=False,
        counts={k:len(v) for k,v in topics.items()},partial_lines_skipped=partial_lines,
        current_state=snapshot.get('latest',{}).get('supervisor_status',{}).get('value',{}),
        active_control_samples=len(active),
        raw_1to1_mapping_error_m=stats(mapping_errors),
        same_control_tick_position_error_m=stats(errors),
        same_control_tick_position_error_rms_m=math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
        source_pair_timestamp_skew_s=stats(skews),
        receiver_header_age_at_supervisor_s=stats(header_ages),
        candidate_to_gate_application_s=stats(gate_delays),
        receiver_header_to_gate_application_s=stats(header_to_gate),
        raw_sign_velocity_100ms_window_mps=stats(velocities),
        active_sign_velocity_100ms_window_mps=stats(active_velocities),
        active_sign_velocity_fraction_above_020mps=(sum(v>.2 for v in active_velocities)/len(active_velocities)
                                                  if active_velocities else None),
        active_goal_clamped_frames=sum(s.get('goal_limited',False) for s in active),
        active_reference_lead_limited_frames=sum(s.get('reference_limited',False) for s in active),
        release_events=leases,source_gaps_over_200ms=gaps,
        wand_timeout_transitions=[t for t in transitions if 'WAND' in t['reason'].upper()],
        max_command_speed_mps=snapshot.get('max_command_speed_mps'),
        mapping_and_movement_exercised=(len(active)>50 and bool(xy) and
            math.hypot(max(p[0] for p in xy)-min(p[0] for p in xy),
                       max(p[1] for p in xy)-min(p[1] for p in xy))>.02),
        synchronization_subjective_acceptance='REQUIRES HUMAN OBSERVATION',
        occlusion_acceptance=('NOT EXERCISED' if not gaps else 'REVIEW RECORDED GAP AND HUMAN REPORT'),
        notes=['Header is receiver publication time, not camera exposure or acquisition time',
               'Model position error is not physical vehicle tracking accuracy',
               'No shifted curves; paired source timestamp skew is explicitly reported',
               'Release timing starts at lease transmission, not physical key release',
               'Gate timing uses FIRST application of each candidate, excluding repeated cached-frame ticks'])
    out=path/'analysis.json';temp=path/'analysis.tmp'
    temp.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');temp.replace(out)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',type=Path,required=True)
    a=p.parse_args();r=analyze(a.session)
    print(json.dumps({k:v for k,v in r.items() if k!='current_state'},indent=2,ensure_ascii=False))
