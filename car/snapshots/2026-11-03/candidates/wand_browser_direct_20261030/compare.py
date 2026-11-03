"""Identical-input nominal replay comparison in independent Python processes.

No ROS context, no hardware. Recorded hand targets are replayed, not recorded
car responses. This isolates the controller change from hand-to-hand variation.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
DOC=ROOT/'docs/wand_browser_direct_2026-10-30'
RECORD=ROOT/'docs/wand_browser_sim12_2026-10-30/prepared_v4/sessions/fast_software_2wZ90A/TRACE.jsonl'


def run(mode):
    if mode=='direct':
        sys.path.insert(0,str(DOC/'prepared_v1'))
        from profile_direct import sim_config
        from mecanum_wand_4wsb_control.direct_target import DirectSession as Session
        from plant_direct import NominalPlant
    else:
        sys.path.insert(0,str(ROOT/'candidates/wand_sim12_20261030'))
        from profile12 import sim_config
        from mecanum_wand_4wsb_control.fast_catchup import FastCatchupSession as Session
        from plant12 import NominalPlant
    from mecanum_wand_4wsb_control.core import Pose
    from mecanum_wand_4wsb_control.rotation_servo import wheel_rpm
    import numpy as np
    cases={
        'long_straight':[(i*.01,2.8*math.sin(.32*i*.01),0.,0.) for i in range(4001)],
        'omni_and_reverse':[(i*.01,1.5*math.sin(.55*i*.01),1.25*(1-math.cos(.55*i*.01)),0.) for i in range(4001)],
        'translation_rotation':[(i*.01,1.5*math.sin(.45*i*.01),1.25*(1-math.cos(.45*i*.01)),1.2*math.sin(.65*i*.01)) for i in range(4001)]}
    recorded=[]
    with RECORD.open() as stream:
        for line in stream:
            r=json.loads(line);s=r.get('supervisor',{});g=s.get('raw_goal')
            if r.get('state')=='ACTIVE' and g:
                recorded.append((r['t'],g['x'],g['y'],g['yaw']))
    start=recorded[0][0];base=recorded[0][1:]
    cases['recorded_human_289s']=[(t-start,x-base[0],y-base[1],a-base[2]) for t,x,y,a in recorded]
    result={}
    for name,samples in cases.items():
        c=sim_config();f=Session(c.follower_config(),c.project_goal,c.project_reference);plant=NominalPlant()
        f.observe('car',Pose(0.,0.,c.ground_z_m),10.,source_stamp=100.)
        f.observe('wand',Pose(0.,0.,1.),10.,source_stamp=100.);f.engage(10.)
        errors=[];angles=[];velocities=[];limited=0;peakrpm=peakacc=peakyawacc=0.;last=0.;ended=''
        for t,x,y,yaw in samples[1:]:
            dt=t-last
            if not 0<dt<=.1:raise ValueError('recording gap outside replay limits')
            last=t;now=10.+t
            f.observe('car',Pose(*plant.pose[:2],c.ground_z_m,plant.pose[2]),now,source_stamp=100.+t)
            f.observe('wand',Pose(x,y,1.,yaw),now,source_stamp=100.+t)
            old=f.command;cmd=f.tick(now)
            if f.state!='ACTIVE':ended=f.reason;break
            peakacc=max(peakacc,math.hypot(cmd.vx-old.vx,cmd.vy-old.vy)/dt)
            peakyawacc=max(peakyawacc,abs(cmd.wz-old.wz)/dt)
            peakrpm=max(peakrpm,max(map(abs,wheel_rpm(cmd))))
            plant.step(cmd,dt)
            errors.append(math.hypot(x-plant.pose[0],y-plant.pose[1]))
            angles.append(abs(yaw-plant.pose[2]))
            velocities.append(math.hypot(cmd.vx,cmd.vy));limited+=int(f.reference_limited)
            if mode=='direct':
                assert (f.goal.x,f.goal.y,f.goal.yaw)==(f.mapped_goal.x,f.mapped_goal.y,f.mapped_goal.yaw)
                assert not f.reference_limited and not f.yaw_limited
        result[name]=dict(samples=len(errors),ended=ended,xy_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),
            xy_p95_m=float(np.percentile(errors,95)),xy_max_m=max(errors),yaw_p95_rad=float(np.percentile(angles,95)),
            command_peak_mps=max(velocities),peak_rpm=peakrpm,peak_accel_mps2=peakacc,peak_yaw_accel_radps2=peakyawacc,
            intermediate_limit_samples=limited)
        assert not ended and peakrpm<=c.limits.target_rpm+1e-6 and peakacc<=4.+1e-7 and peakyawacc<=12.+1e-7
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=('baseline','direct'));a=p.parse_args()
    if a.mode:print(json.dumps(run(a.mode)));sys.exit()
    data={mode:json.loads(subprocess.check_output([sys.executable,'-B',__file__,'--mode',mode],text=True)) for mode in ('baseline','direct')}
    data['scope']='OFFLINE_IDENTICAL_TARGET_REPLAY_NOMINAL_MODEL_NOT_REAL_VEHICLE'
    data['recording_sha256']=hashlib.sha256(RECORD.read_bytes()).hexdigest()
    data['improvement_percent']={k:100*(1-data['direct'][k]['xy_rmse_m']/data['baseline'][k]['xy_rmse_m']) for k in data['direct']}
    assert all(v>10 for v in data['improvement_percent'].values()),data['improvement_percent']
    (DOC/'COMPARISON.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(data,ensure_ascii=False,indent=2))
