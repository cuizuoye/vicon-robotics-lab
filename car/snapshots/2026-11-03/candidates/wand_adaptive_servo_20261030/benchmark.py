"""Deterministic same-input nominal replay. No ROS context or physical IO."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_browser_direct_2026-10-30/prepared_v1'
OUT=ROOT/'docs/wand_adaptive_servo_2026-10-30'
sys.path.insert(0,str(BASE));sys.path.insert(0,str(HERE))
from profile_direct import sim_config
from mecanum_wand_4wsb_control.direct_target import DirectSession
from mecanum_wand_4wsb_control.core import Pose
from mecanum_wand_4wsb_control.rotation_servo import wheel_rpm
from plant_direct import NominalPlant
from adaptive_servo import AdaptiveSession,Policy

def cases(include_record=True):
    paths={
        'slow_hand':[(i*.01,.35*math.sin(.5*i*.01),.2*(1-math.cos(.5*i*.01)),.35*math.sin(.5*i*.01)) for i in range(4001)],
        'fast_reverse':[(i*.01,.7*math.sin(1.25*i*.01),.35*(1-math.cos(.9*i*.01)),0.) for i in range(4001)],
        'large_6m_omni':[(i*.01,3.*math.sin(.25*i*.01),2.5*(1-math.cos(.25*i*.01)),0.) for i in range(6001)],
        'random_rotation':[(i*.01,0.,0.,1.5*math.sin(.9*i*.01)+.4*math.sin(1.7*i*.01)) for i in range(6001)],
        'large_xy_yaw':[(i*.01,3.*math.sin(.25*i*.01),2.5*(1-math.cos(.25*i*.01)),1.4*math.sin(.7*i*.01)) for i in range(6001)],
    }
    if include_record:
        source=ROOT/'docs/wand_browser_sim12_2026-10-30/prepared_v4/sessions/fast_software_2wZ90A/TRACE.jsonl'
        rows=[]
        with source.open() as stream:
            for line in stream:
                r=json.loads(line);g=r.get('supervisor',{}).get('raw_goal')
                if r.get('state')=='ACTIVE' and g:rows.append((r['t'],g['x'],g['y'],g['yaw']))
        first=rows[0];paths['recorded_human_289s']=[(t-first[0],x-first[1],y-first[2],a-first[3]) for t,x,y,a in rows]
    for path in paths.values():
        t,x,y,a=path[-1]
        path.extend((t+i*.01,x,y,a) for i in range(1,601))
    return paths

def run(factory,paths):
    results={}
    for name,samples in paths.items():
        c=sim_config();f=factory(c);plant=NominalPlant()
        _,x0,y0,a0=samples[0]
        for source,pose in (('car',Pose(0.,0.,c.ground_z_m)),('wand',Pose(x0,y0,1.,a0))):f.observe(source,pose,10.,source_stamp=100.)
        assert f.engage(10.)[0]
        xy=[];angles=[];accelerations=[];yawacc=[];speed=[];budget=[];angularbudget=[];last=0.;maxrpm=0.
        for t,x,y,a in samples[1:]:
            dt=t-last;last=t;now=10.+t
            f.observe('car',Pose(*plant.pose[:2],c.ground_z_m,plant.pose[2]),now,source_stamp=100.+t)
            f.observe('wand',Pose(x,y,1.,a),now,source_stamp=100.+t)
            old=f.command;cmd=f.tick(now)
            assert f.state=='ACTIVE',(name,f.reason)
            assert f.goal==f.mapped_goal and not f.reference_limited and not f.yaw_limited
            accelerations.append(math.hypot(cmd.vx-old.vx,cmd.vy-old.vy)/dt);yawacc.append(abs(cmd.wz-old.wz)/dt)
            maxrpm=max(maxrpm,max(map(abs,wheel_rpm(cmd))));speed.append(math.hypot(cmd.vx,cmd.vy))
            plant.step(cmd,dt)
            xy.append(math.hypot(x-x0-plant.pose[0],y-y0-plant.pose[1]));angles.append(abs(a-a0-plant.pose[2]))
            diag=getattr(f,'last_adaptive',{})
            budget.append(diag.get('accel_budget_mps2',c.max_accel_mps2));angularbudget.append(diag.get('yaw_accel_budget_radps2',c.max_yaw_accel_radps2))
        assert maxrpm<=c.limits.target_rpm+1e-6 and max(accelerations)<=4.+1e-6 and max(yawacc)<=12.+1e-6
        results[name]=dict(samples=len(xy),xy_rmse_m=float(np.sqrt(np.mean(np.square(xy)))),xy_p95_m=float(np.percentile(xy,95)),
            yaw_rmse_rad=float(np.sqrt(np.mean(np.square(angles)))),yaw_p95_rad=float(np.percentile(angles,95)),
            final_xy_error_m=xy[-1],final_yaw_error_rad=angles[-1],peak_command_mps=max(speed),peak_rpm=maxrpm,
            peak_accel_mps2=max(accelerations),peak_yaw_accel_radps2=max(yawacc),
            accel_budget_min=min(budget),accel_budget_max=max(budget),yaw_budget_min=min(angularbudget),yaw_budget_max=max(angularbudget))
    return results

def main():
    OUT.mkdir(exist_ok=True)
    parser=argparse.ArgumentParser();parser.add_argument('--sweep',action='store_true');a=parser.parse_args()
    paths=cases();baseline=run(lambda c:DirectSession(c.follower_config(),c.project_goal,c.project_reference),paths)
    choices=[Policy()]
    if a.sweep:choices=[Policy(position_gain=g,angle_gain=yg,accel_floor=floor) for g in (2.,3.,4.) for yg in (3.,4.) for floor in (.6,1.0)]
    candidates=[]
    for policy in choices:
        result=run(lambda c:AdaptiveSession(c.follower_config(),c.project_goal,c.project_reference,policy=policy),paths)
        ratios=[]
        for name,r in result.items():
            b=baseline[name]
            if b['xy_rmse_m']>1e-5:ratios.append(r['xy_rmse_m']/b['xy_rmse_m'])
            if b['yaw_rmse_rad']>1e-5:ratios.append(r['yaw_rmse_rad']/b['yaw_rmse_rad'])
        regressions={name:{metric:r[metric]/baseline[name][metric] for metric in ('xy_rmse_m','yaw_rmse_rad')
            if baseline[name][metric]>1e-5 and r[metric]>1.05*baseline[name][metric]}
            for name,r in result.items()}
        regressions={k:v for k,v in regressions.items() if v}
        candidates.append(dict(policy=asdict(policy),score=float(np.mean(ratios)),results=result,
            eligible=not regressions,regressions_over_5pct=regressions))
        print('policy '+json.dumps(dict(policy=asdict(policy),score=candidates[-1]['score'])),flush=True)
    acceptable=[v for v in candidates if v['eligible']]
    best=min(acceptable or candidates,key=lambda v:v['score'])
    record=dict(scope='OFFLINE_IDENTICAL_INPUT_NOMINAL_TAU_0P12_NOT_HARDWARE',passed=bool(acceptable),hard_limits_unchanged=True,
        baseline=baseline,candidates=candidates,selected=best,real_sign_human_accepted=False,hardware_output=False)
    (OUT/'BENCHMARK_V3.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(best,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
