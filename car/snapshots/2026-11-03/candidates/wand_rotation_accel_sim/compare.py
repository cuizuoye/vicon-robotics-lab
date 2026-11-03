"""Same-time error comparison; identical plant, gains, input and safety gates."""
from dataclasses import dataclass
import math

from accel_core import AccelerationLimits, XY_ACCELS, YAW_ACCELS, ZERO
from harness import RotationHarness
from experiments import Case as YawCase, rms, quantile, smooth


@dataclass(frozen=True)
class Gesture:
    name:str
    duration:float=12.
    mixed:bool=False
    def gesture(self,t):
        p=max(0.,t-.5)
        if self.name=='xy_25cm_reversals':
            x=0.; elapsed=.5
            for target in (.25,0.,-.25,0.):
                if t<elapsed: break
                if t<elapsed+.8:
                    x=x+(target-x)*smooth((t-elapsed)/.8); break
                x=target; elapsed+=1.5
            return x,0.,0.
        envelope=smooth(p/.5)
        return (.20*math.sin(2.5*p)*envelope,.10*(1-math.cos(2.5*p))*envelope,
            .5*math.sin(2.5*p)*envelope if self.mixed else 0.)


def cases():
    return (YawCase('normal_90deg_1p5s',12.,1.5),
        YawCase('faster_90deg_1p0s',10.,1.),
        YawCase('mixed_xy_yaw',12.,1.5,True),
        Gesture('xy_25cm_reversals',8.),Gesture('xy_continuous_reversals'),
        Gesture('rapid_mixed_xy_yaw',mixed=True))


def run(c,case,lag_s=.12,keep=False):
    h=RotationHarness(c,lag_s=lag_s); h.start()
    previous=ZERO; rows=[]; max_a=max_aa=0.
    for i in range(1,round(case.duration/.02)+1):
        t=i*.02; f,l,angle=case.gesture(t); h.set_gesture(f,l,angle)
        out=h.step(); r=dict(h.rows[-1],t=t,hand_turn=angle,
            yaw_error_deg=math.degrees(h.origin.yaw+angle-h.rows[-1]['car_yaw']))
        if h.sup.state!='ACTIVE' or h.gate.fault:
            return dict(case=case.name,passed=False,reason=h.sup.reason,gate_fault=h.gate.fault),rows
        max_a=max(max_a,math.hypot(out.vx-previous.vx,out.vy-previous.vy)/.02)
        max_aa=max(max_aa,abs(out.wz-previous.wz)/.02)
        previous=out; rows.append(r)
        assert c.inside(h.plant.pose)
    xerr=[r['same_time_error_m'] for r in rows]; yaw=[abs(r['yaw_error_deg']) for r in rows]
    peak=max(abs(v) for r in rows for v in r['wheel_rpm'])
    maxv=max(r['command_speed_mps'] for r in rows)
    maxw=max(abs(r['command_wz']) for r in rows)
    h.held=False; zero=h.step()==ZERO
    h.run(2.)
    result=dict(case=case.name,passed=True,model_only=True,plant_lag_s=lag_s,
        xy_rms_m=rms(xerr),xy_p95_m=quantile(xerr,.95),xy_max_m=max(xerr),
        yaw_rms_deg=rms(yaw),yaw_p95_deg=quantile(yaw,.95),yaw_max_deg=max(yaw),
        max_command_speed_mps=maxv,max_command_yaw_rate_radps=maxw,
        max_command_accel_mps2=max_a,max_command_yaw_accel_radps2=max_aa,
        max_wheel_rpm=peak,stop_output_immediate_zero=zero,
        simulated_stop_confirmed=h.sup.stop_confirmed,
        tracking_meets_normal_threshold=(rms(yaw)<=8 and quantile(yaw,.95)<=15 and rms(xerr)<=.08))
    assert maxv<=.7+1e-7 and maxw<=2.+1e-7 and peak<=400.+1e-6
    assert max_a<=c.max_accel_mps2+1e-6 and max_aa<=c.max_yaw_accel_radps2+1e-6
    assert zero and h.sup.stop_confirmed
    return result,rows if keep else []


def sweep():
    rows=[]
    for a in XY_ACCELS:
        for aa in YAW_ACCELS:
            c=AccelerationLimits(a,aa)
            results=[run(c,case)[0] for case in cases()]
            if not all(r['passed'] for r in results): raise ValueError('scenario safety failure: '+c.name)
            rows.append(dict(xy_accel_mps2=a,yaw_accel_radps2=aa,results=results))
    base=rows[0]; normal=cases()[0].name
    def mean_selected(row,names,key):
        return sum(r[key] for r in row['results'] if r['case'] in names)/len(names)
    xy_names=('xy_25cm_reversals','xy_continuous_reversals','rapid_mixed_xy_yaw')
    yaw_names=('normal_90deg_1p5s','faster_90deg_1p0s','mixed_xy_yaw')
    bx=mean_selected(base,xy_names,'xy_rms_m'); by=mean_selected(base,yaw_names,'yaw_rms_deg')
    for row in rows:
        row['xy_error_ratio']=mean_selected(row,xy_names,'xy_rms_m')/bx
        row['yaw_error_ratio']=mean_selected(row,yaw_names,'yaw_rms_deg')/by
        row['score']=.5*(row['xy_error_ratio']+row['yaw_error_ratio'])
        nr=next(r for r in row['results'] if r['case']==normal)
        row['qualifies']=bool(row['xy_error_ratio']<=.90 and row['yaw_error_ratio']<=1.02
            and nr['tracking_meets_normal_threshold'])
    choices=[r for r in rows if r['qualifies']]
    if not choices: raise ValueError('No acceleration pair measurably improves retained-speed model')
    best=min(r['score'] for r in choices)
    # Prefer the smallest pair within 1% of the best normalized error score.
    selected=min((r for r in choices if r['score']<=best*1.01),
        key=lambda r:(r['xy_accel_mps2'],r['yaw_accel_radps2']))
    return dict(model_only=True,selection='>=10% XY RMS improvement, yaw no >2% degradation, normal yaw acceptance; smallest acceleration pair within 1% of best score',
        fixed_speed_mps=.70,fixed_yaw_rate_radps=2.,fixed_plant_lag_s=.12,
        same_time_error_no_alignment=True,rows=rows,recommended=selected,
        hardware_capability_proven=False)
