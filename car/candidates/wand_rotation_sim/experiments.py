"""Deterministic software-only XY/yaw comparison; no live-input launch mode."""
from dataclasses import asdict,dataclass
import math

from harness import RotationHarness
from rotation_core import SimRotationLimits,RATES,ACCELS,ZERO,MAX_RPM


def smooth(t):
    t=max(0.,min(1.,t));return 10*t**3-15*t**4+6*t**5


@dataclass(frozen=True)
class Case:
    name:str
    duration:float
    move_s:float=1.5
    mixed:bool=False
    spin:bool=False
    lag_s:float=.12

    def gesture(self,t):
        if self.spin:angle=4*math.pi*smooth(t/10.)
        else:
            angle=0.;elapsed=.5
            for target in (math.pi/2,0.,-math.pi/2,0.):
                if t<elapsed:break
                if t<elapsed+self.move_s:
                    angle=angle+(target-angle)*smooth((t-elapsed)/self.move_s);break
                angle=target;elapsed+=self.move_s+.6
        if self.mixed:
            phase=1.2*(t-.5) if t>.5 else 0.
            envelope=smooth(t/1.)
            return .20*math.sin(phase)*envelope,.16*(1-math.cos(phase))*envelope,angle
        return 0.,0.,angle


def cases():
    return (Case('gentle_90deg_2p5s',15.0,2.5),Case('normal_90deg_1p5s',12.,1.5),
        Case('fast_90deg_0p8s_challenge',9.,.8),Case('mixed_xy_yaw',12.,1.5,True),
        Case('continuous_720deg',14.,spin=True))


def rms(values):return math.sqrt(sum(v*v for v in values)/len(values)) if values else None
def quantile(values,q):return sorted(values)[min(len(values)-1,int(q*(len(values)-1)))]


def run_case(c,case,*,feedforward=True,keep_rows=False):
    h=RotationHarness(c,lag_s=case.lag_s,feedforward=feedforward);h.start()
    start=h.now;old=ZERO;rows=[];max_accel=max_yaw_accel=0.
    for i in range(1,round(case.duration/.02)+1):
        t=i*.02;f,l,turn=case.gesture(t);h.set_gesture(f,l,turn)
        output=h.step();row=dict(h.rows[-1],t=t,hand_turn=turn)
        if h.sup.state!='ACTIVE' or h.gate.fault:
            return dict(case=case.name,limits=c.name,passed=False,at_s=t,
                reason=h.sup.reason,gate_fault=h.gate.fault),rows
        error=h.origin.yaw+turn-row['car_yaw']
        row['angular_error_deg']=math.degrees(error)
        max_accel=max(max_accel,math.hypot(output.vx-old.vx,output.vy-old.vy)/.02)
        max_yaw_accel=max(max_yaw_accel,abs(output.wz-old.wz)/.02)
        old=output;rows.append(row)
    errors=[abs(r['angular_error_deg']) for r in rows]
    moving=[abs(r['angular_error_deg']) for a,r in zip(rows,rows[1:]) if abs(r['hand_turn']-a['hand_turn'])>.0001]
    same_time_xy=[r['same_time_error_m'] for r in rows]
    peak_rpm=max(abs(v) for r in rows for v in r['wheel_rpm'])
    release_angle=h.plant.pose.yaw;release_speed=h.plant.velocity.wz
    h.held=False;immediate=h.step()==ZERO;still_at=None
    for i in range(1,101):
        h.step()
        if still_at is None and abs(h.plant.velocity.wz)<.02:still_at=i*.02
    result=dict(case=case.name,limits=c.name,passed=True,model_only=True,
        passed_meaning='No unexpected stop; command/geometry safety bounds respected, NOT automatic tracking acceptance',
        tracking_meets_normal_threshold=(rms(errors)<=8. and quantile(errors,.95)<=15.
            and errors[-1]<=2. and rms(same_time_xy)<=.08),
        yaw_feedforward=feedforward,model_lag_s=case.lag_s,
        angular_rms_deg=rms(errors),angular_moving_rms_deg=rms(moving),
        angular_p95_deg=quantile(errors,.95),angular_max_deg=max(errors),
        angular_final_deg=errors[-1],xy_rms_m=rms(same_time_xy),
        max_command_speed_mps=max(r['command_speed_mps'] for r in rows),
        max_command_yaw_rate_radps=max(abs(r['command_wz']) for r in rows),
        max_command_accel_mps2=max_accel,max_command_yaw_accel_radps2=max_yaw_accel,
        max_wheel_rpm=peak_rpm,release_output_zero_immediate=immediate,
        release_model_yaw_rate_radps=release_speed,
        release_model_additional_angle_deg=math.degrees(h.plant.pose.yaw-release_angle),
        release_model_still_s=still_at,software_feedback_stop_confirmed=h.sup.stop_confirmed)
    assert result['max_command_speed_mps']<=.70+1e-7
    assert max_accel<=.80+1e-6 and max_yaw_accel<=c.max_yaw_accel_radps2+1e-6
    assert result['max_command_yaw_rate_radps']<=c.max_yaw_rate_radps+1e-7 and peak_rpm<=MAX_RPM+1e-6
    return result,rows if keep_rows else []


def sweep():
    rows=[]
    for rate in RATES:
        for accel in ACCELS:
            c=SimRotationLimits(rate,accel)
            results=[run_case(c,case)[0] for case in cases() if case.name in ('normal_90deg_1p5s','mixed_xy_yaw')]
            rows.append(dict(rate_radps=rate,accel_radps2=accel,limits=c.name,results=results,
                qualifies=all(r['passed'] and r['angular_p95_deg']<=15.
                    and r['angular_rms_deg']<=8. and r['angular_final_deg']<=2.
                    and r['xy_rms_m']<=.08 for r in results)))
    choices=[r for r in rows if r['qualifies']]
    selected=min(choices,key=lambda r:(r['rate_radps'],r['accel_radps2'])) if choices else None
    return dict(kind='OFFLINE_ROTATION_PARAMETER_SWEEP',model_only=True,
        xy_limits_mps_mps2=[.70,.80],mapping_xy_yaw=[1.,1.],
        preserved_arena=dict(forward=[-1.5,.5],left=[-.4,.5],buffer_m=.50),
        qualification='Both normal and combined: RMS<=8deg, p95<=15deg, final<=2deg, XY RMS<=8cm',
        rows=rows,recommended=selected,physical_hardware_qualified=False)
