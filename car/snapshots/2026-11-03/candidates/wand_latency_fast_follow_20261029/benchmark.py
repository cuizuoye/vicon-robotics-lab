"""Virtual-time comparison through the unmodified supervisor and final gate.

Reuse the previous approximate 0.12s plant; never call this hardware latency.
All guard inputs in this harness are synthetic. No ROS/socket/serial/SDK I/O.
"""
import argparse
from dataclasses import asdict,dataclass
import json
import math
from pathlib import Path

from common import (BASE,Pose,Velocity,RotationSupervisor,RotationFinalGate,
                    Plant,Lease,ZERO,wheel_rpm,config,protected_hashes,fresh_output,save,digest)
from catchup import FastCatchupSession


class Harness:
    def __init__(self,candidate=False,lag=.12,gains=(.98,.88,.95),origin_yaw=0.):
        self.c=config();self.sup=RotationSupervisor(self.c);self.gate=RotationFinalGate(self.c)
        if candidate:
            self.sup.follower=FastCatchupSession(self.c.follower_config(),self.c.project_goal,self.c.project_reference)
        self.plant=Plant(Pose(0.,0.,self.c.ground_z_m,origin_yaw))
        # Same first-order model, with enumerated uncertainty used in BOTH arms.
        self.lag,self.gains=lag,gains
        self.now,self.ros=10.,1e6;self.seq=0;self.held=False
        self.hand=Pose(10.,10.,1.,0.);self.output=ZERO
        self.pose_on={'car':True,'wand':True};self.pose_stamp={}
        self.feedback_on=self.lease_on=self.command_on=True
        self.graph_ok=True;self.fault_bits=0;self.roll=self.pitch=0.;self.freeze=False

    def gesture(self,x,y,turn=0.,z=1.): self.hand=Pose(10+x,10+y,z,turn)

    def step(self,dt=.02):
        self.now+=dt;self.ros+=dt;self.seq+=1
        for i in (self.sup.inputs,self.gate.inputs):
            i.graph_ok=self.graph_ok
            for name,pose in [('car',self.plant.pose),('wand',self.hand)]:
                if self.pose_on[name]:
                    i.observe_pose(name,pose,self.pose_stamp.get(name,self.ros),self.now,self.ros,
                                   self.roll if name=='car' else 0.,self.pitch if name=='car' else 0.)
            if self.lease_on:i.observe_lease(Lease('synthetic-operator',self.seq,self.now,self.ros,self.held),self.now,self.ros)
            if self.feedback_on:
                i.observe_feedback(self.seq,self.now,self.ros,self.now,self.ros,
                    output_zero=self.output==ZERO,enabled=self.output!=ZERO,
                    speed=math.hypot(self.plant.velocity.vx,self.plant.velocity.vy),
                    yaw_rate=abs(self.plant.velocity.wz),fault_bits=self.fault_bits)
        frame=self.sup.tick(self.now,self.ros)
        if self.command_on:self.gate.receive(frame,self.now,self.ros)
        output=self.gate.tick(self.now,self.ros)
        self.sup.observe_gate(steady=self.now,ros=self.ros,epoch=self.gate.primed_epoch,
                             fault=self.gate.fault,hard_fault=self.gate.hard_fault)
        self.output=output
        if not self.freeze:
            alpha=-math.expm1(-dt/self.lag)
            self.plant.velocity=Velocity(*(a+alpha*(b-a) for a,b in zip(
                asdict(self.plant.velocity).values(),asdict(output).values())))
            v=self.plant.velocity;c,s=math.cos(self.plant.pose.yaw),math.sin(self.plant.pose.yaw)
            vx,vy,w=v.vx*self.gains[0],v.vy*self.gains[1],v.wz*self.gains[2]
            p=self.plant.pose
            self.plant.pose=Pose(p.x+(c*vx-s*vy)*dt,p.y+(s*vx+c*vy)*dt,p.z,p.yaw+w*dt)
        else:self.plant.velocity=ZERO
        return output

    def run(self,seconds):
        for _ in range(round(seconds/.02)):self.step()

    def start(self):
        self.run(1.8);ok,reason=self.sup.request_start(self.now,self.ros)
        if not ok:raise AssertionError(reason)
        self.run(.1);self.held=True;self.step()
        assert self.sup.state=='ACTIVE' and not self.gate.fault


@dataclass(frozen=True)
class Sample:
    t:float
    x:float
    y:float
    yaw:float=0.
    age:float=0.
    source_offset:float|None=None


def synthetic_cases():
    result={}
    for name,amplitude,omega in [('gentle',.25,1.2),('normal',.30,5/3),('fast_reversal',.30,2.5)]:
        result[name]=[Sample(i*.02,amplitude*(1-math.cos(omega*i*.02)),0.) for i in range(1,1001)]
    result['circle']=[Sample(i*.02,.3*math.sin(1.2*i*.02),.3*(1-math.cos(1.2*i*.02))) for i in range(1,1001)]
    result['mixed']=[Sample(i*.02,.3*math.sin(1.2*i*.02),.3*(1-math.cos(1.2*i*.02)),.08*math.sin(i*.02)) for i in range(1,1001)]
    for k in range(8):
        a=k*math.pi/4
        result['burst_hold_'+str(k)]=[Sample(i*.02,min(i*.02,1.)*math.cos(a),min(i*.02,1.)*math.sin(a)) for i in range(1,751)]
    result['jitter']=[Sample(i*.02,.0002*math.sin(31*i*.02),.0002*math.cos(29*i*.02)) for i in range(1,501)]
    return result


def recorded_case(session,seconds=60.):
    path=BASE/session/'observation.jsonl'
    candidates={};statuses=[]
    with path.open() as stream:
        for line in stream:
            if not line.strip('\x00\r\n '):continue
            r=json.loads(line);v=r['data'];v=json.loads(v) if isinstance(v,str) else v
            if r['topic']=='candidate' and v['frame']['state']=='ACTIVE':candidates[v['frame']['issued_steady']]=v
            elif r['topic']=='supervisor_status' and v['state']=='ACTIVE':statuses.append(v)
    start=min(s['issued_steady'] for s in statuses);start_ros=min(s['issued_ros'] for s in statuses)
    first=statuses[0]['anchors']['car_origin'];samples=[]
    for s in statuses:
        if s['issued_steady']-start>=seconds:break
        c=candidates.get(s['issued_steady'])
        if c is None:raise ValueError('exact selected frame missing')
        g=s['mapped_goal']
        if g is None:raise ValueError('mapped goal missing')
        samples.append(Sample(s['issued_steady']-start+.02,g['x']-first['x'],g['y']-first['y'],
            s['anchors'].get('turn',0.),s['issued_ros']-c['sign_stamp'],c['sign_stamp']-start_ros+.02))
    return samples,dict(path=str(path),sha256=digest(path),
        initial_car_yaw_rad=first['yaw'],
        transformation='normalize hand/car origins; keep world XY displacement, relative yaw, original car heading and source timing; old car feedback NOT reused')


def percentile(v,q):return sorted(v)[min(len(v)-1,int(q*(len(v)-1)))]


def compare_case(name,samples,candidate,lag=.12,gains=(.98,.88,.95),origin_yaw=0.):
    h=Harness(candidate,lag,gains,origin_yaw);h.start();origin_ros=h.ros
    previous=ZERO;t_old=0.;errors=[];rows=[];max_accel=0.;settled=None;held_start=None
    last_goal=(samples[-1].x,samples[-1].y)
    for sample in samples:
        dt=sample.t-t_old
        if not 0<dt<=.10:raise ValueError('source/control gap requires split, never interpolate')
        h.gesture(sample.x,sample.y,sample.yaw)
        h.pose_stamp['wand']=(origin_ros+sample.source_offset if sample.source_offset is not None else h.ros+dt-sample.age)
        output=h.step(dt)
        if h.sup.state!='ACTIVE' or h.gate.fault:
            return dict(case=name,candidate=candidate,passed=False,at_s=sample.t,reason=h.sup.reason,gate=h.gate.fault),rows
        a=math.hypot(output.vx-previous.vx,output.vy-previous.vy)/dt
        max_accel=max(max_accel,a);peak=max(abs(r) for r in wheel_rpm(output))
        assert math.hypot(output.vx,output.vy)<=.70+1e-8 and a<=.80+1e-6 and peak<=400+1e-6
        assert abs(output.wz)<=.10+1e-8 and abs(output.wz-previous.wz)/dt<=.40+1e-6
        previous=output;t_old=sample.t
        error=math.hypot(sample.x-h.plant.pose.x,sample.y-h.plant.pose.y);errors.append(error)
        if math.dist((sample.x,sample.y),last_goal)<1e-8:
            if held_start is None:held_start=sample.t
            if error<.03 and settled is None:settled=sample.t
        else:held_start=settled=None
        rows.append(dict(t=sample.t,target_x=sample.x,target_y=sample.y,car_x=h.plant.pose.x,
            car_y=h.plant.pose.y,error=error,speed=math.hypot(output.vx,output.vy),
            catchup=getattr(h.sup.follower,'catchup_active',False),reference_limited=h.sup.follower.reference_limited))
    h.held=False;immediate_zero=h.step()==ZERO;h.run(2.)
    return dict(case=name,candidate=candidate,passed=True,model_only=True,model_lag_s=lag,model_gains=gains,
        xy_rms_m=math.sqrt(sum(e*e for e in errors)/len(errors)),xy_p95_m=percentile(errors,.95),
        xy_max_m=max(errors),final_error_m=errors[-1],max_accel_mps2=max_accel,
        catchup_samples=sum(r['catchup'] for r in rows),
        moving_feedforward_samples=getattr(h.sup.follower,'moving_feedforward_ticks',0),
        hold_to_3cm_s=settled-held_start if settled is not None and held_start is not None else None,
        release_zero_immediate=immediate_zero,model_stop_confirmed=h.sup.stop_confirmed),rows


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',required=True);a=ap.parse_args()
    out=fresh_output(a.output);before=protected_hashes();save(out/'protected_before.json',before)
    cases=synthetic_cases();provenance={};initial_yaws={}
    for name in ['oneclick_session_jiqg5pw1','oneclick_session_ejw7_ctw']:
        cases['recorded_'+name],provenance[name]=recorded_case(name)
        initial_yaws['recorded_'+name]=provenance[name]['initial_car_yaw_rad']
    results=[]
    for lag,gains,label in [(.12,(.98,.88,.95),'nominal'),(.06,(1.,1.,1.),'low_lag'),(.20,(.90,.80,.90),'high_lag')]:
        for name,samples in cases.items():
            for candidate in (False,True):
                r,rows=compare_case(name,samples,candidate,lag,gains,initial_yaws.get(name,0.))
                r['plant_case']=label;results.append(r)
                if label=='nominal':
                    save(out/(name+('_candidate' if candidate else '_baseline')+'.json'),rows)
    save(out/'protected_after.json',protected_hashes())
    summary=dict(kind='SIM_ONLY_FAST_CATCHUP_COMPARISON',results=results,provenance=provenance,
        candidate_source_sha256=digest(Path(__file__).with_name('catchup.py')),
        normal_limits_unchanged=dict(speed=.70,accel=.80,yaw_rate=.10,yaw_accel=.40,rpm=400),
        unmodified_supervisor_and_gate=True,physical_hardware_qualified=False,
        original_sources_unchanged=before==protected_hashes())
    summary['functional_passed']=all(r['passed'] and r.get('release_zero_immediate') and
        r.get('model_stop_confirmed') for r in results) and summary['original_sources_unchanged']
    quality=[]
    for label in ('nominal','low_lag','high_lag'):
        for name in cases:
            pair=[r for r in results if r['plant_case']==label and r['case']==name]
            if not all(r['passed'] for r in pair):continue
            b,n=pair
            quality.append(dict(case=name,plant_case=label,
                rms_improvement_percent=100*(1-n['xy_rms_m']/b['xy_rms_m']) if b['xy_rms_m'] else 0.,
                regression_within_5_percent=n['xy_rms_m']<=b['xy_rms_m']*1.05+1e-12))
    summary['quality_comparisons']=quality
    summary['no_large_rms_regression_on_tested_cases']=all(q['regression_within_5_percent'] for q in quality)
    summary['passed_meaning']='Functional guard/cap tests only; NOT hardware or universal tracking qualification'
    save(out/'summary.json',summary)
    for label in ['nominal','low_lag','high_lag']:
        for name in cases:
            pair=[r for r in results if r['plant_case']==label and r['case']==name]
            if not all(r['passed'] for r in pair):print(label,name,'FAIL',pair);continue
            b,n=pair;improvement=100*(1-n['xy_rms_m']/b['xy_rms_m']) if b['xy_rms_m'] else 0.
            print(label,name,'rms',round(b['xy_rms_m'],4),round(n['xy_rms_m'],4),'improvement%',round(improvement,1),
                  'hold',b['hold_to_3cm_s'],n['hold_to_3cm_s'])
    if not summary['functional_passed'] or not summary['no_large_rms_regression_on_tested_cases']:
        raise SystemExit(1)


if __name__=='__main__':main()
