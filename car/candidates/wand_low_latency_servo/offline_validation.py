"""Deterministic A/B dynamics and full supervisor/gate fault harness.

Virtual time, synthetic poses and an explicitly approximate lag/gain plant.
No ROS initialization, socket, serial, Vicon SDK or hardware access. Physical
latency, braking, current, temperature and coverage are NOT measured here.
"""
import argparse
import csv
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

from .core import Pose, Velocity, WandSession, limit_vector
from .safety_core import FinalGate, Lease, Supervisor, ZERO
from .servo_config import ServoSafetyConfig


class BaselineSession(WandSession):
    def observe(self, source, pose, stamp, *, source_stamp=None):
        return super().observe(source, pose, stamp)


class Plant:
    def __init__(self, pose, lag_s=.12, gains=(.98,.88,.95)):
        self.pose, self.velocity = pose, ZERO
        self.lag_s, self.gains = lag_s, gains

    def step(self, command, dt):
        alpha = -math.expm1(-dt/self.lag_s)
        self.velocity = Velocity(*(a+alpha*(b*g-a) for a,b,g in zip(
            asdict(self.velocity).values(),asdict(command).values(),self.gains)))
        c,s=math.cos(self.pose.yaw),math.sin(self.pose.yaw)
        v=self.velocity
        self.pose=Pose(self.pose.x+(c*v.vx-s*v.vy)*dt,
                       self.pose.y+(s*v.vx+c*v.vy)*dt,self.pose.z,self.pose.yaw+v.wz*dt)


class Harness:
    def __init__(self, config, baseline=False, origin_local=(1.5,1.5), lag_s=.12,
                 gains=(.98,.88,.95)):
        self.c=config
        self.sup,self.gate=Supervisor(config),FinalGate(config)
        if baseline:
            old=replace(config.follower_config(),filter_tau_s=.08)
            self.sup.follower=BaselineSession(old,config.project_goal,config.project_reference)
        self.now,self.ros=10.,1_000_000_000.
        self.sequence=0
        self.origin=Pose(*config.geometry().world(*origin_local),config.ground_z_m,
                         config.arena_heading_rad)
        self.plant=Plant(self.origin,lag_s,gains)
        self.hand_origin=Pose(10.,10.,1.)
        self.hand=self.hand_origin
        self.held=False
        self.pose_on={'car':True,'wand':True}
        self.pose_stamp={}
        self.feedback_on=self.lease_on=self.supervisor_on=self.command_link_on=self.gate_on=True
        self.roll=self.pitch=0.
        self.fault_bits=0
        self.output=self.plant_target=ZERO
        self.last_gate=self.now
        self.rows=[]
        self.modify_frame=lambda f:f

    def set_hand(self, forward, left, z=1., yaw=0.):
        c,s=math.cos(self.origin.yaw),math.sin(self.origin.yaw)
        self.hand=Pose(self.hand_origin.x+c*forward-s*left,
                       self.hand_origin.y+s*forward+c*left,z,yaw)

    def step(self, dt=.02, ros_dt=None):
        self.now+=dt
        self.ros+=dt if ros_dt is None else ros_dt
        self.sequence+=1
        for inputs in (self.sup.inputs,self.gate.inputs):
            for source,pose in (('car',self.plant.pose),('wand',self.hand)):
                if self.pose_on[source]:
                    inputs.observe_pose(source,pose,self.pose_stamp.get(source,self.ros),
                        self.now,self.ros,self.roll if source=='car' else 0.,
                        self.pitch if source=='car' else 0.)
            if self.lease_on:
                inputs.observe_lease(Lease('offline-operator',self.sequence,self.now,self.ros,self.held),
                                     self.now,self.ros)
            if self.feedback_on:
                inputs.observe_feedback(self.sequence,self.now,self.ros,self.now,self.ros,
                    output_zero=self.plant_target==ZERO,enabled=self.plant_target!=ZERO,
                    speed=math.hypot(self.plant.velocity.vx,self.plant.velocity.vy),
                    yaw_rate=abs(self.plant.velocity.wz),fault_bits=self.fault_bits)
        if self.supervisor_on:
            frame=self.sup.tick(self.now,self.ros)
            if self.command_link_on:
                self.gate.receive(self.modify_frame(frame),self.now,self.ros)
        if self.gate_on:
            self.output=self.gate.tick(self.now,self.ros)
            self.plant_target,self.last_gate=self.output,self.now
            self.sup.observe_gate(steady=self.now,ros=self.ros,epoch=self.gate.primed_epoch,
                                 fault=self.gate.fault,hard_fault=self.gate.hard_fault)
        if self.now-self.last_gate>.15:
            self.plant_target=ZERO  # Synthetic independent downstream watchdog.
        f=self.sup.follower
        target=f.mapping.goal(self.hand) if f.mapping else self.origin
        target=self.c.project_goal(target,self.origin,self.c.max_goal_radius_m)
        local=self.c.geometry().local(self.plant.pose.x,self.plant.pose.y)
        goal=f.goal
        self.rows.append(dict(time_s=self.now,source_time_s=self.ros,
            car_x=self.plant.pose.x,car_y=self.plant.pose.y,car_local_f=local[0],car_local_l=local[1],
            target_x=target.x,target_y=target.y,goal_x=goal.x if goal else math.nan,
            goal_y=goal.y if goal else math.nan,
            same_time_error_m=math.hypot(target.x-self.plant.pose.x,target.y-self.plant.pose.y),
            command_vx=self.output.vx,command_vy=self.output.vy,command_wz=self.output.wz,
            command_speed_mps=math.hypot(self.output.vx,self.output.vy),
            plant_speed_mps=math.hypot(self.plant.velocity.vx,self.plant.velocity.vy),
            reference_limited=f.reference_limited,state=self.sup.state,reason=self.sup.reason,
            gate_fault=self.gate.fault))
        self.plant.step(self.plant_target,max(0.,dt))
        return self.output

    def run(self, seconds):
        for _ in range(round(seconds/.02)):
            self.step()

    def start(self):
        self.run(1.8)
        ok,reason=self.sup.request_start(self.now,self.ros)
        assert ok,(reason,self.sup.inputs.health(self.now,self.ros))
        self.run(.1)
        assert self.gate.primed_epoch==self.sup.epoch
        self.held=True
        self.step()
        assert self.sup.state=='ACTIVE',self.sup.reason

    def moving(self):
        self.start()
        for i in range(1,76):
            self.set_hand(.001*i,0.)
            self.step()
        assert math.hypot(self.output.vx,self.output.vy)>.01,(self.sup.reason,self.gate.fault)


def quantile(values,q):
    values=sorted(values)
    return values[min(len(values)-1,int(q*(len(values)-1)))]


def trajectory(name,t):
    if name=='ramp_hold_return':
        x=.1*min(t,3.) if t<6. else max(0.,.3-.1*(t-6.))
        return x,0.
    if name=='circle':
        return .10*math.sin(.6*t),.10*(1-math.cos(.6*t))
    if name=='reversals':
        phase=t%8.
        return (.06*phase if phase<2. else .12-.06*(phase-2.) if phase<6. else -.12+.06*(phase-6.)),0.
    if name=='large_target':
        return min(t*.5,1.),min(t*.25,.5)
    if name=='boundary':
        return min(t*.2,4.),min(t*.2,4.)
    if name=='jitter_hold':
        return .0002*math.sin(t*39.),.0002*math.cos(t*31.)
    raise ValueError(name)


def simulate(config,name,baseline=False,lag_s=.12,gains=(.98,.88,.95)):
    origin=(0.,0.) if name=='boundary' else (1.5,1.5)
    h=Harness(config,baseline,origin,lag_s,gains)
    h.start()
    start=h.now
    index=len(h.rows)
    duration={'ramp_hold_return':16.,'circle':32.,'reversals':24.,
              'large_target':22.,'boundary':40.,'jitter_hold':12.}[name]
    old=ZERO
    maximum_accel=0.
    onset=None
    command_onset=None
    max_reference_step=0.
    old_goal=h.sup.follower.goal
    for i in range(1,round(duration/.02)+1):
        t=i*.02
        x,y=trajectory(name,t)
        h.set_hand(x,y,z=1+.15*math.sin(t),yaw=.5*math.sin(t))
        command=h.step()
        assert h.sup.state=='ACTIVE',(name,baseline,t,h.sup.reason,h.gate.fault)
        maximum_accel=max(maximum_accel,math.hypot(command.vx-old.vx,command.vy-old.vy)/.02)
        old=command
        if command_onset is None and math.hypot(command.vx,command.vy)>=.001:
            command_onset=h.now-start
        if onset is None and math.hypot(h.plant.velocity.vx,h.plant.velocity.vy)>=.01:
            onset=h.now-start
        goal=h.sup.follower.goal
        max_reference_step=max(max_reference_step,math.hypot(goal.x-old_goal.x,goal.y-old_goal.y))
        old_goal=goal
        assert config.inside(h.plant.pose)
        assert config.goal_inside(h.sup.follower.mapped_goal)
    rows=h.rows[index:]
    errors=[r['same_time_error_m'] for r in rows]
    result=dict(case=name,controller='baseline' if baseline else 'continuous',
        model_lag_s=lag_s,model_gains=gains,passed=True,
        physical_hardware_measurement=False,duration_virtual_s=duration,
        command_onset_to_0001mps_s=command_onset,
        onset_to_001mps_s=onset,
        rms_same_time_error_m=math.sqrt(sum(e*e for e in errors)/len(errors)),
        p95_same_time_error_m=quantile(errors,.95),max_same_time_error_m=max(errors),
        final_same_time_error_m=errors[-1],max_speed_mps=max(r['command_speed_mps'] for r in rows),
        max_accel_mps2=maximum_accel,max_reference_step_m=max_reference_step,
        reference_lead_limited_frames=sum(r['reference_limited'] for r in rows))
    assert result['max_speed_mps']<=.20+1e-8 and maximum_accel<=.08+1e-7
    assert max_reference_step<=.20*.02+1e-7
    h.held=False
    assert h.step()==ZERO
    h.run(2.)
    assert h.sup.state=='PAUSED' and h.sup.stop_confirmed
    return result,rows


def load_config(profile):
    value=json.loads(Path(profile).read_text())
    if value.get('offline_only') is not True:
        raise ValueError('offline profile required')
    return ServoSafetyConfig(**value['safety'])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError('fresh evidence directory required')
    args.output.mkdir(parents=True)
    config=load_config(args.profile)
    reports=[]
    all_rows={}
    for name in ('ramp_hold_return','circle','reversals','large_target','boundary','jitter_hold'):
        for baseline in (True,False):
            result,rows=simulate(config,name,baseline)
            reports.append(result)
            all_rows[(name,result['controller'])]=rows
            with (args.output/(name+'_'+result['controller']+'.csv')).open('w') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]))
                writer.writeheader();writer.writerows(rows)
    report=dict(offline_only=True,hardware_started=False,firmware_changed=False,
        source='synthetic trajectories with full Supervisor + unchanged independent FinalGate',
        model='first-order lag .12s, XY gains .98/.88, yaw .95; NOT identified vehicle dynamics',
        cautions=['No live Vicon or camera-to-wheel latency measurement',
                  'No physical braking/current/temperature/coverage acceptance',
                  'Unsafe-speed human motion remains unsynchronizable under the retained .20m/s/.08m/s2 limits'],
        cases=reports)
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    plot_comparison(config,all_rows,args.output/'comparison.png')
    print(json.dumps({k:v for k,v in report.items() if k!='cases'},indent=2))
    for r in reports:
        print(f"{r['case']:18} {r['controller']:10} RMS={r['rms_same_time_error_m']:.5f}m "
              f"P95={r['p95_same_time_error_m']:.5f}m onset={r['onset_to_001mps_s']}s")
    return report


def plot_comparison(config,all_rows,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(12,10),constrained_layout=True)
    for row,name in enumerate(('ramp_hold_return','circle','reversals')):
        first=all_rows[(name,'baseline')];start=first[0]['time_s']-.02
        def forward(x,y):return config.geometry().local(x,y)[0]-1.5
        t=[r['time_s']-start for r in first]
        if name=='circle':
            axes[row,0].plot([forward(r['target_x'],r['target_y']) for r in first],
                [config.geometry().local(r['target_x'],r['target_y'])[1]-1.5 for r in first],
                '--',color='black',label='1:1 wand target')
            axes[row,0].set(xlabel='Forward (m)',ylabel='Left (m)')
            axes[row,0].axis('equal')
        else:
            axes[row,0].plot(t,[forward(r['target_x'],r['target_y']) for r in first],
                             '--',color='black',label='1:1 wand target')
            axes[row,0].set(xlabel='Virtual time (s)',ylabel='Forward displacement (m)')
        for kind,color in (('baseline','tab:orange'),('continuous','tab:blue')):
            rows=all_rows[(name,kind)]
            if name=='circle':
                axes[row,0].plot([r['car_local_f']-1.5 for r in rows],
                                 [r['car_local_l']-1.5 for r in rows],color=color,label=kind)
            else:
                axes[row,0].plot(t,[r['car_local_f']-1.5 for r in rows],color=color,label=kind)
            axes[row,1].plot(t,[r['same_time_error_m']*100 for r in rows],color=color,label=kind)
        axes[row,0].set_title(name)
        axes[row,1].set(xlabel='Virtual time (s)',ylabel='Same-time position error (cm)')
        for ax in axes[row]:ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.suptitle('OFFLINE MODEL ONLY | same limits: 0.20 m/s, 0.08 m/s2 | no time-shifted alignment')
    fig.savefig(path,dpi=130)
    plt.close(fig)


if __name__=='__main__':
    main()
