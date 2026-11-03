"""Changed-path regressions; injected clocks and nominal model, no hardware IO."""
from dataclasses import asdict, replace
import math
from pathlib import Path
import sys
import pytest

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
READY=ROOT/'docs/wand_browser_direct_2026-10-30/prepared_v1'
sys.path.insert(0,str(READY))
from profile_direct import sim_config, config, contract, Signer, Verifier, Policy, COMMAND_RPM
from mecanum_wand_4wsb_control.core import Pose, Velocity
from mecanum_wand_4wsb_control.direct_target import DirectSession, DirectSupervisor, DirectFinalGate
from mecanum_wand_4wsb_control.safety_core import ZERO, Lease
from mecanum_wand_4wsb_control.rotation_servo import wheel_rpm
from plant_direct import NominalPlant


class Harness:
    def __init__(self):
        self.c=sim_config();self.s=DirectSupervisor(self.c);self.g=DirectFinalGate(self.c)
        self.p=NominalPlant();self.i=0;self.held=False
        self.wand=Pose(10.,10.,1.3,.4);self.car=Pose(0.,0.,self.c.ground_z_m,0.)
        self.out=ZERO;self.frame=None
    def step(self, *, modify=None, skip=None, dt=.01):
        self.i+=1;now=10.+self.i*dt;ros=100.+self.i*dt
        self.now,self.ros=now,ros
        self.car=Pose(*self.p.pose[:2],self.c.ground_z_m,self.p.pose[2])
        for core in (self.s,self.g):
            for source,pose in (('wand',self.wand),('car',self.car)):
                if source!=skip:core.inputs.observe_pose(source,pose,ros,now,ros)
            if skip!='lease':core.inputs.observe_lease(Lease('operator',self.i,now,ros,self.held),now,ros)
            if skip!='feedback':core.inputs.observe_feedback(self.i,now,ros,now,ros,output_zero=self.out==ZERO,
                enabled=self.out!=ZERO,speed=math.hypot(self.p.velocity.vx,self.p.velocity.vy),yaw_rate=abs(self.p.velocity.wz))
        self.s.observe_gate(steady=now,ros=ros,epoch=self.g.primed_epoch,fault=self.g.fault,hard_fault=self.g.hard_fault)
        f=self.s.tick(now,ros)
        self.frame=modify(f) if modify else f
        self.g.receive(self.frame,now,ros);self.out=self.g.tick(now,ros)
        self.p.step(self.out,dt)
        return self.out
    def active(self):
        for _ in range(220):self.step()
        ok,why=self.s.request_start(self.now,self.ros)
        assert ok,why
        for _ in range(5):self.step()
        self.held=True;self.step()
        assert self.s.state=='ACTIVE' and not self.g.fault,(self.s.reason,self.g.fault)
        assert self.out==ZERO
        return self


def test_exact_timers_and_command_envelope_unchanged():
    before=asdict(config(6));after=asdict(sim_config())
    for k in before:
        if k not in ('max_speed_mps','max_accel_mps2'):assert before[k]==after[k]
    c=sim_config()
    assert (c.pose_timeout_s,c.command_timeout_s,c.lease_timeout_s,c.feedback_timeout_s,c.max_control_gap_s)==(.2,.1,.15,.3,.1)
    assert c.max_session_s==600. and c.follower_config().workspace_radius_enabled is False
    assert c.follower_config().wand_deadband_m==0.
    assert contract()['intermediate_reference'] is False


@pytest.mark.parametrize('key,value',[('max_speed_mps',1.4),('max_accel_mps2',8.),('pose_timeout_s',.3),
    ('command_timeout_s',.2),('lease_timeout_s',.3),('feedback_timeout_s',.4),('max_session_s',601.)])
def test_profile_rejects_unreviewed_limits(key,value):
    with pytest.raises(ValueError):replace(sim_config(),**{key:value})


def test_direct_target_exact_for_omni_rotation_and_far_target():
    h=Harness().active()
    for i in range(700):
        t=(i+1)*.01
        h.wand=Pose(10.+1.1*math.sin(.6*t),10.+.8*(1-math.cos(.6*t)),2.+.1*t,.4+1.2*math.sin(.9*t))
        previous=h.out;h.step()
        assert h.s.state=='ACTIVE' and not h.g.fault,(h.s.reason,h.g.fault)
        f=h.s.follower
        assert f.goal==f.mapped_goal
        assert not f.reference_limited and not f.yaw_limited
        assert f.goal.x==pytest.approx(h.wand.x-10.,abs=1e-10)
        assert f.goal.y==pytest.approx(h.wand.y-10.,abs=1e-10)
        assert f.goal.z==h.c.ground_z_m  # Z never becomes command.
        assert f.goal.yaw==pytest.approx(h.wand.yaw-.4,abs=1e-10)
        assert max(map(abs,wheel_rpm(h.out)))<=COMMAND_RPM+1e-6
        assert math.hypot(h.out.vx-previous.vx,h.out.vy-previous.vy)<=.04+1e-7
        assert abs(h.out.wz-previous.wz)<=.12+1e-7


@pytest.mark.parametrize('axis',['x','y','yaw','z'])
def test_gate_rejects_forged_target_even_with_valid_sources(axis):
    h=Harness().active();h.wand=replace(h.wand,x=h.wand.x+.01)
    h.step(modify=lambda f:replace(f,goal=replace(f.goal,**{axis:getattr(f.goal,axis)+.05})))
    assert h.out==ZERO and h.g.fault=='FINAL_REFERENCE_STEP_LIMIT'


def test_gate_accepts_raw_goal_over_old_20cm_and_yaw_lead_without_slew_proxy():
    h=Harness().active()
    for i in range(100):
        h.wand=Pose(10.+(i+1)*.018,10.,1.3,.4+(i+1)*.03)
        h.step()
        assert h.s.state=='ACTIVE' and not h.g.fault,(h.s.reason,h.g.fault)
    assert h.s.follower.goal.x>1.7 and h.s.follower.goal.yaw>2.9
    assert h.s.follower.goal==h.s.follower.mapped_goal


@pytest.mark.parametrize('missing,reason',[('wand','WAND_POSE_TIMEOUT'),('car','CAR_POSE_TIMEOUT'),
    ('lease','OPERATOR_LEASE_TIMEOUT'),('feedback','FEEDBACK_TIMEOUT')])
def test_missing_source_or_lease_stops_without_auto_resume(missing,reason):
    h=Harness().active()
    for _ in range(40):h.step(skip=missing)
    assert h.s.state!='ACTIVE' and h.out==ZERO
    assert (reason in h.s.reason or reason in h.g.fault or
        (missing in ('wand','car') and h.s.reason.startswith('car or wand pose timeout:'))),(h.s.reason,h.g.fault)
    for _ in range(120):h.step()
    assert h.s.state!='ACTIVE' and h.out==ZERO


def test_source_jump_still_stops():
    h=Harness().active();h.wand=replace(h.wand,x=h.wand.x+.251);h.step()
    assert h.s.state!='ACTIVE' and h.out==ZERO


def test_stale_feedforward_expires_before_pose_deadline():
    f=DirectSession(sim_config().follower_config())
    f.observe('car',Pose(0.,0.,.1),10.,source_stamp=100.)
    f.observe('wand',Pose(0.,0.,1.),10.,source_stamp=100.)
    assert f.engage(10.)[0]
    for i in range(1,11):
        f.observe('wand',Pose(i*.01,0.,1.),10.+i*.01,source_stamp=100.+i*.01)
        f.observe('car',Pose(0.,0.,.1),10.+i*.01,source_stamp=100.+i*.01)
        f.tick(10.+i*.01)
    assert f.reference_velocity[0]>.9
    f.tick(10.161)
    assert f.reference_velocity==(0.,0.) and f.state=='ACTIVE'
    f.tick(10.301)
    assert f.state=='FAULT' and f.command==ZERO


def test_no_progress_and_command_deadlines_remain():
    f=DirectSession(sim_config().follower_config())
    for source in ('wand','car'):f.observe(source,Pose(0.,0.,.1),10.,source_stamp=100.)
    f.engage(10.)
    for i in range(1,410):
        f.observe('wand',Pose(min(i*.01,.8),0.,.1),10.+i*.01,source_stamp=100.+i*.01)
        f.observe('car',Pose(0.,0.,.1),10.+i*.01,source_stamp=100.+i*.01)
        f.tick(10.+i*.01)
        if f.state=='FAULT':break
    assert f.reason=='CAR_PROGRESS_TIMEOUT' and f.command==ZERO
    h=Harness().active();h.g.tick(h.now+.101,h.ros+.101)
    assert h.g.output==ZERO and (h.g.fault=='COMMAND_TIMEOUT' or h.g.hard_fault)


def test_prepared_lift_reanchors_only_on_manual_start():
    h=Harness()
    for _ in range(220):h.step()
    assert h.s.request_start(h.now,h.ros)[0]
    for i in range(40):
        h.wand=Pose(10.+i*.01,10.+i*.005,1.5+i*.01,.4+i*.01);h.step()
        assert h.out==ZERO
    h.held=True;h.step();assert h.out==ZERO and h.s.state=='ACTIVE'
    h.wand=replace(h.wand,x=h.wand.x+.01);h.step()
    assert h.s.follower.goal.x==pytest.approx(.01,abs=1e-10) and not h.g.fault


def test_independent_signature_rejects_governed_mode_replay_and_tampering():
    from profile12 import Signer as OldSigner,Verifier as OldVerifier
    key='d'*64;data=Signer(key).encode(ZERO,10.,100.)
    assert Verifier(key).decode(data,10.,100.)[1]==ZERO
    with pytest.raises(ValueError):OldVerifier(key).decode(data,10.,100.)
    with pytest.raises(ValueError):Verifier(key).decode(OldSigner(key).encode(ZERO,10.,100.),10.,100.)
    verifier=Verifier(key);verifier.decode(data,10.,100.)
    with pytest.raises(ValueError):verifier.decode(data,10.,100.)


def test_browser_has_no_reference_payload_or_green_target():
    app=(READY/'web/app.js').read_text();html=(READY/'web/index.html').read_text()
    assert '#008b8b' not in app and 'latest.reference' not in app and '青：参考点' not in html
    assert "target_policy='DIRECT_SIGN_1TO1'" in (READY/'browser_operator.py').read_text()
    assert "reference=s.get('goal')" not in (READY/'browser_operator.py').read_text()


@pytest.mark.parametrize('command,expected',[(Velocity(1.21,0.,0.),'FINAL_VECTOR_LIMIT'),
    (Velocity(.1,0.,0.),'FINAL_ACCELERATION_LIMIT'),(Velocity(0.,0.,2.01),'FINAL_VECTOR_LIMIT')])
def test_final_gate_still_rejects_over_speed_or_acceleration(command,expected):
    h=Harness().active();h.step(modify=lambda f:replace(f,command=command))
    assert h.out==ZERO and h.g.fault==expected


def test_final_gate_rejects_changed_owner_and_replay():
    h=Harness().active();last=h.frame
    assert not h.g.receive(last,h.now,h.ros)
    h.step(modify=lambda f:replace(f,source='other_owner'))
    assert h.out==ZERO and h.g.fault=='COMMAND_OWNER_CHANGED'


def test_final_gate_rejects_over_budget_wheels_within_axis_limits():
    h=Harness().active()
    for _ in range(25):h.step(modify=lambda f:replace(f,command=Velocity(.025*(_+1),.025*(_+1),0.)))
    # <1.2m/s combined XY, legal acceleration, but >1.2m/s wheel-rim sum.
    assert h.out==ZERO and h.g.fault=='DIRECT_WHEEL_SPEED_LIMIT'
