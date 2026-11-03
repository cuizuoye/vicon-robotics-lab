"""Changed adaptive path, same independent direct gate and retained guards."""
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import sys
import pytest

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_browser_direct_2026-10-30/prepared_v1'
sys.path.insert(0,str(BASE));sys.path.insert(0,str(HERE))
from profile_direct import sim_config
from adaptive_servo import AdaptiveSession,Policy,acceleration_budget,correction,smooth_level
from mecanum_wand_4wsb_control.core import Pose
from mecanum_wand_4wsb_control.safety_core import ZERO
from mecanum_wand_4wsb_control.rotation_servo import wheel_rpm

spec=importlib.util.spec_from_file_location('accepted_direct_fixture',ROOT/'candidates/wand_browser_direct_20261030/test_direct.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
class Harness(old.Harness):
    def __init__(self):
        super().__init__()
        self.s.follower=AdaptiveSession(self.c.follower_config(),self.c.project_goal,self.c.project_reference)

@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-1.,0.])
def test_bad_policy_rejected(bad):
    with pytest.raises(ValueError):Policy(position_gain=bad)

def test_smooth_schedule_is_continuous_monotonic_and_bounded():
    values=[acceleration_budget(i*.001,0.,0.,0.,0.,1.,4.,.3,.7,.12,.015) for i in range(1001)]
    assert values[0]==1. and values[-1]==4.
    assert all(0<=b-a<.02 for a,b in zip(values,values[1:]))
    angle=[acceleration_budget(i*.001,0.,0.,0.,0.,1.5,12.,.5,1.2,.12,.025) for i in range(1001)]
    assert angle[0]==1.5 and angle[-1]==12.

def test_fast_hand_raises_budget_even_before_large_error():
    slow=acceleration_budget(.01,.05,0.,0.,0.,1.,4.,.3,.7,.12,.015)
    fast=acceleration_budget(.01,.8,0.,0.,0.,1.,4.,.3,.7,.12,.015)
    assert slow<1.1 and fast==4.

def test_near_goal_never_forces_weak_braking_or_wrong_way_drive():
    assert acceleration_budget(.02,0.,1.,1.,0.,1.,4.,.3,.7,.12,.015)==4.
    assert acceleration_budget(.01,.1,0.,1.,-.1,1.,4.,.3,.7,.12,.015)==4.

def test_square_root_correction_join_and_no_one_step_overshoot():
    gain,a,dt=4.,4.,.01;join=a/gain**2
    assert correction(join-1e-8,gain,a,dt)==pytest.approx(correction(join+1e-8,gain,a,dt),abs=1e-6)
    for e in (-10.,-.1,-1e-9,0.,1e-9,.1,10.):
        r=correction(e,gain,a,dt)
        assert abs(r)*dt<=abs(e)+1e-12 and r*e>=0

def test_same_raw_target_source_gate_and_caps_for_omni_reversals_rotation():
    h=Harness().active();budgets=[]
    for i in range(2400):
        t=(i+1)*.01
        h.wand=Pose(10.+1.7*math.sin(.55*t),10.+1.2*(1-math.cos(.6*t)),1.3+.2*math.sin(t),.4+1.5*math.sin(.8*t))
        previous=h.out;h.step();f=h.s.follower;d=f.last_adaptive
        assert h.s.state=='ACTIVE' and not h.g.fault,(h.s.reason,h.g.fault)
        assert f.goal==f.mapped_goal and not f.reference_limited and not f.yaw_limited
        assert f.goal.x==pytest.approx(h.wand.x-10.) and f.goal.y==pytest.approx(h.wand.y-10.)
        assert f.goal.z==h.c.ground_z_m
        assert math.hypot(h.out.vx-previous.vx,h.out.vy-previous.vy)<=d['accel_budget_mps2']*.01+1e-7
        assert abs(h.out.wz-previous.wz)<=d['yaw_accel_budget_radps2']*.01+1e-7
        assert max(map(abs,wheel_rpm(h.out)))<=h.c.limits.target_rpm+1e-6
        budgets.append(d['accel_budget_mps2'])
    for _ in range(600):
        h.step();assert h.s.state=='ACTIVE' and not h.g.fault
        budgets.append(h.s.follower.last_adaptive['accel_budget_mps2'])
    assert min(budgets)<1.2 and max(budgets)>3.9

@pytest.mark.parametrize('missing',['car','wand','lease','feedback'])
def test_retained_loss_stops_and_data_return_never_auto_resumes(missing):
    h=Harness().active()
    for i in range(80):
        h.wand=replace(h.wand,x=h.wand.x+.003);h.step()
    for _ in range(45):h.step(skip=missing)
    assert h.s.state!='ACTIVE' and h.out==ZERO
    for _ in range(150):h.step()
    assert h.s.state!='ACTIVE' and h.out==ZERO

@pytest.mark.parametrize('axis',['x','y','yaw'])
def test_direct_gate_rejects_forged_goal(axis):
    h=Harness().active()
    h.step(modify=lambda f:replace(f,goal=replace(f.goal,**{axis:getattr(f.goal,axis)+.03})))
    assert h.out==ZERO and h.g.fault=='FINAL_REFERENCE_STEP_LIMIT'

def test_prepare_then_pick_up_captures_origins_only_at_enable():
    h=Harness()
    for _ in range(220):h.step()
    assert h.s.request_start(h.now,h.ros)[0]
    for i in range(60):
        h.wand=Pose(10.+i*.01,10.,1.4+i*.01,.4+i*.01);h.step();assert h.out==ZERO
    h.held=True;h.step();assert h.out==ZERO
    h.wand=replace(h.wand,x=h.wand.x+.01);h.step()
    assert h.s.follower.goal.x==pytest.approx(.01) and not h.g.fault

def test_zero_stale_feedforward_and_no_progress_still_active():
    f=AdaptiveSession(sim_config().follower_config())
    for source in ('car','wand'):f.observe(source,Pose(0.,0.,.1),10.,source_stamp=100.)
    f.engage(10.)
    for i in range(1,410):
        f.observe('wand',Pose(min(i*.01,.8),0.,.1),10.+i*.01,source_stamp=100.+i*.01)
        f.observe('car',Pose(0.,0.,.1),10.+i*.01,source_stamp=100.+i*.01)
        f.tick(10.+i*.01)
        if f.state=='FAULT':break
    assert f.reason=='CAR_PROGRESS_TIMEOUT' and f.command==ZERO

def test_random_noise_bounded_and_manual_stop_not_smoothed():
    import random
    rng=random.Random(3010);h=Harness().active()
    for _ in range(1000):
        h.wand=Pose(10.+rng.uniform(-.0005,.0005),10.+rng.uniform(-.0005,.0005),1.3,.4+rng.uniform(-.0005,.0005))
        h.step();assert h.s.state=='ACTIVE' and not h.g.fault
        assert math.hypot(h.out.vx,h.out.vy)<.03 and abs(h.out.wz)<.03
    h.held=False;h.step();assert h.out==ZERO and h.s.state!='ACTIVE'
