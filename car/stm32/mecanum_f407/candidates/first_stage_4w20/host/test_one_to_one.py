"""Pure mapping/contract tests, no vehicle dynamics or ROS node execution."""
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import pytest

ROOT=Path(os.environ['ONE_TO_ONE_GENERATED_ROOT'])
for package in ('mecanum_wand_one_to_one_control','mecanum_wand_one_to_one_hardware'):
    sys.path.insert(0,str(ROOT/package))
from mecanum_wand_one_to_one_control.core import Config,Pose,RelativeMapping,WandSession
from mecanum_wand_one_to_one_control.safety_core import SafetyConfig,Supervisor
from mecanum_wand_one_to_one_control.operator_guidance import wand_preview
from mecanum_wand_one_to_one_hardware.commissioning import Runtime,CONSENT,DOMAIN
PROFILE=ROOT/'mecanum_wand_one_to_one_hardware/config/one_to_one.CURRENT_AREA.SITE_CONFIRMATION_REQUIRED.json'


@pytest.mark.parametrize('yaw',[0.,.7,math.pi/2,-2.1])
@pytest.mark.parametrize('forward,left',[(1.,0.),(0.,1.),(-.3,.2),(.5,.5)])
def test_one_metre_rod_is_one_metre_goal_in_each_initial_heading(yaw,forward,left):
    origin=Pose(.1,-.2,.1,yaw);wand=Pose(1.,2.,1.2)
    dx=math.cos(yaw)*forward-math.sin(yaw)*left
    dy=math.sin(yaw)*forward+math.cos(yaw)*left
    mapping=RelativeMapping(origin,wand)
    goal=mapping.goal(replace(wand,x=wand.x+dx,y=wand.y+dy,z=2.5,yaw=2.))
    assert goal.x==pytest.approx(origin.x+dx) and goal.y==pytest.approx(origin.y+dy)
    assert goal.z==origin.z and goal.yaw==origin.yaw
    assert mapping.goal(wand)==origin
    assert mapping.scale==Config().scale==SafetyConfig().follower_config().scale==1.0


def test_supervisor_and_preview_use_same_scale_not_only_a_caption():
    c=SafetyConfig(arena_x_min=-4,arena_x_max=4,arena_y_min=-4,arena_y_max=4,max_goal_radius_m=3.)
    s=Supervisor(c);assert s.follower.config.scale==1.
    car,wand=Pose(0,0,.1,.7),Pose(1,1,1.2)
    moved=replace(wand,x=1.5,y=1.25,z=2.)
    p=wand_preview(c,car,wand,moved)
    expected=RelativeMapping(car,wand).goal(moved)
    assert p['goal'].x==pytest.approx(expected.x) and p['goal'].y==pytest.approx(expected.y)
    assert p['goal'].z==expected.z and p['goal'].yaw==expected.yaw
    assert p['goal'].x==.5 and p['goal'].y==.25
    assert p['radial_budget_m']==3.


def test_prescribed_samples_retract_to_original_goal_and_pause_clears_it():
    c=Config(scale=1,filter_tau_s=0,wand_deadband_m=0,max_speed_mps=.2,max_accel_mps2=.08)
    s=WandSession(c);car=Pose(0,0,.1);wand=Pose(0,0,1.)
    s.observe('car',car,10);s.observe('wand',wand,10);assert s.engage(10)[0]
    for i in range(1,11):
        t=10+i*.02;s.observe('car',car,t);s.observe('wand',Pose(i*.1,0,1+i*.02),t)
        v=s.tick(t);assert math.hypot(v.vx,v.vy)<=.20+1e-9
    assert s.goal.x==pytest.approx(1.)
    for i in range(1,11):
        t=10.2+i*.02;s.observe('car',car,t);s.observe('wand',Pose(1-i*.1,0,1.2),t);s.tick(t)
    assert s.goal.x==pytest.approx(0.)
    s.pause();assert s.goal is None and not s.tick(10.5).vx


def test_hardware_profile_explicit_scale_and_independent_domain(tmp_path):
    r=Runtime('shadow',str(PROFILE));assert DOMAIN==97 and r.motion_tag=='4W20'
    assert r.namespace=='/wand_one_to_one_acceptance' and r.config.follower_config().scale==1.
    assert r.config.max_speed_mps==.20 and r.config.feedback_timeout_s==.20
    assert r.config.lease_timeout_s==.15 and r.config.max_session_s==120
    v=json.loads(PROFILE.read_text());v['stage020_confirmations']['clear_area_and_power_cut_ready_now']=True
    for scale in (2.,None):
        v['mapping_scale']=scale;p=tmp_path/'mixed_profile.json';p.write_text(json.dumps(v))
        with pytest.raises(ValueError,match='1:1 profile'):
            Runtime('hardware',str(p),hashlib.sha256(p.read_bytes()).hexdigest(),CONSENT)


def test_new_area_remains_plan_not_false_hardware_approval():
    p=json.loads((ROOT/'arena_3x3_buffer050.PLAN_ONLY.json').read_text())
    assert p['center_workspace_local_m']==dict(forward=[0,3],left=[0,3])
    assert p['stopping_buffer_each_side_m']==.5
    assert p['far_corner_distance_m']>3
    with pytest.raises((ValueError,KeyError)):
        path=ROOT/'arena_3x3_buffer050.PLAN_ONLY.json'
        Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),CONSENT)


def test_real_ui_has_no_remaining_two_times_captions_or_status_constant():
    c=ROOT/'mecanum_wand_one_to_one_control/mecanum_wand_one_to_one_control'
    for name in ('operator_console.py','operator_guidance.py'):
        assert '2倍' not in (c/name).read_text()
    assert 'scale=self.core.follower.config.scale' in (c/'safety_nodes.py').read_text()
