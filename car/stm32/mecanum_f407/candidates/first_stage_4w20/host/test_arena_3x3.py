"""Geometry and prescribed-input safety contracts; no motor/vehicle model."""
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import pytest

ROOT=Path(os.environ['ARENA_3X3_GENERATED_ROOT'])
for package in ('mecanum_wand_one_to_one_control','mecanum_wand_one_to_one_hardware'):
    sys.path.insert(0,str(ROOT/package))
from mecanum_wand_one_to_one_control.core import Pose,RelativeMapping,Velocity
from mecanum_wand_one_to_one_control.arena_geometry import FixedArena
from mecanum_wand_one_to_one_control.safety_core import Supervisor,FinalGate,Frame,Lease,ZERO
from mecanum_wand_one_to_one_control.operator_guidance import wand_preview
from mecanum_wand_one_to_one_hardware.commissioning import Runtime,CONSENT
from mecanum_wand_one_to_one_hardware.nodes import HardwareBridge
PROFILE=ROOT/'mecanum_wand_one_to_one_hardware/config/one_to_one.3x3_BUFFER050.SITE_CONFIRMATION_REQUIRED.json'


def runtime():return Runtime('shadow',str(PROFILE))
def pose(a,x,y,yaw=None,z=.10790704618641533):return Pose(*a.world(x,y),z,a.heading if yaw is None else yaw)


@pytest.mark.parametrize('heading',[0.,.7,math.pi/2,-2.1,1.526260405304856])
@pytest.mark.parametrize('x,y',[(0.,0.),(3.,0.),(0.,3.),(3.,3.)])
def test_all_corners_reachable_and_one_to_one_in_oriented_fixed_box(heading,x,y):
    a=FixedArena(.64,-1.05,heading);origin=pose(a,0,0);wand=Pose(1,2,1.2)
    wx,wy=a.world(x,y);dx,dy=wx-a.origin_x,wy-a.origin_y
    requested=RelativeMapping(origin,wand).goal(Pose(wand.x+dx,wand.y+dy,2.5,1.1))
    goal=a.project(requested,origin,4.75)
    assert a.local(goal.x,goal.y)==pytest.approx((x,y),abs=2e-8)
    assert goal.z==origin.z and goal.yaw==origin.yaw and a.inside(goal)
    assert a.project(RelativeMapping(origin,wand).goal(wand),origin,4.75).x==pytest.approx(origin.x,abs=2e-8)


@pytest.mark.parametrize('x,y,expected',[(-1,1,(0,1)),(4,1,(3,1)),(1,-1,(1,0)),
                                        (1,4,(1,3)),(-2,5,(0,3)),(4,4,(3,3))])
def test_all_sides_clamp_final_goals_not_to_the_buffer(x,y,expected):
    c=runtime().config;a=c.geometry();origin=pose(a,0,0)
    goal=c.project_goal(pose(a,x,y),origin,4.75)
    assert a.local(goal.x,goal.y)==pytest.approx(expected,abs=2e-8)
    assert c.goal_inside(goal) and c.inside(goal)


def test_fifty_cm_physical_buffer_preserves_twenty_cm_earlier_trigger():
    c=runtime().config;a=c.geometry()
    assert a.physical_bounds()==pytest.approx((-.595,3.595,-.605,3.605))
    assert a.center_bounds(a.heading)==pytest.approx((-.30,3.30,-.30,3.30))
    assert a.physical_bounds()[1]-(3.+.095)==pytest.approx(.50)
    assert a.physical_bounds()[3]-(3.+.105)==pytest.approx(.50)
    assert c.inside(pose(a,3.29,1.5)) and not c.goal_inside(pose(a,3.29,1.5))
    assert not c.inside(pose(a,3.301,1.5))
    assert not c.inside(pose(a,-.301,1.5))
    assert not c.inside(pose(a,1.5,3.301))
    assert not c.inside(pose(a,1.5,-.301))


def test_actual_rotated_body_and_oriented_corners_not_only_center_or_world_bbox():
    c=runtime().config;a=c.geometry()
    assert c.inside(pose(a,3.29,1.5))
    assert not c.inside(pose(a,3.29,1.5,a.heading+math.pi/4))
    xmin,xmax,ymin,ymax=c.center_bounds(a.heading)
    assert not c.inside(Pose(xmax,ymax,c.ground_z_m,a.heading))
    for x,y in c.goal_polygon():assert c.inside(Pose(x,y,c.ground_z_m,a.heading))


def test_far_corner_no_longer_cut_off_by_three_metre_radius():
    c=runtime().config;a=c.geometry();origin=pose(a,0,0);far=pose(a,3,3)
    goal=c.project_goal(far,origin,c.max_goal_radius_m)
    assert math.hypot(goal.x-origin.x,goal.y-origin.y)==pytest.approx(math.sqrt(18),abs=3e-8)
    assert c.max_goal_radius_m==4.75 and c.follower_config().scale==1.


def test_all_possible_safe_start_centers_can_reach_all_nominal_corners():
    c=runtime().config;a=c.geometry()
    for yaw in (a.heading,a.heading+.3,a.heading+math.pi/2):
        xmin,xmax,ymin,ymax=a.center_bounds(yaw)
        for x,y in ((xmin+1e-5,ymin+1e-5),(xmax-1e-5,ymax-1e-5)):
            origin=pose(a,x,y,yaw)
            for gx,gy in ((0,0),(0,3),(3,0),(3,3)):
                goal=c.project_goal(pose(a,gx,gy,yaw),origin,4.75)
                assert a.local(goal.x,goal.y)==pytest.approx((gx,gy),abs=2e-8)


def test_reprepare_never_moves_fixed_arena_or_goal_box():
    c=runtime().config;a=c.geometry();before=c.goal_polygon();sup=Supervisor(c)
    for location in ((0,0),(1,1),(2.9,2.9)):
        origin=pose(a,*location)
        assert sup.follower.goal_projector(pose(a,4,4),origin,4.75)==c.project_goal(pose(a,4,4),origin,4.75)
        assert c.goal_polygon()==before


def test_reference_can_slew_in_from_start_placement_error_without_fifty_mm_jump():
    c=runtime().config;a=c.geometry();s=Supervisor(c).follower
    car=pose(a,-.05,.02);wand=Pose(1,1,1.)
    s.observe('car',car,10.);s.observe('wand',wand,10.);assert s.engage(10.)[0]
    previous=s.goal
    for i in range(1,31):
        t=10+i*.02
        s.observe('car',car,t);s.observe('wand',wand,t);s.tick(t)
        assert s.state=='ACTIVE'
        assert math.hypot(s.goal.x-previous.x,s.goal.y-previous.y)<=.20*.02+1e-8
        assert c.inside(s.goal) and c.goal_inside(s.mapped_goal)
        previous=s.goal


def feed(inputs,car,t,seq,held=False):
    inputs.observe_pose('car',car,t+100,t,t+100)
    inputs.observe_pose('wand',Pose(0,0,1.),t+100,t,t+100)
    inputs.observe_lease(Lease('test-human',seq,t,t+100,held),t,t+100)
    inputs.observe_feedback(seq,t,t+100,t,t+100,output_zero=True,enabled=False,speed=0.,yaw_rate=0.)


def test_supervisor_and_independent_gate_both_reject_actual_outside_car():
    c=runtime().config;a=c.geometry();sup=Supervisor(c);gate=FinalGate(c)
    car=pose(a,3.29,1.5)
    feed(gate.inputs,car,10.,1)
    gate.receive(Frame('test-supervisor',1,10.,110.,'','STANDBY',False,ZERO,None),10.,110.)
    assert gate.tick(10.,110.)==ZERO
    outside=pose(a,3.301,1.5)
    for inputs in (sup.inputs,gate.inputs):
        feed(inputs,outside,10.02,2,True)
        assert inputs.health(10.02,110.02)=='CAR_OUTSIDE_FIXED_ARENA'
    gate.receive(Frame('test-supervisor',2,10.02,110.02,'e','ACTIVE',True,Velocity(.001),car),10.02,110.02)
    assert gate.tick(10.02,110.02)==ZERO and gate.fault=='CAR_OUTSIDE_FIXED_ARENA'


def test_preview_is_the_same_one_to_one_and_oriented_goal_projection():
    c=runtime().config;a=c.geometry();origin=pose(a,0,0);wand=Pose(1,2,1.)
    moved=replace(wand,x=wand.x+a.world(3,3)[0]-a.origin_x,y=wand.y+a.world(3,3)[1]-a.origin_y,z=2.)
    preview=wand_preview(c,origin,wand,moved)
    actual=c.project_goal(RelativeMapping(origin,wand).goal(moved),origin,4.75)
    assert preview['projected_goal'].x==pytest.approx(actual.x)
    assert preview['projected_goal'].y==pytest.approx(actual.y)
    assert preview['radial_budget_m']==pytest.approx(0.,abs=1e-8)


def confirmed(tmp_path,change=lambda v:None):
    v=json.loads(PROFILE.read_text())
    v['confirmations']['fixed_arena_checked']=True
    v['stage020_confirmations']['clear_area_and_power_cut_ready_now']=True
    v['arena_confirmations']=dict.fromkeys(v['arena_confirmations'],True)
    change(v);p=tmp_path/'synthetic_data_ONLY.json';p.write_text(json.dumps(v))
    return p,hashlib.sha256(p.read_bytes()).hexdigest()


def test_profile_blocked_before_node_socket_and_no_fabricated_site_approval(monkeypatch):
    import mecanum_wand_one_to_one_hardware.nodes as nodes
    for k,v in dict(WAND_ONE_TO_ONE_MODE='hardware',WAND_ONE_TO_ONE_PROFILE=str(PROFILE),
        WAND_PROFILE_APPROVAL=hashlib.sha256(PROFILE.read_bytes()).hexdigest(),WAND_HARDWARE_CONSENT=CONSENT).items():
        monkeypatch.setenv(k,v)
    def forbidden(*a,**k):raise AssertionError('MUST NOT CREATE A SOCKET')
    monkeypatch.setattr(nodes.socket,'socket',forbidden)
    with pytest.raises(ValueError):HardwareBridge.__init__(object.__new__(HardwareBridge))


def test_pure_validator_accepts_only_explicit_new_geometry_and_existing_firmware(tmp_path):
    p,h=confirmed(tmp_path);r=Runtime('hardware',str(p),h,CONSENT)
    assert r.motion_tag=='4W20' and r.config.max_speed_mps==.20
    assert r.config.max_session_s==120 and r.config.feedback_timeout_s==.20
    assert r.config.lease_timeout_s==.15 and r.config.command_timeout_s==.10
    assert r.profile['arena_buffer_policy']['physical_stop_distance_measured'] is False


def test_real_safety_node_constructor_retains_oriented_config_subtype_without_dds(monkeypatch):
    # Run the actual constructor with Node IO stubbed; no rclpy.init, DDS,
    # executor, transport or simulated vehicle. Detect a base-type rebuild.
    from types import SimpleNamespace
    from mecanum_wand_one_to_one_control.safety_nodes import SafetyNode,Node
    r=runtime();parameters={}
    monkeypatch.setattr(Node,'__init__',lambda self,*a,**k:None)
    monkeypatch.setattr(Node,'declare_parameter',lambda self,n,v:parameters.setdefault(n,v))
    monkeypatch.setattr(Node,'get_parameter',lambda self,n:SimpleNamespace(value=parameters[n]))
    monkeypatch.setattr(Node,'create_timer',lambda *a,**k:None)
    monkeypatch.setattr(Node,'create_subscription',lambda *a,**k:None)
    stub=SimpleNamespace(namespace=r.namespace,world=r.world,mode='shadow',config=r.config,guard=lambda node:None)
    for core in (Supervisor,FinalGate):
        node=object.__new__(SafetyNode)
        SafetyNode.__init__(node,'offline_constructor_check',core,stub)
        assert type(node.core.c) is type(r.config)
        assert node.core.c==r.config
        assert parameters['arena_buffer_m']==.50 and parameters['arena_forward_max_m']==3.


@pytest.mark.parametrize('failure',['ratio','buffer','size','bbox','radius','speed','site','old_revision'])
def test_profile_cannot_widen_or_mix_limits_and_claim_old_site_review(tmp_path,failure):
    def change(v):
        if failure=='ratio':v['mapping_scale']=2.
        if failure=='buffer':v['safety']['arena_buffer_m']=.1
        if failure=='size':v['safety']['arena_forward_max_m']=4.
        if failure=='bbox':v['safety']['arena_x_max']+=1.
        if failure=='radius':v['safety']['max_goal_radius_m']=5.
        if failure=='speed':v['safety']['max_speed_mps']=.3
        if failure=='site':v['arena_confirmations']['extended_arena_and_buffer_clear_and_tracked']=False
        if failure=='old_revision':v['arena_revision']='old_area'
    p,h=confirmed(tmp_path,change)
    with pytest.raises(ValueError):Runtime('hardware',str(p),h,CONSENT)


def test_actual_console_draws_oriented_polygons_without_tk_or_keyboard():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from mecanum_wand_one_to_one_control.operator_console import Console
    c=runtime().config;a=c.geometry();ui=object.__new__(Console)
    ui.config=c;ui.virtual=False;ui.wand_reference_margin=.5;ui.map=Mock()
    ui.map_point=lambda x,y:(x*100,y*100)
    ui.node=SimpleNamespace(trace=[],car=pose(a,1.5,1.5),raw_wand=None,
        input_kind='hardware',input_hint=lambda:'')
    ui.model=SimpleNamespace(status={'state':'STANDBY'},gate={},held=False,notice='',closing=False,
        fresh=lambda now:True,can_start=lambda now:False,can_recover=lambda now:False,
        instruction=lambda now:'test')
    ui.hand_text=Mock();ui.status_text=Mock();ui.step_text=Mock();ui.notice=Mock()
    ui.buttons={k:Mock() for k in ('start','recover','acknowledge')}
    ui.draw_pad=lambda:None
    Console.draw(ui,1.)
    assert ui.map.create_polygon.call_count==5  # physical, goals, rod, trigger, body
    ui.map.create_rectangle.assert_not_called()


def review_module():
    import importlib.util
    spec=importlib.util.spec_from_file_location('offline_arena_review',Path(__file__).with_name('review_arena_profile.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_site_preparer_requires_exact_confirmation_and_does_not_modify_template(tmp_path):
    review=review_module();before=PROFILE.read_bytes()
    with pytest.raises(ValueError):review.prepare(tmp_path/'not_created','yes')
    assert not (tmp_path/'not_created').exists() and PROFILE.read_bytes()==before


def test_synthetic_site_copy_only_updates_declarations_not_geometry_or_unmeasured_tests(tmp_path):
    review=review_module();original=json.loads(PROFILE.read_text())
    # Temporary synthetic input test, never an actual site-approved run/file.
    p,h=review.prepare(tmp_path/'SYNTHETIC_ONLY_NEVER_LAUNCHED',review.PHRASE)
    copy=json.loads(p.read_text())
    assert copy['safety']==original['safety'] and copy['hardware']==original['hardware']
    assert copy['mapping_scale']==1. and copy['arena_buffer_policy']['physical_stop_distance_measured'] is False
    assert copy['stage020_policy']==original['stage020_policy']
    Runtime('hardware',str(p),h,CONSENT)  # data validation, no Node or socket
