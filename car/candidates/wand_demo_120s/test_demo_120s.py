"""120s and exact signed geometry; virtual clocks only, never real motion."""
import copy
from dataclasses import asdict, replace
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS
import time

import pytest
from support import ROOT,GENERATED,config,harness,model
from mecanum_wand_4ws8_control.core import Pose,Velocity,RelativeMapping
from mecanum_wand_4ws8_control.pilot_core import POLICY,active_limits
from mecanum_wand_4ws8_control.pilot_ui import reason_text
from mecanum_wand_4ws8_control.safety_core import ZERO
from mecanum_wand_4ws8_control.safety_wire import canonical
from mecanum_wand_4ws8_hardware.commissioning import AuthoritySigner,TransportPolicy,Runtime,CONSENT
from mecanum_wand_4ws8_hardware.authorization import verify_runtime_profile,verify_trial_review

SOURCE=Path(os.environ['ONECLICK_SOURCE'])
spec=importlib.util.spec_from_file_location('signed_app_under_test',SOURCE/'oneclick_app.py')
app=importlib.util.module_from_spec(spec);spec.loader.exec_module(app)

@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import socket,rclpy
    def forbidden(*a,**k):pytest.fail('offline test attempted hardware or ROS init')
    monkeypatch.setattr(socket,'socket',forbidden);monkeypatch.setattr(rclpy,'init',forbidden)

def at(c,f,l,yaw=None):return Pose(*c.geometry().world(f,l),c.ground_z_m,c.arena_heading_rad if yaw is None else yaw)

def prepared():
    h=harness.Harness(config(),origin_local=(0.,0.));h.run(1.8)
    assert h.sup.request_start(h.now,h.ros)[0];h.run(.1)
    assert h.sup.state=='ARMED' and h.gate.primed_epoch==h.sup.epoch
    return h

def moving():
    h=harness.Harness(config(),origin_local=(0.,0.));h.start()
    for i in range(1,16):h.set_hand(.003*i,0.);h.step()
    assert h.output!=ZERO
    return h

def approved_profile():
    p=json.loads((GENERATED/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())
    m=json.loads((ROOT/'docs/wand_servo_4ws8_2026-10-26/oneclick_v2/ready/build_manifest.json').read_text())
    p['trial_review']=app.make_review(m)
    p['confirmations']['fixed_arena_checked']=True
    p['arena_confirmations']={k:True for k in p['arena_confirmations']}
    p['staged_runtime_approval']=dict(purpose='sign',issued_utc_s=time.time(),
        clear_and_power_cut_ready_now=True,
        flash_record=str(app.FLASH),build_manifest=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/oneclick_v2/ready/build_manifest.json'))
    return p,m

def test_exact_confirmed_limits_and_preserved_guards():
    c=config()
    assert c.geometry().goal_bounds()==(-1.5,.5,-.4,.5)
    assert POLICY['max_active_s']==c.max_session_s==120.
    assert POLICY['nominal_goal_forward_m']==[-1.5,.5] and POLICY['nominal_goal_left_m']==[-.4,.5]
    assert (c.max_speed_mps,c.max_accel_mps2)==(.70,.80)
    expected=dict(pose_timeout_s=.20,lease_timeout_s=.15,command_timeout_s=.10,feedback_timeout_s=.20,
        tilt_limit_rad=.25,lift_tolerance_m=.08,boundary_margin_m=.20,arena_buffer_m=.50,
        half_length_m=.095,half_width_m=.105)
    assert all(getattr(c,k)==v for k,v in expected.items())

@pytest.mark.parametrize('local',[(-1.5,-.4),(-1.5,.5),(.5,-.4),(.5,.5),(0.,0.)])
def test_all_new_corners_and_original_start_are_inside_both_fences(local):
    c=config();p=at(c,*local)
    assert c.goal_inside(p) and c.inside(p)
    assert c.geometry().local(p.x,p.y)==pytest.approx(local)
    original=at(c,0.,0.);wand=Pose(10.,10.,1.)
    moved=replace(wand,x=wand.x+p.x-original.x,y=wand.y+p.y-original.y)
    goal=RelativeMapping(original,wand,1.).goal(moved)
    projected=c.project_goal(goal,original,c.max_goal_radius_m)
    assert (projected.x,projected.y)==pytest.approx((p.x,p.y),abs=2e-8)

@pytest.mark.parametrize('f,l,clamped',[(-2.,0.,(-1.5,0.)),(1.,0.,(.5,0.)),
    (0.,-.9,(0.,-.4)),(0.,1.,(0.,.5)),(-3.,2.,(-1.5,.5))])
def test_each_direction_clamps_to_confirmed_edge_not_the_old_3x3_region(f,l,clamped):
    c=config();origin=at(c,0.,0.);p=c.project_goal(at(c,f,l),origin,c.max_goal_radius_m)
    assert c.geometry().local(p.x,p.y)==pytest.approx(clamped,abs=2e-8)
    assert c.goal_inside(p)

@pytest.mark.parametrize('f,l',[(-1.5001,0.),(.5001,0.),(0.,-.4001),(0.,.5001)])
@pytest.mark.parametrize('which',['supervisor','gate'])
def test_independent_goal_guard_rejects_each_edge_even_if_other_side_compromised(f,l,which):
    h=prepared();h.held=True;h.step();core=h.sup if which=='supervisor' else h.gate
    assert core.pilot.active(core.pilot.epoch,core.inputs,at(h.c,f,l),h.now+.02)=='PILOT_GOAL_FENCE'

@pytest.mark.parametrize('f,l',[(-1.801,0.),(.801,0.),(0.,-.701),(0.,.801)])
def test_whole_body_hard_stop_retains_extra20cm_margin(f,l):
    c=config();assert not c.inside(at(c,f,l))
    h=prepared();h.held=True;h.step()
    for core in (h.sup,h.gate):
        core.inputs.poses['car']['pose']=at(c,f,l)
        assert core.pilot.active(core.pilot.epoch,core.inputs,at(c,0.,0.),h.now+.02)=='PILOT_FOOTPRINT_FENCE'
    assert c.geometry().physical_bounds()==pytest.approx((-2.095,1.095,-1.005,1.105))

def test_waiting_and_repositioning_sign_after_prepare_do_not_use120s_budget():
    h=prepared();h.run(125.)
    for i in range(1,61):
        h.hand=replace(h.hand_origin,x=h.hand_origin.x+.6*i/60,
            y=h.hand_origin.y-.3*i/60,z=1.+.6*i/60,yaw=.8*i/60)
        assert h.step()==ZERO and h.sup.state=='ARMED'
    assert not h.sup.pilot.used and h.sup.pilot.started is None
    assert h.sup.pilot_status(h.now)['remaining_s']==120.
    h.held=True;assert h.step()==ZERO
    assert h.sup.state=='ACTIVE' and h.sup.pilot.started==h.gate.pilot.started
    assert h.sup.follower.mapping.wand_origin==h.hand
    assert h.sup.pilot_status(h.now)['remaining_s']==120.

def test_old20s_limit_is_gone_and_full_chain_expires_by120s_once_only():
    h=prepared();h.held=True;h.step();t0=h.sup.pilot.started
    h.run(20.2);assert h.sup.state=='ACTIVE'
    assert h.sup.pilot_status(h.now)['remaining_s']==pytest.approx(99.8,abs=.001)
    h.run(99.7);assert h.sup.state=='ACTIVE'
    h.run(.14);assert h.output==ZERO and h.sup.state!='ACTIVE'
    assert h.now-t0<120.1
    assert h.sup.pilot.started==h.gate.pilot.started==t0
    h.held=False;h.run(2.)
    assert h.sup.stop_confirmed and not h.sup.request_start(h.now,h.ros)[0]
    assert not h.sup.request_recover(h.now,h.ros)[0]
    h.held=True;h.run(.3);assert h.output==ZERO

@pytest.mark.parametrize('which',['supervisor','gate'])
def test_each_monitor_independently_expires_at_exact120s(which):
    h=prepared();h.held=True;h.step();m=(h.sup if which=='supervisor' else h.gate).pilot
    inputs=(h.sup if which=='supervisor' else h.gate).inputs
    assert m.active(m.epoch,inputs,m.car,m.started+119.999)==''
    assert m.active(m.epoch,inputs,m.car,m.started+120.)=='PILOT_TIME_LIMIT_120S'

def test_final_gate_expires_even_if_supervisor_budget_is_compromised():
    h=prepared();h.held=True;h.step()
    h.sup.pilot.active=lambda *a:'';h.sup.active_started+=1000.
    h.run(120.04)
    assert h.output==ZERO and 'PILOT_TIME_LIMIT_120S' in h.gate.fault
    assert not h.gate.primed_epoch

@pytest.mark.parametrize('fault',['release','wand_loss','car_loss','feedback_loss','lease_loss',
    'flip','lift','tilt','estop','actuator','clock','supervisor_loss','gate_loss','owner','overspeed',
    'acceleration','reference_jump','mode'])
def test_all_existing_stop_protections_still_revoke(fault):
    h=moving()
    if fault=='release':h.held=False
    elif fault=='wand_loss':h.pose_on['wand']=False
    elif fault=='car_loss':h.pose_on['car']=False
    elif fault=='feedback_loss':h.feedback_on=False
    elif fault=='lease_loss':h.lease_on=False
    elif fault=='flip':h.roll=math.pi
    elif fault=='lift':h.plant.pose=replace(h.plant.pose,z=h.c.ground_z_m+.09)
    elif fault=='tilt':h.pitch=.3
    elif fault=='estop':h.sup.request_stop(h.now,'ESTOP',fault=True,hard=True)
    elif fault=='actuator':h.fault_bits=0x20
    elif fault=='supervisor_loss':h.supervisor_on=False
    elif fault=='gate_loss':h.gate_on=False
    elif fault=='owner':h.modify_frame=lambda f:replace(f,source='foreign')
    elif fault=='overspeed':h.modify_frame=lambda f:replace(f,command=Velocity(.701))
    elif fault=='acceleration':h.modify_frame=lambda f:replace(f,command=Velocity(.6))
    elif fault=='reference_jump':h.modify_frame=lambda f:replace(f,goal=replace(f.goal,x=f.goal.x+.1)) if f.goal else f
    elif fault=='mode':
        for i in (h.sup.inputs,h.gate.inputs):i.mode='rectangle'
    for step in range(25):h.step(ros_dt=-.02 if fault=='clock' and step==0 else None)
    assert h.plant_target==ZERO,(fault,h.sup.reason,h.gate.fault)
    h.roll=h.pitch=0.;h.pose_on={'car':True,'wand':True};h.feedback_on=h.lease_on=True
    h.run(.5);assert h.plant_target==ZERO

@pytest.mark.parametrize('target',[(-.7,0.),(0.,-.3),(.3,.3)])
def test_real_following_and_return_in_new_negative_and_positive_directions(target):
    h=harness.Harness(config(),origin_local=(0.,0.));h.start()
    for i in range(1,251):
        h.set_hand(target[0]*i/250,target[1]*i/250);h.step()
        assert h.sup.state=='ACTIVE' and h.c.inside(h.plant.pose)
    mapped=h.sup.follower.mapped_goal
    assert h.c.geometry().local(mapped.x,mapped.y)==pytest.approx(target,abs=.0021)
    for i in range(1,251):
        h.set_hand(target[0]*(1-i/250),target[1]*(1-i/250));h.step()
        assert h.sup.state=='ACTIVE'
    h.held=False;assert h.step()==ZERO

def test_new_request_and_existing_real_image_evidence_create_only_a_pure_runtime(tmp_path):
    p,m=approved_profile();assert verify_runtime_profile(p,'sign')
    assert verify_trial_review(p['trial_review'],4,'sign',m['firmware']['sha256'])
    path=tmp_path/'synthetic_profile_for_pure_runtime_only.json'
    raw=json.dumps(p).encode();path.write_bytes(raw)
    r=Runtime('hardware',str(path),hashlib.sha256(raw).hexdigest(),CONSENT)
    assert r.config.geometry().goal_bounds()==(-1.5,.5,-.4,.5)
    assert not hasattr(r,'sock') and not hasattr(r,'node')

@pytest.mark.parametrize('change',['old_review','old_request','wrong_policy','false_qualification','stale_site','wrong_arena','relaxed_pose_timeout'])
def test_old_or_unreviewed_authority_cannot_approve_new_scope(change):
    p,m=approved_profile()
    if change=='old_review':p['trial_review']['kind']='BOUNDED_3X3_DEMO_20S_NOT_QUALIFICATION'
    elif change=='old_request':p['trial_review']['range_request']=str(ROOT/'stm32/mecanum_f407/candidates/servo_4ws8_demo_3x3_20s/USER_REQUEST.json')
    elif change=='wrong_policy':p['trial_review']['pilot_policy']=dict(POLICY,max_active_s=20.)
    elif change=='false_qualification':p['trial_review']['continuous_ground_qualified']=True
    elif change=='stale_site':p['staged_runtime_approval']['issued_utc_s']-=601.
    elif change=='wrong_arena':p['safety']['arena_forward_min_m']=-2.
    else:p['safety']['pose_timeout_s']=.5
    with pytest.raises(ValueError):verify_runtime_profile(p,'sign')

def test_old_signed_scope_and_replayed_packets_are_not_new_motion_authority():
    import hmac
    signer=AuthoritySigner('a'*64,4);p=TransportPolicy('a'*64,4)
    text=signer.encode(ZERO,10.,100.)
    assert p.receive(text,10.,100.)
    assert not p.receive(text,10.01,100.01) and p.hard_fault
    env=json.loads(text);env['payload']['scope']='BOUNDED_3X3_DEMO_20S_4WS8_V4_STAGE_4'
    env['signature']=hmac.new(b'\xaa'*32,canonical(env['payload']),hashlib.sha256).hexdigest()
    # Even a correctly signed legacy scope must fail the new exact scope.
    p=TransportPolicy('a'*64,4)
    assert not p.receive(json.dumps(env),10.,100.) and p.hard_fault

def test_120s_metadata_and_operator_labels_match_actual_runtime():
    m=model();assert m.pilot_valid() and m.can_start(10.)
    m.status.update(state='ARMED',gate_ready=True)
    assert '120秒' in m.instruction(10.)
    m.status['pilot']['max_active_s']=20.;assert not m.pilot_valid() and not m.can_start(10.)
    assert '120秒' in reason_text('PILOT_TIME_LIMIT_120S')
    for name in ('pilot_core.py','pilot_ui.py','operator_console.py'):
        source=(GENERATED/'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control'/name).read_text()
        assert 'PILOT_TIME_LIMIT_20S' not in source and '最长20秒' not in source and '单次≤20秒' not in source
    source=(GENERATED/'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/operator_console.py').read_text()
    assert '2.0×0.9m' in source and '四边50cm' in source

def test_firmware_and_all_unchanged_motion_algorithms_stay_byte_identical():
    old=ROOT/'docs/wand_servo_4ws8_2026-10-26/oneclick_v2/ready/runtime/src'
    c='mecanum_wand_4ws8_control';h='mecanum_wand_4ws8_hardware'
    for name in ('core.py','continuous_servo.py','safety_core.py','safety_nodes.py','servo_config.py','staged_config.py'):
        assert (GENERATED/c/c/name).read_bytes()==(old/c/c/name).read_bytes()
    for name in ('nodes.py','stage0_contract.py','stage0_session.py','continuous_bridge.py','startup_idle.py'):
        assert (GENERATED/h/h/name).read_bytes()==(old/h/h/name).read_bytes()
    fw=ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/arm/mecanum_f407_4ws8_staged_NOT_FLASHED.bin'
    assert hashlib.sha256(fw.read_bytes()).hexdigest()=='de87e3fb01df19af5bae75d479cef03376340bd486154ce4e0d75a2c285993ab'
