"""Offline-only regression, source-time, geometry and safety fault injection."""
import ast
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT=Path(os.environ['SERVO_GENERATED_ROOT'])
for name in ('mecanum_wand_servo_control','mecanum_wand_servo_hardware'):
    sys.path.insert(0,str(ROOT/name))
from mecanum_wand_servo_control.core import Pose,Velocity,RelativeMapping
from mecanum_wand_servo_control.continuous_servo import ContinuousSession
from mecanum_wand_servo_control.offline_validation import Harness,load_config,simulate
from mecanum_wand_servo_control.safety_core import Frame,Supervisor,FinalGate,ZERO
from mecanum_wand_servo_hardware.commissioning import Runtime,AuthoritySigner,AuthorityVerifier

PROFILE=ROOT/'mecanum_wand_servo_hardware/config/servo.3x3_1to1.OFFLINE_ONLY.json'
WORKSPACE=Path('/home/PRIMARY_USER/vicon_mecanum_ws')


@pytest.fixture
def config():
    return load_config(PROFILE)


def session(config):
    c=config.geometry()
    car=Pose(*c.world(1.5,1.5),config.ground_z_m,c.heading)
    hand=Pose(1.,1.,1.)
    s=ContinuousSession(config.follower_config(),config.project_goal,config.project_reference)
    s.observe('car',car,10.,source_stamp=100.)
    s.observe('wand',hand,10.,source_stamp=100.)
    assert s.engage(10.)[0]
    return s,car,hand


def move_hand(hand,car,distance):
    return replace(hand,x=hand.x+math.cos(car.yaw)*distance,
                   y=hand.y+math.sin(car.yaw)*distance)


def test_position_has_no_80ms_low_pass_and_only_one_acceleration_limiter(config):
    s,car,hand=session(config)
    moved=move_hand(hand,car,.10)
    s.observe('car',car,10.02,source_stamp=100.02)
    s.observe('wand',moved,10.02,source_stamp=100.02)
    command=s.tick(10.02)
    assert math.hypot(s.mapped_goal.x-car.x,s.mapped_goal.y-car.y)==pytest.approx(.10)
    assert math.hypot(s.goal.x-car.x,s.goal.y-car.y)==pytest.approx(.004)
    assert math.hypot(command.vx,command.vy)==pytest.approx(.08*.02)
    assert s.config.filter_tau_s==0. and config.max_accel_mps2==.08


def test_velocity_uses_source_timestamps_not_irregular_receipt_intervals(config):
    s,car,hand=session(config)
    for receipt,source,distance in ((10.013,100.01,.001),(10.037,100.02,.002),
                                     (10.049,100.03,.003)):
        s.observe('car',car,receipt,source_stamp=source)
        s.observe('wand',move_hand(hand,car,distance),receipt,source_stamp=source)
        s.tick(receipt)
    vx,vy=s.target_velocity()
    assert math.hypot(vx,vy)==pytest.approx(.1,rel=1e-8)


def test_source_replay_cannot_refresh_receipt_or_estimator(config):
    s,car,hand=session(config)
    s.observe('wand',move_hand(hand,car,.003),10.02,source_stamp=100.02)
    before=list(s.velocity_samples)
    assert not s.observe('wand',move_hand(hand,car,.004),10.04,source_stamp=100.02)
    assert s.samples['wand'][1]==10.02 and list(s.velocity_samples)==before


def test_velocity_drops_before_pose_timeout_without_extrapolating_pose(config):
    s,car,hand=session(config)
    s.observe('wand',move_hand(hand,car,.004),10.02,source_stamp=100.02)
    s.observe('car',car,10.02,source_stamp=100.02);s.tick(10.02)
    assert math.hypot(*s.target_velocity())>.01
    last=s.samples['wand']
    s.observe('car',car,10.09,source_stamp=100.09);s.tick(10.09)
    assert s.target_velocity()==(0.,0.) and s.samples['wand']==last
    s.observe('car',car,10.23,source_stamp=100.23)
    assert s.tick(10.23)==ZERO and s.state=='FAULT'


def test_pause_and_new_engage_clear_velocity_history_and_old_target(config):
    s,car,hand=session(config)
    s.observe('wand',move_hand(hand,car,.004),10.02,source_stamp=100.02)
    s.observe('car',car,10.02,source_stamp=100.02);s.tick(10.02)
    s.pause()
    assert not s.velocity_samples and s.reference_velocity==(0.,0.)
    s.observe('car',car,10.04,source_stamp=100.04)
    s.observe('wand',move_hand(hand,car,.12),10.04,source_stamp=100.04)
    assert s.engage(10.04)[0] and s.target_velocity()==(0.,0.)
    s.observe('car',car,10.06,source_stamp=100.06)
    s.observe('wand',move_hand(hand,car,.12),10.06,source_stamp=100.06)
    assert s.tick(10.06)==ZERO


def test_velocity_history_cannot_grow_without_bound_or_retain_pre_gap_motion(config):
    s,car,hand=session(config)
    for i in range(1,101):
        t=10.+i*.0001
        s.observe('wand',move_hand(hand,car,i*.00001),t,source_stamp=100.+i*.0001)
    assert len(s.velocity_samples)<=16
    s.observe('wand',move_hand(hand,car,.01),10.10,source_stamp=100.10)
    assert len(s.velocity_samples)==1 and s.target_velocity()==(0.,0.)


@pytest.mark.parametrize('source',['car','wand'])
@pytest.mark.parametrize('bad_stamp',[math.nan,math.inf])
def test_invalid_source_time_never_reaches_feedforward(config,source,bad_stamp):
    s,car,hand=session(config)
    assert not s.observe(source,car if source=='car' else hand,10.02,source_stamp=bad_stamp)
    assert s.state=='FAULT' and s.command==ZERO and not s.velocity_samples


@pytest.mark.parametrize('local',[(0,0),(3,0),(0,3),(3,3),(-1,1),(4,1),(1,-1),(1,4)])
def test_one_to_one_nominal_goals_clamp_and_fifty_cm_buffer(config,local):
    a=config.geometry()
    origin=Pose(*a.world(0,0),config.ground_z_m,a.heading)
    hand=Pose(1,1,1)
    x,y=a.world(*local)
    moved=replace(hand,x=hand.x+x-a.origin_x,y=hand.y+y-a.origin_y,z=2.,yaw=1.)
    goal=config.project_goal(RelativeMapping(origin,hand).goal(moved),origin,4.75)
    assert a.local(goal.x,goal.y)==pytest.approx(tuple(max(0,min(3,v)) for v in local),abs=2e-8)
    assert goal.z==origin.z and goal.yaw==origin.yaw and config.inside(goal)
    assert a.physical_bounds()==pytest.approx((-.595,3.595,-.605,3.605))
    assert a.center_bounds(a.heading)==pytest.approx((-.30,3.30,-.30,3.30))


def test_rotated_footprint_and_fixed_reference_never_recenter(config):
    a=config.geometry();before=config.goal_polygon()
    p=Pose(*a.world(3.29,1.5),config.ground_z_m,a.heading)
    assert config.inside(p) and not config.inside(replace(p,yaw=a.heading+math.pi/4))
    h=Harness(config,origin_local=(-.05,.02));h.start();h.run(.3)
    assert h.sup.state=='ACTIVE' and config.goal_inside(h.sup.follower.mapped_goal)
    assert config.goal_polygon()==before


@pytest.mark.parametrize('field,value',[('max_speed_mps',.3),('max_accel_mps2',.12),
    ('arena_buffer_m',.1),('arena_forward_max_m',4.),('goal_reference_policy','direct'),
    ('goal_boundary_policy','fault'),('max_session_s',150.)])
def test_candidate_cannot_relax_existing_envelope(config,field,value):
    with pytest.raises(ValueError):replace(config,**{field:value})


FAULTS=('wand_loss','car_loss','source_replay','feedback_loss','operator_loss','operator_release',
    'wand_jump','car_jump','lift','tilt','car_boundary','mode_conflict','source_conflict',
    'command_link_loss','supervisor_loss','delayed_command','command_replay','overspeed',
    'acceleration_jump','reference_jump','owner_changed','actuator_fault','estop',
    'session_timeout','clock_backwards','clock_forward','timer_gap')


@pytest.mark.parametrize('fault',FAULTS)
def test_full_supervisor_and_independent_gate_faults_stop_without_auto_resume(config,fault):
    h=Harness(config);h.moving();start=h.now
    if fault=='wand_loss':h.pose_on['wand']=False
    elif fault=='car_loss':h.pose_on['car']=False
    elif fault=='source_replay':h.pose_stamp['wand']=h.ros
    elif fault=='feedback_loss':h.feedback_on=False
    elif fault=='operator_loss':h.lease_on=False
    elif fault=='operator_release':h.held=False
    elif fault=='wand_jump':h.hand=replace(h.hand,x=h.hand.x+.3)
    elif fault=='car_jump':h.plant.pose=replace(h.plant.pose,x=h.plant.pose.x+.21)
    elif fault=='lift':h.plant.pose=replace(h.plant.pose,z=config.ground_z_m+.09)
    elif fault=='tilt':h.roll=.3
    elif fault=='car_boundary':h.plant.pose=replace(h.plant.pose,
        x=config.geometry().world(3.31,1.5)[0],y=config.geometry().world(3.31,1.5)[1])
    elif fault=='mode_conflict':
        for inputs in (h.sup.inputs,h.gate.inputs):inputs.mode='rectangle'
    elif fault=='source_conflict':
        for inputs in (h.sup.inputs,h.gate.inputs):inputs.graph_ok=False
    elif fault=='command_link_loss':h.command_link_on=False
    elif fault=='supervisor_loss':h.supervisor_on=False
    elif fault=='delayed_command':h.modify_frame=lambda f:replace(f,issued_steady=f.issued_steady-.15,
                                                                issued_ros=f.issued_ros-.15)
    elif fault=='command_replay':
        replay=h.gate.frame;h.modify_frame=lambda f:replay
    elif fault=='overspeed':h.modify_frame=lambda f:replace(f,command=Velocity(.201))
    elif fault=='acceleration_jump':h.modify_frame=lambda f:replace(f,command=Velocity(.19))
    elif fault=='reference_jump':h.modify_frame=lambda f:replace(f,goal=replace(f.goal,x=f.goal.x+.05)) if f.goal else f
    elif fault=='owner_changed':h.modify_frame=lambda f:replace(f,source='unsigned-foreign-owner')
    elif fault=='actuator_fault':h.fault_bits=0x20
    elif fault=='estop':h.sup.request_stop(h.now,'offline estop',fault=True,hard=True)
    elif fault=='session_timeout':h.sup.active_started=h.now-config.max_session_s
    first_zero=None
    for i in range(1,21):
        output=h.step(dt=.12 if fault=='timer_gap' and i==1 else .02,
            ros_dt=-.02 if fault=='clock_backwards' and i==1 else
                   .4 if fault=='clock_forward' and i==1 else None)
        if output==ZERO and first_zero is None:first_zero=h.now-start
    assert first_zero is not None and first_zero<=.24+1e-8,(fault,h.sup.reason,h.gate.fault)
    assert h.output==ZERO
    # Restoring data alone must never arm/resume motion. Release/restart is
    # deliberately NOT requested here; persistent hard faults remain latched.
    h.pose_on={'car':True,'wand':True};h.pose_stamp.clear()
    h.feedback_on=h.lease_on=h.supervisor_on=h.command_link_on=True
    h.modify_frame=lambda f:f;h.roll=h.pitch=0.;h.fault_bits=0
    for inputs in (h.sup.inputs,h.gate.inputs):inputs.mode='wand';inputs.graph_ok=True
    h.run(.6)
    assert h.output==ZERO and h.sup.state not in ('ACTIVE','ARMED')


def test_gate_loss_uses_only_simulated_independent_downstream_watchdog(config):
    h=Harness(config);h.moving();h.gate_on=False;h.supervisor_on=False
    h.run(.18)
    assert h.plant_target==ZERO  # This is not a physical MCU watchdog test.


def test_release_reprepare_never_auto_moves_and_resets_velocity_estimator(config):
    h=Harness(config);h.moving();h.held=False
    assert h.step()==ZERO
    h.run(2.)
    assert h.sup.state=='PAUSED' and h.sup.stop_confirmed
    h.set_hand(.20,.05);h.run(1.)
    ok,reason=h.sup.request_start(h.now,h.ros);assert ok,reason
    h.run(.1);parked=h.plant.pose
    assert h.sup.state=='ARMED' and h.output==ZERO
    h.held=True;h.run(.4)
    assert h.sup.state=='ACTIVE' and h.output==ZERO
    assert math.hypot(h.plant.pose.x-parked.x,h.plant.pose.y-parked.y)<.001


def test_occlusion_recovery_requires_release_explicit_reprepare_and_new_hold(config):
    h=Harness(config);h.moving();h.pose_on['wand']=False;h.run(.3)
    assert h.output==ZERO and h.sup.fault_latched
    h.pose_on['wand']=True;h.run(2.)
    assert not h.sup.request_recover(h.now,h.ros)[0]  # Still held, not allowed.
    assert h.output==ZERO and h.sup.state=='FAULT'
    h.held=False;h.run(1.8)
    assert h.sup.request_recover(h.now,h.ros)[0]
    h.run(1.)
    assert h.sup.state=='ARMED' and h.output==ZERO
    parked=h.plant.pose
    h.held=True;h.run(.3)
    assert h.sup.state=='ACTIVE' and h.output==ZERO
    assert math.hypot(h.plant.pose.x-parked.x,h.plant.pose.y-parked.y)<.001


def test_servo_policy_profile_cannot_silently_reenable_filter_or_change_gain(config,tmp_path):
    p=json.loads(PROFILE.read_text());p['servo_policy']['position_filter_tau_s']=.08
    target=tmp_path/'synthetic_only.json';target.write_text(json.dumps(p))
    with pytest.raises(ValueError,match='policy mismatch'):Runtime('shadow',str(target))


@pytest.mark.parametrize('mode',['hardware','emulator'])
def test_no_profile_or_old_consent_can_open_transport(monkeypatch,mode):
    import mecanum_wand_servo_hardware.nodes as nodes
    monkeypatch.setenv('WAND_SERVO_MODE',mode)
    monkeypatch.setenv('WAND_SERVO_PROFILE',str(PROFILE))
    monkeypatch.setenv('WAND_PROFILE_APPROVAL',hashlib.sha256(PROFILE.read_bytes()).hexdigest())
    monkeypatch.setenv('WAND_HARDWARE_CONSENT','ONE_TO_ONE_3X3_020_CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY')
    def forbidden(*a,**k):raise AssertionError('offline candidate must not open a socket')
    monkeypatch.setattr(nodes.socket,'socket',forbidden)
    with pytest.raises(ValueError,match='OFFLINE_ONLY'):
        nodes.HardwareBridge.__init__(object.__new__(nodes.HardwareBridge))


def test_profile_and_real_node_constructor_keep_same_fixed_arena_without_dds(config,monkeypatch):
    from mecanum_wand_servo_control.safety_nodes import SafetyNode,Node
    r=Runtime('shadow',str(PROFILE));parameters={}
    assert r.source_domain==113 and r.ip==''
    assert r.profile['arena_confirmations']['extended_arena_and_buffer_clear_and_tracked'] is False
    monkeypatch.setattr(Node,'__init__',lambda self,*a,**k:None)
    monkeypatch.setattr(Node,'declare_parameter',lambda self,n,v:parameters.setdefault(n,v))
    monkeypatch.setattr(Node,'get_parameter',lambda self,n:SimpleNamespace(value=parameters[n]))
    monkeypatch.setattr(Node,'create_timer',lambda *a,**k:None)
    monkeypatch.setattr(Node,'create_subscription',lambda *a,**k:None)
    stub=SimpleNamespace(namespace=r.namespace,world=r.world,mode='shadow',config=r.config,guard=lambda n:None)
    for core in (Supervisor,FinalGate):
        node=object.__new__(SafetyNode);SafetyNode.__init__(node,'offline_constructor',core,stub)
        assert node.core.c==config and type(node.core.c) is type(config)
    node=object.__new__(SafetyNode);SafetyNode.__init__(node,'offline_constructor',Supervisor,stub)
    assert isinstance(node.core.follower,ContinuousSession)


def test_final_gate_and_authority_verifier_are_not_weakened():
    report=json.loads((ROOT/'generation_report.json').read_text())
    assert report['independent_final_gate_unchanged']
    baseline=WORKSPACE/'docs/wand_one_to_one_3x3_2026-10-23/verified_runtime/src'
    for relative,sha in report['existing_source_sha256'].items():
        assert hashlib.sha256((baseline/relative).read_bytes()).hexdigest()==sha
    def gate_source(path):
        source=path.read_text()
        node=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='FinalGate')
        return ast.get_source_segment(source,node)
    assert gate_source(baseline/'mecanum_wand_one_to_one_control/mecanum_wand_one_to_one_control/safety_core.py')==gate_source(
        ROOT/'mecanum_wand_servo_control/mecanum_wand_servo_control/safety_core.py')
    key='ab'*32;sign=AuthoritySigner(key);verify=AuthorityVerifier(key)
    baseline_message=sign.encode(ZERO,10.,100.)
    verify.decode(baseline_message,10.,100.)
    active=sign.encode(Velocity(.1),10.02,100.02,epoch='test',state='ACTIVE',permit=True)
    mutated=json.loads(active);mutated['payload']['command']['vx']=.19
    with pytest.raises(ValueError,match='signature'):verify.decode(json.dumps(mutated),10.02,100.02)
    verify.decode(active,10.02,100.02)
    with pytest.raises(ValueError):verify.decode(active,10.02,100.02)


def test_generator_reproduces_the_verified_candidate_without_touching_old_inputs(tmp_path):
    import importlib.util
    path=WORKSPACE/'candidates/wand_low_latency_servo/build_candidate.py'
    spec=importlib.util.spec_from_file_location('servo_generator_under_test',path)
    generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
    target=tmp_path/'synthetic_generation_only'
    generator.generate(target)
    for name in ('mecanum_wand_servo_control','mecanum_wand_servo_hardware'):
        for relative,sha in generator.hashes(target/name).items():
            assert hashlib.sha256((ROOT/name/relative).read_bytes()).hexdigest()==sha


@pytest.mark.parametrize('name',['ramp_hold_return','circle','reversals','large_target','boundary','jitter_hold'])
def test_full_closed_loop_bounds_and_same_time_quality(config,name):
    old,_=simulate(config,name,True);new,_=simulate(config,name,False)
    if name in ('ramp_hold_return','circle'):
        assert new['rms_same_time_error_m']<old['rms_same_time_error_m']*.85
        assert new['onset_to_001mps_s']<old['onset_to_001mps_s']
    if name=='reversals':
        assert new['rms_same_time_error_m']<old['rms_same_time_error_m']
        assert new['p95_same_time_error_m']<old['p95_same_time_error_m']
    if name in ('large_target','boundary'):
        assert new['rms_same_time_error_m']<old['rms_same_time_error_m']
    if name in ('ramp_hold_return','large_target','boundary'):
        assert new['final_same_time_error_m']<.015
    if name=='jitter_hold':
        assert new['max_speed_mps']<1e-6


@pytest.mark.parametrize('lag,gains',[(.04,(1.,1.,1.)),(.2,(.9,.8,.9)),(.3,(.85,.75,.9))])
def test_model_mismatch_stays_bounded_not_a_vehicle_identification(config,lag,gains):
    r,_=simulate(config,'circle',False,lag,gains)
    assert r['p95_same_time_error_m']<.06
