"""File-only new-host regression; no ROS init, programmer, serial or UDP IO."""
import copy
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
from collections import deque
from unittest.mock import patch
import pytest

from bootstrap import OUT,ROOT,CONTROL,HARDWARE,config,synthetic_packet
from mecanum_wand_4wsb_control.core import Pose,Velocity
from mecanum_wand_4wsb_control.safety_core import ZERO
from mecanum_wand_4wsb_control.cached_packet import decode_packet
from mecanum_wand_4wsb_control.bound_history import RetainedPackets
from mecanum_wand_4wsb_hardware.commissioning import AuthoritySigner,AuthorityVerifier,TransportPolicy
from mecanum_wand_4wsb_hardware.stage0_contract import decode_status,SIZE
from mecanum_wand_4wsb_hardware.stages import get_stage
from mecanum_wand_4wsb_hardware.authorization import verify_runtime_profile,verify_flash_record


@pytest.fixture(autouse=True)
def no_external_io(monkeypatch):
    import socket
    import subprocess
    import rclpy
    def denied(*a,**k): raise AssertionError('offline host test attempted external IO')
    monkeypatch.setattr(socket,'socket',denied)
    monkeypatch.setattr(subprocess,'Popen',denied)
    monkeypatch.setattr(rclpy,'init',denied)


@pytest.mark.parametrize('stage',range(8))
def test_stage_and_all_timing_guards(stage):
    c=config(stage);s=get_stage(stage)
    assert c.stage==stage and c.max_session_s==600 and c.feedback_timeout_s==.30
    assert c.pose_timeout_s==.2 and c.command_timeout_s==.1 and c.lease_timeout_s==.15
    assert c.follower_config().workspace_radius_enabled is False
    assert c.follower_config().max_wheel_rpm==s.target_rpm
    for name,value in [('max_control_gap_s',.2),('pose_timeout_s',.3),('feedback_timeout_s',.31),
                       ('lease_timeout_s',.2),('command_timeout_s',.2),('max_session_s',601.)]:
        with pytest.raises(ValueError): replace(c,**{name:value})
    with pytest.raises(ValueError):
        replace(c,max_accel_mps2=3.14159)


@pytest.mark.parametrize('stage',range(8))
def test_real_c_snapshot_new_wire_and_wrong_policies(stage):
    p=ROOT/f'docs/wand_firmware_4wsb_2026-10-29/ready/native/active{stage}.bin'
    raw=p.read_bytes();s=decode_status(raw)
    assert len(raw)==SIZE==208 and s['stage']==stage and s['output_authority_version']==7
    assert s['wheel_target_slew_guard_cps_s']==get_stage(stage).wheel_slew_guard
    for n in (100,156,184,200,204,207,209):
        with pytest.raises(ValueError): decode_status(raw[:n] if n<=208 else raw+b'0')
    for offset,word in ((0,int.from_bytes(b'4WSA','little')),(200,150000),(204,0x46ce4000)):
        bad=bytearray(raw);struct.pack_into('<I',bad,offset,word)
        # Offset204=26400float, invalid for both allowed guards.
        with pytest.raises(ValueError): decode_status(bad)


@pytest.mark.parametrize('stage',range(8))
def test_hmac_baseline_scoping_replay_expiry_and_latch(stage):
    key='a'*64;s=AuthoritySigner(key,stage);v=AuthorityVerifier(key,stage)
    first=s.encode(ZERO,10.,100.)
    assert v.decode(first,10.,100.)[1]==ZERO
    with pytest.raises(ValueError): v.decode(first,10.,100.)
    with pytest.raises(ValueError):
        AuthorityVerifier(key,(stage+1)%8).decode(s.encode(ZERO,10.,100.),10.,100.)
    with pytest.raises(ValueError): v.decode(s.encode(ZERO,10.,100.),10.101,100.101)
    p=TransportPolicy(key,stage)
    p.receive(s.encode(ZERO,10.,100.),10.,100.)
    p.receive(s.encode(ZERO,10.01,100.01,epoch='e',state='ARMED',permit=True),10.01,100.01)
    cmd=Velocity(.03,0.,0.)
    assert p.receive(s.encode(cmd,10.02,100.02,epoch='e',state='ACTIVE',permit=True),10.02,100.02)
    assert p.active(10.03)
    assert not p.active(10.13)
    p.receive(s.encode(cmd,10.14,100.14,epoch='e',state='ACTIVE',permit=True),10.14,100.14)
    assert p.command==ZERO and not p.active(10.14)


def test_legacy_scope_rejected_and_initial_motion_rejected():
    sys.path.insert(0,str(ROOT/'docs/wand_sync_600s_2026-10-28/prepared/runtime/src/mecanum_wand_4wsa_hardware'))
    sys.path.insert(0,str(ROOT/'docs/wand_sync_600s_2026-10-28/prepared/runtime/src/mecanum_wand_4wsa_control'))
    from mecanum_wand_4wsa_hardware.commissioning import AuthoritySigner as OldSigner
    key='a'*64
    with pytest.raises(ValueError): AuthorityVerifier(key,4).decode(OldSigner(key,4).encode(ZERO,10.,100.),10.,100.)
    with pytest.raises(ValueError):
        AuthorityVerifier(key,4).decode(AuthoritySigner(key,4).encode(Velocity(.1,0.,0.),10.,100.,
            epoch='e',state='ACTIVE',permit=True),10.,100.)


def test_combined_wheel_envelope_and_valid_estop_semantics():
    key='a'*64;s=AuthoritySigner(key,6);v=AuthorityVerifier(key,6)
    v.decode(s.encode(ZERO,10.,100.),10.,100.)
    with pytest.raises(ValueError,match='wheel RPM'):
        v.decode(s.encode(Velocity(.49,.49,2.),10.01,100.01,epoch='e',state='ACTIVE',permit=True),10.01,100.01)
    policy=TransportPolicy(key,6)
    assert policy.receive(s.encode(ZERO,10.,100.),10.,100.)
    assert not policy.receive(s.encode(ZERO,10.01,100.01,estop=True),10.01,100.01)
    assert policy.hard_fault and policy.fault=='ESTOP' and policy.command==ZERO


def test_future_physical_plans_cannot_manufacture_qualification():
    from acceptance import plan,blank_observation
    p=plan(7);o=blank_observation(7,'suspended')
    assert p['stop_planning']['reaction_travel_m']==pytest.approx(.63)
    assert not p['stop_planning']['fifty_cm_automatically_qualified']
    assert o['physical_trial'] is False and o['reviewed_by_human'] is False
    assert o['measured_current_A'] is None and o['motor_temperature_C'] is None


def test_packet_memo_never_memoizes_age_or_accepts_mutation():
    p=synthetic_packet(1.,2.,.3,10.,100.)
    a=decode_packet(p.payload(),10.,100.);b=decode_packet(p.payload(),10.,100.)
    assert a is not b and a._pose is not b._pose
    assert a.pose(10.19,100.19)==b.pose(10.19,100.19)
    with pytest.raises(ValueError): a.pose(10.201,100.201)
    bad=p.payload();bad['samples'][1]['position']=[0.,0.,0.]
    with pytest.raises(ValueError): decode_packet(bad,10.,100.)
    bad=p.payload();bad['digest']='0'*64
    with pytest.raises(ValueError): decode_packet(bad,10.,100.)


def test_source_relay_reuses_only_exact_wire_and_rechecks_age(monkeypatch):
    from mecanum_wand_4wsb_hardware import source_relay as module
    clocks=[10.,100.];calls=[]
    value=dict(packet=synthetic_packet(1.,2.,.1,*clocks).payload(),car=None,conflicts=[],rejected={})
    outputs=[]
    class Pub:
        def publish(self,msg):outputs.append(msg)
    clock=NS(now=lambda:NS(nanoseconds=round(clocks[1]*1e9),to_msg=lambda:NS(sec=int(clocks[1]),nanosec=0)))
    relay=NS(reader=NS(poll=lambda:value),runtime=NS(yaw_offset=0.),get_clock=lambda:clock,
        last_packet=None,last_car=None,cached_wire=None,cached_packet=None,
        rejected=0,last_status=None,status_at=-math.inf,
        packet_pub=Pub(),wand_pub=Pub(),car_pub=Pub(),mode_pub=Pub(),info=Pub())
    relay.header=lambda v:module.SourceRelay.header(relay,v)
    original=module.decode_packet
    def counted(*a):calls.append(1);return original(*a)
    monkeypatch.setattr(module,'decode_packet',counted)
    monkeypatch.setattr(module.time,'monotonic',lambda:clocks[0])
    module.SourceRelay.tick(relay)
    clocks[:]=[10.01,100.01];module.SourceRelay.tick(relay)
    assert len(calls)==1 and relay.rejected==0
    clocks[:]=[10.201,100.201];module.SourceRelay.tick(relay)
    assert len(calls)==1 and relay.rejected==1  # cached geometry cannot renew freshness
    clocks[:]=[10.02,100.02]
    value['packet']['samples'][1]['position']=[0.,0.,0.]  # keep old claimed digest
    value['car']=dict(pose=dict(x=0.,y=0.,z=.1,yaw=0.),stamp=100.02,roll=0.,pitch=0.)
    cars=[]
    relay.car_pub=NS(publish=cars.append)
    module.SourceRelay.tick(relay)
    assert len(calls)==2 and relay.rejected==2
    assert len(cars)==1 and relay.last_car==100.02  # Bad Sign must not erase STOP feedback.


def test_retention_out_of_order_conflict_and_no_receipt_renewal():
    cache=RetainedPackets()
    a=synthetic_packet(1.,2.,0.,10.,100.)
    b=synthetic_packet(1.01,2.,0.,10.01,100.01)
    assert cache.add(b,10.01,100.01) and cache.add(a,10.01,100.01)
    assert cache.last_stamp==100.01 and cache.exact(a.digest,a.stamp,10.01,100.01).received==10.
    assert not cache.add(a,10.02,100.02)
    with pytest.raises(ValueError): cache.exact(a.digest,a.stamp,10.201,100.201)
    different=synthetic_packet(1.02,2.,0.,10.,100.)
    with pytest.raises(ValueError): cache.add(different,10.02,100.02)


@pytest.mark.parametrize('stage',range(8))
def test_draft_rejected_and_constructor_tier_plumbing_without_io(stage,tmp_path,monkeypatch):
    p=OUT/'runtime/src'/HARDWARE/'config'/f'stage{stage}.NOT_APPROVED.json'
    v=json.loads(p.read_text())
    with pytest.raises(ValueError): verify_runtime_profile(v,'sign')
    with pytest.raises(ValueError): verify_flash_record(None,None)
    # Test-local verifier stub proves constructor plumbing, never creates a
    # hardware transport or changes any deliverable/admission evidence.
    from mecanum_wand_4wsb_hardware import commissioning
    v['confirmations']={k:True for k in v['confirmations']}
    v['tracking_confirmations']={k:True for k in v['tracking_confirmations']}
    file=tmp_path/'ONLY_TEST_CONSTRUCTOR_NOT_APPROVED.json';file.write_text(json.dumps(v))
    monkeypatch.setattr(commissioning,'verify_runtime_profile',lambda *a:True)
    r=commissioning.Runtime('hardware',str(file),hashlib.sha256(file.read_bytes()).hexdigest(),commissioning.CONSENT)
    assert r.config.stage==stage and r.motion_status_size==208


def test_no_fixed_fence_does_not_remove_finiteness_and_dynamic_guards():
    c=config(7);p=Pose(20.,-20.,c.ground_z_m,0.)
    assert c.inside(p) and c.project_goal(p,p,4.75)==p
    assert not c.inside(Pose(math.nan,0.))
    assert c.goal_polygon()==c.fence_polygon()==()


def test_protected_inputs_unchanged():
    for p,digest in json.loads((OUT/'PROTECTED_BEFORE.json').read_text()).items():
        assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest,p


@pytest.mark.parametrize('purpose',('sign','suspended'))
def test_launch_without_approval_fails_before_ros_or_socket(monkeypatch,purpose):
    import cli
    m=json.loads((OUT/'HOST_MANIFEST.json').read_text())
    monkeypatch.setattr(cli,'check',lambda _:m)
    monkeypatch.setattr(sys,'argv',['cli','--launch','--purpose',purpose])
    with pytest.raises(ValueError,match='NO APPROVED PROFILE'):cli.main()


@pytest.mark.parametrize('purpose',('sign','suspended'))
def test_draft_launch_fails_before_ros_or_socket(monkeypatch,purpose):
    import cli
    m=json.loads((OUT/'HOST_MANIFEST.json').read_text())
    monkeypatch.setattr(cli,'check',lambda _:m)
    p=OUT/'runtime/src'/HARDWARE/'config/stage0.NOT_APPROVED.json'
    consent=('4WSB_SUSPENDED_SINGLE5S_CLEAR_POWER_CUT_READY' if purpose=='suspended' else
             '4WSB_SPACE_TOGGLE_600S_CLEAR_POWER_CUT_READY')
    monkeypatch.setattr(sys,'argv',['cli','--launch','--purpose',purpose,'--profile',str(p),
        '--profile-sha',hashlib.sha256(p.read_bytes()).hexdigest(),'--consent',consent])
    with pytest.raises(ValueError,match='fresh site approval'):cli.main()


def test_setup_entrypoints_never_fall_back_to_old_rotation_adapters():
    text=(OUT/'runtime/src'/HARDWARE/'setup.py').read_text()
    assert 'optimized_nodes:source_main' in text and 'optimized_nodes:gate_main' in text
    assert '.rotation_nodes:source_main' not in text
    text=(OUT/'runtime/src'/HARDWARE/'launch/wand_4wsb_staged.launch.py').read_text()
    assert "'link_receive_timeout_s': .30" in text
    assert 'clear_latched_faults' not in text


def test_native_snapshot_follows_production_isr_request_tick_snapshot_order():
    from native_twin import NativeController
    c=NativeController(ROOT/'docs/wand_firmware_4wsb_2026-10-29/ready/native/libstage0_oracle.so',7)
    c.velocity((.2,0.,0.),True)
    c.step(c.now+10)
    for _ in range(30):
        c.velocity((.2,0.,0.),True);c.step(c.now+10)
    c.velocity((.002,0.,0.),True)
    # This intentionally proves why the old fixture failed. Production does
    # not publish the half-applied state; it finishes the100Hz update first.
    with pytest.raises(ValueError,match='oracle failed'):c.snapshot()
    c.step(c.now+10)
    assert c.status['tag']=='4WSB' and c.status['raw_duty']==[0]*4
    assert c.wheels.fault_bits==0
    c.velocity((0.,0.,0.),False);c.step(c.now+10)


def test_authorized_launch_description_supplies_observer_path_but_never_runs_ros(tmp_path,monkeypatch):
    import cli
    from mecanum_wand_4wsb_hardware import commissioning
    m=json.loads((OUT/'HOST_MANIFEST.json').read_text())
    monkeypatch.setattr(cli,'check',lambda _:m)
    # Test-only constructor and process boundary substitutions. No approved
    # profile is generated, no Runtime/Node/socket is constructed.
    monkeypatch.setattr(commissioning,'Runtime',lambda *a:None)
    calls=[]
    monkeypatch.setattr(cli.subprocess,'run',lambda argv,**kw:calls.append((argv,kw)))
    p=OUT/'runtime/src'/HARDWARE/'config/stage0.NOT_APPROVED.json'
    destination=tmp_path/'launch_description_only'
    monkeypatch.setattr(sys,'argv',['cli','--launch','--profile',str(p),'--evidence-dir',str(destination)])
    cli.main()
    assert len(calls)==1
    args,kw=calls[0]
    assert 'wand_4wsb_staged.launch.py' in args
    assert kw['env']['WAND_PILOT_TRACE']==str(destination/'observation.jsonl')
    assert kw['env']['WAND_AUTOPREP_FLOW']=='AUTO_PREPARE_SINGLE_SPACE_V1'
    assert not (destination/'observation.jsonl').exists()


@pytest.mark.parametrize('state,enabled,command,bypass',[
    ('STANDBY',False,ZERO,True),('ARMED',True,ZERO,False),
    ('STANDBY',False,Velocity(.1,0.,0.),False)])
def test_only_disabled_zero_standby_can_skip_source_wait(state,enabled,command,bypass):
    from dataclasses import asdict
    from mecanum_wand_4wsb_control.safety_core import Frame
    from mecanum_wand_4wsb_hardware.rotation_nodes import RotationGateNode
    from std_msgs.msg import String
    received=[];outputs=[]
    frame=Frame('test-supervisor',1,10.,100.,'',state,enabled,command,None)
    core=NS(last_sequence=-1,receive=lambda *v:received.append(v),trip=lambda *_:None)
    gate=NS(core=core,pending=deque(),clocks=lambda:(10.,100.),publish_output=outputs.append)
    value=dict(frame=asdict(frame),sign_digest='a'*64,sign_stamp=100.,car_stamp=100.)
    RotationGateNode.on_candidate(gate,String(data=json.dumps(value)))
    assert bool(received)==bypass
    assert len(gate.pending)==(0 if bypass else 1)
    assert outputs==([ZERO] if bypass else [])


@pytest.mark.parametrize('change,ignored',[
    ({},True),({'fresh':True},False),({'primed_epoch':'e'},False),
    ({'blocked_epoch':'e'},False),({'hard_fault':True},False),
    ({'fault':'ESTOP'},False),({'owner':'different'},False),
    ({'active':True},False),({'previous_active':True},False)])
def test_cold_discovery_drop_is_zero_only_and_cannot_renew_an_epoch(change,ignored):
    from mecanum_wand_4wsb_hardware.optimized_nodes import stale_virgin_zero
    from mecanum_wand_4wsb_control.safety_core import Frame
    from dataclasses import replace
    f=Frame('owner',4,9.8,99.8,'','STANDBY',False,ZERO,None)
    core=NS(frame=None,owner=None,last_sequence=-1,primed_epoch='',blocked_epoch='',fault='',hard_fault=False)
    change=change.copy()
    if change.pop('fresh',False):f=replace(f,issued_steady=10.,issued_ros=100.)
    if change.pop('active',False):f=replace(f,state='ACTIVE',enabled=True,epoch='e')
    if change.pop('previous_active',False):core.frame=replace(f,state='ACTIVE',enabled=True,epoch='e')
    for k,v in change.items():setattr(core,k,v)
    before=vars(core).copy()
    assert stale_virgin_zero(f,core,10.,100.) is ignored
    assert vars(core)==before  # No timestamp, frame, owner, counter or epoch refresh.


def test_raw_history_is_not_a_continuous_command_backlog(monkeypatch):
    from mecanum_wand_4wsb_hardware.optimized_nodes import Supervisor,Gate,COMMAND_QOS
    from mecanum_wand_4wsb_hardware.bound_rotation import HistoryQosMixin
    from mecanum_wand_4wsb_control.bound_history import HISTORY_QOS
    from rclpy.node import Node
    assert COMMAND_QOS.depth==4 and HISTORY_QOS.depth==32
    assert HistoryQosMixin not in Supervisor.__mro__
    monkeypatch.setattr(Node,'create_subscription',lambda self,t,topic,cb,qos,*a,**kw:qos)
    assert Gate.create_subscription(NS(),object,'candidate',None,None).depth==4


@pytest.mark.parametrize('state,issued,flag,accepted',[
    ('ACTIVE',11.,True,False),('STOPPING',11.,True,False),
    ('STOP_UNCONFIRMED',11.,True,False),('PAUSED',9.,True,False),
    ('PAUSED',11.,False,False),('PAUSED',float('nan'),True,False),
    ('PAUSED',11.,True,True),('FAULT',11.,True,True)])
def test_stop_ack_is_post_request_and_from_completed_stop_state(state,issued,flag,accepted):
    from split_runtime import stop_acknowledged
    assert stop_acknowledged(dict(state=state,issued_steady=issued,stop_confirmed=flag),10.) is accepted


def test_cold_expired_authenticated_zero_is_discarded_without_any_renewal():
    key='f'*64;signer=AuthoritySigner(key,1);p=TransportPolicy(key,1)
    old=signer.encode(ZERO,10.,100.)
    before=(vars(p).copy(),vars(p.verifier).copy())
    assert p.receive(old,10.2,100.2) is False
    assert (vars(p),vars(p.verifier))==before
    assert p.receive(signer.encode(ZERO,10.21,100.21),10.21,100.21)
    before=(vars(p).copy(),vars(p.verifier).copy())
    assert not p.receive(signer.encode(ZERO,10.22,100.22),10.5,100.5)
    assert (vars(p),vars(p.verifier))==before
    assert p.receive(signer.encode(ZERO,10.51,100.51,epoch='e',state='ARMED',permit=True),10.51,100.51)
    assert p.receive(signer.encode(Velocity(.02,0.,0.),10.52,100.52,epoch='e',state='ACTIVE',permit=True),10.52,100.52)
    assert p.active(10.53)
    assert not p.receive(signer.encode(ZERO,10.54,100.54),10.8,100.8)
    assert p.hard_fault and p.command==ZERO and 'e' in p.blocked


@pytest.mark.parametrize('kind',('future','mixed_clock','signature','armed','epoch','estop','unknown_state','wrong_owner','wrong_scope'))
def test_cold_discard_never_bypasses_other_validation(kind):
    key='f'*64;signer=AuthoritySigner(key,1);p=TransportPolicy(key,1)
    kw={};now,ros=10.2,100.2
    if kind=='future':now,ros=9.8,99.8
    if kind=='mixed_clock':now,ros=10.2,99.8
    if kind=='armed':kw=dict(state='ARMED',permit=True,epoch='e')
    if kind=='epoch':kw=dict(epoch='e')
    if kind=='estop':kw=dict(estop=True)
    if kind=='unknown_state':kw=dict(state='UNKNOWN')
    if kind=='wrong_owner':p.verifier.owner='someone-else'
    if kind=='wrong_scope':signer=AuthoritySigner(key,2)
    raw=signer.encode(ZERO,10.,100.,**kw)
    if kind=='signature':
        env=json.loads(raw);env['signature']='0'*64;raw=json.dumps(env)
    assert not p.receive(raw,now,ros)
    assert p.hard_fault and p.fault=='SIGNED_AUTHORITY_INVALID' and p.command==ZERO


@pytest.mark.parametrize('scenario,still',[
    ('zero',True),('single_quantized_pulse',True),('fast_pulse',False),
    ('continuous_slow_motion',False),('continuous_yaw',False),
    ('out_and_back',False),('enabled',False),('nonzero_output',False),('fault',False)])
def test_still_feedback_uses_absolute_window_not_cancelling_or_one_tick(scenario,still):
    from mecanum_wand_4wsb_control.stop_feedback import StopFeedbackInputs
    i=StopFeedbackInputs(config(1))
    for n in range(61):
        t=n*.01;speed=yaw=0.;enabled=False;zero=True;fault=0
        if scenario=='single_quantized_pulse' and n==30:speed,yaw=.02,.1
        if scenario=='fast_pulse' and n==30:speed,yaw=.5,1.
        if scenario=='continuous_slow_motion':speed=.006
        if scenario=='continuous_yaw':yaw=.021
        if scenario=='out_and_back' and 20<=n<=40:speed=.04
        if scenario=='enabled' and n==30:enabled=True
        if scenario=='nonzero_output' and n==30:zero=False
        if scenario=='fault' and n==30:fault=1
        i.observe_feedback(n,10+t,100+t,10+t,100+t,
            output_zero=zero,enabled=enabled,speed=speed,yaw_rate=yaw,fault_bits=fault)
    assert i.feedback_still(10.6,100.6) is still
    assert not i.feedback_still(10.901,100.901)


def test_standstill_requires_vicon_freshness_full_dwell_and_feedback_progress():
    from mecanum_wand_4wsb_control.stop_feedback import StopFeedbackInputs
    i=StopFeedbackInputs(config(1));qualified=[]
    for n in range(151):
        t=n*.01
        assert i.observe_pose('car',Pose(0.,0.,i.c.ground_z_m,0.),100+t,10+t,100+t)
        i.observe_feedback(n,10+t,100+t,10+t,100+t,
            output_zero=True,enabled=False,speed=0.,yaw_rate=0.)
        qualified.append(i.update_stationarity(10+t,100+t))
    assert not any(qualified[:50]) and qualified[51]
    before=list(i.feedback_history)
    assert not i.observe_feedback(150,11.5,101.5,11.6,101.6,
        output_zero=True,enabled=False,speed=0.,yaw_rate=0.)
    assert list(i.feedback_history)==before
    assert not i.update_stationarity(11.801,101.801)


def test_zero_feedback_cannot_hide_vicon_out_and_back_excursion():
    from mecanum_wand_4wsb_control.stop_feedback import StopFeedbackInputs
    i=StopFeedbackInputs(config(1))
    for n in range(101):
        t=n*.01;x=.02 if 65<=n<=75 else 0.
        i.observe_pose('car',Pose(x,0.,i.c.ground_z_m,0.),100+t,10+t,100+t)
        i.observe_feedback(n,10+t,100+t,10+t,100+t,
            output_zero=True,enabled=False,speed=0.,yaw_rate=0.)
    assert i.feedback_still(11.,101.)
    assert not i.update_stationarity(11.,101.)


def test_still_history_rejects_missing_coverage_and_clock_rewind():
    from mecanum_wand_4wsb_control.stop_feedback import StopFeedbackInputs
    i=StopFeedbackInputs(config(1))
    for n,t in enumerate((0.,.4,.5,.6)):
        i.observe_feedback(n,10+t,100+t,10+t,100+t,
            output_zero=True,enabled=False,speed=0.,yaw_rate=0.)
    assert not i.feedback_still(10.6,100.6)
    i.observe_feedback(4,10.59,100.59,10.6,100.6,
        output_zero=True,enabled=False,speed=0.,yaw_rate=0.)
    assert len(i.feedback_history)==1 and not i.feedback_still(10.6,100.6)


@pytest.mark.parametrize('stage',(6,7))
def test_demo_recipe_is_continuous_and_includes_fast_hands_and_rotation(stage):
    from split_runtime import hand_gesture
    values=[hand_gesture(n*.01,stage,'demo') for n in range(6001)]
    assert all(all(math.isfinite(v) for v in row) for row in values)
    speeds=[math.hypot(b[0]-a[0],b[1]-a[1])/.01 for a,b in zip(values,values[1:])]
    assert .40<max(speeds)<.70 and max(speeds)*.01<.01
    assert max(abs(b[2]-a[2]) for a,b in zip(values,values[1:]))<.012
    assert max(abs(v[2]) for v in values)>(.5 if stage==6 else .02)


def test_timing_report_joins_only_matching_active_sequences():
    from software_timing import summarize
    p=[dict(kind='control_publish',seq=1,issued=10.,at=10.001,state='ACTIVE'),
       dict(kind='control_publish',seq=2,issued=11.,at=11.001,state='STANDBY')]
    g=[dict(kind='gate_callback_enter',seq=1,issued=10.,at=10.002),
       dict(kind='gate_core_receive',seq=1,issued=10.,at=10.003,accepted=True)]
    s=summarize(p,g)
    assert s['matched']==1 and s['active_publications']==1 and not s['rejected_matched']
    assert s['issued_to_core_return']['max_ms']==pytest.approx(3.)
    g[1]['issued']=9.
    with pytest.raises(ValueError):summarize(p,g)
