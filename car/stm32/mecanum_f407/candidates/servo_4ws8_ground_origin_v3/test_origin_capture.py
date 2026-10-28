"""Preparation/engagement lifecycle, raw-record replay and real codec tests."""
from dataclasses import asdict,replace
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import pytest

from test_pilot_bounds import config,harness,live,ROOT,GENERATED
from mecanum_wand_4ws8_control.core import Pose,Velocity
from mecanum_wand_4ws8_control.safety_core import ZERO
from mecanum_wand_4ws8_control.safety_nodes import decode_frame,SupervisorNode,GateNode
from mecanum_wand_4ws8_control.pilot_core import POLICY
from mecanum_wand_4ws8_hardware.commissioning import AuthoritySigner,TransportPolicy
from mecanum_wand_4ws8_hardware.nodes import AcceptanceGate
from replay_trial import run as replay


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import socket,rclpy
    def forbidden(*a,**k):pytest.fail('offline regression attempted hardware/ROS initialization')
    monkeypatch.setattr(socket,'socket',forbidden)
    monkeypatch.setattr(rclpy,'init',forbidden)


def prepared():
    h=harness.Harness(config(),origin_local=(.1,.1))
    h.run(1.8);assert h.sup.request_start(h.now,h.ros)[0]
    h.run(.1);assert h.sup.state=='ARMED' and h.gate.primed_epoch==h.sup.epoch
    return h


def adjust(h,dx,dy,z=1.6,yaw=.8):
    initial=h.hand
    for i in range(1,61):
        fraction=i/60
        h.hand=Pose(initial.x+dx*fraction,initial.y+dy*fraction,
            initial.z+(z-initial.z)*fraction,yaw*fraction)
        assert h.step()==ZERO and h.sup.state=='ARMED',(h.sup.reason,h.gate.fault)
    return h.hand


def enable(h):
    h.held=True;assert h.step()==ZERO
    assert h.sup.state=='ACTIVE',(h.sup.reason,h.gate.fault)
    assert h.sup.pilot.wand==h.sup.follower.mapping.wand_origin
    assert h.sup.pilot.car==h.sup.follower.mapping.car_origin
    assert h.sup.pilot.started==h.gate.pilot.started==h.sup.active_started
    return h.hand


def move_after_enable(h,anchor,forward=.05,left=0.):
    c,s=math.cos(h.origin.yaw),math.sin(h.origin.yaw)
    for i in range(1,16):
        fraction=i/15
        h.hand=Pose(anchor.x+(c*forward-s*left)*fraction,
                    anchor.y+(s*forward+c*left)*fraction,anchor.z)
        h.step()


def test_prepare_does_not_capture_origins_or_start_time_path_or_budget():
    h=prepared();adjust(h,.60,.40)
    h.run(7.)
    for core in (h.sup,h.gate):
        assert core.pilot.car is None and core.pilot.wand is None and core.pilot.last is None
        assert core.pilot.started is None and not core.pilot.used and core.pilot.path==0
    status=h.sup.pilot_status(h.now)
    assert status['remaining_s']==5. and status['capture_phase']=='WAIT_HELD_EDGE'
    assert h.output==ZERO


@pytest.mark.parametrize('motion',[(.60,.40,.65),(0.,0.,1.6),(-.70,.30,1.4),(.30,-.60,.7)])
def test_pick_up_adjust_after_prepare_then_enable_is_neutral_and_one_to_one(motion):
    h=prepared();adjust(h,*motion)
    anchor=enable(h);mapping=h.sup.follower.mapping
    h.run(.3);assert h.output==ZERO and h.plant.pose==mapping.car_origin
    move_after_enable(h,anchor)
    assert h.sup.state=='ACTIVE' and h.output!=ZERO
    goal=h.sup.follower.mapped_goal
    assert math.hypot(goal.x-mapping.car_origin.x,goal.y-mapping.car_origin.y)==pytest.approx(.05,abs=1e-8)
    assert h.sup.pilot.wand==anchor and h.gate.pilot.wand==anchor
    assert h.sup.follower.mapping.scale==1.


def test_repeated_prepare_adjustments_do_not_accumulate_into_trial_budget():
    h=prepared();adjust(h,.4,.4);adjust(h,-.5,.3);adjust(h,.2,-.7)
    enable(h)
    assert h.sup.pilot.path==h.gate.pilot.path==0.
    assert h.output==ZERO


def test_cancel_before_enable_then_new_manual_prepare_uses_no_stale_anchor():
    h=prepared();old=h.sup.epoch
    h.sup.request_stop(h.now,'cancel before engagement');h.run(2.)
    adjust_start=h.hand
    for i in range(1,61):
        h.hand=replace(adjust_start,x=adjust_start.x+.5*i/60);h.step()
    assert h.sup.request_start(h.now,h.ros)[0];h.run(.1)
    assert h.sup.epoch!=old and not h.sup.pilot.used
    anchor=enable(h);assert h.sup.pilot.wand==anchor


@pytest.mark.parametrize('fault',['wand_loss','car_loss','feedback_loss','lease_loss','wand_jump','lift','tilt','actuator_fault'])
def test_adjustment_permission_does_not_relax_pre_enable_health_checks(fault):
    h=prepared()
    if fault=='wand_loss':h.pose_on['wand']=False
    elif fault=='car_loss':h.pose_on['car']=False
    elif fault=='feedback_loss':h.feedback_on=False
    elif fault=='lease_loss':h.lease_on=False
    elif fault=='wand_jump':h.hand=replace(h.hand,x=h.hand.x+.3)
    elif fault=='lift':h.plant.pose=replace(h.plant.pose,z=h.c.ground_z_m+.09)
    elif fault=='tilt':h.roll=.3
    elif fault=='actuator_fault':h.fault_bits=0x20
    h.run(.3);h.held=True;h.run(.2)
    assert h.output==ZERO and h.sup.state!='ACTIVE'
    assert h.sup.pilot.started is None and not h.sup.pilot.used


@pytest.mark.parametrize('which',['supervisor','gate'])
def test_post_enable_fifteen_cm_limit_still_relative_to_new_anchor(which):
    h=prepared();adjust(h,.7,.4);anchor=enable(h)
    # Isolate each new monitor from the other monitor and base jump limit.
    monitor=h.sup.pilot if which=='supervisor' else h.gate.pilot
    inputs=h.sup.inputs if which=='supervisor' else h.gate.inputs
    frozen=(monitor.car,monitor.wand,monitor.started)
    inputs.poses['wand']['pose']=replace(anchor,x=anchor.x+.151)
    assert monitor.active(monitor.epoch,inputs,monitor.car,h.now+.02)=='PILOT_HAND_LIMIT'
    assert (monitor.car,monitor.wand,monitor.started)==frozen


def test_hand_limit_stops_full_chain_after_successful_adjusted_enable():
    h=prepared();adjust(h,.6,.4);anchor=enable(h)
    move_after_enable(h,anchor,forward=.151)
    assert h.output==ZERO and 'PILOT_HAND_LIMIT' in h.sup.reason
    h.hand=anchor;h.run(.4);assert h.output==ZERO


def test_five_seconds_count_from_enable_not_prepare_and_cannot_be_renewed():
    h=prepared();h.run(8.);anchor=enable(h);t0=h.now
    h.run(4.8);assert h.sup.state=='ACTIVE'
    assert 0<h.sup.pilot_status(h.now)['remaining_s']<=.21
    h.run(.24);assert h.output==ZERO and h.sup.state!='ACTIVE'
    assert 'PILOT_TIME_LIMIT' in h.sup.reason or 'PILOT_TIME_LIMIT' in h.gate.fault
    assert h.sup.pilot.started==h.gate.pilot.started==t0
    h.held=False;h.run(2.)
    assert not h.sup.request_start(h.now,h.ros)[0] and not h.sup.request_recover(h.now,h.ros)[0]


def test_gate_uses_current_pose_not_arm_pose_and_original_frame_start_time():
    h=prepared();adjust(h,.6,.4)
    h.gate_on=False;anchor=h.hand
    h.held=True;h.step();frame=h.sup.frame(h.now,h.ros)
    # Base command remains neutral at engage. Deliver a normally delayed
    # frame; do not give the gate another five seconds from receipt.
    assert frame.command==ZERO
    assert h.gate.receive(frame,h.now+.04,h.ros+.04)
    assert h.gate.tick(h.now+.04,h.ros+.04)==ZERO
    assert h.gate.pilot.wand==anchor
    assert h.gate.pilot.started==frame.issued_steady==h.sup.active_started
    assert h.gate.pilot.active(frame.epoch,h.gate.inputs,frame.goal,frame.issued_steady+5.001)=='PILOT_TIME_LIMIT_5S'


def test_gate_rejects_missing_neutral_first_frame_before_any_signed_permission():
    h=prepared();h.gate_on=False;h.held=True;h.step()
    first=h.sup.frame(h.now,h.ros)
    forged=replace(first,sequence=first.sequence+1,command=Velocity(.001))
    assert h.gate.receive(forged,h.now,h.ros)
    assert h.gate.tick(h.now,h.ros)==ZERO
    assert h.gate.fault=='PILOT_NEUTRAL_CAPTURE_REQUIRED' and not h.gate.primed_epoch
    assert not h.gate.pilot.used
    published=[];signer=AuthoritySigner('a'*64,4)
    fake=SimpleNamespace(core=h.gate,runtime=SimpleNamespace(mode='hardware'),
        commands=SimpleNamespace(publish=lambda m:None),
        actuator=SimpleNamespace(publish=lambda m:published.append(m.data)),
        authority_signer=signer,clocks=lambda:(h.now,h.ros))
    AcceptanceGate.publish_output(fake,ZERO)
    policy=TransportPolicy('a'*64,4)
    assert policy.receive(published[-1],h.now,h.ros)
    assert not policy.envelope['permit'] and not policy.active(h.now)


def test_real_ros_codec_and_status_adapter_publish_capture_phase_without_io():
    h=prepared();adjust(h,.6,.4)
    h.modify_frame=lambda frame:decode_frame(json.dumps(asdict(frame),allow_nan=False))
    anchor=enable(h);move_after_enable(h,anchor)
    assert h.output!=ZERO
    statuses=[];candidates=[]
    fake=SimpleNamespace(core=h.sup,clocks=lambda:(h.now,h.ros),
        candidate=SimpleNamespace(publish=lambda m:candidates.append(m.data)),
        status=SimpleNamespace(publish=lambda m:statuses.append(json.loads(m.data))),
        transport_report=None,operator_gate_ready=lambda *a:True)
    SupervisorNode.tick(fake)
    assert statuses[-1]['pilot']['capture_phase']=='CAPTURED'
    assert statuses[-1]['pilot']['capture_started_steady']==h.sup.pilot.started
    assert decode_frame(candidates[-1]).epoch==h.sup.epoch


def test_actual_signed_output_chain_requires_prepare_enable_and_revokes_on_release():
    h=prepared();adjust(h,.6,.4)
    signer=AuthoritySigner('b'*64,4);policy=TransportPolicy('b'*64,4);published=[]
    assert policy.receive(signer.encode(ZERO,h.now,h.ros),h.now,h.ros)
    fake=SimpleNamespace(core=h.gate,runtime=SimpleNamespace(mode='hardware'),
        commands=SimpleNamespace(publish=lambda m:None),
        actuator=SimpleNamespace(publish=lambda m:published.append(m.data)),
        authority_signer=signer,clocks=lambda:(h.now,h.ros))
    def send():
        AcceptanceGate.publish_output(fake,h.output)
        assert policy.receive(published[-1],h.now,h.ros)
    send();assert policy.epoch==h.sup.epoch and not policy.active(h.now)
    anchor=enable(h);send();assert policy.active(h.now) and policy.command==ZERO
    move_after_enable(h,anchor);send();assert policy.active(h.now) and policy.command!=ZERO
    h.held=False;assert h.step()==ZERO
    send();assert not policy.active(h.now) and policy.command==ZERO and not policy.envelope['permit']


@pytest.mark.parametrize('which',['supervisor','gate'])
def test_neither_repeated_arm_nor_begin_can_rebase_a_consumed_observation(which):
    h=prepared();adjust(h,.6,.4);enable(h)
    core=h.sup if which=='supervisor' else h.gate
    m=core.pilot;original=(m.car,m.wand,m.started)
    assert m.arm('replacement-epoch',core.inputs).startswith('PILOT_CONSUMED')
    assert m.begin(m.epoch,core.inputs,h.now,h.ros,h.now,
        car_origin=m.car,wand_origin=replace(m.wand,x=m.wand.x+.1)).startswith('PILOT_CONSUMED')
    assert (m.car,m.wand,m.started)==original


def test_post_enable_wand_height_and_orientation_are_not_xy_motion():
    h=prepared();adjust(h,.6,.4);anchor=enable(h)
    for i in range(1,21):
        h.hand=replace(anchor,z=anchor.z+.5*i/20,yaw=anchor.yaw+.8*i/20)
        assert h.step()==ZERO and h.sup.state=='ACTIVE'
    assert h.sup.follower.mapped_goal==h.sup.follower.mapping.car_origin


def test_original_firmware_arena_mapping_and_thresholds_unchanged():
    c=config()
    assert (c.max_speed_mps,c.max_accel_mps2)==(.70,.80)
    assert (c.arena_forward_max_m,c.arena_left_max_m,c.arena_buffer_m)==(3.,3.,.5)
    assert (POLICY['max_hand_xy_m'],POLICY['max_car_xy_m'],POLICY['max_car_path_m'])==(.15,.20,.25)
    old=ROOT/'docs/wand_servo_4ws8_2026-10-23/ground_pilot_070_080/ui_fix_v2_ready/runtime/src'
    for package,files in [('mecanum_wand_4ws8_control',('safety_core.py','core.py','continuous_servo.py','safety_wire.py')),
                          ('mecanum_wand_4ws8_hardware',('authorization.py','stage0_contract.py','stage0_session.py'))]:
        for file in files:assert (GENERATED/package/package/file).read_bytes()==(old/package/package/file).read_bytes()


def test_historical_prepare_pickup_record_now_enables_neutrally_then_follows_5cm():
    result=replay(GENERATED)
    assert result['recorded_pre_enable_xy_m']>.65 and result['recorded_pre_enable_z_m']>.5
    assert result['enable_state']=='ACTIVE' and result['enable_command_zero']
    assert result['post_enable_5cm_command_nonzero'] and result['supervisor_origin_matches_mapping']
    assert result['mapped_goal_displacement_m']==pytest.approx(.05,abs=1e-8)
    assert result['shared_start_steady'] and not result['hardware_started']


def test_identical_historical_record_still_reproduces_old_v2_hand_limit():
    old=ROOT/'docs/wand_servo_4ws8_2026-10-23/ground_pilot_070_080/ui_fix_v2_ready/runtime/src'
    result=subprocess.run([sys.executable,str(Path(__file__).with_name('replay_trial.py')),'--runtime',str(old)],
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=15,check=True)
    data=json.loads(result.stdout)
    assert data['enable_reason']=='PILOT_HAND_LIMIT' and data['enable_state']=='STOPPING'
    assert data['enable_command_zero'] and not data['hardware_started']
