"""Recorded flip and injected guard/UI regressions; no live ROS or hardware."""
import importlib.util
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from builtin_interfaces.msg import Time
from test_pilot_bounds import config, harness, ROOT, GENERATED
from test_pilot_ui import model
from mecanum_wand_4ws8_control.core import Pose
from mecanum_wand_4ws8_control.safety_core import Inputs, ZERO
from mecanum_wand_4ws8_control.pose_quality import CarPoseQuality, stop_hint
from mecanum_wand_4ws8_control.safety_nodes import SafetyNode, SupervisorNode, decode_pose, pose_message

EVIDENCE=ROOT/'docs/wand_servo_4ws8_2026-10-26/oneclick_v1/ready/oneclick_session_87zu239_'

@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import socket, rclpy
    def denied(*a,**k):pytest.fail('offline only: attempted real socket or ROS init')
    monkeypatch.setattr(socket,'socket',denied)
    monkeypatch.setattr(rclpy,'init',denied)

def stamp(t):
    return Time(sec=int(t),nanosec=round((t-int(t))*1e9))

def sample(c):return Pose(*c.geometry().world(.1,.1),c.ground_z_m,c.arena_heading_rad)

def observe(q, pose, t, roll=0., pitch=0.):
    return q.observe(pose,t,10.+t, t+.001,roll,pitch)

def test_recorded_trial_has_zero_native_faults_and_16_real_pose_guard_violations():
    from mecanum_vicon_control.transport_protocol import unpack_wheel_telemetry
    q=CarPoseQuality(config());states=[]
    for line in (EVIDENCE/'observation.jsonl').open():
        r=json.loads(line);d=r['data']
        if r['topic']=='car_pose':
            msg=pose_message(Pose(0.,0.),stamp(d['stamp']),world='map')
            p,o=msg.pose.position,msg.pose.orientation
            p.x,p.y,p.z=d['x'],d['y'],d['z']
            o.x,o.y,o.z,o.w=(d[k] for k in ('qx','qy','qz','qw'))
            pose,t,roll,pitch=decode_pose(msg,'map')
            q.observe(pose,t,r['received_steady'],t+.001,roll,pitch)
        elif r['topic']=='supervisor_status':states.append(d)
    s=q.snapshot();f=s['first_failure']
    assert s['failures']==16 and f['classification']=='ORIENTATION_FLIP_SUSPECT'
    assert 170.<f['orientation_step_deg']<=180. and f['source_step_s']<.015
    assert abs(f['height_error_m'])<.005
    assert 'Vicon刚体翻转' in q.preflight_issue()
    assert any(d['state']=='ACTIVE' for d in states)
    assert next(d for d in states if d['state']=='STOPPING')['reason']=='CAR_LIFT_OR_TILT'
    for line in (EVIDENCE/'mcu_receive.jsonl').open():
        r=json.loads(line)
        if r['kind']=='WHEEL_TELEMETRY':
            assert unpack_wheel_telemetry(bytes.fromhex(r['payload_hex'])).fault_bits==0

@pytest.mark.parametrize('fault',['flip','real_tilt','lift','timestamp','invalid'])
def test_bad_frame_cannot_be_hidden_by_a_final_good_frame(fault):
    c=config();q=CarPoseQuality(c);p=sample(c)
    for i in range(120):observe(q,p,100.+i*.01)
    if fault=='flip':observe(q,p,101.20,math.pi)
    elif fault=='real_tilt':observe(q,p,101.20,.3)
    elif fault=='lift':observe(q,Pose(p.x,p.y,p.z+.081,p.yaw),101.20)
    elif fault=='timestamp':q.observe(p,101.20,111.20,101.50,0.,0.)
    else:q.observe(Pose(math.nan,p.y,p.z),101.20,111.20,101.201,0.,0.)
    for i in range(1,10):observe(q,p,101.20+i*.01)
    assert q.latest['classification']=='HEALTHY'
    assert q.failures and q.preflight_issue() and q.first_failure

def test_good_preflight_counts_unique_continuous_samples_without_filtering():
    c=config();q=CarPoseQuality(c);p=sample(c)
    for i in range(121):assert observe(q,p,100.+i*.01)
    assert q.preflight_issue()=='' and q.snapshot()['samples']==121
    before=q.snapshot()
    for _ in range(20):assert not observe(q,p,101.20)
    assert q.snapshot()==before

@pytest.mark.parametrize('times',[[100.]*120,[100.+i*.001 for i in range(120)],
    [100.+i*.01 for i in range(60)]+[101.+i*.01 for i in range(60)]])
def test_replayed_short_or_gapped_stream_never_passes_preflight(times):
    c=config();q=CarPoseQuality(c)
    for t in times:observe(q,sample(c),t)
    assert q.preflight_issue()

def test_yaw_wrap_is_not_misclassified_as_a_180_degree_roll_flip():
    c=config();q=CarPoseQuality(c);p=sample(c)
    observe(q,Pose(p.x,p.y,p.z,math.pi-.001),100.)
    observe(q,Pose(p.x,p.y,p.z,-math.pi+.001),100.01)
    assert q.failures==0 and q.latest['orientation_step_deg']<1.

@pytest.mark.parametrize('roll,pitch,dz',[(math.pi,0.,0.),(.30,0.,0.),(0.,.30,0.),(0.,0.,.09)])
def test_actual_ros_pose_callback_retains_unchanged_guard_rejection(roll,pitch,dz):
    c=config();node=object.__new__(SafetyNode)
    node.world='map';node.core=NS(inputs=Inputs(c));node.car_pose_quality=CarPoseQuality(c)
    node.clocks=lambda:(10.,100.001)
    p=sample(c);msg=pose_message(Pose(p.x,p.y,p.z+dz,p.yaw),stamp(100.),roll,pitch,world='map')
    node.on_pose('car',msg)
    assert node.core.inputs.pose_errors['car']=='CAR_LIFT_OR_TILT'
    assert 'car' not in node.core.inputs.poses and node.car_pose_quality.failures==1

def test_sign_vertical_motion_does_not_enter_car_pose_diagnostics_or_tilt_guard():
    c=config();node=object.__new__(SafetyNode)
    node.world='map';node.core=NS(inputs=Inputs(c));node.car_pose_quality=CarPoseQuality(c)
    node.clocks=lambda:(10.,100.001)
    node.on_pose('wand',pose_message(Pose(1.,1.,1.8),stamp(100.),math.pi,.5,world='map'))
    assert 'wand' in node.core.inputs.poses and node.car_pose_quality.samples==0

def test_flip_stops_both_real_guard_algorithms_and_good_return_never_resumes():
    h=harness.Harness(config(),origin_local=(1.5,1.5));h.start()
    for i in range(1,16):h.set_hand(.003*i,0.);h.step()
    assert h.output!=ZERO
    h.roll=math.pi
    assert h.step()==ZERO
    assert h.sup.reason=='CAR_LIFT_OR_TILT'
    assert h.gate.output==ZERO and h.sup.fault_latched
    h.roll=0.;h.run(.6)
    assert h.output==ZERO and h.sup.state!='ACTIVE'
    assert not h.sup.request_start(h.now,h.ros)[0]

def test_consumed_trial_shows_flip_reason_without_offering_recovery_or_enable():
    m=model('FAULT',True)
    m.status.update(reason='CAR_LIFT_OR_TILT',first_stop_reason='CAR_LIFT_OR_TILT',
        car_pose_quality=dict(first_failure=dict(classification='ORIENTATION_FLIP_SUSPECT',
            orientation_step_deg=179.,source_step_s=.009)))
    text=m.instruction(10.)
    assert '179.0°' in text and 'Vicon刚体翻转' in text and '不能再次准备' in text
    assert '自动恢复' in text and not m.can_start(10.) and not m.can_recover(10.)
    m.status['reason']='operator stop'
    assert 'Vicon刚体翻转' in m.instruction(10.)
    m.status['hard_fault']=True
    assert '保护已锁存' in m.instruction(10.) and '不能通过确认' in m.instruction(10.)

def test_real_supervisor_status_keeps_first_stop_reason_after_later_stop_click():
    from mecanum_wand_4ws8_control.pilot_core import PilotSupervisor
    node=object.__new__(SupervisorNode);node.core=PilotSupervisor(config())
    node.car_pose_quality=CarPoseQuality(config());node.first_stop_reason='';node.transport_report=None
    node.candidate=NS(publish=lambda m:None);messages=[]
    node.status=NS(publish=lambda m:messages.append(json.loads(m.data)))
    node.clocks=lambda:(10.,100.)
    node.core.state='FAULT';node.core.reason='CAR_LIFT_OR_TILT';node.core.fault_latched=True
    node.tick();node.core.reason='operator stop';node.clocks=lambda:(10.02,100.02);node.tick()
    assert messages[-1]['first_stop_reason']=='CAR_LIFT_OR_TILT'
    assert messages[-1]['reason']=='operator stop' and 'car_pose_quality' in messages[-1]

@pytest.mark.parametrize('flip', [False,True])
def test_actual_readonly_preflight_rejects_one_bad_frame_even_when_last_frame_good(flip,monkeypatch,capsys):
    import rclpy
    path=Path(os.environ['ONECLICK_SOURCE'])/'pose_check.py'
    spec=importlib.util.spec_from_file_location('quality_preflight',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    c=config();p=sample(c);callbacks={};clock=NS(now=0.,step=0)
    node=NS(create_subscription=lambda ty,topic,cb,q:callbacks.update({topic:cb}),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=round((100.+clock.now)*1e9))),
        get_publishers_info_by_topic=lambda topic:[NS()],destroy_node=lambda:None)
    def spin(node,timeout_sec):
        clock.now+=.02;clock.step+=1
        for key in ('Sign','car01'):
            pose=Pose(p.x,p.y,p.z,c.arena_heading_rad-2.356194490192345) if key=='car01' else Pose(1.,1.,1.)
            msg=pose_message(pose,stamp(100.+clock.now),math.pi if flip and key=='car01' and clock.step==40 else 0.,world='map')
            callbacks[f'/vicon/{key}/{key}'](msg)
    monkeypatch.setattr(rclpy,'init',lambda:None)
    monkeypatch.setattr(rclpy,'create_node',lambda *a,**k:node)
    monkeypatch.setattr(rclpy,'spin_once',spin)
    monkeypatch.setattr(rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:clock.now))
    module.main();report=json.loads(capsys.readouterr().out)
    assert report['ready'] is not flip
    assert report['car_pose_quality']['latest']['classification']=='HEALTHY'
    assert bool(report['car_pose_quality']['failures']) is flip
    if flip:assert any('Vicon刚体翻转' in s for s in report['issues'])

def test_safety_algorithms_and_firmware_boundary_are_byte_identical():
    old=ROOT/'docs/wand_servo_4ws8_2026-10-26/oneclick_v1/ready/runtime/src'
    c='mecanum_wand_4ws8_control';h='mecanum_wand_4ws8_hardware'
    for name in ('core.py','continuous_servo.py','safety_core.py','pilot_core.py','demo_policy.py','staged_config.py'):
        assert (GENERATED/c/c/name).read_bytes()==(old/c/c/name).read_bytes()
    for name in ('nodes.py','stage0_contract.py','stage0_session.py','authorization.py'):
        assert (GENERATED/h/h/name).read_bytes()==(old/h/h/name).read_bytes()
    assert config().tilt_limit_rad==.25 and config().lift_tolerance_m==.08
