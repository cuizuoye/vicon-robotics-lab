"""Actual single-click model/adapter/lease chain, virtual clocks and fake I/O."""
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from support import config,harness,GENERATED,ROOT,model as held_model
from test_demo_120s import approved_profile
from mecanum_wand_4ws8_control.toggle_ui import ToggleOperatorModel
from mecanum_wand_4ws8_control.demo_policy import INPUT_MODE,POLICY
from mecanum_wand_4ws8_control import operator_console as ui
from mecanum_wand_4ws8_control.safety_core import Lease,ZERO
from mecanum_wand_4ws8_control.safety_wire import canonical
from mecanum_wand_4ws8_hardware.commissioning import AuthoritySigner,TransportPolicy
from mecanum_wand_4ws8_hardware.authorization import verify_runtime_profile


@pytest.fixture(autouse=True)
def no_real_io(monkeypatch):
    import socket,rclpy
    def forbidden(*a,**k):pytest.fail('offline toggle tests: real socket/ROS init forbidden')
    monkeypatch.setattr(socket,'socket',forbidden);monkeypatch.setattr(rclpy,'init',forbidden)

def model(state='ARMED',at=10.,ready=True):
    m=ToggleOperatorModel();m.focused=True
    old=held_model(state)
    s=dict(old.status,gate_ready=ready,input_health='')
    s['pilot']=dict(s['pilot'],operator_input_mode=INPUT_MODE,capture_started_steady=None)
    m.receive_status(s,at);m.receive_gate(dict(fault='',hard_fault=False),at)
    return m

def click(m,t=10.):
    m.tick(t);m.press(t)
    return m.release(t+.03)

def active(m,t=10.04):
    m.receive_status(dict(m.status,state='ACTIVE',pilot=dict(m.status['pilot'],used=True,
        capture_started_steady=t,remaining_s=120.)),t)
    m.receive_gate(dict(fault='',hard_fault=False),t)

def test_complete_tap_starts_once_with_physical_key_up_not_a_synthetic_hold():
    m=model();m.tick(10.)
    assert not m.press(10.) and m.click_pending and m.key_down and not m.session_requested
    assert m.release(10.03) and not m.key_down and m.session_requested
    active(m);m.tick(10.04)
    m.release(10.05);m.observe_physical_release(10.06)
    assert m.session_requested and not m.key_down and m.activation_used and not m.ended
    assert '松键继续' in m.instruction(10.06)

def test_second_space_stops_on_key_down_without_waiting_for_key_up():
    m=model();assert click(m);active(m)
    m.press(10.06)
    assert not m.session_requested and m.key_down and m.ended
    m.release(10.07);m.press(10.08);m.release(10.09)
    assert not m.session_requested and not m.can_start(10.09) and not m.can_recover(10.09)

@pytest.mark.parametrize('condition',['not_ready','standby','stale','unfocused','fault','hard_fault','input_health','wrong_mode','used'])
def test_press_when_not_ready_cannot_become_a_delayed_start(condition):
    m=model()
    if condition=='not_ready':m.status['gate_ready']=False
    elif condition=='standby':m.status['state']='STANDBY'
    elif condition=='stale':m.status_at=9.
    elif condition=='unfocused':m.focused=False
    elif condition=='fault':m.gate['fault']='FAULT'
    elif condition=='hard_fault':m.gate['hard_fault']=True
    elif condition=='input_health':m.status['input_health']='WAND_POSE_TIMEOUT'
    elif condition=='wrong_mode':m.status['pilot'].pop('operator_input_mode')
    else:m.pilot_used=True
    m.press(10.)
    m.status['gate_ready']=True;m.status['state']='ARMED';m.status_at=10.01;m.focused=True
    m.gate.update(fault='',hard_fault=False);m.status['input_health']=''
    m.status['pilot']['operator_input_mode']=INPUT_MODE;m.pilot_used=False
    assert not m.release(10.03) and not m.session_requested

@pytest.mark.parametrize('fault',['gate','hard','pose','freshness','focus','closing'])
def test_condition_changing_between_press_and_release_cancels_click(fault):
    m=model();m.press(10.);assert m.click_pending
    if fault=='gate':m.gate['fault']='FAULT'
    elif fault=='hard':m.status['hard_fault']=True
    elif fault=='pose':m.status['input_health']='CAR_LIFT_OR_TILT'
    elif fault=='freshness':m.status_at=9.
    elif fault=='focus':m.focus(False,10.01)
    else:m.closing=True
    assert not m.release(10.03) and not m.session_requested

def test_polling_a_missing_key_release_never_synthesizes_a_start():
    m=model();m.press(10.)
    m.observe_physical_release(10.02)
    assert not m.key_down and not m.click_pending and not m.session_requested
    assert not m.release(10.03)  # A late real release cannot complete a cancelled click.

def test_long_press_and_autorepeat_do_not_toggle_or_start_on_expired_release():
    m=model();m.press(10.)
    for i in range(1,31):
        t=10.+i*.005;m.press(t)
        assert m.click_pending and not m.session_requested
    assert m.release(10.16) and m.session_requested
    for i in range(10):
        # A repeated release without a new accepted press cannot toggle.
        m.release(10.17+i*.001)
    assert m.session_requested
    m=model();m.press(10.)
    m.status_at=m.gate_at=11.01
    assert not m.release(11.01) and not m.session_requested

@pytest.mark.parametrize('reason',['focus','ui_gap','backend_stale','backend_paused','gate_fault','hard_fault','pose_fault','close'])
def test_active_latch_revokes_on_existing_guards_and_never_resumes(reason):
    m=model();assert click(m);active(m);m.tick(10.04)
    if reason=='focus':m.focus(False,10.05)
    elif reason=='ui_gap':m.tick(10.15)
    elif reason=='backend_stale':m.status_at=9.
    elif reason=='backend_paused':m.status['state']='PAUSED'
    elif reason=='gate_fault':m.gate['fault']='CAR_LIFT_OR_TILT'
    elif reason=='hard_fault':m.status['hard_fault']=True
    elif reason=='pose_fault':m.status['input_health']='WAND_POSE_TIMEOUT'
    else:m.closing=True
    m.tick(10.16)
    assert not m.session_requested and m.ended
    m.closing=False;m.focused=True;active(m,10.18);m.gate.update(fault='',hard_fault=False)
    m.status.update(hard_fault=False,input_health='');m.release(10.19);m.press(10.20);m.release(10.21)
    assert not m.session_requested

def test_activation_without_active_ack_expires_without_late_motion():
    m=model();assert click(m)
    for i in range(1,60):
        t=10.03+i*.02
        m.receive_status(dict(m.status),t);m.receive_gate(dict(fault='',hard_fault=False),t);m.tick(t)
    assert not m.session_requested and m.ended
    active(m,11.24);m.press(11.25);m.release(11.26)
    assert not m.session_requested

def adapter(monkeypatch,m=None):
    m=m or model();clock=NS(now=10.);down=NS(value=True)
    monkeypatch.setattr(ui.time,'monotonic',lambda:clock.now)
    from mecanum_wand_4ws8_control import toggle_keys
    monkeypatch.setattr(toggle_keys,'time',NS(monotonic=lambda:clock.now))
    events=[];c=object.__new__(ui.Console)
    c.model=m;c.closing=False;c.release_job=None;c.loop_job=None;c.last_draw=10.
    c.root=NS(after=lambda *a:1,after_cancel=lambda *a:None)
    c.key_state=NS(down=lambda:down.value)
    c.node=NS(heartbeat=lambda:events.append(('lease',m.session_requested)),
        request=lambda action:events.append(('service',action)),pump=lambda:events.append(('pump',m.session_requested)))
    c.draw=lambda now:None
    return c,clock,down,events

def test_actual_adapter_repeat_pairs_start_once_and_stop_second_press(monkeypatch):
    c,t,down,e=adapter(monkeypatch)
    c.press();assert not c.model.session_requested
    for _ in range(15):
        t.now+=.01;c.release();c.press()  # Physical down: synthetic release ignored.
    assert c.model.key_down and c.model.click_pending and not c.model.session_requested
    down.value=False;t.now+=.01;c.release()
    assert c.model.session_requested and not c.model.key_down
    down.value=True;t.now+=.01;c.press()
    assert not c.model.session_requested and e[-1]==('service','stop')
    assert e[-2]==('lease',False)

def test_actual_adapter_escape_stops_even_with_stale_feedback_and_no_key_edge(monkeypatch):
    m=model();assert click(m);active(m);m.status_at=1.
    c,t,down,e=adapter(monkeypatch,m);c.escape()
    assert not m.session_requested and m.ended
    assert e[:2]==[('lease',False),('service','stop')]

def test_actual_ui_loop_can_only_cancel_click_when_key_release_event_missing(monkeypatch):
    c,t,down,e=adapter(monkeypatch);c.press()
    down.value=False;t.now+=.02;c.loop()
    assert not c.model.session_requested and not c.model.click_pending
    c.release();assert not c.model.session_requested

def test_stop_button_releases_before_waiting_for_an_unavailable_stop_service():
    m=model();assert click(m);active(m)
    node=object.__new__(ui.OperatorNode);node.model=m;node.pending=set()
    released=[];node.heartbeat=lambda:released.append(m.session_requested)
    node.service_clients={'stop':NS(service_is_ready=lambda:False)}
    assert not node.request('stop')  # Service not ready is NOT a reason to keep enabling.
    assert released==[False] and m.ended and not m.session_requested

def test_keyboard_state_read_failure_in_real_adapter_revokes_and_closes(monkeypatch):
    m=model();assert click(m);active(m)
    c,t,down,e=adapter(monkeypatch,m)
    def broken():raise OSError('offline injected X keyboard read failure')
    c.key_state=NS(down=broken)
    c.node.get_logger=lambda:NS(error=lambda message:e.append(('error',message)))
    def close():
        c.closing=True;m.end_trial(t.now,'键盘读取失败关闭');c.node.heartbeat()
    c.close=close
    c.loop()
    assert c.closing and m.ended and not m.session_requested
    assert ('lease',False) in e and not any(k=='pump' for k,v in e)

def test_queued_tap_after_ui_stall_is_rejected_by_real_adapter(monkeypatch):
    c,t,down,e=adapter(monkeypatch);c.model.tick(9.8)
    c.press();down.value=False;t.now+=.01;c.release()
    assert not c.model.session_requested
    assert all(not value for kind,value in e if kind=='lease')

class LeaseDriver:
    """Runs the actual OperatorNode.heartbeat codec into BOTH real safety cores."""
    def __init__(self):
        self.h=harness.Harness(config(),origin_local=(0.,0.));self.h.lease_on=False
        self.m=model('STANDBY',self.h.now,False);self.sent=[]
        self.node=object.__new__(ui.OperatorNode)
        self.node.model=self.m;self.node.owner='offline-toggle-human';self.node.sequence=0
        self.node.hand_pub=None;self.node.last_ui_diagnostic=None
        self.node.clocks=lambda:(self.h.now,self.h.ros)
        self.node.get_logger=lambda:NS(info=lambda text:None)
        self.node.lease_pub=NS(publish=self.receive)
        self.sync()
    def receive(self,msg):
        lease=Lease(**json.loads(msg.data));self.sent.append(lease)
        for inputs in (self.h.sup.inputs,self.h.gate.inputs):
            inputs.observe_lease(lease,self.h.now,self.h.ros)
    def sync(self):
        h=self.h
        self.m.receive_status(dict(state=h.sup.state,reason=h.sup.reason,hard_fault=h.sup.hard_fault,
            input_health=h.sup.inputs.health(h.now,h.ros),stop_confirmed=h.sup.stop_confirmed,
            ready_to_start=h.sup.ready_since is not None,gate_ready=bool(h.sup.epoch and
                h.sup.epoch==h.gate.primed_epoch and not h.gate.fault),pilot=h.sup.pilot_status(h.now)),h.now)
        self.m.receive_gate(dict(fault=h.gate.fault,hard_fault=h.gate.hard_fault),h.now)
    def step(self):
        self.node.heartbeat();out=self.h.step();self.sync();return out
    def run(self,seconds):
        for _ in range(round(seconds/.02)):self.step()
    def prepare(self):
        self.run(1.8);assert self.h.sup.request_start(self.h.now,self.h.ros)[0];self.run(.1)
        assert self.m.start_ready(self.h.now)
    def tap(self):
        self.m.press(self.h.now);return self.m.release(self.h.now)

def test_real_heartbeat_releases_physical_key_but_renews_logical_lease_and120s_timer():
    d=LeaseDriver();d.prepare();d.run(23.)
    assert not d.h.sup.pilot.used and not d.m.session_requested
    assert d.tap() and d.step()==ZERO and d.h.sup.state=='ACTIVE'
    t0=d.h.sup.pilot.started
    assert t0==d.h.gate.pilot.started and not d.m.key_down
    d.run(20.2);assert d.h.sup.state=='ACTIVE' and d.sent[-1].held and not d.m.key_down
    d.run(99.9);d.step()
    assert d.h.output==ZERO and not d.m.session_requested and d.m.ended
    assert d.h.now-t0>=120.
    assert not d.tap() and not d.m.can_start(d.h.now)

def test_second_tap_revokes_the_real_lease_chain_while_moving_and_never_restarts():
    d=LeaseDriver();d.prepare();assert d.tap();d.step()
    for i in range(1,16):d.h.set_hand(.003*i,0.);d.step()
    assert d.h.output!=ZERO
    d.m.press(d.h.now);assert not d.m.session_requested and d.step()==ZERO
    assert not d.sent[-1].held and d.h.sup.state!='ACTIVE'
    d.m.release(d.h.now);d.run(2.)
    assert not d.tap() and d.h.sup.stop_confirmed and d.h.output==ZERO

def test_pickup_and_adjustment_before_single_tap_still_capture_neutral_origins():
    d=LeaseDriver();d.prepare()
    from dataclasses import replace
    for i in range(1,61):
        d.h.hand=replace(d.h.hand_origin,x=d.h.hand_origin.x+.6*i/60,
            y=d.h.hand_origin.y-.4*i/60,z=1.+.6*i/60)
        assert d.step()==ZERO and d.h.sup.state=='ARMED'
    assert d.tap() and d.step()==ZERO
    assert d.h.sup.follower.mapping.wand_origin==d.h.hand
    assert d.h.sup.pilot.started==d.h.gate.pilot.started
    d.run(.4);assert d.h.output==ZERO and d.h.sup.state=='ACTIVE'

def test_stop_between_enable_lease_and_first_active_frame_cannot_start_later():
    d=LeaseDriver();d.prepare();assert d.tap()
    d.node.heartbeat()  # True lease delivered; neither core has ticked ACTIVE.
    assert d.sent[-1].held and d.h.sup.state=='ARMED'
    d.m.end_trial(d.h.now,'Esc before activation acknowledgement')
    d.node.heartbeat();assert not d.sent[-1].held
    assert d.step()==ZERO and d.h.sup.state!='ACTIVE'
    d.run(.6);assert not d.tap() and d.h.output==ZERO

@pytest.mark.parametrize('fault',['occlusion','car_loss','flip','feedback','lease','focus','esc','hard'])
def test_actual_toggle_lease_chain_keeps_existing_fault_stops(fault):
    d=LeaseDriver();d.prepare();d.tap();d.step()
    for i in range(1,16):d.h.set_hand(.003*i,0.);d.step()
    assert d.h.output!=ZERO
    if fault=='occlusion':d.h.pose_on['wand']=False
    elif fault=='car_loss':d.h.pose_on['car']=False
    elif fault=='flip':d.h.roll=math.pi
    elif fault=='feedback':d.h.feedback_on=False
    elif fault=='lease':d.node.lease_pub=NS(publish=lambda m:None)
    elif fault=='focus':d.m.focus(False,d.h.now)
    elif fault=='esc':d.m.end_trial(d.h.now,'Esc停止')
    else:d.h.fault_bits=0x20
    d.run(.5)
    assert d.h.output==ZERO and not d.m.session_requested and d.m.ended
    d.h.pose_on={'car':True,'wand':True};d.h.roll=0.;d.h.feedback_on=True;d.m.focused=True
    d.run(.5);assert not d.tap() and d.h.output==ZERO

@pytest.mark.parametrize('change',['missing_mode','held_mode','no_ack','held_review'])
def test_explicit_mode_and_current_human_ack_required(change):
    p,m=approved_profile()
    if change=='missing_mode':p.pop('operator_input_mode')
    elif change=='held_mode':p['operator_input_mode']='SPACE_HOLD'
    elif change=='no_ack':p['staged_runtime_approval'].pop('single_click_mode_acknowledged_now')
    else:p['trial_review']['kind']='BOUNDED_OFFSET_ARENA_120S_NOT_QUALIFICATION'
    with pytest.raises(ValueError):verify_runtime_profile(p,'sign')

def test_correctly_signed_held_mode_scope_is_rejected_in_toggle_mode():
    signer=AuthoritySigner('a'*64,4);env=json.loads(signer.encode(ZERO,10.,100.))
    env['payload']['scope']='BOUNDED_SIGNED_ARENA_120S_4WS8_V4_STAGE_4'
    env['signature']=hmac.new(b'\xaa'*32,canonical(env['payload']),hashlib.sha256).hexdigest()
    p=TransportPolicy('a'*64,4)
    assert not p.receive(json.dumps(env),10.,100.) and p.hard_fault

def test_real_console_model_and_stop_bindings_are_dedicated_and_widgets_are_consistent(monkeypatch):
    import tkinter as tk
    import time
    m=model('STANDBY',time.monotonic(),False);now=time.monotonic()
    m.status_at=m.gate_at=now
    keys=NS(down=lambda:False,close=lambda:None)
    monkeypatch.setattr(ui,'X11KeyState',lambda *a:keys)
    from mecanum_wand_4ws8_control.core import Pose
    c=config();events=[]
    fake=NS(runtime=NS(config=c,wand_reference_margin_m=.5),input_kind='hardware',trace=[],
        car=Pose(*c.geometry().world(0.,0.),c.ground_z_m,c.arena_heading_rad),raw_wand=Pose(1.,1.,1.),
        request=lambda name:events.append(name),input_hint=lambda:'')
    root=tk.Tk();root.withdraw();console=ui.Console(root,fake,m)
    try:
        console.draw(now);root.update_idletasks()
        assert ui.OperatorModel is ToggleOperatorModel and '单击' in root.title()
        assert str(console.buttons['start']['state'])=='normal' and not m.session_requested
        assert 'SafeWandKeys'==console.buttons['start'].bindtags()[0]
        assert root.bind_class('SafeWandKeys','<Escape>')
        assert '单击模式' in console.step_text.get()
        assert not events
    finally:
        for name in ('loop_job','focus_job','release_job','shutdown_job','destroy_job'):
            job=getattr(console,name,None)
            if job is not None:root.after_cancel(job)
        root.destroy()

def test_core_firmware_and_all_short_safety_guards_are_byte_identical_to_held_version():
    old=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_offset/ready/runtime/src'
    c='mecanum_wand_4ws8_control';h='mecanum_wand_4ws8_hardware'
    for name in ('core.py','continuous_servo.py','safety_core.py','safety_nodes.py','pilot_core.py',
                 'operator_model.py','pilot_ui.py','arena_config.py','arena_geometry.py','pose_quality.py'):
        assert (GENERATED/c/c/name).read_bytes()==(old/c/c/name).read_bytes()
    for name in ('nodes.py','continuous_bridge.py','stage0_contract.py','stage0_session.py'):
        assert (GENERATED/h/h/name).read_bytes()==(old/h/h/name).read_bytes()
