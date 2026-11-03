"""Replay the real failed boot and exercise unchanged lease/safety cores."""
import copy
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from support import ROOT,GENERATED,config
from test_toggle import LeaseDriver,model
from test_autoprep import standby,fresh,poll
from mecanum_wand_4ws8_control import auto_prepare as ap
from mecanum_wand_4ws8_control.safety_core import ZERO

TRACE=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_autoprep/ready_v4/oneclick_session_9rsmnc5n/observation.jsonl'


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import rclpy,socket,subprocess
    def denied(*a,**kw):pytest.fail('offline startup test attempted hardware/ROS/process IO')
    monkeypatch.setattr(rclpy,'init',denied);monkeypatch.setattr(socket,'socket',denied)
    monkeypatch.setattr(subprocess,'Popen',denied)


def boot_model():
    m=standby();m.status.update(graph_ok=True,epoch='',first_stop_reason='')
    m.gate.update(fault='COMMAND_TIMEOUT',output_zero=True,speed_mps=0.,primed_epoch='',graph_ok=True)
    m.transport_status.update(ready=True,baseline_current=True,command_progress_age_s=.01)
    m.transport_status['startup_clock']['qualified']=True
    return m


def test_boot_timeout_waits_then_requests_only_disabled_prepare_once():
    m=boot_model();p=ap.AutoPrepareOnce()
    m.transport_status['fresh']=False
    assert not poll(p,m,10.) and not p.cancelled and p.phase=='WAITING_ZERO_BASELINE'
    m.transport_status['fresh']=True
    assert not poll(p,m,10.1)
    assert poll(p,m,10.4) and p.attempted
    assert not m.held and not m.activation_used and not m.ended
    assert m.gate['fault']=='COMMAND_TIMEOUT' # Never silently cleared in UI.
    for t in (10.42,10.5):assert not poll(p,m,t) and not p.cancelled
    m.status.update(state='ARMED',epoch='fresh-epoch',gate_ready=False)
    assert not poll(p,m,10.52) and not p.cancelled
    m.gate.update(fault='',primed_epoch='fresh-epoch');m.status['gate_ready']=True
    assert not poll(p,m,10.54) and p.phase=='READY'
    m.gate['fault']='COMMAND_TIMEOUT'
    assert not poll(p,m,10.56) and p.cancelled and not m.held


@pytest.mark.parametrize('bad',['gate_unknown','nonzero','gate_speed','primed','previous_stop','epoch',
    'hard','armed_without_request','active','used','ended','activation','safety_enabled','motion_requested'])
def test_only_virgin_observed_zero_timeout_is_eligible(bad):
    m=boot_model();p=ap.AutoPrepareOnce()
    if bad=='gate_unknown':m.gate['fault']='COMMAND_OWNER_CHANGED'
    elif bad=='nonzero':m.gate['output_zero']=False
    elif bad=='gate_speed':m.gate['speed_mps']=.01
    elif bad=='primed':m.gate['primed_epoch']='old'
    elif bad=='previous_stop':m.status['first_stop_reason']='old fault'
    elif bad=='epoch':m.status['epoch']='old'
    elif bad=='hard':m.gate['hard_fault']=True
    elif bad=='armed_without_request':m.status['state']='ARMED'
    elif bad=='active':m.status['state']='ACTIVE'
    elif bad=='used':m.pilot_used=True
    elif bad=='ended':m.ended=True
    elif bad=='activation':m.activation_used=True
    elif bad=='safety_enabled':m.transport_status['safety_enabled']=True
    else:m.transport_status['motion_gate_requested']=True
    assert not poll(p,m,10.) and not p.attempted and not m.held
    if bad!='activation':assert p.cancelled


@pytest.mark.parametrize('bad',['sup_graph','gate_graph','zero_unknown','clock_unqualified','baseline_false',
    'no_progress','stale_progress','nan_progress','inf_progress','future_progress','not_ready',
    'feedback_not_ready','pose_fault','unfocused','held_key','input_issue','service'])
def test_boot_does_not_prepare_until_complete_fresh_zero_baseline(bad):
    m=boot_model();p=ap.AutoPrepareOnce();kw={}
    if bad=='sup_graph':m.status['graph_ok']=False
    elif bad=='gate_graph':m.gate['graph_ok']=False
    elif bad=='zero_unknown':m.transport_status.pop('safety_enabled')
    elif bad=='clock_unqualified':m.transport_status['startup_clock']['qualified']=False
    elif bad=='baseline_false':m.transport_status['baseline_current']=False
    elif bad=='no_progress':m.transport_status.pop('command_progress_age_s')
    elif bad=='stale_progress':m.transport_status['command_progress_age_s']=.101
    elif bad=='nan_progress':m.transport_status['command_progress_age_s']=math.nan
    elif bad=='inf_progress':m.transport_status['command_progress_age_s']=math.inf
    elif bad=='future_progress':m.transport_status['command_progress_age_s']=-.001
    elif bad=='not_ready':m.status['ready_to_start']=False
    elif bad=='feedback_not_ready':m.transport_status['ready']=False
    elif bad=='pose_fault':m.status['input_health']='WAND_POSE_TIMEOUT'
    elif bad=='unfocused':m.focused=False
    elif bad=='held_key':kw['space_down']=True
    elif bad=='input_issue':kw['input_issue']='duplicate source'
    else:kw['service_ready']=False
    for t in (10.,10.5):assert not poll(p,m,t,**kw)
    assert not p.cancelled and not p.attempted and not m.held


def test_boot_wait_and_prepare_handoff_both_bounded_no_retry():
    m=boot_model();p=ap.AutoPrepareOnce();m.transport_status['fresh']=False
    assert not poll(p,m,10.)
    assert not poll(p,m,40.01) and p.cancelled and not p.attempted
    m.transport_status['fresh']=True;assert not poll(p,m,40.03)
    m=boot_model();p=ap.AutoPrepareOnce();poll(p,m,10.);assert poll(p,m,10.3)
    assert not poll(p,m,15.31) and p.cancelled
    m.gate['fault']='';m.status.update(state='ARMED',gate_ready=True)
    assert not poll(p,m,15.32) and not m.held


def test_partial_baseline_instability_restarts_full_300ms():
    m=boot_model();p=ap.AutoPrepareOnce();assert not poll(p,m,10.)
    m.transport_status['baseline_current']=False;assert not poll(p,m,10.25)
    m.transport_status['baseline_current']=True
    assert not poll(p,m,10.27) and not poll(p,m,10.56)
    assert poll(p,m,10.57)


def test_original_failed_live_trace_reaches_one_prepare_without_ending_trial(monkeypatch):
    rows=[json.loads(line) for line in TRACE.read_text().splitlines()]
    first=rows[0]['received_steady'];m=model('STANDBY',10.,False)
    m.status={};m.gate={};m.transport_status={};m.transport_at=-math.inf
    p=ap.AutoPrepareOnce();clock=NS(now=10.)
    monkeypatch.setattr(ap,'time',NS(monotonic=lambda:clock.now))
    class Base:
        def loop(self):m.tick(clock.now)
    class Console(ap.AutoPrepareConsoleMixin,Base):pass
    c=object.__new__(Console);c.closing=False;c.model=m;c.auto_prepare=p;c.auto_prepare_stop_sent=False
    c.server_space_down=lambda:False;calls=[]
    c.node=NS(service_clients={'start':NS(service_is_ready=lambda:True)},pending=set(),
        input_hint=lambda:m.status.get('input_health',''),
        request=lambda action:(calls.append((clock.now,action)) or True),heartbeat=lambda:None)
    index=0
    while clock.now<19.:
        while index<len(rows) and rows[index]['received_steady']-first<=clock.now-10.:
            r=rows[index];at=10.+r['received_steady']-first;data=copy.deepcopy(r['data'])
            if r['topic']=='supervisor_status':m.receive_status(data,at)
            elif r['topic']=='gate_status':m.receive_gate(data,at)
            elif r['topic']=='transport_status':m.transport_status=data;m.transport_at=at
            index+=1
        c.loop()
        if calls:break
        clock.now+=.02
    assert len(calls)==1 and calls[0][1]=='start'
    assert calls[0][0]>14.3 # Cannot prepare before recorded readiness.
    assert not p.cancelled and not m.ended and not m.held and not m.activation_used
    assert m.gate['fault']=='COMMAND_TIMEOUT'


def test_actual_lease_and_gate_boot_latch_clears_only_on_fresh_disabled_epoch():
    d=LeaseDriver();d.h.gate.trip('COMMAND_TIMEOUT');original_sync=d.sync
    def sync():
        original_sync();h=d.h
        d.m.status.update(graph_ok=True,epoch=h.sup.epoch,first_stop_reason='')
        d.m.gate.update(graph_ok=True,output_zero=h.gate.output==ZERO,speed_mps=0.,primed_epoch=h.gate.primed_epoch)
        d.m.transport_status=copy.deepcopy(boot_model().transport_status);d.m.transport_at=h.now
    d.sync=sync;d.run(1.8);p=ap.AutoPrepareOnce();requests=[]
    for _ in range(40):
        if p.poll(d.m,d.h.now,space_down=False,service_ready=True):
            requests.append(d.h.now);assert d.h.sup.request_start(d.h.now,d.h.ros)[0]
        assert d.step()==ZERO
    assert len(requests)==1 and p.phase=='READY' and not d.h.gate.fault
    assert not d.m.held and not d.h.sup.pilot.used
    # Adjusting Sign while ARMED must remain zero. A new human click captures
    # the current origins; no automatic motion and no preloaded old target.
    from dataclasses import replace
    for i in range(1,41):
        d.h.hand=replace(d.h.hand_origin,x=d.h.hand_origin.x+i*.003,z=1.+i*.004)
        assert d.step()==ZERO
    assert d.tap();assert d.step()==ZERO;assert d.h.sup.state=='ACTIVE'
    for _ in range(20):d.h.hand=replace(d.h.hand,x=d.h.hand.x+.004);d.step()
    assert d.h.output!=ZERO
    d.m.end_trial(d.h.now,'offline Esc')
    assert d.step()==ZERO
    assert [e['event'] for e in d.m.events].count('automatic_prepare_request_only_no_enable')==1
    assert not d.m.held


def test_only_auto_prepare_module_changed_in_runtime():
    old=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_autoprep/ready_v4/runtime/src'
    changed=[]
    for p in old.rglob('*'):
        if not p.is_file() or any(k=='__pycache__' or k.endswith('.egg-info') for k in p.parts):continue
        q=GENERATED/p.relative_to(old)
        if q.read_bytes()!=p.read_bytes():changed.append(str(p.relative_to(old)))
    assert changed==['mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/auto_prepare.py']
