"""Reuse timeout regressions; add real native-C retained-budget startup."""
import importlib.util
import json
import os
from pathlib import Path
from collections import deque
from types import SimpleNamespace as NS
import pytest

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');OUT=Path(os.environ['FEEDBACK_OUTPUT'])
spec=importlib.util.spec_from_file_location('timeout_parent_tests',ROOT/'candidates/wand_feedback_300ms/test_timeout.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
offline_only=base.offline_only
for name in dir(base):
    if name.startswith('test_') and name not in ('test_all_parents_untouched_and_exact_change_inventory',
                                               'test_old_instance_cannot_be_silently_reused'):
        globals()['test_reused_'+name[5:]]=getattr(base,name)


def test_actual_native_retained_used_budget_prepares_without_power_cycle_or_motion(monkeypatch):
    p=base.parent;native=base.native
    old=p.armed(monkeypatch)
    from mecanum_wand_4ws8_control.core import Velocity
    for _ in range(20):old.signed('ACTIVE','regression-epoch',True,Velocity(.08,0.,0.));old.step()
    assert old.b.last_motion_status['started_ms']>0
    old.signed();old.run(.7)
    assert not old.b.last_motion_status['permitted']
    old.socket.oracle.tick(old.socket.ms()+10)
    # New host process, same running physical MCU C state and MCU time. The
    # fixture does NOT oracle_init a second time; no production reset is faked.
    monkeypatch.setattr(native,'Oracle',lambda:old.socket.oracle)
    h=p.actual(monkeypatch);h.clock.now=old.clock.now+.02
    h.socket.next_sample=(h.socket.ms()//100+1)*100
    h.signed();h.run(1.2)
    s=h.status()
    assert s['startup_clock']['ready'] and s['startup_reset_ready'] and not s['ready'],(s,h.logs)
    assert not h.b.user_arm_seen and h.b.last_motion_status['started_ms']>0
    from mecanum_vicon_control.transport_protocol import MessageType as M,FrameFlags as F
    assert not any(f.message_type==M.CONTROL_ENABLE and f.payload[1] for _,f,_ in h.socket.sent)
    for _ in range(130):h.signed('ARMED','new-human-preparation',True);h.step()
    assert h.status()['ready'] and h.b.last_motion_status['started_ms']==0,(h.status(),h.logs)
    assert h.b.last_motion_status['selected'] and not h.b.last_motion_status['permitted']
    assert not any(f.flags&F.ENABLED for _,f,_ in h.socket.sent)
    assert any(f.message_type==M.CONTROL_ENABLE and f.payload==b'\0\1\0\0' for _,f,_ in h.socket.sent)


@pytest.mark.parametrize('mutation',['native_fault','watchdog','feedback_reason','arm_seen','frozen','permitted','nonzero'])
def test_startup_reset_evidence_never_granted_for_fault_or_arm(monkeypatch,mutation):
    h=base.parent.actual(monkeypatch);h.run(1.2)
    h.b.last_motion_status['started_ms']=123  # Change diagnostic fixture only.
    if mutation=='native_fault':h.b.last_mcu_fault_bits=0x20
    elif mutation=='watchdog':h.b.last_motion_status['watchdog_reset']=1
    elif mutation=='feedback_reason':h.b.last_motion_status['first_feedback_fault']['reason']=1
    elif mutation=='arm_seen':h.b.user_arm_seen=True
    elif mutation=='frozen':h.b.sock.clock.frozen=True
    elif mutation=='permitted':h.b.last_motion_status['permitted']=True
    else:h.b.last_motion_status['raw_duty'][0]=1
    h.b.safety_tick()
    assert h.status()['startup_reset_ready'] is False


def model():
    from mecanum_wand_4ws8_control.toggle_ui import ToggleOperatorModel
    m=ToggleOperatorModel();m.focused=True;m.blocked=False;m.transport_at=10.
    m.status=dict(state='STANDBY',epoch='',first_stop_reason='',hard_fault=False,recovery_pending=False,
        graph_ok=True,input_health='',ready_to_start=True,stop_confirmed=True)
    m.gate=dict(fault='COMMAND_TIMEOUT',hard_fault=False,output_zero=True,speed_mps=0.,primed_epoch='',graph_ok=True)
    m.transport_status=dict(fresh=True,ready=False,startup_reset_ready=True,baseline_current=True,
        safety_enabled=False,motion_gate_requested=False,command_progress_age_s=.02,
        startup_clock=dict(ready=True,qualified=True,fault=''))
    m.fresh=lambda now:True
    return m


@pytest.mark.parametrize('ready',[False,True])
def test_only_genuine_new_or_used_zero_baseline_can_request_prepare(ready):
    from mecanum_wand_4ws8_control.auto_prepare import AutoPrepareOnce
    m=model();m.transport_status['startup_reset_ready']=ready
    assert AutoPrepareOnce.zero_baseline_qualified(m,10.) is ready
    assert not m.session_requested and not m.held


@pytest.mark.parametrize('enabled,epoch,primed,bypass',[
    (False,'','',True),(True,'','',False),(False,'arm','',False),(False,'','prior',False)])
def test_standby_fast_path_only_accepts_never_armed_zero(enabled,epoch,primed,bypass):
    from dataclasses import replace,asdict
    from mecanum_wand_4ws8_hardware.rotation_nodes import RotationGateNode
    from mecanum_wand_4ws8_control.rotation_core import RotationFinalGate
    from mecanum_wand_4ws8_control.rotation_source import canonical
    from mecanum_wand_4ws8_control.safety_core import Supervisor
    c=base.config();core=RotationFinalGate(c);core.primed_epoch=primed
    gate=RotationGateNode.__new__(RotationGateNode)
    gate.core=core;gate.pending=deque();gate.clocks=lambda:(10.,100.);outputs=[]
    gate.publish_output=outputs.append
    f=Supervisor(c).frame(10.,100.);f=replace(f,enabled=enabled,epoch=epoch)
    gate.on_candidate(NS(data=canonical(dict(frame=asdict(f),sign_digest='not-in-cache',sign_stamp=99.,car_stamp=99.))))
    assert (not gate.pending) is bypass
    if bypass:assert core.frame==f and outputs and core.output==base.parent.nodes.ZERO


def test_protected_parents_and_changed_files_exact():
    import hashlib
    m=json.loads((OUT/'build_manifest.json').read_text())
    for p,digest in m['protected_input_hashes'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest,p
    assert {Path(p).name for p in json.loads((OUT/'PRIVATE_CHANGE_INVENTORY.json').read_text())}=={
        'auto_prepare.py','rotation_nodes.py','nodes.py'}
    assert 'SHOW_FEEDBACK_300MS_READY_V1' in (OUT/'oneclick_app.py').read_text()
    assert m['feedback_protection_ms']==300
