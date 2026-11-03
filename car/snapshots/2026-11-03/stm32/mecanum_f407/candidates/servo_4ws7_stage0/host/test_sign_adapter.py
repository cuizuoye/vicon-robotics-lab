"""Generated Sign transport authority hooks; no executor/socket/real MCU."""
from types import SimpleNamespace,MethodType
import json
import os
from pathlib import Path
import sys
import pytest
ROOT=Path(os.environ['STAGE0_GENERATED'])
for name in ('mecanum_wand_4ws7_control','mecanum_wand_4ws7_hardware'):sys.path.insert(0,str(ROOT/name))
from mecanum_wand_4ws7_control.safety_core import ZERO
from mecanum_wand_4ws7_hardware.commissioning import AuthoritySigner,TransportPolicy
from mecanum_wand_4ws7_hardware.nodes import HardwareBridge,CmdVelUdpBridge

def transport(monkeypatch,selected=True):
    import mecanum_wand_4ws7_hardware.nodes as nodes
    clock=[10.];monkeypatch.setattr(nodes.time,'monotonic',lambda:clock[0])
    signer=AuthoritySigner('23'*32);policy=TransportPolicy('23'*32)
    policy.receive(signer.encode(ZERO,10.,100.),10.,100.)
    status=dict(tag='4WS7',started_ms=0,stationary_ms=500,permitted=0,
        compares=[0]*8,raw_duty=[0]*4,target_cps=[0]*4,body_milli=[0]*3,
        effective_cps=[0]*4,granted_direction=[0]*4,output_authority_version=3,
        selected=selected,action_mode=1 if selected else 0)
    reports=[];calls=[]
    f=SimpleNamespace(policy=policy,session=SimpleNamespace(connected=True,session_id=123,select_mode=lambda t:b'SELECT'),
        sock=object(),last_motion_status=status,last_motion_status_monotonic=10.,
        last_wheel_telemetry_monotonic=10.,last_command_progress_monotonic=10.,
        last_receive_monotonic=10.,last_telemetry_command_sequence=55,last_mcu_fault_bits=0,
        last_wheel_rpm=(0,)*4,last_armed_epoch='',enabled_epoch='',rearm_epoch='',rearm_started=None,
        permit_observed_epoch='',local_stop_latched=False,control_enable_reset_seen=False,
        readonly_mode=False,motion_gate_requested=False,safety_enabled=False,pending_control_action=None,
        last_feedback_identity=None,feedback_sequence=0,runtime=SimpleNamespace(mode='emulator'),
        stage0_selection_session=None,stage0_selection_at=None,
        monotonic_ms=lambda:int(clock[0]*1000),send_immediate=lambda *a:calls.append(a) or True,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int((clock[0]+90)*1e9))),
        get_logger=lambda:SimpleNamespace(error=lambda text:calls.append(('error',text)),info=lambda t:None,warn=lambda t:None),
        transport_info=SimpleNamespace(publish=lambda m:reports.append(m.data)),
        feedback=SimpleNamespace(publish=lambda m:None),force_local_disabled=lambda:None)
    monkeypatch.setattr(CmdVelUdpBridge,'on_cmd',lambda *a:None)
    monkeypatch.setattr(CmdVelUdpBridge,'on_enable',lambda *a:None)
    def controlled_stop(fake,msg):
        calls.append(('control',msg.data))
        if msg.data:fake.queue_control_action('enable')
        else:fake.motion_gate_requested=False;fake.pending_control_action=None
    monkeypatch.setattr(CmdVelUdpBridge,'on_control_enable',controlled_stop)
    for name in ('reset_preparation','feedback_is_fresh','baseline_is_current','baseline_is_prepared',
                 'queue_control_action','stop_wire','safety_tick'):
        setattr(f,name,MethodType(getattr(HardwareBridge,name),f))
    f.reset_preparation()
    def update(state='ARMED',step=.1,fresh=True):
        clock[0]+=step
        if fresh:
            f.last_motion_status_monotonic=f.last_wheel_telemetry_monotonic=f.last_command_progress_monotonic=clock[0]
        policy.receive(signer.encode(ZERO,clock[0],clock[0]+90,state=state,epoch='e',permit=True),clock[0],clock[0]+90)
        f.safety_tick()
    return f,clock,update,calls,reports

def test_select_is_once_and_ready_requires_actual_new_mode_status(monkeypatch):
    f,clock,update,calls,reports=transport(monkeypatch,False)
    for _ in range(8):update()
    assert len([a for a in calls if a[0]==b'SELECT'])==1
    assert not json.loads(reports[-1])['ready']
    f.last_motion_status.update(selected=True,action_mode=1);update()
    assert json.loads(reports[-1])['ready'] and not f.motion_gate_requested
    update('ACTIVE',.02)
    assert f.motion_gate_requested and f.pending_control_action=='enable'

@pytest.mark.parametrize('defect',['unselected','bench_mode','stale','wrong_epoch','new_session','nonzero','fault'])
def test_no_enable_with_incomplete_or_wrong_authority(monkeypatch,defect):
    f,clock,update,calls,reports=transport(monkeypatch)
    for _ in range(8):update()
    if defect=='unselected':f.last_motion_status['selected']=False
    if defect=='bench_mode':f.last_motion_status['action_mode']=2
    if defect=='stale':f.last_motion_status_monotonic-=.21
    if defect=='wrong_epoch':f.prepared_epoch='old'
    if defect=='new_session':f.session.session_id=456
    if defect=='nonzero':f.last_motion_status['body_milli'][0]=1
    if defect=='fault':f.last_mcu_fault_bits=32
    f.policy.envelope={**f.policy.envelope,'state':'ACTIVE'}
    f.queue_control_action('enable')
    assert f.policy.fault=='MCU_NOT_PREPARED' and not f.motion_gate_requested

def test_active_feedback_timeout_and_select_timeout_fail_closed(monkeypatch):
    f,clock,update,calls,reports=transport(monkeypatch)
    for _ in range(8):update()
    update('ACTIVE',.02)
    update('ACTIVE',.21,False)
    assert f.policy.fault and not f.motion_gate_requested
    f,clock,update,calls,reports=transport(monkeypatch,False)
    for _ in range(12):update()
    assert f.policy.fault=='STAGE0_SELECT_NOT_CONFIRMED'
