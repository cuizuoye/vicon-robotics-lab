"""Offline startup/reopen/UI regression; no ROS init, SDK or real UDP."""
import copy,importlib.util,json,os,time
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
import socket
from test_pilot_bounds import GENERATED,ROOT,config
from test_pilot_ui import model
from mecanum_wand_4ws8_hardware.startup_idle import may_retry_unarmed_startup
from mecanum_wand_4ws8_control.novice_ui import backend_hint,prepare_connection_ok,acknowledge_physical_release

SOURCE=Path(os.environ['ONECLICK_SOURCE'])
REAL_SOCKET=socket.socket
spec=importlib.util.spec_from_file_location('oneclick_app_test',SOURCE/'oneclick_app.py')
app=importlib.util.module_from_spec(spec);spec.loader.exec_module(app)

@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import socket,rclpy
    def denied(*a,**k):pytest.fail('offline test attempted real hardware/ROS/SDK launch')
    monkeypatch.setattr(socket,'socket',denied);monkeypatch.setattr(rclpy,'init',denied)

def bridge():
    s=dict(tag='4WS8',output_authority_version=4,permitted=False,watchdog_reset=0,
        fault_wheel=0,first_feedback_fault=dict(reason=0),compares=[0]*8,raw_duty=[0]*4,
        target_cps=[0]*4,body_milli=[0]*3,effective_cps=[0]*4,granted_direction=[0]*4)
    return NS(user_arm_seen=False,local_stop_latched=False,safety_enabled=False,
        motion_gate_requested=False,pending_control_action=None,last_mcu_fault_bits=0,
        policy=NS(epoch='',fault='',hard_fault=False,envelope=dict(state='STANDBY',permit=False,command=dict(vx=0.,vy=0.,wz=0.))),
        last_motion_status=s,last_wheel_rpm=[0.]*4,last_mcu_state_flags=(1<<3)|(1<<9),
        last_motion_status_monotonic=9.8,last_wheel_telemetry_monotonic=9.8)

def test_healthy_used_but_disabled_zero_reopen_can_retry_without_fault_clear():
    b=bridge();before=copy.deepcopy(b.__dict__)
    assert may_retry_unarmed_startup(b,'遥测超时',10.01)
    assert may_retry_unarmed_startup(b,'STM32命令序号停止推进',10.01)
    assert b.__dict__==before

@pytest.mark.parametrize('field,value',[
    ('user_arm_seen',True),('local_stop_latched',True),('safety_enabled',True),
    ('motion_gate_requested',True),('pending_control_action','enable'),
    ('pending_control_action','clear_faults'),('last_mcu_fault_bits',0x200),
    ('last_mcu_state_flags',8|2),('last_mcu_state_flags',8|256),
    ('last_mcu_state_flags',512),('last_wheel_rpm',[0.,.001,0.,0.]),
    ('last_wheel_rpm',None),('last_motion_status_monotonic',8.),
    ('last_wheel_telemetry_monotonic',10.1)])
def test_reconnect_exception_is_impossible_after_arm_motion_fault_or_missing_zero(field,value):
    b=bridge();setattr(b,field,value)
    assert not may_retry_unarmed_startup(b,'遥测超时',10.01)

@pytest.mark.parametrize('field,value',[
    ('permitted',True),('tag','4WS7'),('output_authority_version',3),
    ('watchdog_reset',1),('fault_wheel',1),('first_feedback_fault',dict(reason=2)),
    ('raw_duty',[0,0,1,0]),('compares',[]),('effective_cps',[0.,0.,.001,0.])])
def test_any_nonzero_or_unreviewed_native_output_blocks_startup_retry(field,value):
    b=bridge();b.last_motion_status[field]=value
    assert not may_retry_unarmed_startup(b,'遥测超时',10.01)

@pytest.mark.parametrize('field,value',[
    ('epoch','old'),('fault','ESTOP'),('hard_fault',True),
    ('envelope',dict(state='ARMED')),('envelope',dict(state='ACTIVE')),
    ('envelope',dict(state='STANDBY',permit=True)),
    ('envelope',dict(state='STANDBY',command=dict(vx=.001)))])
def test_no_policy_epoch_permit_or_hard_fault_can_be_reconnected_as_startup(field,value):
    b=bridge();setattr(b.policy,field,value)
    assert not may_retry_unarmed_startup(b,'遥测超时',10.01)

def test_send_failure_identity_mismatch_or_estop_always_use_original_latch():
    b=bridge()
    for reason in ('UDP发送失败','收到急停请求','只读诊断校验失败','控制帧发送失败'):
        assert not may_retry_unarmed_startup(b,reason,10.01)

def test_actual_transport_startup_retry_never_sets_enable_or_clears_anything(monkeypatch):
    from mecanum_wand_4ws8_hardware import nodes
    from mecanum_wand_4ws8_hardware.continuous_bridge import ContinuousCmdVelUdpBridge
    b=object.__new__(nodes.HardwareBridge);b.__dict__.update(bridge().__dict__)
    events=[];b.sock=None
    b.force_local_disabled=lambda:events.append('disabled_zero')
    b.get_logger=lambda:NS(warn=lambda msg:events.append(msg))
    monkeypatch.setattr(nodes,'time',NS(monotonic=lambda:10.01))
    monkeypatch.setattr(ContinuousCmdVelUdpBridge,'latch_local_stop',lambda self,reason:events.append('original_latch'))
    b.latch_local_stop('遥测超时')
    assert 'disabled_zero' in events and 'original_latch' not in events
    assert b.reconnect_after==pytest.approx(10.66) and not b.local_stop_latched
    b.user_arm_seen=True;b.latch_local_stop('遥测超时')
    assert events[-1]=='original_latch'

def test_actual_signed_prepare_intent_disables_exception_permanently():
    from mecanum_wand_4ws8_hardware.nodes import HardwareBridge
    b=object.__new__(HardwareBridge);b.__dict__.update(bridge().__dict__)
    b.policy.receive=lambda *a:None;b.safety_tick=lambda:None
    b.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000))
    b.policy.envelope={'state':'ARMED'};b.on_signed(NS(data='already validated by policy'))
    assert b.user_arm_seen
    b.policy.envelope={'state':'STANDBY'};b.on_signed(NS(data=''))
    assert b.user_arm_seen and not may_retry_unarmed_startup(b,'遥测超时',10.01)

def test_real_failed_reopen_trace_was_never_armed_but_had_feedback_gaps():
    folder=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_3x3_20s/ready/ground_stage4_single_9vd3kzd9'
    states=set();gap=False;stamps=[]
    for line in (folder/'observation.jsonl').open():
        r=json.loads(line)
        if r['topic']=='supervisor_status':states.add(r['data']['state'])
    for line in (folder/'mcu_receive.jsonl').open():
        r=json.loads(line)
        if r['kind']=='LINK_STATUS':stamps.append(r['received_steady'])
    assert 'ARMED' not in states and 'ACTIVE' not in states
    assert max(b-a for a,b in zip(stamps,stamps[1:]))>.20


def test_real_native_used_zero_snapshot_is_eligible_only_before_new_prepare():
    from mecanum_wand_4ws8_hardware.stage0_contract import decode_motion_status
    from mecanum_vicon_control.transport_protocol import unpack_wheel_telemetry
    folder=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_3x3_20s/ready/ground_stage4_single_9vd3kzd9'
    t=next(r['received_steady'] for line in (folder/'observation.jsonl').open()
        if (r:=json.loads(line))['topic']=='transport_status' and r['data'].get('local_stop'))
    raw=[json.loads(s) for s in (folder/'mcu_receive.jsonl').read_text().splitlines()]
    diagnostic=next(r for r in reversed(raw) if r['kind']=='LINK_STATUS' and r['received_steady']<t)
    wheel=next(r for r in reversed(raw) if r['kind']=='WHEEL_TELEMETRY' and r['received_steady']<t)
    b=bridge();b.last_motion_status=decode_motion_status(bytes.fromhex(diagnostic['payload_hex']))
    v=unpack_wheel_telemetry(bytes.fromhex(wheel['payload_hex']))
    b.last_mcu_fault_bits=v.fault_bits;b.last_mcu_state_flags=v.state_flags;b.last_wheel_rpm=v.wheel_rpm
    b.last_motion_status_monotonic=diagnostic['received_steady'];b.last_wheel_telemetry_monotonic=wheel['received_steady']
    assert v.state_flags&(1<<9) and not b.last_motion_status['permitted']
    assert may_retry_unarmed_startup(b,'遥测超时',t)
    b.user_arm_seen=True
    assert not may_retry_unarmed_startup(b,'遥测超时',t)

def test_prepare_unlock_can_use_real_released_key_but_never_synthesizes_a_hold():
    m=model();m.blocked=True
    assert acknowledge_physical_release(m,False,10.)
    assert not m.blocked and not m.held and not m.key_down
    m.blocked=True
    assert not acknowledge_physical_release(m,True,10.) and m.blocked
    m.focused=False;assert not acknowledge_physical_release(m,False,10.)
    m.focused=True;m.pilot_used=True;assert not acknowledge_physical_release(m,False,10.)

@pytest.mark.parametrize('health',['FEEDBACK_TIMEOUT','WAND_POSE_TIMEOUT','CAR_POSE_TIMEOUT','ACTUATOR_FAULT'])
def test_concrete_backend_blocker_replaces_generic_wait_hint(health):
    m=model();m.status['input_health']=health
    text=backend_hint(m,10.)
    assert text and '等待位姿稳定' not in text
    if health=='ACTUATOR_FAULT':assert '不会自动清故障' in text

def test_transport_checks_do_not_deadlock_healthy_used_budget_before_prepare():
    m=model();m.transport_at=10.
    m.transport_status=dict(fresh=True,local_stop=False,fault='',safety_enabled=False,
        motion_gate_requested=False,ready=False,preparation_stage='WAIT_BASELINE')
    assert prepare_connection_ok(m,10.) and m.can_start(10.)
    m.transport_status['local_stop']=True
    assert not m.can_start(10.) and '锁止' in backend_hint(m,10.)
    m.transport_status['local_stop']=False;m.transport_status['fresh']=False
    assert not m.can_start(10.) and '反馈' in backend_hint(m,10.)

@pytest.mark.parametrize('confirmations,at,now',[([False]*3,10.,10.),([True,True],10.,10.),([True]*3,10.,131.),([1,1,1],10.,10.)])
def test_gui_cannot_replace_fresh_human_site_confirmation(confirmations,at,now):
    with pytest.raises(ValueError):app.create_session(confirmations,at,now)

def test_file_entry_verification_and_review_do_not_start_processes(monkeypatch):
    started=[]
    monkeypatch.setattr(app,'runtime_imports',lambda:dict(firmware=dict(sha256='valid')))
    # Test the input barrier before it can reach any subprocess or hardware.
    monkeypatch.setattr(app.subprocess,'Popen',lambda *a,**k:started.append(a))
    with pytest.raises(ValueError):app.create_session([True,False,True],10.,10.)
    assert not started


def test_complete_gui_profile_flow_is_explicitly_unarmed_and_never_starts_a_process(tmp_path,monkeypatch):
    import mecanum_wand_4ws8_hardware.authorization as authorization
    import mecanum_wand_4ws8_hardware.commissioning as commissioning
    profile=GENERATED/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json'
    monkeypatch.setattr(app,'READY',tmp_path)
    monkeypatch.setattr(app,'check_port',lambda:None)
    monkeypatch.setattr(app,'verify_files',lambda:dict(draft_profiles=[str(profile)]*7,hardware_package='mecanum_wand_4ws8_hardware'))
    monkeypatch.setattr(app,'make_review',lambda m:dict(test_only_synthetic=True))
    seen=[]
    monkeypatch.setattr(authorization,'verify_runtime_profile',lambda p,purpose:seen.append((p,purpose)))
    monkeypatch.setattr(commissioning,'Runtime',lambda *a:seen.append(a))
    monkeypatch.setattr(app.subprocess,'Popen',lambda *a,**k:pytest.fail('profile builder must not launch anything'))
    folder,command,env=app.create_session([True]*3,10.,10.)
    data=json.loads((folder/'site_confirmed.json').read_text())
    assert data['mapping_scale']==1. and data['stage']==4 and seen
    assert 'mode:=hardware' in command and 'gui:=true' in command
    assert env['ROS_DOMAIN_ID']=='120' and not any('enable:=true' in v for v in command)
    assert not list(folder.glob('mcu_receive*'))

def test_real_tk_preview_is_unconfirmed_and_has_no_live_start_path(monkeypatch):
    import tkinter as tk
    monkeypatch.setattr(app,'verify_files',lambda:pytest.fail('preview must not access hardware configuration'))
    root=tk.Tk();root.withdraw();window=app.App(root,preview=True)
    try:
        for v in window.checks:v.set(True)
        window.good_at=time.monotonic();window.update_buttons()
        assert str(window.open_button['state'])=='disabled'
        assert str(window.check_button['state'])=='disabled'
        window.open_console();assert window.backend is None
        assert '离线' in window.message.get()
    finally:
        window.closed=True;root.after_cancel(window.poll_job);root.destroy()


def test_normal_gui_requires_all_three_click_confirmations_and_never_auto_opens(monkeypatch):
    import tkinter as tk
    root=tk.Tk();root.withdraw();window=app.App(root,preview=True)
    calls=[]
    try:
        window.preview=False;window.good_at=time.monotonic()
        window.update_buttons();assert str(window.open_button['state'])=='disabled'
        for v in window.checks[:2]:v.set(True)
        window.update_buttons();assert str(window.open_button['state'])=='disabled'
        window.checks[2].set(True);window.update_buttons()
        assert str(window.open_button['state'])=='normal' and window.backend is None
        window.good_at=time.monotonic()-121.;window.update_buttons()
        assert str(window.open_button['state'])=='disabled'
    finally:
        window.closed=True;root.after_cancel(window.poll_job);root.destroy()


def test_duplicate_command_raises_existing_ui_instead_of_opening_second_backend(tmp_path,monkeypatch):
    def unix_only(family,*args,**kwargs):
        assert family==socket.AF_UNIX,'no Internet/robot sockets permitted in IPC test'
        return REAL_SOCKET(family,*args,**kwargs)
    monkeypatch.setattr(socket,'socket',unix_only)
    monkeypatch.setenv('XDG_RUNTIME_DIR',str(tmp_path))
    instance=app.single_instance()
    try:
        assert instance is not None
        assert app.single_instance() is None
        connection,_=instance[1].accept()
        with connection:assert connection.recv(16)==b'SHOW'
    finally:
        instance[1].close();instance[0].close();instance[2].unlink()


def test_busy_transport_is_explained_and_never_killed_or_replaced(monkeypatch):
    closed=[]
    class Busy:
        def bind(self,*a):raise OSError('address already in use')
        def close(self):closed.append(True)
    monkeypatch.setattr(socket,'socket',lambda *a,**k:Busy())
    with pytest.raises(ValueError,match='不会自动抢占或杀进程'):app.check_port()
    assert closed==[True]

def test_accepted_algorithm_firmware_and_active_timeout_parameters_unchanged():
    old=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_3x3_20s/ready/runtime/src'
    package='mecanum_wand_4ws8_control'
    for name in ('core.py','continuous_servo.py','safety_core.py','safety_nodes.py','pilot_core.py','demo_policy.py','staged_config.py','arena_geometry.py'):
        assert (GENERATED/package/package/name).read_bytes()==(old/package/package/name).read_bytes()
    c=config();assert c.feedback_timeout_s==.20 and c.command_timeout_s==.10 and c.lease_timeout_s==.15
    assert (c.max_speed_mps,c.max_accel_mps2)==(.70,.80)
    package='mecanum_wand_4ws8_hardware'
    for name in ('stage0_contract.py','stage0_session.py','authorization.py','pilot_authorization.py'):
        assert (GENERATED/package/package/name).read_bytes()==(old/package/package/name).read_bytes()
