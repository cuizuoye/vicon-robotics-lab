"""Auto-open/prepare plus genuine Space activation, with no real robot I/O."""
import ast
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from support import ROOT, GENERATED, config
from test_toggle import model, click, active, LeaseDriver
from mecanum_wand_4ws8_control import auto_prepare as ap
from mecanum_wand_4ws8_control import operator_console as ui
from mecanum_wand_4ws8_control.safety_core import ZERO

OUT=Path(os.environ['AUTOPREP_OUTPUT'])
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'
spec=importlib.util.spec_from_file_location('autoprep_app_test',OUT/'oneclick_app.py')
app=importlib.util.module_from_spec(spec);spec.loader.exec_module(app)
from auto_open import AutomaticOpenMixin


@pytest.fixture(autouse=True)
def no_real_io(monkeypatch):
    import socket,rclpy
    def denied(*a,**k):pytest.fail('automatic prepare tests attempted real socket/ROS/process I/O')
    monkeypatch.setattr(socket,'socket',denied)
    monkeypatch.setattr(rclpy,'init',denied)
    monkeypatch.setattr(app.subprocess,'Popen',denied)


def fresh(m,t):
    m.status_at=m.gate_at=m.transport_at=t


def standby(t=10.):
    m=model('STANDBY',t,False)
    m.transport_status=dict(fresh=True,startup_clock=dict(ready=True,fault=''),
        local_stop=False,fault='',safety_enabled=False,motion_gate_requested=False)
    m.transport_at=t
    return m


def poll(p,m,t,**kw):
    fresh(m,t)
    return p.poll(m,t,**{'space_down':False,'service_ready':True,**kw})


def test_automatic_prepare_is_one_request_after_stable_released_key_not_enable():
    m=standby();p=ap.AutoPrepareOnce()
    assert not poll(p,m,10.) and not poll(p,m,10.299)
    assert poll(p,m,10.300)
    assert p.attempted and p.phase=='PREPARING' and not m.session_requested
    assert not m.activation_used and not m.key_down
    m.status.update(state='ARMED',gate_ready=True)
    for t in (10.32,11.,20.):
        assert not poll(p,m,t) and p.phase=='READY' and not m.session_requested
    assert [e['event'] for e in m.events].count('automatic_prepare_request_only_no_enable')==1
    assert not m.pilot_used


@pytest.mark.parametrize('bad', ['physical_down','model_down','unfocused','stale_status',
    'stale_gate','stale_transport','future_transport','not_fresh','clock_unready',
    'stop_unconfirmed','service_unready','input_issue','pose_health','enabled','motion_gate',
    'backend_not_ready','blocked','wrong_mode','recovery_pending','fault','hard_fault',
    'transport_fault','clock_fault','gate_fault','closed','used','ended'])
def test_any_unready_condition_blocks_auto_prepare_without_creating_authority(bad):
    m=standby();p=ap.AutoPrepareOnce();p.poll(m,10.,space_down=False,service_ready=True)
    fresh(m,10.31);kw=dict(space_down=False,service_ready=True)
    if bad=='physical_down':kw['space_down']=True
    elif bad=='model_down':m.key_down=True
    elif bad=='unfocused':m.focused=False
    elif bad=='stale_status':m.status_at=9.
    elif bad=='stale_gate':m.gate_at=9.
    elif bad=='stale_transport':m.transport_at=9.
    elif bad=='future_transport':m.transport_at=11.
    elif bad=='not_fresh':m.transport_status['fresh']=False
    elif bad=='clock_unready':m.transport_status['startup_clock']['ready']=False
    elif bad=='stop_unconfirmed':m.status['stop_confirmed']=False
    elif bad=='service_unready':kw['service_ready']=False
    elif bad=='input_issue':kw['input_issue']='Sign missing'
    elif bad=='pose_health':m.status['input_health']='WAND_POSE_TIMEOUT'
    elif bad=='enabled':m.transport_status['safety_enabled']=True
    elif bad=='motion_gate':m.transport_status['motion_gate_requested']=True
    elif bad=='backend_not_ready':m.status['ready_to_start']=False
    elif bad=='blocked':m.blocked=True
    elif bad=='wrong_mode':m.status['pilot']['operator_input_mode']='SPACE_HOLD'
    elif bad=='recovery_pending':m.status['recovery_pending']=True
    elif bad=='fault':m.status['state']='FAULT'
    elif bad=='hard_fault':m.status['hard_fault']=True
    elif bad=='transport_fault':m.transport_status['fault']='RX_FAILURE'
    elif bad=='clock_fault':m.transport_status['startup_clock']['fault']='CLOCK_FAILURE'
    elif bad=='gate_fault':m.gate['fault']='COMMAND_SOURCE_CONFLICT'
    elif bad=='closed':m.closing=True
    elif bad=='used':m.pilot_used=True
    else:m.ended=True
    assert not p.poll(m,10.31,**kw) and not p.attempted
    assert not m.held and not m.activation_used


def test_instability_resets_stable_window_and_fault_is_never_auto_recovered():
    m=standby();p=ap.AutoPrepareOnce();poll(p,m,10.)
    assert not poll(p,m,10.29,input_issue='occluded')
    assert not poll(p,m,10.31) and not poll(p,m,10.60)
    assert poll(p,m,10.61)
    m.status['state']='FAULT';assert not poll(p,m,10.62) and p.cancelled
    m.status.update(state='STANDBY',ready_to_start=True)
    assert not poll(p,m,11.) and p.cancelled and not m.session_requested


def test_prepare_timeouts_cannot_become_late_requests_or_retries():
    m=standby();p=ap.AutoPrepareOnce()
    assert not poll(p,m,10.,service_ready=False)
    assert not poll(p,m,40.01) and p.cancelled and not p.attempted
    m=standby();p=ap.AutoPrepareOnce();poll(p,m,10.);assert poll(p,m,10.3)
    assert not poll(p,m,15.31) and p.cancelled
    m.status.update(state='ARMED',gate_ready=True)
    assert not poll(p,m,15.32) and not m.session_requested


def test_early_space_is_not_queued_and_a_new_complete_click_is_required():
    m=standby();p=ap.AutoPrepareOnce()
    m.press(10.);assert not m.click_pending
    assert not poll(p,m,10.,space_down=True)
    m.release(10.02);assert not poll(p,m,10.03)
    assert poll(p,m,10.33) and not m.session_requested
    m.status.update(state='ARMED',gate_ready=True);poll(p,m,10.35)
    assert not m.release(10.36) and not m.session_requested
    assert click(m,10.37) and m.session_requested
    active(m,10.41)
    assert not poll(p,m,10.42) and p.phase=='MANUAL_ACTIVE'
    assert m.pilot_used and m.session_requested and not p.cancelled


def test_actual_lease_chain_auto_prepare_then_adjustment_waits_for_new_human_space():
    d=LeaseDriver();d.run(1.8);p=ap.AutoPrepareOnce()
    d.m.transport_status=standby().transport_status
    requested=[]
    for _ in range(30):
        d.m.transport_at=d.h.now
        if p.poll(d.m,d.h.now,space_down=False,service_ready=True):
            requested.append(d.h.now)
            assert d.h.sup.request_start(d.h.now,d.h.ros)[0]
        assert d.step()==ZERO
    assert len(requested)==1 and d.m.start_ready(d.h.now) and p.phase=='READY'
    # The same neutral origin timing is preserved: picking up/moving Sign AFTER
    # automatic preparation is not motion authority or a preloaded target.
    from dataclasses import replace
    for i in range(1,51):
        d.h.hand=replace(d.h.hand_origin,x=d.h.hand_origin.x+.40*i/50,
            y=d.h.hand_origin.y-.20*i/50,z=1.+.35*i/50)
        assert d.step()==ZERO
    assert not d.h.sup.pilot.used and not d.sent[-1].held
    assert d.tap() and d.step()==ZERO and d.h.sup.state=='ACTIVE'
    for i in range(1,16):
        d.h.hand=replace(d.h.hand,x=d.h.hand.x+.004)
        d.step()
        d.m.transport_at=d.h.now
        assert not p.poll(d.m,d.h.now,space_down=False,service_ready=True)
    assert d.h.output!=ZERO and d.sent[-1].held and not p.cancelled
    d.m.press(d.h.now);assert d.step()==ZERO and not d.sent[-1].held
    d.m.release(d.h.now);d.run(2.)
    assert not d.tap() and not p.poll(d.m,d.h.now,space_down=False,service_ready=True)
    assert d.h.output==ZERO and d.h.sup.stop_confirmed


def test_real_console_loop_requests_only_prepare_once_and_stops_after_prepare_failure(monkeypatch):
    clock=NS(now=10.);monkeypatch.setattr(ap,'time',NS(monotonic=lambda:clock.now))
    class Base:
        def loop(self):pass
    class C(ap.AutoPrepareConsoleMixin,Base):pass
    c=object.__new__(C);c.closing=False;c.model=standby();c.auto_prepare=ap.AutoPrepareOnce()
    c.auto_prepare_stop_sent=False;c.server_space_down=lambda:False
    calls=[];c.node=NS(service_clients={'start':NS(service_is_ready=lambda:True)},pending=set(),
        input_hint=lambda:'',request=lambda action:(calls.append(action) or True),
        heartbeat=lambda:calls.append('lease_false' if not c.model.held else 'LEASE_TRUE'))
    c.loop();clock.now=10.3;fresh(c.model,clock.now);c.loop()
    assert calls==['start'] and not c.model.held
    c.model.status['state']='FAULT';clock.now=10.32;fresh(c.model,clock.now);c.loop();c.loop()
    assert calls==['start','lease_false','stop'] and c.model.ended and not c.model.held
    c.model.status['state']='STANDBY';clock.now=10.4;fresh(c.model,clock.now);c.loop()
    assert calls.count('start')==1


class Flag:
    def __init__(self,value=False):self.value=value
    def get(self):return self.value


def opening():
    class Manual:
        def update_buttons(self):pass
        def preflight(self):self.good_at=-1e9
        def open_console(self):self.opens+=1
    class C(AutomaticOpenMixin,Manual):pass
    c=object.__new__(C);c.preview=c.closed=c.busy=c.auto_open_queued=c.auto_open_attempted=False
    c.backend=None;c.good_at=10.;c.checks=[Flag() for _ in range(3)];c.opens=0
    jobs=[];c.root=NS(after_idle=lambda cb:jobs.append(cb))
    return c,jobs


def test_auto_open_needs_all_actual_checkboxes_and_fresh_preflight_once(monkeypatch):
    import auto_open
    monkeypatch.setattr(auto_open,'time',NS(monotonic=lambda:10.1))
    c,jobs=opening()
    for flag in c.checks:
        c.update_buttons();assert jobs==[];flag.value=True
    c.update_buttons();c.update_buttons();assert len(jobs)==1
    jobs.pop()();assert c.opens==1 and c.auto_open_attempted
    c.update_buttons();assert not jobs
    c.preflight();assert not c.auto_open_attempted and not c.auto_open_allowed()


@pytest.mark.parametrize('bad',['preview','closed','busy','backend','stale','future','not_boolean'])
def test_check_failure_or_missing_human_ack_never_queues_open(monkeypatch,bad):
    import auto_open
    monkeypatch.setattr(auto_open,'time',NS(monotonic=lambda:10.1))
    c,jobs=opening()
    for flag in c.checks:flag.value=True
    if bad in ('preview','closed','busy'):setattr(c,bad,True)
    elif bad=='backend':c.backend=object()
    elif bad=='stale':c.good_at=-120.
    elif bad=='future':c.good_at=11.
    else:c.checks[0].value=1
    c.update_buttons();assert not jobs and c.opens==0


def test_confirmation_revoked_before_idle_callback_cancels_open(monkeypatch):
    import auto_open
    monkeypatch.setattr(auto_open,'time',NS(monotonic=lambda:10.1))
    c,jobs=opening()
    for flag in c.checks:flag.value=True
    c.update_buttons();c.checks[1].value=False;jobs.pop()()
    assert not c.auto_open_attempted and c.opens==0


def test_failed_or_finished_console_does_not_get_automatic_second_attempt(monkeypatch):
    import auto_open
    monkeypatch.setattr(auto_open,'time',NS(monotonic=lambda:10.1))
    c,jobs=opening()
    for flag in c.checks:flag.value=True
    c.update_buttons();jobs.pop()()
    c.backend=None;c.update_buttons()
    assert c.opens==1 and not jobs


def test_preview_ui_requests_1280x720_and_keeps_every_confirmation_false(monkeypatch):
    import tkinter as tk
    root=tk.Tk();root.withdraw();calls=[];real_geometry=root.geometry
    monkeypatch.setattr(root,'geometry',lambda value:(calls.append(value),real_geometry(value))[1])
    window=app.App(root,preview=True)
    try:
        root.update_idletasks()
        assert calls==['1280x720'] and tuple(root.minsize())==(1000,640)
        assert root.winfo_reqheight()<=720 and root.winfo_reqwidth()<=1280
        assert not window.open_button.winfo_manager() and not window.auto_open_attempted
        assert not any(v.get() for v in window.checks)
        assert str(window.check_button['state'])=='disabled' and window.backend is None
        assert not window.auto_open_allowed()
    finally:
        root.after_cancel(window.poll_job);root.destroy()


def test_real_console_construction_has_correct_size_hidden_prepare_and_no_requests(monkeypatch):
    import tkinter as tk
    import time
    from mecanum_wand_4ws8_control.core import Pose
    monkeypatch.setenv('WAND_AUTOPREP_FLOW',ap.TOKEN)
    monkeypatch.setattr(ui,'X11KeyState',lambda *a:NS(down=lambda:False,close=lambda:None))
    m=standby(time.monotonic());c=config();events=[]
    node=NS(runtime=NS(config=c,wand_reference_margin_m=.5),input_kind='hardware',trace=[],
        car=Pose(*c.geometry().world(0.,0.),c.ground_z_m,c.arena_heading_rad),raw_wand=Pose(1.,1.,1.),
        request=lambda name:events.append(name),input_hint=lambda:'')
    root=tk.Tk();root.withdraw();calls=[];real_geometry=root.geometry
    monkeypatch.setattr(root,'geometry',lambda value:(calls.append(value),real_geometry(value))[1])
    console=ui.Console(root,node,m)
    try:
        console.draw(time.monotonic());root.update_idletasks()
        assert calls==['1280x720'] and tuple(root.minsize())==(1000,640)
        assert root.winfo_reqheight()<=720 and root.winfo_reqwidth()<=1280
        assert console.auto_prepare is not None and not console.buttons['start'].winfo_manager()
        assert console.buttons['stop'].winfo_manager()=='pack'
        assert console.buttons['estop'].winfo_manager()=='pack'
        assert '自动检查/准备' in console.step_text.get() and not events and not m.session_requested
    finally:
        for name in ('loop_job','focus_job','release_job','shutdown_job','destroy_job'):
            job=getattr(console,name,None)
            if job is not None:root.after_cancel(job)
        root.destroy()


def test_all_safety_hardware_and_mapping_code_is_byte_identical_except_explicit_ui_wrapper():
    old=OLD/'runtime/src'
    changed={'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/operator_console.py'}
    for p in old.rglob('*'):
        if not p.is_file() or any(k=='__pycache__' or k.endswith('.egg-info') for k in p.parts):continue
        relative=p.relative_to(old)
        if str(relative) not in changed:assert (GENERATED/relative).read_bytes()==p.read_bytes(),str(relative)
    def classes(p):
        return {n.name:{m.name:ast.dump(m) for m in n.body if isinstance(m,ast.FunctionDef)}
            for n in ast.parse(p.read_text()).body if isinstance(n,ast.ClassDef)}
    relative=next(iter(changed));a=classes(old/relative);b=classes(GENERATED/relative)
    for name,methods in a.items():
        for method,value in methods.items():
            if name=='Console' and method in ('__init__','draw','draw_pad'):continue
            assert b[name][method]==value,(name,method)
    old_text=(old/relative).read_text()
    expected=old_text.replace("root.geometry('1120x790')","root.geometry('1280x720')").replace(
        'root.minsize(1000, 740)','root.minsize(1000, 640)').replace(
        'width=690, height=430','width=690, height=260').replace(
        '单击模式：准备 → 就绪 → 轻按松开空格启动；松键不停车。再次空格/Esc/停止结束。',
        '自动检查与准备 → 看到就绪后单击空格开始；再次空格/Esc/停止结束；松键不停车。')
    for points,pixels in ((18,24),(11,16),(10,14),(9,13)):
        for space in (' ', ''):
            expected=expected.replace(f"'Noto Sans CJK SC',{space}{points}",f"'Noto Sans CJK SC',{space}-{pixels}")
    for method in ('__init__','draw','draw_pad'):
        assert classes_from_text(expected)['Console'][method]==b['Console'][method]


def classes_from_text(text):
    return {n.name:{m.name:ast.dump(m) for m in n.body if isinstance(m,ast.FunctionDef)}
        for n in ast.parse(text).body if isinstance(n,ast.ClassDef)}
