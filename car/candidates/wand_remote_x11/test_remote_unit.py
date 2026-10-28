"""Remote scheduling/click invariants. No physical socket or ROS init."""
import threading
import time
from types import SimpleNamespace as NS
import pytest
from mecanum_wand_4ws8_control import async_x11,remote_keys,operator_console as ui
from mecanum_wand_4ws8_control.async_x11 import AsyncX11KeyState,KeySample
from mecanum_wand_4ws8_control.toggle_ui import ToggleOperatorModel
from mecanum_wand_4ws8_control.demo_policy import INPUT_MODE
from mecanum_wand_4ws8_control.remote_render import ChangedVariable,ChangedButton


def ready():
    m=ToggleOperatorModel();m.focused=True
    m.status=dict(state='ARMED',gate_ready=True,input_health='',hard_fault=False,
        pilot=dict(max_active_s=120.,used=False,remaining_s=120.,operator_input_mode=INPUT_MODE,
                   spatial_policy='TRACKING_ONLY_NO_FIXED_FENCE_V1'))
    m.status_at=m.gate_at=10.;m.gate=dict(fault='',hard_fault=False)
    m.fresh=lambda now:True;m.pilot_valid=lambda:True
    return m


def console(monkeypatch):
    c=ui.Console.__new__(ui.Console);c.model=ready();c.closing=False;c.release_job=None
    c._remote_release=None;c.key_state=AsyncX11KeyState.__new__(AsyncX11KeyState)
    c.key_state.snapshot=lambda:KeySample(False,10.,10.)
    calls=[];c.node=NS(heartbeat=lambda:calls.append('lease'),request=lambda k:calls.append(k))
    clock=NS(now=10.);monkeypatch.setattr(remote_keys,'time',NS(monotonic=lambda:clock.now))
    # Inherited key event methods use this module's time as well.
    import mecanum_wand_4ws8_control.toggle_keys as old_keys
    monkeypatch.setattr(old_keys,'time',NS(monotonic=lambda:clock.now))
    return c,clock,calls


def test_queries_run_on_owned_worker_and_reads_are_nonblocking():
    thread_ids=[]
    class Reader:
        def __init__(self,name):thread_ids.append(threading.get_ident())
        def down(self):time.sleep(.025);thread_ids.append(threading.get_ident());return False
        def close(self):thread_ids.append(threading.get_ident())
    s=AsyncX11KeyState(':test',reader_factory=Reader)
    try:
        reads=[];end=time.monotonic()+.20
        while time.monotonic()<end:
            at=time.monotonic();assert s.down() is False;reads.append(time.monotonic()-at);time.sleep(.005)
        assert max(reads)<.01 and len(reads)>15
        assert set(thread_ids)=={s.thread.ident} and s.thread.ident!=threading.get_ident()
    finally:s.close()


def test_frozen_query_cannot_freshen_old_state_or_block_stop():
    freeze=threading.Event();release=threading.Event()
    class Reader:
        def __init__(self,name):pass
        def down(self):
            if freeze.is_set():release.wait(1.)
            return False
        def close(self):pass
    s=AsyncX11KeyState(':test',reader_factory=Reader);freeze.set()
    try:
        old=s.snapshot().started;time.sleep(.10)
        with pytest.raises(RuntimeError):s.down()
        with s._lock:assert s._sample.started==old
        at=time.monotonic();s.close();assert time.monotonic()-at<.15
    finally:release.set();s.thread.join(.2)


def test_slow_response_is_not_accepted_at_its_late_completion():
    class Reader:
        def __init__(self,name):pass
        def down(self):time.sleep(.070);return False
        def close(self):pass
    with pytest.raises(RuntimeError,match='exceeded60ms'):
        AsyncX11KeyState(':test',reader_factory=Reader)


def test_actual_release_waits_for_a_post_event_key_up_sample(monkeypatch):
    c,t,calls=console(monkeypatch);c.press();assert c.model.click_pending
    t.now=10.02;c.release()
    assert not c._confirm_remote_release(10.03,KeySample(False,10.01,10.02))
    assert not c.model.session_requested
    t.now=10.04;assert c._confirm_remote_release(t.now,KeySample(False,10.03,10.035))
    assert c.model.session_requested and c.model.activation_used


def test_polling_without_real_release_cannot_start(monkeypatch):
    c,t,calls=console(monkeypatch);c.press();assert c.model.click_pending
    assert not c._confirm_remote_release(10.03,KeySample(False,10.02,10.025))
    c.model.observe_physical_release(10.03)
    assert not c.model.session_requested and not c.model.activation_used


def test_cached_key_up_does_not_cancel_a_real_remote_click(monkeypatch):
    c,t,calls=console(monkeypatch);c.press()
    c._observe_cached_release(10.01,KeySample(False,9.99,10.))
    assert c.model.click_pending and c.model.key_down
    # Separate X11 connections can observe physical release before Tk sees it.
    c._observe_cached_release(10.03,KeySample(False,10.02,10.025))
    assert c.model.click_pending and not c.model.session_requested
    t.now=10.04;c.release()
    t.now=10.06;c._confirm_remote_release(t.now,KeySample(False,10.05,10.055))
    assert c.model.session_requested


def test_lost_release_times_out_without_synthesizing_start(monkeypatch):
    c,t,calls=console(monkeypatch);c.press()
    for i in range(1,23):
        now=10.+i*.05;c._observe_cached_release(now,KeySample(False,now-.01,now-.005))
        c.model.tick(now)
    assert not c.model.click_pending and not c.model.session_requested
    assert not c.model.activation_used and not c.model.key_down


@pytest.mark.parametrize('failure',['timeout','focus','fault','closing','click_changed'])
def test_pending_release_never_delays_enable_until_conditions_recover(monkeypatch,failure):
    c,t,calls=console(monkeypatch);c.press();t.now=10.02;c.release()
    if failure=='timeout':t.now=10.13
    elif failure=='focus':c.model.focused=False;t.now=10.04
    elif failure=='fault':c.model.gate['fault']='FAULT';t.now=10.04
    elif failure=='closing':c.closing=True;t.now=10.04
    else:c.model.click_started=9.;t.now=10.04
    c._confirm_remote_release(t.now,KeySample(False,t.now-.005,t.now-.001))
    assert not c.model.session_requested
    c.model.focused=True;c.model.gate['fault']='';c.closing=False
    c._confirm_remote_release(10.14,KeySample(False,10.135,10.139))
    assert not c.model.session_requested


def test_repeat_pair_does_not_commit_but_second_real_click_stops(monkeypatch):
    c,t,calls=console(monkeypatch);c.press();t.now=10.02;c.release()
    t.now=10.025;c.press();assert c._remote_release is None and c.model.key_down
    assert not c.model.session_requested
    t.now=10.1;c.release();t.now=10.12
    c._confirm_remote_release(t.now,KeySample(False,10.11,10.115))
    assert c.model.session_requested
    # The real GUI continues ticking between input events; do not accidentally
    # turn this second-click test into the separately tested 100ms stall case.
    c.model.tick(10.12)
    t.now=10.15;c.press();assert not c.model.session_requested and 'stop' in calls


def test_escape_is_immediate_and_needs_no_x11_query(monkeypatch):
    c,t,calls=console(monkeypatch);c.model.held=True
    c.key_state.snapshot=lambda:(_ for _ in ()).throw(AssertionError('Esc queried X11'))
    c.escape();assert c.model.ended and not c.model.session_requested and 'stop' in calls


def test_ui_stall_guard_still_revokes_at100ms():
    m=ready();m.tick(10.);m.held=True;m.activation_used=True
    m.tick(10.101)
    assert not m.session_requested and m.ended and '调度中断' in m.notice


def test_unchanged_widgets_are_not_reconfigured():
    text=[];v=ChangedVariable(NS(set=text.append,get=lambda:'value'))
    for _ in range(20):v.set('same')
    v.set('different');assert text==['same','different']
    configurations=[];b=ChangedButton(NS(configure=lambda **k:configurations.append(k)))
    for _ in range(20):b.configure(state='disabled')
    b.configure(state='normal');assert configurations==[dict(state='disabled'),dict(state='normal')]
