"""Regression for the two observed failures, no physical socket/ROS init."""
from collections import deque,OrderedDict
from dataclasses import asdict,replace
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import pytest
from std_msgs.msg import String
from mecanum_wand_4ws8_control.async_x11 import AsyncX11KeyState,KeySample
from mecanum_wand_4ws8_control import async_x11,remote_keys
from mecanum_wand_4ws8_control.safety_core import Supervisor,ZERO
from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig
from mecanum_wand_4ws8_hardware.rotation_nodes import RotationGateNode
from mecanum_wand_4ws8_hardware.bound_rotation import SourceFaultGate
from mecanum_wand_4ws8_control.bound_history import RetainedPackets

OUT=Path(os.environ['FEEDBACK_OUTPUT'])


def config():
    m=json.loads((OUT/'build_manifest.json').read_text())
    return TrackingSafetyConfig(**json.loads(Path(m['draft_profiles'][4]).read_text())['safety'])


def test_normal40ms_queries_may_have_no_fresh_sample_but_do_not_lose_liveness():
    class Reader:
        def __init__(self,*a):pass
        def down(self):time.sleep(.040);return False
        def close(self):pass
    keys=AsyncX11KeyState(':software',reader_factory=Reader);pending=fresh=0
    try:
        until=time.monotonic()+.55
        while time.monotonic()<until:
            s=keys.sample_or_pending()
            if s is None:pending+=1
            else:fresh+=1;assert not s.down and time.monotonic()-s.started<=.080
            time.sleep(.002)
        assert pending and fresh and keys.diagnostics()['queries']>=8
        assert keys.MAX_AGE_S==.080 and keys.MAX_RTT_S==.060 and keys.MAX_REPLY_GAP_S==.080
    finally:keys.close()


def cached(monkeypatch,*,now=10.1,sample=None,inflight=None):
    k=AsyncX11KeyState.__new__(AsyncX11KeyState);k._lock=threading.Lock()
    k._closed=False;k._error='';k._sample=sample or KeySample(False,10.,10.04);k._inflight=inflight
    monkeypatch.setattr(async_x11,'time',NS(monotonic=lambda:now))
    return k


def test_pending_query_does_not_relabel_old_input_or_unlock(monkeypatch):
    k=cached(monkeypatch,inflight=10.055)
    assert k.sample_or_pending() is None and k.down() is True
    with pytest.raises(RuntimeError,match='no fresh'):k.snapshot()


def test_inflight_query_deadline_is_independent_of_reply_completion(monkeypatch):
    k=cached(monkeypatch,now=10.101,sample=KeySample(False,10.03,10.04),inflight=10.040)
    with pytest.raises(RuntimeError,match='in-flight'):k.sample_or_pending()


def test_lost_reply_progress_still_revokes_at80ms(monkeypatch):
    k=cached(monkeypatch,now=10.121)
    with pytest.raises(RuntimeError,match='progress expired80ms'):k.sample_or_pending()


def test_pending_input_cannot_complete_real_click(monkeypatch):
    from mecanum_wand_4ws8_control.operator_console import Console
    from mecanum_wand_4ws8_control.toggle_ui import ToggleOperatorModel
    w=Console.__new__(Console);w.closing=False;w.release_job=None;w._remote_release=(10.,10.)
    w.model=ToggleOperatorModel();w.model.key_down=True;w.model.click_pending=True;w.model.click_started=10.
    w.node=NS(heartbeat=lambda:pytest.fail('pending input tried to complete click'))
    assert not w._confirm_remote_release(10.05,None) and not w.model.session_requested
    assert w._confirm_remote_release(10.101,None)
    assert not w.model.click_pending and not w.model.session_requested
    assert not w._confirm_remote_release(10.11,KeySample(False,10.105,10.109))


def test_link_failure_revokes_and_closes_without_retrying_input():
    from mecanum_wand_4ws8_control.operator_console import Console
    w=Console.__new__(Console);w.closing=False;calls=[]
    w.key_state=AsyncX11KeyState.__new__(AsyncX11KeyState)
    w.key_state.sample_or_pending=lambda:(_ for _ in ()).throw(RuntimeError('lost remote fixture'))
    w.model=NS(revoke=lambda *a:calls.append('revoke'))
    w.node=NS(get_logger=lambda:NS(error=lambda *a:calls.append('log')))
    w.close=lambda:calls.append('close')
    remote_keys.RemoteKeysMixin.loop(w);assert calls==['revoke','log','close']


def gate():
    g=RotationGateNode.__new__(RotationGateNode);c=config();g.core=SourceFaultGate(c)
    g.pending=deque();g.processing_bound=False;g.bound_drained=0;g.selection_failure=None
    g.packet_cache=RetainedPackets();g.cars=OrderedDict()
    g.startup_frame_window=10.;g.startup_fresh_frame=False;g.startup_stale_zero_dropped=0
    g.startup_envelope_error_logged=False
    g.clocks=lambda:(10.25,100.25);g.logs=[];g.outputs=[]
    g.get_logger=lambda:NS(warn=g.logs.append);g.publish_output=g.outputs.append
    return g,Supervisor(c).frame(10.,100.)


def wire(f):
    return String(data=json.dumps(dict(frame=asdict(f),sign_digest=None,sign_stamp=None,car_stamp=None)))


def test_stale_virgin_zero_is_discarded_without_establishing_owner_or_fault():
    g,old=gate();g.core.fault='COMMAND_TIMEOUT';g.on_candidate(wire(old))
    assert g.startup_stale_zero_dropped==1 and g.core.fault=='COMMAND_TIMEOUT'
    assert g.core.frame is None and g.core.owner is None and g.core.last_sequence==-1
    fresh=replace(old,sequence=old.sequence+1,issued_steady=10.25,issued_ros=100.25)
    g.on_candidate(wire(fresh))
    assert g.startup_fresh_frame and g.core.frame==fresh and g.core.owner==fresh.source
    # The exception did NOT erase a gate fault or prime/enable an epoch.
    assert g.core.fault=='COMMAND_TIMEOUT' and not g.core.primed_epoch and g.outputs[-1]==ZERO


@pytest.mark.parametrize('case',['armed','enabled','nonzero','goal','hard_frame','real_fault',
    'native_fault','primed','blocked','used','angle','old_owner','fresh_seen','old_window','overflow','future','clock_skew'])
def test_startup_discard_exception_never_bypasses_real_fault_or_motion(case):
    from mecanum_wand_4ws8_control.core import Velocity,Pose
    g,f=gate()
    if case=='armed':f=replace(f,state='ARMED',epoch='arm')
    elif case=='enabled':f=replace(f,enabled=True,state='ACTIVE')
    elif case=='nonzero':f=replace(f,command=Velocity(.01,0.,0.))
    elif case=='goal':f=replace(f,goal=Pose(0.,0.,.1,0.))
    elif case=='hard_frame':f=replace(f,hard_fault=True)
    elif case=='real_fault':g.core.fault='COMMAND_ENVELOPE_INVALID'
    elif case=='native_fault':g.core.hard_fault=True;g.core.fault='ACTUATOR_FAULT'
    elif case=='primed':g.core.primed_epoch='armed'
    elif case=='blocked':g.core.blocked_epoch='past'
    elif case=='used':g.core.pilot.used=True
    elif case=='angle':g.core.angle_epoch='past'
    elif case=='old_owner':g.core.owner='someone_else'
    elif case=='fresh_seen':g.startup_fresh_frame=True
    elif case=='old_window':g.startup_frame_window=7.
    elif case=='overflow':g.startup_stale_zero_dropped=32
    elif case=='future':f=replace(f,issued_steady=11.,issued_ros=101.)
    else:f=replace(f,issued_ros=99.)
    assert not g.stale_virgin_zero(f,10.25,100.25)
    before=g.startup_stale_zero_dropped;g.on_candidate(wire(f))
    assert g.startup_stale_zero_dropped==before and g.core.output==ZERO
    assert not g.core.primed_epoch or g.core.primed_epoch=='armed'


def test_stale_command_after_first_fresh_frame_is_not_a_startup_exception():
    g,f=gate();fresh=replace(f,issued_steady=10.25,issued_ros=100.25)
    g.on_candidate(wire(fresh));assert g.startup_fresh_frame
    g.on_candidate(wire(replace(f,sequence=f.sequence+1)))
    assert g.core.fault=='COMMAND_ENVELOPE_INVALID' and g.startup_stale_zero_dropped==0
    assert any('CANDIDATE_ENVELOPE_REJECT_DIAGNOSTIC' in v for v in g.logs)
