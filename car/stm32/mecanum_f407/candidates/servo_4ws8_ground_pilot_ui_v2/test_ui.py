"""Private UI lifecycle and neutral placement regressions, no live endpoints."""
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import sys
import pytest

GENERATED=Path(os.environ['STAGED_GENERATED'])
for name in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
    sys.path.insert(0,str(GENERATED/name))
from mecanum_wand_4ws8_control.pilot_core import PilotSupervisor,PilotFinalGate,start_issue,POLICY
from mecanum_wand_4ws8_control.pilot_ui import PilotOperatorModel,real_instruction,reason_text
from mecanum_wand_4ws8_control.safety_core import Inputs,ZERO
from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig
from mecanum_wand_4ws8_control.core import Pose


@pytest.fixture(autouse=True)
def no_io(monkeypatch):
    import socket
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('offline: no sockets'))


def config():
    return StagedSafetyConfig(**json.loads((GENERATED/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())['safety'])


def model(state='STANDBY',used=False,blocked='',stopped=True):
    m=PilotOperatorModel();m.focused=True
    m.receive_status(dict(state=state,ready_to_start=True,stop_confirmed=stopped,gate_ready=state=='ARMED',
        pilot=dict(used=used,max_active_s=5.0,remaining_s=4.2,prepare_blocked=blocked)),10.)
    m.receive_gate(dict(fault='',hard_fault=False),10.)
    return m


def test_first_prepare_and_ready_held_edge_work():
    m=model();assert m.can_start(10.)
    assert '点击准备' in m.instruction(10.)
    m.status['state']='ARMED';m.status['gate_ready']=False
    assert not m.press(10.) and not m.held
    m.release(10.01);m.status['gate_ready']=True
    assert m.press(10.02) and m.held


@pytest.mark.parametrize('state',['PAUSED','FAULT','STOPPING','STOP_UNCONFIRMED','STANDBY'])
def test_used_session_never_offers_prepare_recover_or_generic_recovery_hint(state):
    m=model(state,True)
    assert not m.can_start(10.) and not m.can_recover(10.)
    text=real_instruction(m,10.,True)
    assert '不能再次准备' in text and '点击确认' not in text
    assert not m.press(10.) and not m.held


def test_waits_for_actual_standstill_after_consumed():
    m=model('STOPPING',True,stopped=False)
    assert '等待停车确认' in m.instruction(10.)
    assert not m.can_recover(10.)


def test_active_countdown_does_not_release_current_authorized_hold():
    m=model('ACTIVE',True);m.held=m.key_down=True
    assert not m.pilot_done and '4.2秒' in m.instruction(10.)
    assert m.tick(10.) and m.held
    m.release(10.01);assert not m.held and m.pilot_used


def test_consumed_latch_cannot_clear_by_acknowledge_or_late_old_status():
    m=model('FAULT',True)
    value=dict(m.status,state='PAUSED',pilot=dict(m.status['pilot'],used=False))
    m.receive_status(value,10.1);m.receive_gate(dict(fault=''),10.1)
    assert m.pilot_used and not m.can_start(10.1) and not m.can_recover(10.1)


def test_missing_pilot_metadata_and_stale_feedback_fail_closed():
    m=model();m.status.pop('pilot')
    assert not m.can_start(10.) and '专用' in m.instruction(10.)
    m=model();assert not m.can_start(10.3) and not m.press(10.3)


def test_prepare_block_reason_is_visible_not_just_silent_disabled_button():
    msg='起点在绿色目标区外，请人工向内挪几厘米'
    m=model(blocked=msg)
    assert not m.can_start(10.) and m.instruction(10.)==msg


@pytest.mark.parametrize('local',[(-.016,-.004),(-.01,.05),(.05,-.01),(3.01,.05),(.05,3.01)])
def test_outside_nominal_start_blocked_in_supervisor_and_independent_gate(local):
    c=config();car=Pose(*c.geometry().world(*local),c.ground_z_m,c.arena_heading_rad)
    sup=PilotSupervisor(c);gate=PilotFinalGate(c)
    for core in (sup,gate):
        core.inputs.poses={'car':dict(pose=car),'wand':dict(pose=Pose(1,1,1))}
    assert start_issue(c,sup.inputs).startswith('PILOT_START_REQUIRES_INTERIOR')
    ok,msg=sup.request_start(10.,100.);assert not ok and '人工' in msg
    assert gate.pilot.arm('epoch',gate.inputs).startswith('PILOT_START_REQUIRES_INTERIOR')
    assert sup.command==gate.output==ZERO


@pytest.mark.parametrize('local',[(.05,.05),(.10,.10),(1.5,1.5),(2.9,2.9)])
def test_inside_start_projects_to_same_point_not_automatic_recentering(local):
    c=config();car=Pose(*c.geometry().world(*local),c.ground_z_m,c.arena_heading_rad)
    inputs=Inputs(c);inputs.poses={'car':dict(pose=car),'wand':dict(pose=Pose(1,1,1))}
    assert start_issue(c,inputs)==''
    goal=c.project_goal(car,car,c.max_goal_radius_m)
    assert math.hypot(goal.x-car.x,goal.y-car.y)<1e-10
    assert PilotFinalGate(c).pilot.arm('epoch',inputs)==''


def test_neutral_origin_holds_zero_then_one_to_one_short_move():
    from test_pilot_bounds import harness
    h=harness.Harness(config(),origin_local=(.05,.05));h.start();h.run(.3)
    assert h.output==ZERO
    car=h.plant.pose
    for i in range(1,11):h.set_hand(.005*i,0.);h.step()
    assert h.output!=ZERO and h.sup.follower.mapping.scale==1.
    goal=h.sup.follower.mapped_goal
    assert math.hypot(goal.x-car.x,goal.y-car.y)==pytest.approx(.05,abs=1e-8)


def test_normal_timeout_label_and_policy_are_consistent():
    assert POLICY['max_active_s']==5. and POLICY['engagements_per_process']==1
    assert '正常限时' in reason_text('FINAL_GATE: PILOT_TIME_LIMIT_5S')
    source=(GENERATED/'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/operator_console.py').read_text()
    assert '本轮最多5秒、一次使能' in source and '单次≤5秒' in source
    assert '无额外位移限位' not in source and 'and not self.model.pilot_done' in source


@pytest.mark.parametrize('from_gate',[False,True])
def test_true_hard_fault_is_not_hidden_by_normal_pilot_finished_hint(from_gate):
    m=model('FAULT',True)
    if from_gate:m.gate.update(hard_fault=True,fault='ACTUATOR_FAULT')
    else:m.status.update(hard_fault=True,reason='ACTUATOR_FAULT')
    assert '保护已锁存' in m.instruction(10.) and '不能通过确认' in m.instruction(10.)
    assert not m.can_start(10.) and not m.can_recover(10.)


def test_actual_tk_widgets_follow_consumed_status_without_motion(monkeypatch):
    import tkinter as tk
    import time
    from types import SimpleNamespace
    from mecanum_wand_4ws8_control.operator_console import Console
    now=time.monotonic();m=model()
    m.status_at=m.gate_at=now
    c=config();car=Pose(*c.geometry().world(.05,.05),c.ground_z_m,c.arena_heading_rad)
    fake=SimpleNamespace(runtime=SimpleNamespace(config=c,wand_reference_margin_m=.5),
        input_kind='hardware',trace=[],car=car,raw_wand=Pose(1.,1.,1.),
        request=lambda *a:pytest.fail('GUI smoke never sends service requests'),input_hint=lambda:'')
    root=tk.Tk();root.withdraw();app=Console(root,fake,m)
    try:
        app.draw(now)
        assert str(app.buttons['start']['state'])=='normal'
        m.receive_status(dict(m.status,state='PAUSED',pilot=dict(m.status['pilot'],used=True)),now)
        app.draw(now)
        assert str(app.buttons['start']['state'])=='disabled'
        assert str(app.buttons['recover']['state'])=='disabled'
        assert str(app.buttons['acknowledge']['state'])=='disabled'
        assert '不能再次准备' in app.step_text.get()
        root.update_idletasks()
    finally:
        if app.loop_job is not None:root.after_cancel(app.loop_job)
        if app.key_state is not None:app.key_state.close()
        root.destroy()
