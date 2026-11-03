"""Explicit 20s timing, one-shot and unchanged spatial/health safeguards."""
from dataclasses import replace
import json
import pytest
from test_origin_capture import prepared,adjust,enable,move_after_enable
from test_pilot_bounds import GENERATED,ROOT
from mecanum_wand_4ws8_control.pilot_core import POLICY
from mecanum_wand_4ws8_control.pilot_ui import PilotOperatorModel,reason_text
from mecanum_wand_4ws8_control.safety_core import ZERO


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import socket,rclpy
    def forbidden(*a,**k):pytest.fail('offline test: hardware/ROS prohibited')
    monkeypatch.setattr(socket,'socket',forbidden);monkeypatch.setattr(rclpy,'init',forbidden)


def test_twenty_seconds_starts_at_enable_not_prepare_or_previous_five_second_limit():
    h=prepared();h.run(23.);adjust(h,.6,.4)
    assert not h.sup.pilot.used and h.sup.pilot.started is None
    enable(h);t0=h.now
    h.run(5.2);assert h.sup.state=='ACTIVE' and h.sup.pilot_status(h.now)['remaining_s']>14.7
    h.run(14.6);assert h.sup.state=='ACTIVE' and h.sup.pilot_status(h.now)['remaining_s']<=.21
    h.run(.24);assert h.output==ZERO and h.sup.state!='ACTIVE'
    assert h.sup.pilot.started==h.gate.pilot.started==t0
    assert 'PILOT_TIME_LIMIT_20S' in h.sup.reason or 'PILOT_TIME_LIMIT_20S' in h.gate.fault
    h.held=False;h.run(2.)
    assert not h.sup.request_start(h.now,h.ros)[0] and not h.sup.request_recover(h.now,h.ros)[0]


@pytest.mark.parametrize('which',['supervisor','gate'])
def test_each_independent_guard_expires_at_twenty_seconds(which):
    h=prepared();enable(h)
    core=h.sup if which=='supervisor' else h.gate;m=core.pilot
    assert m.active(m.epoch,core.inputs,m.car,m.started+5.1)==''
    assert m.active(m.epoch,core.inputs,m.car,m.started+19.999)==''
    assert m.active(m.epoch,core.inputs,m.car,m.started+20.)=='PILOT_TIME_LIMIT_20S'


def test_twenty_second_backend_metadata_unlocks_the_actual_ui_model():
    h=prepared();m=PilotOperatorModel();m.focused=True
    status=dict(state='STANDBY',ready_to_start=True,pilot=h.sup.pilot_status(h.now))
    m.receive_status(status,10.);m.receive_gate(dict(fault='',hard_fault=False),10.)
    assert m.pilot_valid() and m.can_start(10.)
    m.status['state']='ARMED';m.status['gate_ready']=True
    assert '最长20秒' in m.instruction(10.)
    assert '20秒' in reason_text('FINAL_GATE: PILOT_TIME_LIMIT_20S')
    m.status['pilot']['max_active_s']=5.
    assert not m.pilot_valid() and not m.can_start(10.)


def test_duration_change_does_not_relax_geometry_limits_or_watchdogs():
    h=prepared();c=h.c
    assert (c.max_speed_mps,c.max_accel_mps2)==(.70,.80)
    assert (c.arena_forward_max_m,c.arena_left_max_m,c.arena_buffer_m)==(3.,3.,.5)
    assert (c.command_timeout_s,c.lease_timeout_s,c.feedback_timeout_s)==(.10,.15,.20)
    assert (POLICY['max_hand_xy_m'],POLICY['max_goal_xy_m'],POLICY['max_car_xy_m'],POLICY['max_car_path_m'])==(.15,.15,.20,.25)
    old=ROOT/'docs/wand_servo_4ws8_2026-10-26/ground_origin_sync_v3/ready/runtime/src'
    for package,files in [('mecanum_wand_4ws8_control',('safety_core.py','core.py','continuous_servo.py','safety_nodes.py')),
                          ('mecanum_wand_4ws8_hardware',('authorization.py','nodes.py','stage0_session.py','stage0_contract.py'))]:
        for file in files:assert (GENERATED/package/package/file).read_bytes()==(old/package/package/file).read_bytes()


@pytest.mark.parametrize('cause',['release','occlusion','feedback','hand_limit'])
def test_spatial_and_stop_guards_still_end_the_twenty_second_observation_early(cause):
    h=prepared();anchor=enable(h);move_after_enable(h,anchor)
    assert h.output!=ZERO
    if cause=='release':h.held=False
    elif cause=='occlusion':h.pose_on['wand']=False
    elif cause=='feedback':h.feedback_on=False
    else:h.hand=replace(anchor,x=anchor.x+.151)
    h.run(.4)
    assert h.output==ZERO and h.sup.state!='ACTIVE'


def test_all_console_and_gate_duration_labels_are_twenty_seconds():
    for name in ('pilot_core.py','pilot_ui.py','operator_console.py'):
        s=(GENERATED/'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control'/name).read_text()
        assert 'PILOT_TIME_LIMIT_5S' not in s and '最长5秒' not in s and '单次≤5秒' not in s
