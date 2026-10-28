"""Offline MCU-C/real-host regression. Physical sockets and ROS init denied."""
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
OUT=Path(os.environ['FEEDBACK_OUTPUT'])


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


parent=load('feedback_guard_reused',ROOT/'candidates/wand_feedback_guard/test_feedback.py')
native=load('feedback_300_native_harness',OUT/'native_harness.py')
parent.native=native;parent.ORIGINAL_ACTUAL=native.actual_bridge
offline_only=parent.offline_only
excluded={'test_real_214ms_feedback_gap_still_stops_and_sends_exact_disable',
    'test_queued_replay_cannot_prevent_real_timeout',
    'test_only_reviewed_transport_files_changed_all_motion_ui_and_parents_preserved'}
for name in dir(parent):
    if name.startswith('test_') and name not in excluded:globals()['test_reused_'+name[5:]]=getattr(parent,name)


def at_age(monkeypatch,age):
    h=parent.armed(monkeypatch)
    h.socket.allow_telemetry=False;h.socket.queue.clear()
    h.clock.now=min(h.b.sock.clock.accepted_at.values())+age
    h.signed('ACTIVE','regression-epoch',True)
    return h


@pytest.mark.parametrize('age',[.201,.214,.250,.299])
def test_under_300ms_is_still_fresh_in_actual_bridge(monkeypatch,age):
    h=at_age(monkeypatch,age)
    assert h.b.link_receive_timeout_s==.3
    assert h.b.feedback_is_fresh(h.clock.now) and not h.b.policy.fault
    assert h.b.motion_gate_requested and h.b.safety_enabled
    assert not h.b.sock.clock.fault


@pytest.mark.parametrize('age',[.301,.350,.600])
def test_over_300ms_stops_exact_disable_no_automatic_restart(monkeypatch,age):
    from mecanum_vicon_control.transport_protocol import MessageType as M
    h=at_age(monkeypatch,age)
    assert h.b.policy.fault=='MCU_FEEDBACK_TIMEOUT'
    assert not h.b.safety_enabled and not h.b.motion_gate_requested
    controls=[f for _,f,_ in h.socket.sent if f.message_type==M.CONTROL_ENABLE]
    assert controls[-1].payload==b'\0'*4
    count=sum(f.message_type==M.HELLO for _,f,_ in h.socket.sent)
    h.socket.allow_telemetry=True;h.run(.4)
    assert not h.b.safety_enabled and h.b.policy.fault
    assert sum(f.message_type==M.HELLO for _,f,_ in h.socket.sent)==count


def test_replayed_queue_never_revives_source_age(monkeypatch):
    from mecanum_vicon_control.transport_protocol import MessageType as M
    h=parent.armed(monkeypatch);raw=h.socket.oracle.packet(M.WHEEL_TELEMETRY)
    h.socket.allow_telemetry=False;h.socket.queue.clear()
    h.clock.now=min(h.b.sock.clock.accepted_at.values())+.301
    h.socket.queue.extend([(h.clock.now,raw)]*64)
    h.signed('ACTIVE','regression-epoch',True)
    assert h.b.policy.fault=='MCU_FEEDBACK_TIMEOUT' and not h.b.safety_enabled


@pytest.mark.parametrize('kind_name',['WHEEL_TELEMETRY','LINK_STATUS'])
def test_either_native_stream_expiring_is_sufficient(monkeypatch,kind_name):
    from mecanum_vicon_control.transport_protocol import MessageType as M
    h=parent.armed(monkeypatch);kind=M[kind_name]
    h.b.sock.clock.accepted_at[kind]=h.clock.now-.301
    # Mirrored caches are intentionally stale too; no packet is invented.
    h.socket.allow_telemetry=False;h.socket.queue.clear()
    h.signed('ACTIVE','regression-epoch',True)
    assert not h.b.feedback_is_fresh(h.clock.now) and not h.b.safety_enabled


@pytest.mark.parametrize('age,valid',[(.214,True),(.250,True),(.299,True),(.301,False)])
def test_clock_age_acceptance_is_not_rebased_to_receipt(age,valid):
    from mecanum_wand_4ws8_hardware.startup_clock import HandshakeSampleClock
    from mecanum_vicon_control.transport_protocol import MessageType as M
    c=HandshakeSampleClock();native.qualify(c);anchor=c.local_baseline
    result=c.observe(M.WHEEL_TELEMETRY,1800,10.8+age)
    assert (result is not None)==valid
    assert c.local_baseline==anchor
    if valid:
        assert result==pytest.approx(10.8)
        assert c.observe(M.WHEEL_TELEMETRY,1800,10.8+age+.001) is None


def config():
    from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig
    return TrackingSafetyConfig(**json.loads((OUT/'runtime/src/mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())['safety'])


def test_supervisor_and_gate_have_same_300ms_only():
    from mecanum_wand_4ws8_control.rotation_core import RotationSupervisor,RotationFinalGate
    from mecanum_wand_4ws8_control.safety_core import Inputs
    c=config()
    assert (c.feedback_timeout_s,c.pose_timeout_s,c.command_timeout_s,c.lease_timeout_s,c.max_session_s)==(.3,.2,.1,.15,120.)
    assert (c.max_speed_mps,c.max_accel_mps2,c.max_yaw_rate_radps,c.max_yaw_accel_radps2)==(.7,.8,.1,.4)
    for core in (RotationSupervisor(c),RotationFinalGate(c)):
        for age,expected in ((.250,True),(.301,False)):
            i=core.inputs
            assert i.observe_feedback(sequence=int(age*1000),issued_steady=10.,issued_ros=100.,
                steady=10.+age,ros=100.+age,output_zero=True,enabled=False,speed=0.,yaw_rate=0.) is expected
            assert i.feedback_fresh(10.+age,100.+age) is expected


@pytest.mark.parametrize('value',[.2,.301,.5,float('nan')])
def test_candidate_rejects_other_feedback_timeouts(value):
    from dataclasses import replace
    with pytest.raises(ValueError):replace(config(),feedback_timeout_s=value)


def test_all_parents_untouched_and_exact_change_inventory():
    import hashlib
    m=json.loads((OUT/'build_manifest.json').read_text())
    for path,digest in m['protected_input_hashes'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
    changes=json.loads((OUT/'PRIVATE_CHANGE_INVENTORY.json').read_text())
    expected={'feedback_policy.py','safety_core.py','servo_config.py','startup_clock.py','nodes.py','wand_4ws8_staged.launch.py'}
    assert {Path(p).name for p in changes}==expected
    assert m['feedback_protection_ms']==300 and not m['firmware_changed']
    assert m['parent_entry_changed'] is False


def test_old_instance_cannot_be_silently_reused():
    s=(OUT/'oneclick_app.py').read_text()
    assert 'SHOW_FEEDBACK_300MS_V1' in s and "b'FEEDBACK_300MS_V1'" in s
    assert 'SHOW_FEEDBACK_GUARD_V1' not in s
