"""Real callbacks/core with deterministic software scheduling, no ROS init."""
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from std_msgs.msg import String
from mecanum_wand_4ws8_hardware.rotation_nodes import RotationGateNode
from mecanum_wand_4ws8_hardware.bound_rotation import SourceFaultGate
from mecanum_wand_4ws8_control.bound_history import RetainedPackets,HISTORY_QOS
from mecanum_wand_4ws8_control.rotation_source import canonical
from mecanum_wand_4ws8_control.safety_core import ZERO
from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');OUT=Path(os.environ['FEEDBACK_OUTPUT'])
spec=importlib.util.spec_from_file_location('bound_software_fixture',ROOT/'tools/diagnose_ros_bound_source_offline_20261027.py')
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)


class CallbackGate(RotationGateNode):
    def __new__(cls):
        g=super().__new__(cls)
        g.processing_bound=False;g.bound_drained=0;g.selection_failure=None
        return g


fixture.RotationGateNode=CallbackGate
fixture.RotationFinalGate=SourceFaultGate
fixture.PacketCache=RetainedPackets


def harness():
    m=json.loads((OUT/'build_manifest.json').read_text())
    c=TrackingSafetyConfig(**json.loads(Path(m['draft_profiles'][4]).read_text())['safety'])
    return fixture.Harness(c)


@pytest.mark.parametrize('which',['car','sign','both'])
def test_late_selected_source_after_newer_source_is_retained_and_drains_fifo(which):
    h=harness();base=h.started;before=h.g.bound_drained
    h.now=base+.010;v=h.source(deliver_sign=which=='car',deliver_car=which=='sign')
    h.refresh();h.candidate(v);assert len(h.g.pending)==1
    h.now=base+.020;new=h.source();h.refresh();h.candidate(new)
    assert len(h.g.pending)==2
    h.now=base+.035;h.refresh()
    if which in ('sign','both'):h.g.on_packet(String(data=canonical(v['packet'].payload())))
    if which in ('car','both'):h.g.on_pose('car',v['car_message'])
    assert not h.g.pending and not h.g.core.fault
    assert h.g.core.last_sequence==h.seq and h.g.bound_drained-before==2
    assert h.g.packet_cache.latest(h.now,h.ros).stamp==new['sign_stamp']
    assert h.g.core.inputs.poses['car']['source_ros']==new['car_stamp']


def test_each_batched_frame_is_validated_including_first_neutral_active():
    h=harness();h.g.core=SourceFaultGate(h.g.core.c)
    # Re-establish a genuine released/standstill ARMED baseline on this core.
    for _ in range(60):
        h.now+=.02;v=h.source();h.refresh(False);h.candidate(v,False);h.tick()
    assert h.g.core.primed_epoch and not h.g.core.pilot.used
    h.now+=.01;first=h.source(deliver_car=False);h.refresh();h.candidate(first)
    h.now+=.01;second=h.source();h.refresh();h.candidate(second)
    assert not h.g.core.pilot.used and len(h.g.pending)==2
    h.now+=.01;h.g.on_pose('car',first['car_message'])
    assert h.g.core.pilot.used and h.g.core.angle_epoch and not h.g.core.fault
    assert not h.g.pending and h.g.core.output==ZERO


@pytest.mark.parametrize('which',['car','sign','digest'])
def test_missing_exact_source_never_falls_back_and_keeps_first_failure(which):
    h=harness();base=h.started
    h.now=base+.01;v=h.source(deliver_sign=which!='sign',deliver_car=which!='car')
    if which=='digest':v['sign_digest']='0'*64
    h.refresh();h.candidate(v)
    h.now=base+.03;new=h.source();h.refresh();h.candidate(new)
    assert len(h.g.pending)==2
    h.now=base+.055;h.source();h.refresh();h.tick()
    assert h.g.core.fault=='ROTATION_SELECTED_SOURCE_INVALID'
    detail=h.g.selection_failure
    assert detail['reason']=='SELECTED_SOURCE_MISSING'
    assert detail['missing_car']==(which=='car')
    assert detail['missing_sign']==(which!='car')
    assert not h.g.pending and h.outputs[-1]==ZERO
    h.now=base+.12;h.source();h.refresh();h.tick()
    assert h.g.core.fault=='ROTATION_SELECTED_SOURCE_INVALID'
    # A real native/hard fault still overrides the descriptive source fault.
    h.g.core.trip('ACTUATOR_FAULT',hard=True)
    assert h.g.core.hard_fault and h.g.core.fault=='ACTUATOR_FAULT'


def test_duplicate_sources_never_refresh_receipt_age():
    h=harness();h.now+=.01;v=h.source()
    old=h.g.cars[v['car_stamp']][2];packet=h.g.packet_cache.exact(v['sign_digest'],v['sign_stamp'],h.now,h.ros)
    h.now+=.05;h.g.on_pose('car',v['car_message']);h.g.on_packet(String(data=canonical(v['packet'].payload())))
    assert h.g.cars[v['car_stamp']][2]==old
    assert h.g.packet_cache.exact(v['sign_digest'],v['sign_stamp'],h.now,h.ros).received==packet.received


def test_retention_is_bounded_by_original_age_and_count():
    h=harness();h.now+=.01;v=h.source();stamp=v['car_stamp'];digest=v['sign_digest']
    for _ in range(100):h.now+=.002;h.source()
    assert len(h.g.cars)<=64 and len(h.g.packet_cache.packets)<=64
    assert stamp not in h.g.cars and digest not in h.g.packet_cache.packets


@pytest.mark.parametrize('source',['car','sign'])
def test_same_stamp_conflict_does_not_replace_cached_source(source):
    h=harness();h.now+=.01;v=h.source()
    if source=='car':
        msg=v['car_message'];msg.pose.position.x+=.01;h.g.on_pose('car',msg)
        assert 'CAR_DECODE_INVALID' in h.g.core.inputs.events
    else:
        from dataclasses import replace
        p=v['packet'];raw=replace(p.samples[0],received=p.received+.001)
        p=type(p)((raw,*p.samples[1:]));h.g.on_packet(String(data=canonical(p.payload())))
        assert 'ROTATION_RAW_SOURCE_INVALID' in h.g.core.inputs.events


def test_stop_bypasses_unknown_source_queue():
    from dataclasses import replace,asdict
    h=harness();h.now+=.01;v=h.source(deliver_car=False);h.refresh();h.candidate(v)
    assert h.g.pending
    f=replace(h.g.core.frame,sequence=h.seq+1,issued_steady=h.now,issued_ros=h.ros,
        enabled=False,state='STOPPING',command=ZERO,goal=None)
    h.g.on_candidate(String(data=canonical(dict(frame=asdict(f),sign_digest=None,sign_stamp=None,car_stamp=None))))
    assert not h.g.pending and h.g.core.frame==f and h.outputs[-1]==ZERO


def test_unknown_head_and_queue_overflow_still_fail_closed():
    h=harness();h.now+=.01;v=h.source(deliver_car=False);h.refresh()
    for _ in range(5):h.candidate(v)
    assert not h.g.pending and h.g.core.fault=='ROTATION_BOUND_COMMAND_INVALID'
    assert h.outputs[-1]==ZERO


def test_internal_history_qos_is_reliable_32_not_global_sensor_change():
    from mecanum_wand_4ws8_control.safety_nodes import SENSOR,LATEST
    assert HISTORY_QOS.depth==32 and HISTORY_QOS.reliability.name=='RELIABLE'
    assert SENSOR.depth==1 and SENSOR.reliability.name=='BEST_EFFORT' and LATEST.depth==1


def test_expired_remote_sample_revokes_and_closes_without_a_second_read():
    from mecanum_wand_4ws8_control.operator_console import Console
    from mecanum_wand_4ws8_control.async_x11 import AsyncX11KeyState
    w=Console.__new__(Console);w.closing=False;calls=[]
    w.key_state=AsyncX11KeyState.__new__(AsyncX11KeyState)
    w.key_state.snapshot=lambda:(_ for _ in ()).throw(RuntimeError('expired fixture'))
    w.model=NS(revoke=lambda *a:calls.append('revoke'))
    w.node=NS(get_logger=lambda:NS(error=lambda *a:calls.append('log')))
    w.server_space_down=lambda:(_ for _ in ()).throw(AssertionError('Must not re-read input'))
    w.close=lambda:calls.append('close')
    # Invoke the remote loop directly: the outer auto-prepare mixin is not a
    # part of this exception-path unit fixture.
    from mecanum_wand_4ws8_control.remote_keys import RemoteKeysMixin
    RemoteKeysMixin.loop(w)
    assert calls==['revoke','log','close']


def test_per_frame_validation_cannot_hide_a_stalled_output_timer():
    h=harness();timer_at=h.g.core.clock.previous
    h.now+=.030;v=h.source();h.refresh();h.candidate(v)
    assert not h.g.core.fault and h.g.core.clock.previous==timer_at
    # Another live source/command callback is NOT a publication timer tick.
    h.now+=.075;v=h.source();h.refresh();h.candidate(v)
    assert h.g.core.hard_fault and h.g.core.fault=='CONTROL_TIMER_GAP'
    assert h.g.core.output==ZERO and h.outputs[-1]==ZERO


@pytest.mark.parametrize('field,value',[
    ('sign_digest',[]),('sign_digest',{'invalid':float('nan')}),
    ('sign_stamp',float('nan')),('sign_stamp',float('inf')),('car_stamp',[])])
def test_invalid_bound_identifiers_cannot_crash_fault_diagnostics(field,value):
    from dataclasses import asdict,replace
    h=harness();h.now+=.01;v=h.source();h.refresh()
    f=replace(h.g.core.frame,sequence=h.seq+1,issued_steady=h.now,issued_ros=h.ros)
    data=dict(frame=asdict(f),**{k:v[k] for k in ('sign_digest','sign_stamp','car_stamp')})
    data[field]=value
    # Python's ROS JSON decoder can encounter NaN despite the producer's
    # canonical serializer forbidding it: exercise the actual wire callback.
    h.g.on_candidate(String(data=json.dumps(data)))
    h.now+=.045;h.source();h.refresh();h.tick()
    assert h.g.core.fault=='ROTATION_SELECTED_SOURCE_INVALID' and h.outputs[-1]==ZERO
    assert h.g.selection_failure is not None
    canonical(h.g.selection_failure)  # finite, bounded, valid diagnostic JSON
