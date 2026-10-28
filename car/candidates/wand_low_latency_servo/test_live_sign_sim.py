"""Read-only input adapter / isolated runtime checks; no ROS init or hardware."""
import ast
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped

PATH=Path(__file__).with_name('live_sign_sim.py')
spec=importlib.util.spec_from_file_location('servo_live_under_test',PATH)
live=importlib.util.module_from_spec(spec);spec.loader.exec_module(live)


def test_live_runtime_only_adds_authorized_real_sign_input_without_hardware():
    r=live.LiveRuntime()
    assert r.mode=='shadow' and r.ip=='' and r.source_domain==0
    assert r.config.follower_config().scale==1. and r.config.follower_config().filter_tau_s==0.
    assert r.config.arena_forward_max_m==r.config.arena_left_max_m==3.
    assert r.config.max_speed_mps==.2 and r.config.max_accel_mps2==.08
    assert r.expected['car_pose']=='safe_sim_plant' and r.expected['plant_feedback']=='safe_sim_plant'


def test_center_start_moves_only_software_car_not_fixed_fence_or_mapping():
    corner=live.LiveRuntime();center=live.LiveRuntime('center')
    assert center.config==corner.config and center.config.goal_polygon()==corner.config.goal_polygon()
    assert center.config.geometry().local(center.initial_pose.x,center.initial_pose.y)==pytest.approx((1.5,1.5))
    assert center.config.goal_inside(center.initial_pose) and center.config.inside(center.initial_pose)
    assert center.initial_pose.yaw==corner.initial_pose.yaw and center.config.follower_config().scale==1.
    assert center.ip=='' and center.mode=='shadow'


@pytest.mark.parametrize('bad',['domain','namespace','sim_time','topic','service'])
def test_live_guard_refuses_domain_or_remapping_changes(monkeypatch,bad):
    monkeypatch.setenv('ROS_DOMAIN_ID','114')
    n=SimpleNamespace(context=SimpleNamespace(get_domain_id=lambda:114),
        get_namespace=lambda:live.NAMESPACE,get_parameter=lambda _:SimpleNamespace(value=False),
        resolve_topic_name=lambda name:live.NAMESPACE+'/'+name,
        resolve_service_name=lambda name:live.NAMESPACE+'/'+name)
    live.LiveRuntime().guard(n)
    if bad=='domain':n.context.get_domain_id=lambda:0
    if bad=='namespace':n.get_namespace=lambda:'/car'
    if bad=='sim_time':n.get_parameter=lambda _:SimpleNamespace(value=True)
    if bad=='topic':n.resolve_topic_name=lambda name:'/car/cmd_vel'
    if bad=='service':n.resolve_service_name=lambda name:'/car/enable'
    with pytest.raises(RuntimeError):live.LiveRuntime().guard(n)


def test_domain_zero_reader_has_only_the_sign_subscription_and_no_motion_api():
    source=PATH.read_text();tree=ast.parse(source)
    reader=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SignReader')
    attrs={n.func.attr for n in ast.walk(reader) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
    assert 'create_subscription' in attrs
    assert not attrs.intersection({'create_publisher','create_client','create_service'})
    assert 'HardwareBridge' not in source and 'import socket' not in source and 'import serial' not in source


def message():
    m=PoseStamped();m.header.frame_id='map';m.header.stamp.sec=100
    m.pose.orientation.w=1.;m.pose.position.z=1.
    return m


def test_raw_header_is_preserved_and_replay_does_not_refresh():
    m=message()
    reader=SimpleNamespace(sample=None,last_stamp=-math.inf,rejected=0,received=0,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_010_000_000)))
    live.SignReader.receive(reader,m)
    assert reader.sample.header==m.header and reader.sample is not m and reader.received==1
    old=reader.sample
    live.SignReader.receive(reader,m)
    assert reader.sample is old and reader.received==1
    m.header.stamp.sec=99
    live.SignReader.receive(reader,m)
    assert reader.sample is None and reader.rejected==1


def test_relay_does_not_republish_old_pose_or_create_car_pose_or_lease():
    m=message()
    reader=SimpleNamespace(sample=m,last_stamp=100.,received=1,rejected=0,audit=lambda:[])
    relay=SimpleNamespace(reader=reader,source_executor=SimpleNamespace(spin_once=lambda **k:None),
        forwarded=-math.inf,audit_at=-math.inf,conflicts=[],ready_at=0.,
        pose_pub=Mock(),mode_pub=Mock(),status_pub=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_010_000_000)))
    live.SignRelay.tick(relay)
    assert relay.pose_pub.publish.call_args.args[0] is m
    live.SignRelay.tick(relay)
    assert relay.pose_pub.publish.call_count==1
    assert not hasattr(relay,'car_pub') and not hasattr(relay,'lease_pub')


def test_relay_keeps_a_strong_executor_reference_without_shadowing_node_executor():
    source=PATH.read_text();tree=ast.parse(source)
    relay=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SignRelay')
    text=ast.get_source_segment(source,relay)
    assert 'self.source_executor=SingleThreadedExecutor' in text
    assert 'self.executor' not in text


def test_trace_gate_still_uses_the_real_final_gate_and_signed_output():
    assert issubclass(live.TraceGate,live.GateNode)
    source=ast.parse(PATH.read_text())
    gate=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='TraceGate')
    text=ast.get_source_segment(PATH.read_text(),gate)
    assert 'super().publish_output(output)' in text
    assert 'core.tick' not in text and 'core.fault =' not in text
