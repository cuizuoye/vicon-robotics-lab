"""No ROS init: isolated runtime/ROS constructor and signed channel checks."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import matching_sim as m
from matching_wire import MatchingOutputSigner, MatchingOutputVerifier
from mecanum_wand_servo_control.core import Velocity
from mecanum_wand_servo_control.safety_wire import OutputSigner, OutputVerifier

PATH=m.WORKSPACE/'docs/wand_servo_matching_sim_2026-10-23/live_runtime_v1/live_matching_sim.py'
spec=importlib.util.spec_from_file_location('matching_live_under_test',PATH)
live=importlib.util.module_from_spec(spec)
spec.loader.exec_module(live)
KEY='ab'*32


def test_exact_sim_runtime_and_original_runtime_still_limited():
    runtime=live.LiveRuntime('center')
    assert runtime.mode=='shadow' and runtime.ip=='' and runtime.source_domain==0
    assert runtime.config.max_speed_mps==.70 and runtime.config.max_accel_mps2==2.
    assert runtime.config.follower_config().scale==1.
    assert runtime.config.geometry().local(runtime.initial_pose.x,runtime.initial_pose.y)==pytest.approx((1.5,1.5))
    assert runtime.config.goal_polygon()==m.accepted_config().goal_polygon()
    assert m.accepted_config().max_speed_mps==.2
    with pytest.raises(ValueError):replace(m.accepted_config(),max_speed_mps=.7)
    with pytest.raises(TypeError):live.LiveRuntime(mode='hardware')


@pytest.mark.parametrize('bad',['domain','namespace','simtime','topic','service'])
def test_fail_closed_scope(monkeypatch,bad):
    monkeypatch.setenv('ROS_DOMAIN_ID','115')
    n=SimpleNamespace(context=SimpleNamespace(get_domain_id=lambda:115),get_namespace=lambda:live.NAMESPACE,
        get_parameter=lambda _:SimpleNamespace(value=False),
        resolve_topic_name=lambda name:live.NAMESPACE+'/'+name,
        resolve_service_name=lambda name:live.NAMESPACE+'/'+name)
    runtime=live.LiveRuntime('center')
    runtime.guard(n)
    if bad=='domain':n.context.get_domain_id=lambda:0
    if bad=='namespace':n.get_namespace=lambda:'/car'
    if bad=='simtime':n.get_parameter=lambda _:SimpleNamespace(value=True)
    if bad=='topic':n.resolve_topic_name=lambda _: '/car/cmd_vel'
    if bad=='service':n.resolve_service_name=lambda _: '/car/enable'
    with pytest.raises(RuntimeError):runtime.guard(n)


def test_ros_config_reconstruction_keeps_all_nonexperimental_guards(monkeypatch):
    from mecanum_wand_servo_control.safety_nodes import SafetyNode,Node
    from mecanum_wand_servo_control.safety_core import Supervisor,FinalGate
    params={}
    monkeypatch.setattr(Node,'__init__',lambda *a,**k:None)
    monkeypatch.setattr(Node,'declare_parameter',lambda self,n,v:params.setdefault(n,v))
    monkeypatch.setattr(Node,'get_parameter',lambda self,n:SimpleNamespace(value=params[n]))
    monkeypatch.setattr(Node,'create_timer',lambda *a,**k:None)
    monkeypatch.setattr(Node,'create_subscription',lambda *a,**k:None)
    runtime=live.LiveRuntime('center')
    stub=SimpleNamespace(namespace=runtime.namespace,world=runtime.world,config=runtime.config,
                         guard=lambda node:None,mode='shadow')
    for core in (Supervisor,FinalGate):
        n=object.__new__(SafetyNode)
        SafetyNode.__init__(n,'constructor_test',core,stub)
        assert n.core.c==runtime.config
        assert n.core.c.pose_timeout_s==.20
        assert n.core.c.follower_config().reference_lead_m==.20
        assert n.core.c.follower_config().scale==1.


def channel():
    signer=MatchingOutputSigner(KEY)
    verifier=MatchingOutputVerifier(KEY)
    verifier.decode(signer.encode(m.ZERO,10.,100.),10.,100.)
    return signer,verifier


def test_new_scoped_signed_channel_permits_diagonal_only_within_vector_cap():
    s,v=channel()
    cmd=Velocity(.6,.3,.1)
    assert v.decode(s.encode(cmd,10.02,100.02),10.02,100.02)==cmd
    with pytest.raises(ValueError,match='hard limit'):
        v.decode(s.encode(Velocity(.7,.7),10.04,100.04),10.04,100.04)


@pytest.mark.parametrize('kind',['signature','replay','expired','future','owner','scope','initial'])
def test_signatures_owner_freshness_replay_and_zero_baseline_retained(kind):
    s,v=channel()
    text=s.encode(Velocity(.4),10.02,100.02)
    now=(10.02,100.02)
    if kind=='signature':
        data=json.loads(text);data['payload']['command']['vx']=.5;text=json.dumps(data)
    elif kind=='replay':v.decode(text,*now)
    elif kind=='expired':now=(10.2,100.2)
    elif kind=='future':now=(10.,100.)
    elif kind=='owner':text=MatchingOutputSigner(KEY).encode(m.ZERO,*now)
    elif kind=='scope':text=OutputSigner(KEY).encode(m.ZERO,*now)
    elif kind=='initial':v=MatchingOutputVerifier(KEY)
    with pytest.raises(ValueError):v.decode(text,*now)


def test_old_verifier_unmodified_and_channels_cannot_cross():
    old_s,old_v=OutputSigner(KEY),OutputVerifier(KEY)
    old_v.decode(old_s.encode(m.ZERO,10.,100.),10.,100.)
    with pytest.raises(ValueError):old_v.decode(old_s.encode(Velocity(.21),10.02,100.02),10.02,100.02)
    with pytest.raises(ValueError):OutputVerifier(KEY).decode(MatchingOutputSigner(KEY).encode(m.ZERO,10.,100.),10.,100.)
    with pytest.raises(ValueError):MatchingOutputVerifier(KEY).decode(old_s.encode(m.ZERO,10.,100.),10.,100.)


def test_generated_adapter_cannot_import_hardware_transport_or_enable_from_manager():
    source=PATH.read_text()
    assert 'HardwareBridge' not in source and 'import socket' not in source and 'import serial' not in source
    assert 'self.signer=MatchingOutputSigner' in source and 'self.verifier=MatchingOutputVerifier' in source
    assert "plant=lambda:MatchingPlantNode(runtime)" in source
    assert 'self.model.press' not in source
    report=json.loads((PATH.parent/'generation.json').read_text())
    assert report['no_hardware_mode'] and report['domain']==115
    for path,sha in report['inputs_sha256'].items():assert m.digest(Path(path))==sha
    assert m.digest(PATH)==report['output_sha256']
