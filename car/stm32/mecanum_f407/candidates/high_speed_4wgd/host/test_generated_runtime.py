"""Import/contract checks only; do not create ROS nodes or network transports."""
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import pytest

ROOT=os.environ.get('HIGH_SPEED_GENERATED_ROOT')
if not ROOT:pytest.skip('private runtime not generated',allow_module_level=True)
ROOT=Path(ROOT)
for package in ('mecanum_wand_highspeed_control','mecanum_wand_highspeed_hardware'):
    sys.path.insert(0,str(ROOT/package))

from mecanum_wand_highspeed_control.core import Velocity
from mecanum_wand_highspeed_control.safety_core import SafetyConfig,ZERO
from mecanum_wand_highspeed_hardware.commissioning import Runtime,CONSENT,AuthoritySigner,AuthorityVerifier
from mecanum_wand_highspeed_hardware.continuous_status import decode_motion_status,disabled_motion_status_ready
from mecanum_wand_highspeed_hardware.nodes import HardwareBridge

PROFILE=ROOT/'mecanum_wand_highspeed_hardware/config/high_speed_020.REVIEW_REQUIRED.json'


def test_high_speed_runtime_and_guards_have_independent_domain_namespace_and_tag():
    r=Runtime('shadow',str(PROFILE))
    assert r.namespace=='/wand_highspeed_acceptance' and r.motion_tag=='4WGD'
    assert r.config.max_speed_mps==.20 and r.config.max_accel_mps2==.08
    assert r.config.feedback_timeout_s==.20 and r.config.lease_timeout_s==.15
    assert r.config.command_timeout_s==.10 and r.config.max_session_s==120
    assert r.motion_status_size==156
    with pytest.raises(ValueError): SafetyConfig(max_speed_mps=.201)


def test_real_draft_rejected_before_even_socket_or_node_construction(monkeypatch):
    import mecanum_wand_highspeed_hardware.nodes as nodes
    for name,value in dict(WAND_HIGHSPEED_MODE='hardware',WAND_HIGHSPEED_PROFILE=str(PROFILE),
        WAND_PROFILE_APPROVAL=hashlib.sha256(PROFILE.read_bytes()).hexdigest(),
        WAND_HARDWARE_CONSENT=CONSENT).items():monkeypatch.setenv(name,value)
    def forbidden(*args,**kwargs):raise AssertionError('MUST NOT CONSTRUCT A SOCKET')
    monkeypatch.setattr(nodes.socket,'socket',forbidden)
    with pytest.raises(ValueError,match='HIGH_SPEED_REVIEW_INCOMPLETE'):
        HardwareBridge.__init__(object.__new__(HardwareBridge))


def reviewed(tmp_path,change=None):
    v=json.loads(PROFILE.read_text())
    v['high_speed_confirmations']=dict.fromkeys(v['high_speed_confirmations'],True)
    v['stopping_review'].update(test_speed_mps=.20,measured_coast_distance_m=.10)
    v['safety'].update(boundary_margin_m=.50,arena_x_min=-4.,arena_x_max=4.,arena_y_min=-4.,arena_y_max=4.)
    if change:change(v)
    p=tmp_path/'synthetic_review_ONLY_NOT_SITE_APPROVED.json';p.write_text(json.dumps(v))
    return p,hashlib.sha256(p.read_bytes()).hexdigest()


def test_synthetic_review_can_only_validate_data_not_grant_motion(tmp_path):
    p,d=reviewed(tmp_path)
    r=Runtime('hardware',str(p),d,CONSENT)
    assert r.config.max_speed_mps==.20 and r.profile_hash==d
    # Runtime is a pure data validator, not a ROS node/transport or arm action.


@pytest.mark.parametrize('failure',['speed','identity','watchdog','feedback','margin','unknown_brake'])
def test_review_cannot_widen_first_stage_or_omit_measured_coast_and_margin(tmp_path,failure):
    def change(v):
        if failure=='speed':v['safety']['max_speed_mps']=.30
        if failure=='identity':v['hardware']['required_motion_tag']='4WGC'
        if failure=='watchdog':v['hardware']['wire_timeout_ms']=601
        if failure=='feedback':v['safety']['feedback_timeout_s']=.5
        if failure=='margin':v['safety']['boundary_margin_m']=.30
        if failure=='unknown_brake':v['stopping_review']['measured_coast_distance_m']=None
    p,d=reviewed(tmp_path,change)
    with pytest.raises((ValueError,TypeError)):Runtime('hardware',str(p),d,CONSENT)


def test_high_speed_signed_vector_limit_owner_replay_and_timeout_are_preserved():
    key='ab'*32;s=AuthoritySigner(key);v=AuthorityVerifier(key)
    v.decode(s.encode(ZERO,10.,100.),10.,100.)
    def message(command,t=10.01):return s.encode(command,t,t+90,state='ACTIVE',permit=True,epoch='e')
    good=message(Velocity(.20))
    assert v.decode(good,10.01,100.01)[1]==Velocity(.20)
    with pytest.raises(ValueError):v.decode(good,10.01,100.01)
    for command in (Velocity(.201),Velocity(.20,.20),Velocity(0,0,.101)):
        with pytest.raises(ValueError):v.decode(message(command),10.01,100.01)
    with pytest.raises(ValueError):v.decode(message(ZERO),10.12,100.12)


def test_private_decoder_is_strict_D_not_a_C_alias():
    path=os.environ.get('HIGH_SPEED_STATIC_FIXTURES')
    if not path:pytest.skip('C fixtures not built')
    idle=(Path(path)/'idle_status.bin').read_bytes()
    d=decode_motion_status(idle)
    assert d['tag']=='4WGD' and disabled_motion_status_ready(d)
    from mecanum_wand_hardware.continuous_status import decode_motion_status as original_decode
    from mecanum_vicon_control.transport_protocol import ProtocolError
    with pytest.raises(ProtocolError):original_decode(idle)
