"""Import/static input contracts only. Never execute a ROS node or socket."""
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import pytest

ROOT=Path(os.environ['STAGE020_GENERATED_ROOT'])
FIX=Path(os.environ['STAGE020_STATIC_FIXTURES'])
for package in ('mecanum_wand_stage020_control','mecanum_wand_stage020_hardware'):
    sys.path.insert(0,str(ROOT/package))
from mecanum_wand_stage020_control.core import Velocity
from mecanum_wand_stage020_control.safety_core import SafetyConfig,ZERO
from mecanum_wand_stage020_hardware.commissioning import Runtime,CONSENT,DOMAIN,AuthoritySigner,AuthorityVerifier
from mecanum_wand_stage020_hardware.nodes import HardwareBridge
from mecanum_wand_stage020_hardware.continuous_status import decode_motion_status,disabled_motion_status_ready
from mecanum_wand_stage020_hardware.high_speed_contract import STATUS,AUTHORITY,SNAPSHOT
from mecanum_wand_stage020_hardware.stage020_quantization import inward_velocity,FirstStageSession
from mecanum_vicon_control.transport_protocol import ProtocolError,decode_frame,unpack_cmd_vel

PROFILE=ROOT/'mecanum_wand_stage020_hardware/config/stage020.SITE_CONFIRMATION_REQUIRED.json'


def test_private_domain_identity_envelope_and_preserved_timing():
    r=Runtime('shadow',str(PROFILE))
    assert DOMAIN==96 and r.namespace=='/wand_stage020_acceptance' and r.motion_tag=='4W20'
    assert r.config.max_speed_mps==.20 and r.config.max_accel_mps2==.08
    assert r.config.feedback_timeout_s==.20 and r.config.lease_timeout_s==.15
    assert r.config.command_timeout_s==.10 and r.config.max_session_s==120
    assert r.motion_status_size==156
    with pytest.raises(ValueError): SafetyConfig(max_speed_mps=.201)


def test_site_confirmation_blocks_before_node_or_socket(monkeypatch):
    import mecanum_wand_stage020_hardware.nodes as nodes
    for key,value in dict(WAND_STAGE020_MODE='hardware',WAND_STAGE020_PROFILE=str(PROFILE),
        WAND_PROFILE_APPROVAL=hashlib.sha256(PROFILE.read_bytes()).hexdigest(),
        WAND_HARDWARE_CONSENT=CONSENT).items():monkeypatch.setenv(key,value)
    def forbidden(*args,**kwargs):raise AssertionError('must not create socket')
    monkeypatch.setattr(nodes.socket,'socket',forbidden)
    with pytest.raises(ValueError,match='STAGE020_SITE_CONFIRMATION_REQUIRED'):
        HardwareBridge.__init__(object.__new__(HardwareBridge))


def confirmed(tmp_path,change=lambda v:None):
    v=json.loads(PROFILE.read_text())
    v['stage020_confirmations']['clear_area_and_power_cut_ready_now']=True
    change(v)
    p=tmp_path/'synthetic_input_ONLY.json';p.write_text(json.dumps(v))
    return p,hashlib.sha256(p.read_bytes()).hexdigest()


def test_pure_site_input_validator_does_not_invent_physical_acceptance(tmp_path):
    p,d=confirmed(tmp_path);r=Runtime('hardware',str(p),d,CONSENT)
    assert r.config.max_speed_mps==.20
    assert r.profile['stage020_policy']['braking_current_thermal_acceptance'].startswith('DEFERRED')
    assert 'stopping_review' not in r.profile


@pytest.mark.parametrize('failure',['speed','tag','pwm','rpm','wheel','cps','watchdog','feedback','consent'])
def test_first_stage_cannot_be_widened_or_mistaken_for_full_highspeed(tmp_path,failure):
    def change(v):
        if failure=='speed':v['safety']['max_speed_mps']=.30
        if failure=='tag':v['hardware']['required_motion_tag']='4WGD'
        if failure=='pwm':v['hardware']['pwm_cap_permille']=800
        if failure=='rpm':v['hardware']['motor_target_rpm_cap']=400
        if failure=='wheel':v['hardware']['wheel_rim_limit_mps']=1.25
        if failure=='cps':v['hardware']['target_limit_cps']=7100
        if failure=='watchdog':v['hardware']['wire_timeout_ms']=601
        if failure=='feedback':v['safety']['feedback_timeout_s']=.5
    p,d=confirmed(tmp_path,change)
    with pytest.raises((ValueError,TypeError)):
        Runtime('hardware',str(p),d,CONSENT if failure!='consent' else 'old_consent')


def test_signature_vector_owner_replay_and_deadline_preserved():
    s,v=AuthoritySigner('ab'*32),AuthorityVerifier('ab'*32)
    v.decode(s.encode(ZERO,10.,100.),10.,100.)
    def message(command):return s.encode(command,10.01,100.01,state='ACTIVE',permit=True,epoch='e')
    good=message(Velocity(.20))
    assert v.decode(good,10.01,100.01)[1]==Velocity(.20)
    with pytest.raises(ValueError):v.decode(good,10.01,100.01)
    for command in (Velocity(.201),Velocity(.2,.2),Velocity(0,0,.101)):
        with pytest.raises(ValueError):v.decode(message(command),10.01,100.01)
    with pytest.raises(ValueError):v.decode(message(ZERO),10.12,100.12)


def change_base(payload,index,value):
    v=list(STATUS.unpack_from(payload));v[index]=value
    return STATUS.pack(*v)+payload[STATUS.size:]


def test_actual_c_fixtures_and_old_identity_rejection():
    idle=(FIX/'idle_status.bin').read_bytes();moving=(FIX/'moving_status.bin').read_bytes()
    assert disabled_motion_status_ready(decode_motion_status(idle))
    d=decode_motion_status(moving)
    assert d['tag']=='4W20' and max(map(abs,d['raw_duty']))==350
    from mecanum_wand_hardware.continuous_status import decode_motion_status as old_decode
    for payload in (idle,moving):
        with pytest.raises(ProtocolError):old_decode(payload)


@pytest.mark.parametrize('index,value',[(0,0x44475734),(0,0x43475734),(17,-360),
    (20,1701),(28,201),(29,200),(30,101),(33,1),(34,0),(3,31)])
def test_decoder_rejects_wrong_firmware_pwm_cps_vector_yaw_watchdog_and_permit(index,value):
    with pytest.raises(ProtocolError):decode_motion_status(change_base((FIX/'moving_status.bin').read_bytes(),index,value))


def test_decoder_does_not_trust_latest_request_in_place_of_granted_output():
    payload=(FIX/'moving_status.bin').read_bytes()
    a=list(AUTHORITY.unpack_from(payload,STATUS.size+SNAPSHOT.size));a[4]=-1
    with pytest.raises(ProtocolError):decode_motion_status(payload[:STATUS.size+SNAPSHOT.size]+AUTHORITY.pack(*a))
    with pytest.raises(ProtocolError):decode_motion_status(change_base(payload,20,-1000))


def test_all_heading_quantisation_never_exceeds_mcu_vector_cap():
    session=FirstStageSession();session.connected=True
    for i in range(3600):
        a=math.radians(i/10);vx,vy=.2*math.cos(a),.2*math.sin(a)
        x,y,z=inward_velocity(vx,vy,.10)
        assert math.hypot(x,y)<=.2+1e-12 and abs(z)<=.10
        frame=decode_frame(session.cmd_vel(vx,vy,.1,600,True,1000))
        wire=unpack_cmd_vel(frame.payload)
        assert math.hypot(wire.vx_mps,wire.vy_mps)<=.2+1e-12
    with pytest.raises(ProtocolError):inward_velocity(.2,.2,0)
