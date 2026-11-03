"""Offline contract/signature/model tests; no ROS context or motor IO."""
from dataclasses import asdict,replace
import math
from types import SimpleNamespace as NS
import pytest
from profile12 import sim_config,config,contract,COMMAND_RPM,ABSOLUTE_RPM,Signer,Verifier,Policy,runtime
from mecanum_wand_4wsb_control.core import Velocity
from mecanum_wand_4wsb_control.safety_core import ZERO
from mecanum_wand_4wsb_control.rotation_servo import coupled_shape,wheel_rpm
from mecanum_wand_4wsb_hardware.commissioning import AuthoritySigner,AuthorityVerifier
from plant12 import NominalPlant

def test_exact_non_motion_guards_and_no_fixed_fence():
    c=sim_config();before=asdict(config(6));after=asdict(c)
    for key in before:
        if key not in ('max_speed_mps','max_accel_mps2'):assert after[key]==before[key]
    assert c.pose_timeout_s==.2 and c.command_timeout_s==.1 and c.feedback_timeout_s==.3
    assert c.lease_timeout_s==.15 and c.max_control_gap_s==.1 and c.max_session_s==600.
    assert c.inside(__import__('mecanum_wand_4wsb_control.core',fromlist=['Pose']).Pose(100.,100.,.1))
    assert c.follower_config().workspace_radius_enabled is False

@pytest.mark.parametrize('key,value',[('pose_timeout_s',.3),('command_timeout_s',.2),('feedback_timeout_s',.4),
    ('lease_timeout_s',.3),('max_session_s',601.),('max_control_gap_s',.2),('max_speed_mps',1.4),('max_accel_mps2',8.)])
def test_profile_cannot_relax_guards_or_enable_abandoned_speed(key,value):
    with pytest.raises(ValueError):replace(sim_config(),**{key:value})

def test_straight_and_diagonal_share_wheel_budget_and_rotation_scales_every_axis():
    c=sim_config().follower_config()
    straight=coupled_shape(Velocity(1.2,0.,0.),ZERO,100.,c)
    diagonal=coupled_shape(Velocity(1.2/math.sqrt(2),1.2/math.sqrt(2),0.),ZERO,100.,c)
    assert straight.vx==pytest.approx(1.2)
    assert math.hypot(diagonal.vx,diagonal.vy)==pytest.approx(1.2/math.sqrt(2))
    raw=Velocity(1.2/math.sqrt(2),1.2/math.sqrt(2),2.)
    combined=coupled_shape(raw,ZERO,100.,c)
    assert combined.vx/raw.vx==pytest.approx(combined.wz/raw.wz)
    assert max(map(abs,wheel_rpm(combined)))<=COMMAND_RPM+1e-6<ABSOLUTE_RPM

def test_signed_scope_cannot_be_used_by_real_4wsb_or_abandoned_sim14():
    key='a'*64
    good=Signer(key).encode(ZERO,10.,100.)
    assert Verifier(key).decode(good,10.,100.)[1]==ZERO
    with pytest.raises(ValueError):AuthorityVerifier(key,6).decode(good,10.,100.)
    with pytest.raises(ValueError):Verifier(key).decode(AuthoritySigner(key,6).encode(ZERO,10.,100.),10.,100.)

def test_signed_speed_wheel_replay_and_stale_checks():
    key='b'*64;s=Signer(key);v=Verifier(key);first=s.encode(ZERO,10.,100.);v.decode(first,10.,100.)
    with pytest.raises(ValueError):v.decode(first,10.,100.)
    with pytest.raises(ValueError):v.decode(s.encode(ZERO,10.,100.),10.101,100.101)
    for command in (Velocity(1.21,0.,0.),Velocity(.9,.9,0.),Velocity(.84,.84,2.)):
        with pytest.raises(ValueError):v.decode(s.encode(command,10.,100.,epoch='e',state='ACTIVE',permit=True),10.,100.)

def test_revoked_epoch_never_automatically_resumes():
    key='c'*64;s=Signer(key);p=Policy(key)
    p.receive(s.encode(ZERO,10.,100.),10.,100.)
    p.receive(s.encode(ZERO,10.01,100.01,epoch='e',state='ARMED',permit=True),10.01,100.01)
    p.receive(s.encode(Velocity(.1,0.,0.),10.02,100.02,epoch='e',state='ACTIVE',permit=True),10.02,100.02)
    assert p.active(10.03) and not p.active(10.13)
    p.receive(s.encode(Velocity(.1,0.,0.),10.14,100.14,epoch='e',state='ACTIVE',permit=True),10.14,100.14)
    assert not p.active(10.14) and p.command==ZERO

def test_runtime_rejects_public_domain_namespace_and_hardware_bridge(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','0');monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    with pytest.raises(ValueError):runtime('/host4wsb_0123456789')
    monkeypatch.setenv('ROS_DOMAIN_ID','191')
    with pytest.raises(ValueError):runtime('/real_car')
    rt=runtime('/host4wsb_0123456789')
    node=NS(context=NS(get_domain_id=lambda:191),get_namespace=lambda:rt.namespace,
        get_parameter=lambda _:NS(value=False),get_name=lambda:'wand_hardware_bridge')
    with pytest.raises(ValueError):rt.guard(node)

def test_nominal_plant_has_finite_response_and_stops_without_minimum_start_threshold():
    p=NominalPlant()
    for _ in range(100):p.step(Velocity(0.,0.,.1),.01)
    assert p.pose[2]>.08
    for _ in range(100):p.step(ZERO,.01)
    assert abs(p.velocity.wz)<.001
    assert contract()['original_4wsb_firmware_used'] is False

def test_rebuilt_geometry_uses_new_junction_and_retains_every_source_check():
    from geometry12 import install_geometry,synthetic_packet
    from mecanum_wand_4wsb_control.cached_packet import decode_packet
    install_geometry();packet=synthetic_packet(1.,2.,.3,10.,100.)
    pose=packet.pose(10.,100.)
    assert (pose.x,pose.y,pose.z)==pytest.approx((1.,2.,1.3))
    assert tuple(s.name for s in packet.samples)==('sign','Sign1','Sign2','Sign3','Sign4','Sign5')
    with pytest.raises(ValueError):packet.pose(10.201,100.201)
    bad=packet.payload();bad['samples'][2]['position']=list(bad['samples'][2]['position']);bad['samples'][2]['position'][0]+=.1
    with pytest.raises(ValueError):decode_packet(bad,10.,100.)

def test_rebuilt_sign_rotation_never_introduces_stem_length_translation():
    from geometry12 import install_geometry,synthetic_packet
    install_geometry()
    for yaw in (-3.14,-1.57,0.,1.57,3.14):
        pose=synthetic_packet(1.,2.,yaw,10.,100.).pose(10.,100.)
        assert (pose.x,pose.y)==pytest.approx((1.,2.))

@pytest.mark.parametrize('dt',[-1.,.11,float('nan')])
def test_model_rejects_unbounded_time_steps(dt):
    with pytest.raises(ValueError):NominalPlant().step(ZERO,dt)
