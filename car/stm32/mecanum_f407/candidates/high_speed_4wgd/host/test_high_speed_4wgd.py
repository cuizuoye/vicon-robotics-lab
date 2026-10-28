from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path

import pytest

from high_speed_4wgd import (HighSpeedPlan,CPR,MAX_WHEEL_MPS,STAGES_MPS,
    decode_high_speed_status,provisional_stopping_reserve,require_hardware_admission)
from mecanum_wand_control.core import Pose,Velocity
from mecanum_wand_control.safety_core import SafetyConfig,ZERO
from mecanum_wand_hardware.commissioning import Runtime,CONSENT,AuthoritySigner,AuthorityVerifier
from mecanum_wand_hardware.continuous_status import decode_motion_status,STATUS,AUTHORITY,SNAPSHOT
from mecanum_vicon_control.transport_protocol import ProtocolError

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]


def test_motor_conversion_does_not_apply_gear_ratio_twice_or_equate_pwm_and_rpm():
    p=HighSpeedPlan();table=p.speed_table()
    assert table['target_rpm']==400 and table['first_speed_mps']==.20
    assert table['first_wheel_rpm']==pytest.approx(63.661977)
    assert table['theoretical_straight_mps']==pytest.approx(1.2566370614)
    assert table['theoretical_diagonal_mps']==pytest.approx(.8885765876)
    assert table['pwm_cap_is_not_rpm_percent']
    assert table['approximate_full_rpm_ramp_s']>8.8


@pytest.mark.parametrize('field,value',[('target_rpm',401),('max_pwm_permille',810),
    ('wire_timeout_ms',601),('feedback_timeout_s',.21),('scale',3.),('first_speed_mps',.3)])
def test_new_candidate_constants_cannot_be_relaxed_by_parameter_override(field,value):
    with pytest.raises(ValueError): replace(HighSpeedPlan(),**{field:value})


def test_braking_estimate_is_explicitly_provisional_and_includes_fallback_latency():
    assert provisional_stopping_reserve(.2,.2)==pytest.approx(.42)
    assert provisional_stopping_reserve(MAX_WHEEL_MPS,.2)>4.9
    assert provisional_stopping_reserve(.5,.2)>provisional_stopping_reserve(.2,.2)
    for speed,decel in ((-.1,.2),(math.inf,.2),(.2,0),(2.,.2)):
        with pytest.raises(ValueError): provisional_stopping_reserve(speed,decel)


def test_prescribed_measured_pose_trace_keeps_mapping_and_first_stage_vector_limits():
    # Prescribed measurements, not a command-driven vehicle simulation.
    p=HighSpeedPlan();c=SafetyConfig(arena_x_min=-3,arena_x_max=3,arena_y_min=-3,arena_y_max=3)
    s=p.follower(c)
    s.observe('car',Pose(0,0,.1),10.);s.observe('wand',Pose(0,0,1.),10.)
    assert s.engage(10.)[0]
    old=ZERO
    for i in range(1,801):
        t=10+i*.02
        s.observe('car',Pose(min(1.2,.10*i*.02),0,.1),t)
        s.observe('wand',Pose(min(.6,i*.01),0,.5+.3*math.sin(t)),t)
        out=s.tick(t)
        assert s.state=='ACTIVE',s.reason
        assert math.hypot(out.vx,out.vy)<=.20+1e-9
        assert math.hypot(out.vx-old.vx,out.vy-old.vy)<=.08*.02+1e-9
        old=out
    assert s.mapped_goal.x==pytest.approx(1.2)
    s.pause();assert s.goal is None and s.mapped_goal is None and s.tick(26.02)==ZERO


@pytest.mark.parametrize('stage',STAGES_MPS)
def test_future_stages_are_offline_only_and_always_block_real_admission(stage):
    p=HighSpeedPlan();f=p.follower(SafetyConfig(),stage_mps=stage)
    assert f.config.max_speed_mps==stage
    with pytest.raises(RuntimeError,match='OFFLINE ONLY'):
        require_hardware_admission(profile_approved=True,site_clear=True,stage=stage)


def test_current_real_entry_and_signed_channel_still_reject_high_speed_draft_and_commands():
    path=HERE/'high_speed_4wgd.draft.json'
    with pytest.raises(ValueError):
        Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),CONSENT)
    s,v=AuthoritySigner('a1'*32),AuthorityVerifier('a1'*32)
    v.decode(s.encode(ZERO,10.,100.),10.,100.)
    with pytest.raises(ValueError,match='speed limit'):
        v.decode(s.encode(Velocity(.2),10.01,100.01,state='ACTIVE',permit=True,epoch='e'),10.01,100.01)


def fixture(name):
    root=os.environ.get('HIGH_SPEED_STATIC_FIXTURES')
    if not root: pytest.skip('C-generated fixtures are created by run_offline.py')
    return (Path(root)/name).read_bytes()


def change_base(payload,index,value):
    v=list(STATUS.unpack_from(payload));v[index]=value
    return STATUS.pack(*v)+payload[STATUS.size:]


def test_actual_c_idle_and_fast_diagnostics_match_independent_identity_and_caps():
    idle,fast=(fixture(n) for n in ('idle_status.bin','fast_status.bin'))
    assert decode_high_speed_status(idle)['permitted']==0
    s=decode_high_speed_status(fast)
    assert s['tag']=='4WGD' and s['permitted']==1 and s['output_authority_version']==1
    assert max(abs(v) for v in s['raw_duty'])==800
    for cpr,value in zip(CPR,s['effective_cps']):
        assert abs(value)*60/cpr==pytest.approx(400,abs=.0001)
    for payload in (idle,fast):
        with pytest.raises(ProtocolError): decode_motion_status(payload)


@pytest.mark.parametrize('index,value',[(0,0x43475734),(17,-810),(21,7100),(29,1257),
    (33,1),(34,0),(3,31)])
def test_high_speed_decoder_rejects_bad_identity_power_rpm_body_watchdog_or_permit(index,value):
    payload=change_base(fixture('fast_status.bin'),index,value)
    with pytest.raises(ProtocolError): decode_high_speed_status(payload)


def test_high_speed_decoder_rejects_wrong_grant_and_forged_reversal_permission():
    payload=fixture('fast_status.bin')
    a=list(AUTHORITY.unpack_from(payload,STATUS.size+SNAPSHOT.size));a[4]=-1
    damaged=payload[:STATUS.size+SNAPSHOT.size]+AUTHORITY.pack(*a)
    with pytest.raises(ProtocolError): decode_high_speed_status(damaged)
    target=change_base(payload,20,-7000)
    with pytest.raises(ProtocolError): decode_high_speed_status(target)
