"""Offline regression of all removed spatial gates and retained stop gates."""
from dataclasses import replace
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import pytest

OUT=Path(os.environ['TRACKING_OUTPUT']);ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
for package in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
    sys.path.insert(0,str(OUT/'runtime/src'/package))
sys.path.insert(0,str(ROOT/'candidates/wand_rotation_hardware'))
import test_migration as legacy
from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig,SPATIAL_POLICY
from mecanum_wand_4ws8_control.core import Pose,Velocity,Config
from mecanum_wand_4ws8_control.safety_core import ZERO
from mecanum_wand_4ws8_control.offline_suite import Plant
from mecanum_wand_4ws8_hardware.authorization import verify_runtime_profile

# Reuse the exact independently ticked supervisor + final gate software plant
# harness, but construct the new explicit policy, not the old fixed config.
legacy.RotationSafetyConfig=TrackingSafetyConfig
config=legacy.config


@pytest.fixture(autouse=True)
def no_external_io(monkeypatch):
    import socket,subprocess,rclpy
    def denied(*a,**kw):pytest.fail('pure test attempted hardware/network/ROS IO')
    monkeypatch.setattr(socket,'socket',denied);monkeypatch.setattr(subprocess,'Popen',denied)
    monkeypatch.setattr(rclpy,'init',denied)


def far_harness():
    h=legacy.Harness();h.origin=Pose(12.,-8.,h.c.ground_z_m,0.)
    h.plant=Plant(h.origin);h.hand=Pose(30.,-20.,1.2,.7)
    return h


def app():
    spec=importlib.util.spec_from_file_location('tracking_app',OUT/'oneclick_app.py')
    a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a);return a


@pytest.mark.parametrize('xy',[(0,0),(10,-20),(-100,100),(10000,-10000)])
def test_no_fixed_arena_or_projection_or_radius(xy):
    c=config();p=Pose(*xy,c.ground_z_m,1.2);origin=Pose(12,-8,c.ground_z_m,0.)
    assert c.inside(p) and c.goal_inside(p)
    assert c.project_goal(p,origin,4.75)==p
    assert c.project_reference(p,origin,4.75)==p
    assert not c.follower_config().workspace_radius_enabled
    assert c.fence_polygon()==c.goal_polygon()==c.rod_reference_polygon()==()
    with pytest.raises(ValueError):c.center_bounds(0.)


@pytest.mark.parametrize('field',['x','y','z','yaw'])
@pytest.mark.parametrize('value',[math.nan,math.inf,-math.inf])
def test_unbounded_is_not_permission_for_invalid_pose(field,value):
    c=config();p=replace(Pose(12,-8,c.ground_z_m,0.),**{field:value})
    assert not c.inside(p) and not c.goal_inside(p)
    with pytest.raises(ValueError):c.project_goal(p,Pose(0,0),4.75)


def test_continuous_8m_follow_crosses_old_radius_and_boundary_without_stop():
    h=far_harness();h.start(pick_up=True);origin=h.plant.pose;hand=h.hand
    for i in range(1,801):
        h.hand=replace(hand,x=hand.x+.01*i)
        h.step()
        assert h.sup.state=='ACTIVE' and not h.gate.fault,(i,h.sup.reason,h.gate.fault)
    h.run(2.)
    assert h.plant.pose.x-origin.x>7.5
    assert h.sup.follower.mapped_goal.x==pytest.approx(origin.x+8.)
    assert not h.sup.follower.goal_limited
    assert h.sup.pilot.path>7.5 and h.gate.pilot.path>7.5
    h.held=False;h.run(1.);assert h.output==ZERO and not h.sup.request_start(h.now,h.ros)[0]


@pytest.mark.parametrize('fault',['wand_loss','car_loss','feedback_loss','lease_loss','release','yaw_jump',
    'lift','tilt','mode','source','actuator','reference_spoof','overspeed','accel','command_loss','estop','timeout'])
def test_old_nonspatial_faults_still_stop_and_cannot_auto_resume(fault):
    legacy.test_faults_fail_stop_and_never_resume_on_input_return(fault)


def test_position_jump_still_stops_even_without_a_fence():
    h=far_harness();h.start();h.hand=replace(h.hand,x=h.hand.x+.3);h.run(.1)
    assert h.output==ZERO and h.sup.state!='ACTIVE'
    assert 'JUMP' in h.sup.reason or 'jump' in h.sup.reason


@pytest.mark.parametrize('kw',[dict(pose_timeout_s=.3),dict(max_session_s=140.),
    dict(max_accel_mps2=2.),dict(max_yaw_rate_radps=2.),dict(lift_tolerance_m=.2),
    dict(goal_boundary_policy='clamp'),dict(goal_reference_policy='direct')])
def test_other_limits_cannot_be_relaxed_by_profile(kw):
    with pytest.raises(ValueError):replace(config(),**kw)


def test_new_radius_switch_is_explicit_boolean_and_legacy_defaults_still_bounded():
    assert Config().workspace_radius_enabled is True
    with pytest.raises(ValueError):Config(workspace_radius_enabled='no')


def profile(now=12345.):
    a=app();m=json.loads((OUT/'build_manifest.json').read_text())
    p=json.loads(Path(m['draft_profiles'][4]).read_text());p['trial_review']=a.make_review(m)
    p['staged_runtime_approval']=dict(purpose='sign',issued_utc_s=now,clear_and_power_cut_ready_now=True,
        single_click_mode_acknowledged_now=True,flash_record=str(a.FLASH),build_manifest=str(OUT/'build_manifest.json'))
    p['tracking_confirmations']={k:True for k in p['tracking_confirmations']}
    return p


def test_runtime_uses_matching_existing_4ws8_readback_without_fake_coverage_claim(tmp_path):
    from mecanum_wand_4ws8_hardware.commissioning import Runtime,CONSENT
    import hashlib,time
    p=profile(now=time.time());assert verify_runtime_profile(p,'sign')
    p['confirmations']['tracking_only_no_obstacle_detection_acknowledged']=True
    path=tmp_path/'SOFTWARE_TEST_ONLY.json';path.write_text(json.dumps(p))
    runtime=Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),CONSENT)
    assert isinstance(runtime.config,TrackingSafetyConfig)
    assert runtime.config.inside(Pose(12.,-8.,runtime.config.ground_z_m,2.))
    assert not runtime.config.follower_config().workspace_radius_enabled
    assert p['input_workspace']['full_vicon_coverage_measured'] is False
    assert p['arena_revision']==SPATIAL_POLICY
    assert 'fixed_arena_checked' not in p['confirmations']


@pytest.mark.parametrize('bad',['old_policy','old_review','unconfirmed','expired','coverage_claim','endpoint','firmware'])
def test_old_site_scope_or_malformed_new_approval_cannot_launch(bad):
    p=profile()
    if bad=='old_policy':p['arena_revision']='fixed_reference_signed_120s_buffer050_v1'
    elif bad=='old_review':p['trial_review']['kind']='BOUNDED_XY_YAW_4WS8_EXISTING_ENVELOPE_NOT_QUALIFICATION'
    elif bad=='unconfirmed':p['tracking_confirmations'][next(iter(p['tracking_confirmations']))]=False
    elif bad=='expired':p['staged_runtime_approval']['issued_utc_s']=1.
    elif bad=='coverage_claim':p['input_workspace']['full_vicon_coverage_measured']=True
    elif bad=='endpoint':p['hardware']['esp_ip']='127.0.0.1'
    else:p['hardware']['required_motion_tag']='4WS9'
    with pytest.raises(ValueError):verify_runtime_profile(p,'sign',now=12345.)


def test_exact_source_binding_and_stop_priority_unchanged():
    legacy.test_missing_source_frame_cannot_be_overwritten_by_later_good_frame()
    legacy.test_stop_bypasses_pending_source_and_publishes_zero_immediately()


def test_preflight_has_no_saved_area_or_saved_heading_check():
    source=(OUT/'pose_check.py').read_text()
    assert 'goal_inside(car)' not in source and 'abs(error)>.20' not in source
    assert 'quality.preflight_issue()' in source and 'car.z-c.ground_z_m' in source
    assert 'create_publisher' not in source
