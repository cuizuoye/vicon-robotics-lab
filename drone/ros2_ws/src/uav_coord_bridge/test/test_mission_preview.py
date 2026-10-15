from dataclasses import replace
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uav_coord_bridge.mission_preview import MissionPreview, Observation, State


def obs(t=0.):
    return Observation(t, (2., 3., .2), yaw=.8)


def test_no_automatic_start_or_hardware_adapter():
    p = MissionPreview()
    assert p.tick(obs())['state'] == 'WAIT_START'
    assert not p.start(obs())
    assert not p.result()['hardware_adapter_present'] and p.result()['actual_sent_to_fc'] == 0


@pytest.mark.parametrize('field,value', [('pose_age_s', .2), ('status_age_s', 1.), ('health_ok', False),
    ('emergency_available', False), ('mapping_valid', False), ('position_valid', False),
    ('velocity_valid', False), ('landed', False), ('armed', True), ('yaw', float('nan'))])
def test_start_gates_are_all_required(field, value):
    assert not MissionPreview().start(replace(obs(), **{field: value}), confirmed=True)


def test_freeze_xy_yaw_and_height_relative_to_nonzero_start():
    p = MissionPreview()
    assert p.start(obs(), confirmed=True)
    assert p.target == (2., 3., .7) and p.yaw == .8
    p.tick(replace(obs(1.), position=(2.1, 3.1, .4), armed=True, landed=False, yaw=1.))
    assert p.target == (2., 3., .7) and p.yaw == .8


def test_hover_then_landing_requires_landed_and_disarmed_feedback():
    p = MissionPreview(hover_seconds=1.)
    p.start(obs(), confirmed=True)
    high = replace(obs(1.), position=p.target, armed=True, landed=False)
    p.tick(high)
    assert p.state == State.HOVER
    p.tick(replace(high, t=2.1))
    assert p.state == State.LAND
    p.tick(replace(obs(3.), landed=True, armed=True))
    assert p.state == State.LAND
    p.tick(replace(obs(4.), landed=False, armed=False))
    assert p.state == State.LAND
    p.tick(replace(obs(5.), landed=True, armed=False))
    assert p.state == State.LANDED


def test_emergency_release_cannot_resume_or_restart():
    p = MissionPreview()
    p.start(obs(), confirmed=True)
    p.tick(replace(obs(1.), home_pressed=True))
    p.tick(obs(2.))
    assert p.state == State.STOP_LATCHED and not p.start(obs(3.), confirmed=True)


@pytest.mark.parametrize('bad', ['pose_loss', 'excursion', 'climb_timeout'])
def test_failure_requests_advisory_land_not_forced_disarm(bad):
    p = MissionPreview(climb_timeout=2.)
    p.start(obs(), confirmed=True)
    o = replace(obs(1.), armed=True, landed=False)
    if bad == 'pose_loss': o = replace(o, pose_age_s=1.)
    if bad == 'excursion': o = replace(o, position=(4., 3., .5))
    if bad == 'climb_timeout': o = replace(o, t=3.)
    result = p.tick(o)
    assert p.state == State.ABORT_LAND and result['intent'] == 'preview_land_and_alert'
    assert result['actual_sent_to_fc'] == 0


def test_landing_timeout_is_not_ground_detection_or_forced_disarm():
    p = MissionPreview(land_timeout=2.)
    p.start(obs(), confirmed=True)
    o = replace(obs(1.), pose_age_s=1., armed=True, landed=False)
    p.tick(o)
    result = p.tick(replace(o, t=4.))
    assert p.state == State.LAND_TIMEOUT and result['intent'] == 'preview_land_and_alert'
    assert 'never force disarm' in result['reason']


def test_hover_instability_resets_dwell_instead_of_advancing_timer():
    p = MissionPreview(hover_seconds=2.)
    p.start(obs(), confirmed=True)
    o = replace(obs(1.), position=p.target, armed=True, landed=False)
    p.tick(o)
    p.tick(replace(o, t=2., velocity=(.2, 0., 0.)))
    p.tick(replace(o, t=3.))
    p.tick(replace(o, t=4.))
    assert p.state == State.HOVER
    p.tick(replace(o, t=5.1))
    assert p.state == State.LAND


def test_hover_cannot_wait_forever_if_stability_never_returns():
    p = MissionPreview(hover_seconds=1., hover_timeout=3.)
    p.start(obs(), confirmed=True)
    o = replace(obs(1.), position=p.target, armed=True, landed=False)
    p.tick(o)
    p.tick(replace(o, t=2., velocity=(.2, 0., 0.)))
    p.tick(replace(o, t=4.1, velocity=(.2, 0., 0.)))
    assert p.state == State.ABORT_LAND
