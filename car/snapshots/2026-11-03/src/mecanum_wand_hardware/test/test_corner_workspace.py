"""Static profile/geometry checks only: no plant, ROS node, socket or hardware."""

from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import pytest

from mecanum_wand_control.core import Pose, RelativeMapping
from mecanum_wand_hardware.commissioning import CONSENT, Runtime


CONFIG = Path(__file__).resolve().parents[1] / 'config'
PROFILE = CONFIG / 'hardware_acceptance.corner_2x2_2026-10-23.json'


def load_runtime(mode='shadow'):
    digest = hashlib.sha256(PROFILE.read_bytes()).hexdigest()
    return Runtime(mode, str(PROFILE), digest, CONSENT)


def test_corner_profile_has_true_half_extents_and_explicit_budgets():
    shadow, emulator, hardware = (load_runtime(mode) for mode in ('shadow', 'emulator', 'hardware'))
    assert shadow.config == emulator.config == hardware.config
    c = hardware.config
    assert (2*c.half_length_m, 2*c.half_width_m) == (.19, .21)
    assert (c.max_goal_radius_m, c.max_session_s) == (3., 120.)
    assert c.max_speed_mps == .05 and c.boundary_margin_m == .20
    assert shadow.ip == '' and emulator.ip == '127.0.0.1'
    assert hardware.profile['hardware']['required_motion_tag'] == '4WGB'
    assert hardware.profile['hardware']['wire_timeout_ms'] == 600


@pytest.mark.parametrize('forward,left', [(0., 0.), (2., 0.), (2., 2.), (0., 2.)])
def test_entire_two_metre_corner_workspace_survives_footprint_and_radius(forward, left):
    r = load_runtime()
    origin = r.initial_pose
    # 1m of Sign movement must yield the complete 2m car displacement.
    mapping = RelativeMapping(origin, Pose(0., 0.), scale=2.)
    goal = mapping.goal_from_local(forward/2, left/2)
    assert math.hypot(goal.x-origin.x, goal.y-origin.y) < r.config.max_goal_radius_m
    bounded = r.config.project_goal(goal, origin, r.config.max_goal_radius_m)
    assert (bounded.x, bounded.y) == pytest.approx((goal.x, goal.y), abs=1e-8)
    # Circumradius expansion protects the physical footprint at any car yaw.
    for degrees in range(0, 360, 5):
        assert r.config.inside(Pose(goal.x, goal.y, goal.z, math.radians(degrees)))


def test_fixed_world_fence_does_not_follow_a_new_session_origin():
    r = load_runtime()
    c = r.config
    original = asdict(c)
    far_goal = Pose(r.initial_pose.x+20., r.initial_pose.y+20., r.initial_pose.z, r.initial_pose.yaw)
    bounded = c.project_goal(far_goal, r.initial_pose, c.max_goal_radius_m)
    assert c.inside(bounded) and not c.inside(far_goal)
    assert math.hypot(bounded.x-r.initial_pose.x, bounded.y-r.initial_pose.y) <= c.max_goal_radius_m
    assert original == asdict(c)


def test_no_other_safety_limits_or_historical_profiles_changed():
    old = Runtime('shadow', str(CONFIG/'hardware_acceptance.wide_wand_2026-10-22.json'))
    new = load_runtime()
    changed = {'arena_x_min', 'arena_x_max', 'arena_y_min', 'arena_y_max',
               'half_length_m', 'half_width_m', 'max_goal_radius_m', 'max_session_s'}
    for key, value in asdict(old.config).items():
        if key not in changed:
            assert getattr(new.config, key) == value
    assert old.config.max_session_s == 30. and old.config.max_goal_radius_m == .5
    assert new.wand_reference_margin_m >= .5


def test_timeout_explanation_does_not_claim_every_profile_stops_after_30_seconds():
    from mecanum_wand_control.operator_guidance import reason_text
    assert reason_text('SESSION_TIME_LIMIT') == '本轮会话已达到配置时限'


@pytest.mark.parametrize('field,value', [('max_session_s', 120.01), ('max_goal_radius_m', 3.01),
                                        ('lease_timeout_s', .3), ('feedback_timeout_s', .5)])
def test_expansion_cannot_relax_other_guards_or_exceed_reviewed_budgets(tmp_path, field, value):
    data = json.loads(PROFILE.read_text())
    data['safety'][field] = value
    raw = json.dumps(data).encode()
    path = tmp_path/'invalid.json'
    path.write_bytes(raw)
    with pytest.raises(ValueError):
        Runtime('hardware', str(path), hashlib.sha256(raw).hexdigest(), CONSENT)
