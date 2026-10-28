from dataclasses import replace
import math

import pytest

from mecanum_wand_control.core import Pose
from mecanum_wand_control.safety_core import SafetyConfig, ZERO
from mecanum_wand_control.safety_suite import Harness


def config():
    return SafetyConfig(arena_x_min=-.7, arena_x_max=.7, arena_y_min=-.7,
        arena_y_max=.7, half_length_m=.19, half_width_m=.21, goal_boundary_policy='clamp')


@pytest.mark.parametrize('yaw', [0., 1.5164931576573302, math.pi/4, -1.2])
@pytest.mark.parametrize('xy', [(2., 0.), (-2., 0.), (0., 2.), (0., -2.), (2., 2.), (-2., -2.)])
def test_projection_always_preserves_rotated_footprint_and_radius(yaw, xy):
    c = config()
    for origin in (Pose(0., 0., .1, yaw), Pose(.20, .15, .1, yaw)):
        assert c.inside(origin)
        goal = c.project_goal(Pose(*xy, 1.8, -2.), origin, .5)
        assert c.inside(goal)
        assert math.hypot(goal.x-origin.x, goal.y-origin.y) <= .5
        assert goal.z == origin.z and goal.yaw == origin.yaw


@pytest.mark.parametrize('dx,dy', [(1.2, 0.), (-1.2, 0.), (0., 1.2), (0., -1.2), (1.2, 1.2)])
def test_wide_hand_input_saturates_without_latching_and_returns_to_origin(dx, dy):
    h = Harness(config())
    h.start()
    start = h.plant.pose
    for i in range(1, 101):
        h.hand = Pose(-.4+dx*i/100, -.4+dy*i/100, .1+i*.01)
        h.step()
        assert h.sup.state == 'ACTIVE' and not h.gate.fault
        assert h.c.inside(h.sup.follower.goal)
    h.run(4.)
    assert h.sup.follower.goal_limited
    # Reverse to the same absolute origin; no implicit input rebase or
    # unbounded filter wind-up is permitted while held at the boundary.
    for i in range(1, 101):
        h.hand = Pose(-.4+dx*(1-i/100), -.4+dy*(1-i/100), 1.2)
        h.step()
        assert h.sup.state == 'ACTIVE'
    # A diagonal corner is ~0.42m from the origin: at 0.05m/s it needs
    # >8s before convergence/deceleration, not the old single-axis 6s.
    h.run(10.5)
    assert not h.sup.follower.goal_limited and not h.gate.fault
    assert math.dist((h.plant.pose.x, h.plant.pose.y), (start.x, start.y)) < .025
    assert max(r['command_speed_mps'] for r in h.rows) <= .05+1e-9


def test_unsaturated_mapping_is_still_exactly_two_x():
    h = Harness(config())
    h.start()
    h.hand = replace(h.hand, x=-.30, y=-.36, z=1.8)
    h.run(2.)
    assert h.sup.state == 'ACTIVE' and not h.sup.follower.goal_limited
    assert h.sup.follower.goal.x == pytest.approx(.20, abs=1e-6)
    assert h.sup.follower.goal.y == pytest.approx(.08, abs=1e-6)


def test_clamp_never_hides_raw_pose_jump_or_loss():
    for failure in ('jump', 'loss'):
        h = Harness(config())
        h.moving()
        if failure == 'jump': h.hand = replace(h.hand, x=h.hand.x+.3)
        else: h.pose_on['wand'] = False
        h.run(.3)
        assert h.sup.state != 'ACTIVE' and h.output == ZERO


def test_independent_final_gate_still_rejects_outside_goal():
    h = Harness(config())
    h.moving()
    h.modify_frame = lambda f: replace(f, goal=Pose(3., 3., .1))
    h.step()
    assert h.output == ZERO and h.gate.fault == 'GOAL_OUTSIDE_FIXED_ARENA'


def test_clamp_never_allows_actual_car_outside_fixed_arena():
    h = Harness(config())
    h.start()
    h.plant.pose = replace(h.plant.pose, x=.30)
    h.step()
    assert h.output == ZERO and h.sup.state != 'ACTIVE'


def test_unknown_boundary_policy_is_not_silently_accepted():
    with pytest.raises(ValueError): SafetyConfig(goal_boundary_policy='off')
