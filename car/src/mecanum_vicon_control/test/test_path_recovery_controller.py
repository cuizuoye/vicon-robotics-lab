import math

import pytest
from types import SimpleNamespace
from std_msgs.msg import Bool

from mecanum_vicon_control.path_recovery_controller import (
    RECOVERY,
    TRACKING,
    bounded_world_velocity,
    segment_geometry,
    update_recovery_state,
)


def test_segment_projection_signed_error_and_lookahead():
    geometry = segment_geometry((0.4, 0.2), (0.0, 0.0), (1.0, 0.0), 0.25)
    assert geometry.projection_x == pytest.approx(0.4)
    assert geometry.projection_y == pytest.approx(0.0)
    assert geometry.target_x == pytest.approx(0.65)
    assert geometry.target_y == pytest.approx(0.0)
    assert geometry.progress == pytest.approx(0.4)
    assert geometry.cross_track_m == pytest.approx(0.2)


def test_projection_is_clamped_but_along_error_preserves_overshoot():
    geometry = segment_geometry((1.2, -0.1), (0.0, 0.0), (1.0, 0.0), 0.2)
    assert geometry.projection_x == pytest.approx(1.0)
    assert geometry.target_x == pytest.approx(1.0)
    assert geometry.along_m == pytest.approx(1.2)
    assert geometry.cross_track_m == pytest.approx(-0.1)


def test_degenerate_segment_is_finite_and_complete():
    geometry = segment_geometry((0.3, 0.4), (0.0, 0.0), (0.0, 0.0), 0.2)
    assert geometry.length_m == 0.0
    assert geometry.progress == 1.0
    assert geometry.cross_track_m == pytest.approx(0.5)


def test_recovery_hysteresis_requires_exit_band():
    state, settled = update_recovery_state(TRACKING, 0.09, 0.08, 0.03, 1.0)
    assert state == RECOVERY and settled is None
    state, settled = update_recovery_state(state, 0.04, 0.08, 0.03, 2.0, settled)
    assert state == RECOVERY and settled is None
    state, settled = update_recovery_state(state, 0.02, 0.08, 0.03, 3.0, settled)
    assert state == RECOVERY and settled == 3.0


def test_tracking_velocity_progresses_and_corrects_toward_path():
    geometry = segment_geometry((0.4, 0.1), (0.0, 0.0), (1.0, 0.0), 0.2)
    vx, vy = bounded_world_velocity(
        geometry, TRACKING, 0.1, 0.03, 0.9, 1.2, 0.06, 0.1, 0.1)
    assert vx > 0.0
    assert vy < 0.0
    assert math.hypot(vx, vy) <= 0.1 + 1e-12


def test_recovery_limits_tangent_and_prioritizes_cross_track():
    geometry = segment_geometry((0.4, 0.2), (0.0, 0.0), (1.0, 0.0), 0.2)
    vx, vy = bounded_world_velocity(
        geometry, RECOVERY, 0.1, 0.03, 0.9, 1.2, 0.06, 0.1, 0.1)
    assert 0.0 < vx <= 0.03
    assert vy < 0.0
    assert abs(vy) > vx
    assert math.hypot(vx, vy) <= 0.1 + 1e-12


def test_velocity_reverses_after_segment_overshoot():
    geometry = segment_geometry((1.2, 0.0), (0.0, 0.0), (1.0, 0.0), 0.2)
    vx, vy = bounded_world_velocity(
        geometry, TRACKING, 0.1, 0.03, 0.9, 1.2, 0.06, 0.1, 0.1)
    assert vx < 0.0
    assert vy == pytest.approx(0.0)


@pytest.mark.parametrize('point', [(math.nan, 0), (0, math.inf)])
def test_nonfinite_geometry_rejected(point):
    with pytest.raises(ValueError, match='finite'):
        segment_geometry(point, (0, 0), (1, 0), 0.2)


def test_invalid_hysteresis_rejected():
    with pytest.raises(ValueError, match='thresholds'):
        update_recovery_state(TRACKING, 0.1, 0.03, 0.08, 1.0)


def test_trajectory_enable_preserves_goal_that_arrived_first():
    goal = object()
    node = SimpleNamespace(
        trajectory_active=False,
        pose=None,
        segment_start=None,
        goal=goal,
        fault_latched=True,
        settled_since_s=1.0,
        set_state=lambda state: None,
    )
    from mecanum_vicon_control.path_recovery_controller import PathRecoveryController
    PathRecoveryController.on_trajectory_active(node, Bool(data=True))
    assert node.trajectory_active
    assert node.goal is goal
    assert not node.fault_latched
