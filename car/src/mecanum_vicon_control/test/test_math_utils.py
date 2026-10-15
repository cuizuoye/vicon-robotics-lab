import math

import pytest

from mecanum_vicon_control.math_utils import (
    clamp,
    quaternion_from_yaw,
    slew,
    world_error_to_body,
    wrap_angle,
    yaw_from_quaternion,
)


def test_angle_round_trip():
    for yaw in (-math.pi, -1.2, 0.0, 1.4, math.pi):
        quaternion = quaternion_from_yaw(yaw)
        recovered = yaw_from_quaternion(*quaternion)
        assert wrap_angle(recovered - yaw) == pytest.approx(0.0, abs=1e-9)


def test_world_error_to_body_at_ninety_degrees():
    forward, left = world_error_to_body(0.0, 1.0, math.pi / 2.0)
    assert forward == pytest.approx(1.0)
    assert left == pytest.approx(0.0, abs=1e-9)


def test_clamp_and_slew():
    assert clamp(2.0, -1.0, 1.0) == 1.0
    assert slew(0.0, 1.0, 0.2) == pytest.approx(0.2)
    assert slew(0.0, -1.0, 0.2) == pytest.approx(-0.2)

