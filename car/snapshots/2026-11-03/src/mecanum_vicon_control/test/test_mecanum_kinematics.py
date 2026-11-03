import pytest

from mecanum_vicon_control.mecanum_kinematics import (
    WheelSpeeds,
    body_to_wheel_angular,
    wheel_angular_to_body,
)


RADIUS = 0.03
HALF_LENGTH = 0.10
HALF_WIDTH = 0.08


def test_pure_forward_has_four_equal_positive_wheels():
    wheels = body_to_wheel_angular(
        0.3, 0.0, 0.0, RADIUS, HALF_LENGTH, HALF_WIDTH)
    assert wheels.as_tuple() == pytest.approx((10.0, 10.0, 10.0, 10.0))


def test_left_translation_sign_pattern():
    wheels = body_to_wheel_angular(
        0.0, 0.3, 0.0, RADIUS, HALF_LENGTH, HALF_WIDTH)
    assert wheels.as_tuple() == pytest.approx((-10.0, 10.0, 10.0, -10.0))


def test_counter_clockwise_yaw_sign_pattern():
    wheels = body_to_wheel_angular(
        0.0, 0.0, 1.0, RADIUS, HALF_LENGTH, HALF_WIDTH)
    assert wheels.as_tuple() == pytest.approx((-6.0, 6.0, -6.0, 6.0))


@pytest.mark.parametrize('body', [
    (0.2, -0.1, 0.4),
    (-0.15, 0.08, -0.7),
    (0.0, 0.0, 0.0),
])
def test_inverse_forward_round_trip(body):
    wheels = body_to_wheel_angular(
        *body, RADIUS, HALF_LENGTH, HALF_WIDTH)
    recovered = wheel_angular_to_body(
        wheels, RADIUS, HALF_LENGTH, HALF_WIDTH)
    assert recovered == pytest.approx(body)


def test_common_scale_saturation_preserves_ratios():
    unlimited = body_to_wheel_angular(
        0.4, 0.2, 0.8, RADIUS, HALF_LENGTH, HALF_WIDTH)
    limited = body_to_wheel_angular(
        0.4, 0.2, 0.8, RADIUS, HALF_LENGTH, HALF_WIDTH,
        max_wheel_radps=8.0)
    assert max(abs(value) for value in limited.as_tuple()) == pytest.approx(8.0)
    ratios = [
        limited_value / unlimited_value
        for limited_value, unlimited_value in zip(
            limited.as_tuple(), unlimited.as_tuple())
        if unlimited_value != 0.0
    ]
    assert ratios == pytest.approx([ratios[0]] * len(ratios))


@pytest.mark.parametrize('geometry', [
    (0.0, HALF_LENGTH, HALF_WIDTH),
    (RADIUS, 0.0, 0.0),
    (RADIUS, -0.1, HALF_WIDTH),
])
def test_invalid_geometry_is_rejected(geometry):
    with pytest.raises(ValueError):
        body_to_wheel_angular(0.0, 0.0, 0.0, *geometry)


def test_forward_kinematics_rejects_nonfinite_wheel_speed():
    with pytest.raises(ValueError, match='finite'):
        wheel_angular_to_body(
            WheelSpeeds(float('nan'), 0.0, 0.0, 0.0),
            RADIUS, HALF_LENGTH, HALF_WIDTH)
