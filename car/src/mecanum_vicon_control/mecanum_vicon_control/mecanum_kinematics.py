"""Reference kinematics for an X-configured four-wheel mecanum chassis."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WheelSpeeds:
    """Wheel angular speeds in fixed FL, FR, RL, RR order (rad/s)."""

    front_left: float
    front_right: float
    rear_left: float
    rear_right: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (
            self.front_left,
            self.front_right,
            self.rear_left,
            self.rear_right,
        )


def _validate_geometry(
        wheel_radius_m: float,
        half_length_m: float,
        half_width_m: float,
):
    values = (wheel_radius_m, half_length_m, half_width_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('mecanum geometry must be finite')
    if wheel_radius_m <= 0.0:
        raise ValueError('wheel_radius_m must be positive')
    if half_length_m < 0.0 or half_width_m < 0.0:
        raise ValueError('half dimensions cannot be negative')
    if half_length_m + half_width_m <= 0.0:
        raise ValueError('half_length_m + half_width_m must be positive')


def body_to_wheel_angular(
        vx_mps: float,
        vy_mps: float,
        wz_radps: float,
        wheel_radius_m: float,
        half_length_m: float,
        half_width_m: float,
        max_wheel_radps: float | None = None,
) -> WheelSpeeds:
    """Convert body velocity to X-mecanum wheel angular speeds.

    Coordinates are ROS REP-103: +x forward, +y left and +z/yaw
    counter-clockwise. Positive wheel speed means the wheel drives the robot
    forward. If a limit is supplied, all wheels are scaled by one common
    factor so the requested motion direction is preserved.
    """
    _validate_geometry(wheel_radius_m, half_length_m, half_width_m)
    if not all(math.isfinite(value) for value in (vx_mps, vy_mps, wz_radps)):
        raise ValueError('body velocity must be finite')
    lever = half_length_m + half_width_m
    raw = [
        (vx_mps - vy_mps - lever * wz_radps) / wheel_radius_m,
        (vx_mps + vy_mps + lever * wz_radps) / wheel_radius_m,
        (vx_mps + vy_mps - lever * wz_radps) / wheel_radius_m,
        (vx_mps - vy_mps + lever * wz_radps) / wheel_radius_m,
    ]
    if max_wheel_radps is not None:
        if not math.isfinite(max_wheel_radps) or max_wheel_radps <= 0.0:
            raise ValueError('max_wheel_radps must be finite and positive')
        peak = max(abs(value) for value in raw)
        if peak > max_wheel_radps:
            scale = max_wheel_radps / peak
            raw = [value * scale for value in raw]
    return WheelSpeeds(*raw)


def wheel_angular_to_body(
        wheel_speeds: WheelSpeeds,
        wheel_radius_m: float,
        half_length_m: float,
        half_width_m: float,
) -> tuple[float, float, float]:
    """Forward kinematics used for odometry and reference round-trip tests."""
    _validate_geometry(wheel_radius_m, half_length_m, half_width_m)
    values = wheel_speeds.as_tuple()
    if not all(math.isfinite(value) for value in values):
        raise ValueError('wheel speeds must be finite')
    fl, fr, rl, rr = values
    lever = half_length_m + half_width_m
    vx = wheel_radius_m * (fl + fr + rl + rr) / 4.0
    vy = wheel_radius_m * (-fl + fr + rl - rr) / 4.0
    wz = wheel_radius_m * (-fl + fr - rl + rr) / (4.0 * lever)
    return vx, vy, wz
