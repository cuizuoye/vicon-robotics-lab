"""Offline body-velocity planning reference. No ROS, serial or network output."""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

from .mecanum_kinematics import (
    WheelSpeeds, body_to_wheel_angular, wheel_angular_to_body,
)


WHEEL_NAMES = ('FL', 'FR', 'RL', 'RR')


@dataclass(frozen=True)
class ChassisGeometry:
    wheelbase_mm: float | None
    track_width_mm: float | None
    wheel_diameter_mm: float = 60.0
    dimensions_confirmed: bool = False
    x_layout_confirmed: bool = False


@dataclass(frozen=True)
class WheelCalibration:
    name: str
    counts_per_rev: float
    motor_sign: int
    encoder_sign: int
    diameter_m: float = 0.06
    adopted: bool = False


@dataclass(frozen=True)
class BodyVelocity:
    vx_m_s: float
    vy_m_s: float
    wz_rad_s: float


@dataclass(frozen=True)
class DriveLimits:
    max_wheel_m_s: float = 0.05
    target_limit_cps: float = 300.0
    feedback_limit_cps: float = 10000.0


@dataclass(frozen=True)
class DrivePlan:
    wheel_m_s: tuple[float, ...]
    forward_cps: tuple[float, ...]
    expected_raw_encoder_cps: tuple[float, ...]
    motor_direction_reference: tuple[int, ...]
    applied_body: BodyVelocity
    common_scale: float


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f'{name} must be a finite number')
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(result):
        raise ValueError(f'{name} must be a finite number')
    return result


def _validated(geometry, wheels, limits):
    # RViz示意尺寸不用于实车规划；CPR采用不等于尺寸/X排列或运动授权。
    if geometry.dimensions_confirmed is not True or geometry.x_layout_confirmed is not True:
        raise ValueError('measured dimensions and X wheel layout must both be confirmed')
    length = _number(geometry.wheelbase_mm, 'wheelbase_mm') / 2000.0
    width = _number(geometry.track_width_mm, 'track_width_mm') / 2000.0
    radius = _number(geometry.wheel_diameter_mm, 'wheel_diameter_mm') / 2000.0
    if not (0.0 < length <= 1.0 and 0.0 < width <= 1.0 and 0.001 <= radius <= 1.0):
        raise ValueError('chassis geometry outside the planning range')
    speed = _number(limits.max_wheel_m_s, 'max_wheel_m_s')
    target = _number(limits.target_limit_cps, 'target_limit_cps')
    feedback = _number(limits.feedback_limit_cps, 'feedback_limit_cps')
    if not (0.0 < speed <= 0.05 and 0.0 < target <= 300.0 and target <= feedback <= 10000.0):
        raise ValueError('planning limits exceed the restricted bench bounds')
    if len(wheels) != 4 or tuple(w.name for w in wheels) != WHEEL_NAMES:
        raise ValueError('wheel order must be FL, FR, RL, RR')
    for wheel in wheels:
        cpr = _number(wheel.counts_per_rev, 'counts_per_rev')
        diameter = _number(wheel.diameter_m, 'diameter_m')
        if (wheel.adopted is not True or not 1.0 <= cpr <= 10000000.0 or
                abs(diameter - 2.0 * radius) > 0.000001 or
                type(wheel.motor_sign) is not int or wheel.motor_sign not in (-1, 1) or
                type(wheel.encoder_sign) is not int or wheel.encoder_sign not in (-1, 1)):
            raise ValueError('invalid/unadopted per-wheel CPR, diameter or sign')
    return radius, length, width


def plan_body_velocity(geometry, wheels, requested, limits=DriveLimits()):
    """Return normalized PI targets, not PWM, an enable or a transmitted command."""
    radius, length, width = _validated(geometry, wheels, limits)
    velocity = tuple(_number(v, 'body velocity') for v in
                     (requested.vx_m_s, requested.vy_m_s, requested.wz_rad_s))
    if abs(velocity[0]) > 1.0 or abs(velocity[1]) > 1.0 or abs(velocity[2]) > 10.0:
        raise ValueError('body velocity outside the planning range')
    raw = body_to_wheel_angular(*velocity, radius, length, width).as_tuple()
    maximum = limits.max_wheel_m_s / radius
    angular = body_to_wheel_angular(*velocity, radius, length, width,
                                    max_wheel_radps=maximum).as_tuple()
    peak = max(abs(v) for v in raw)
    base_scale = maximum / peak if peak > maximum else 1.0
    speeds = tuple(v * radius for v in angular)
    cps = tuple(v * w.counts_per_rev / (math.pi * w.diameter_m)
                for v, w in zip(speeds, wheels))
    # CPR不同的计数速度上限同样用共同缩放，不能逐轮截断破坏运动方向。
    factors = [1.0]
    factors.extend(limits.max_wheel_m_s / abs(v) for v in speeds
                   if abs(v) > limits.max_wheel_m_s)
    factors.extend(limits.target_limit_cps / abs(v) for v in cps
                   if abs(v) > limits.target_limit_cps)
    extra = min(factors)
    if extra < 1.0:
        extra = math.nextafter(extra, 0.0)
    speeds = tuple(v * extra for v in speeds)
    cps = tuple(v * extra for v in cps)
    if (max(abs(v) for v in speeds) > limits.max_wheel_m_s or
            max(abs(v) for v in cps) > limits.target_limit_cps):
        raise ValueError('numeric limit violation')
    scale = base_scale * extra
    return DrivePlan(
        speeds, cps, tuple(v * w.encoder_sign for v, w in zip(cps, wheels)),
        tuple((1 if v > 0 else -1 if v < 0 else 0) * w.motor_sign
              for v, w in zip(cps, wheels)),
        BodyVelocity(*(v * scale for v in velocity)), scale,
    )


def body_from_raw_cps(geometry, wheels, raw_cps, limits=DriveLimits()):
    """Encoder-model body velocity only; slip and Vicon correction are not modeled."""
    radius, length, width = _validated(geometry, wheels, limits)
    if len(raw_cps) != 4:
        raise ValueError('four raw encoder channels required')
    cps = tuple(_number(v, 'raw_cps') for v in raw_cps)
    if any(abs(v) > limits.feedback_limit_cps for v in cps):
        raise ValueError('raw encoder speed outside feedback limit')
    angular = WheelSpeeds(*(v * w.encoder_sign * 2.0 * math.pi / w.counts_per_rev
                            for v, w in zip(cps, wheels)))
    return BodyVelocity(*wheel_angular_to_body(angular, radius, length, width))


def load_geometry(path):
    payload = json.loads(Path(path).read_text(encoding='utf-8'))
    if (not isinstance(payload, dict) or type(payload.get('schema_version')) is not int or
            payload['schema_version'] != 1):
        raise ValueError('unsupported chassis geometry schema')
    layout = payload.get('roller_layout')
    if payload.get('x_layout_confirmed') is True and layout == 'X':
        expected = {'FL': '\\', 'FR': '/', 'RL': '/', 'RR': '\\'}
        observed = payload.get('observed_roller_axes_top_view', expected)
        if observed != expected:
            raise ValueError('reported roller axes conflict with X layout')
    return ChassisGeometry(
        payload['wheelbase_mm'], payload['track_width_mm'],
        payload['nominal_wheel_diameter_mm'], payload['dimensions_confirmed'],
        payload['x_layout_confirmed'] is True and layout == 'X',
    )


def load_adopted_wheels(path):
    payload = json.loads(Path(path).read_text(encoding='utf-8'))
    diameter = _number(payload['nominal_wheel_diameter_m'], 'nominal_wheel_diameter_m')
    if not isinstance(payload.get('user_decision'), str) or not payload['user_decision'].strip():
        raise ValueError('approximate wheel profile needs the recorded adoption decision')
    # 读取已采用的近似值，不修改原始测量/重复性状态，不构成精密认证或使能授权。
    return tuple(WheelCalibration(w['name'], w['counts_per_rev'], w['motor_sign'],
                                  w['encoder_sign'], diameter, True) for w in payload['wheels'])


def main(argv=None):
    parser = argparse.ArgumentParser(description='Offline mecanum velocity planner; never opens serial/network')
    parser.add_argument('--geometry', required=True)
    parser.add_argument('--wheel-profile', required=True)
    parser.add_argument('--vx', type=float, default=0.05, help='body forward m/s')
    parser.add_argument('--vy', type=float, default=0.0, help='body left m/s')
    parser.add_argument('--wz', type=float, default=0.0, help='body CCW rad/s')
    args = parser.parse_args(argv)
    report = {'scope': 'offline planning only', 'motion_authorized': False,
              'serial_opened': False, 'network_opened': False, 'wheel_order': WHEEL_NAMES}
    try:
        geometry = load_geometry(args.geometry)
        wheels = load_adopted_wheels(args.wheel_profile)
        requested = BodyVelocity(args.vx, args.vy, args.wz)
        plan = plan_body_velocity(geometry, wheels, requested)
        report.update(planning_ready=True, requested_body=asdict(requested), plan=asdict(plan),
                      model_feedback_roundtrip=asdict(body_from_raw_cps(
                          geometry, wheels, plan.expected_raw_encoder_cps)))
        code = 0
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        report.update(planning_ready=False, error=str(exc))
        code = 2
    print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
