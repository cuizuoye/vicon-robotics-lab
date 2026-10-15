import ctypes as ct
from dataclasses import astuple, replace
import json
import math
from pathlib import Path
import random
import socket
import subprocess

import pytest

from mecanum_vicon_control.drive_plan import (
    BodyVelocity, ChassisGeometry, DriveLimits, WheelCalibration,
    body_from_raw_cps, load_adopted_wheels, load_geometry, main, plan_body_velocity,
)


ROOT = Path(__file__).resolve().parents[3]
MCU = ROOT / 'stm32/mecanum_f407'
GEOMETRY = ChassisGeometry(122.0, 170.0, 60.0, True, True)
CPR = (3179/3, 1049, 3167/3, 1061)
SIGNS = (1, -1, 1, -1)
WHEELS = tuple(WheelCalibration(name, cpr, sign, sign, 0.06, True)
               for name, cpr, sign in zip(('FL', 'FR', 'RL', 'RR'), CPR, SIGNS))


@pytest.mark.parametrize('command, expected', [
    ((0.05, 0, 0), (1, 1, 1, 1)),
    ((-0.05, 0, 0), (-1, -1, -1, -1)),
    ((0, 0.03, 0), (-1, 1, 1, -1)),
    ((0, -0.03, 0), (1, -1, -1, 1)),
    ((0, 0, 0.1), (-1, 1, -1, 1)),
    ((0, 0, -0.1), (1, -1, 1, -1)),
    ((0, 0, 0), (0, 0, 0, 0)),
])
def test_direction_patterns_and_single_sign_conversion(command, expected):
    plan = plan_body_velocity(GEOMETRY, WHEELS, BodyVelocity(*command))
    assert tuple(1 if v > 0 else -1 if v < 0 else 0 for v in plan.forward_cps) == expected
    assert plan.motor_direction_reference == tuple(a*b for a, b in zip(expected, SIGNS))
    assert plan.expected_raw_encoder_cps == pytest.approx(tuple(a*b for a, b in zip(plan.forward_cps, SIGNS)))
    assert plan.common_scale == pytest.approx(1.0)


def test_forward_uses_distinct_cpr_not_one_wheel_copy():
    plan = plan_body_velocity(GEOMETRY, WHEELS, BodyVelocity(0.05, 0, 0))
    assert plan.wheel_m_s == pytest.approx((0.05,)*4)
    assert plan.forward_cps == pytest.approx(tuple(0.05*c/(math.pi*0.06) for c in CPR))
    assert len(set(plan.forward_cps)) == 4


def test_cpr_limit_scales_all_wheels_together():
    wheels = (replace(WHEELS[0], counts_per_rev=5000),) + WHEELS[1:]
    plan = plan_body_velocity(GEOMETRY, wheels, BodyVelocity(0.05, 0, 0))
    assert plan.common_scale < 1.0
    assert plan.forward_cps[0] == pytest.approx(300)
    assert plan.wheel_m_s == pytest.approx((plan.wheel_m_s[0],)*4)
    assert max(abs(v) for v in plan.forward_cps) <= 300


@pytest.mark.parametrize('command', [(0.2, 0.1, 0.4), (-0.4, 0.2, -0.8), (0, 0, 0), (0, 0, 1)])
def test_limited_model_round_trip(command):
    plan = plan_body_velocity(GEOMETRY, WHEELS, BodyVelocity(*command))
    recovered = body_from_raw_cps(GEOMETRY, WHEELS, plan.expected_raw_encoder_cps)
    assert astuple(recovered) == pytest.approx(astuple(plan.applied_body), abs=1e-12)
    assert max(abs(v) for v in plan.wheel_m_s) <= 0.05
    assert max(abs(v) for v in plan.forward_cps) <= 300


@pytest.mark.parametrize('changes', [
    {'dimensions_confirmed': False}, {'x_layout_confirmed': False},
    {'wheelbase_mm': None}, {'track_width_mm': 0}, {'track_width_mm': 2001},
    {'wheel_diameter_mm': float('nan')}, {'wheelbase_mm': True},
])
def test_invalid_geometry_is_not_ready(changes):
    with pytest.raises(ValueError):
        plan_body_velocity(replace(GEOMETRY, **changes), WHEELS, BodyVelocity(0.05, 0, 0))


@pytest.mark.parametrize('changes', [
    {'counts_per_rev': 0}, {'counts_per_rev': 0.5}, {'counts_per_rev': 10**500},
    {'counts_per_rev': float('inf')}, {'encoder_sign': 0}, {'motor_sign': True},
    {'adopted': False}, {'diameter_m': 0.08}, {'name': 'RR'},
])
def test_invalid_wheel_is_not_ready(changes):
    wheels = (replace(WHEELS[0], **changes),) + WHEELS[1:]
    with pytest.raises(ValueError):
        plan_body_velocity(GEOMETRY, wheels, BodyVelocity(0.05, 0, 0))


@pytest.mark.parametrize('limits', [DriveLimits(0.051), DriveLimits(0), DriveLimits(target_limit_cps=301),
                                        DriveLimits(feedback_limit_cps=10001)])
def test_limits_cannot_expand_bench_bounds(limits):
    with pytest.raises(ValueError):
        plan_body_velocity(GEOMETRY, WHEELS, BodyVelocity(0.05, 0, 0), limits)


@pytest.mark.parametrize('command', [(float('nan'), 0, 0), (1.1, 0, 0), (0, 0, 11)])
def test_invalid_body_request(command):
    with pytest.raises(ValueError):
        plan_body_velocity(GEOMETRY, WHEELS, BodyVelocity(*command))


@pytest.mark.parametrize('raw', [(0, 0, 0), (0, float('nan'), 0, 0), (10001, 0, 0, 0)])
def test_feedback_rejects_bad_channels(raw):
    with pytest.raises(ValueError):
        body_from_raw_cps(GEOMETRY, WHEELS, raw)


def test_geometry_metadata_cannot_contradict_x_layout(monkeypatch):
    data = {'schema_version': 1, 'roller_layout': 'X', 'x_layout_confirmed': True,
            'observed_roller_axes_top_view': {'FL': '\\', 'FR': '\\', 'RL': '/', 'RR': '/'}}
    monkeypatch.setattr(Path, 'read_text', lambda *a, **k: json.dumps(data))
    with pytest.raises(ValueError, match='conflict'):
        load_geometry('unused.json')
    data['schema_version'] = True
    with pytest.raises(ValueError, match='schema'):
        load_geometry('unused.json')


def test_dryrun_opens_no_network_and_never_authorizes_motion(monkeypatch, capsys):
    def forbidden(*a, **k):
        raise AssertionError('offline planner attempted a socket')
    monkeypatch.setattr(socket, 'socket', forbidden)
    monkeypatch.setattr('mecanum_vicon_control.drive_plan.load_geometry', lambda p: GEOMETRY)
    monkeypatch.setattr('mecanum_vicon_control.drive_plan.load_adopted_wheels', lambda p: WHEELS)
    args = ['--geometry', 'unused', '--wheel-profile', 'unused']
    assert main(args) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['planning_ready'] is True
    assert report['motion_authorized'] is False
    assert report['serial_opened'] is report['network_opened'] is False
    assert main(args + ['--vx', 'nan']) == 2
    assert json.loads(capsys.readouterr().out)['planning_ready'] is False
    monkeypatch.setattr('mecanum_vicon_control.drive_plan.load_geometry',
                        lambda p: replace(GEOMETRY, x_layout_confirmed=False))
    assert main(args) == 2
    assert json.loads(capsys.readouterr().out)['motion_authorized'] is False


def test_existing_adopted_profile_has_expected_order_and_signs():
    wheels = load_adopted_wheels(ROOT / 'docs/four_speed_bench_adopted_profile_20261009.json')
    assert tuple(w.name for w in wheels) == ('FL', 'FR', 'RL', 'RR')
    assert tuple(w.counts_per_rev for w in wheels) == pytest.approx(CPR)
    assert tuple(w.motor_sign for w in wheels) == SIGNS


class CBody(ct.Structure):
    _fields_ = [('vx_m_s', ct.c_float), ('vy_m_s', ct.c_float), ('wz_rad_s', ct.c_float)]


class CKinematics(ct.Structure):
    _fields_ = [('wheel_radius_m', ct.c_float), ('half_length_m', ct.c_float), ('half_width_m', ct.c_float)]


class CGeometry(ct.Structure):
    _fields_ = [('kinematics', CKinematics), ('dimensions_confirmed', ct.c_bool), ('x_layout_confirmed', ct.c_bool)]


class CWheel(ct.Structure):
    _fields_ = [('counts_per_rev', ct.c_float), ('diameter_m', ct.c_float),
                ('motor_sign', ct.c_int8), ('encoder_sign', ct.c_int8), ('calibration_confirmed', ct.c_bool),
                ('kp_permille_per_cps', ct.c_float), ('ki_permille_per_count', ct.c_float)]


class CConfig(ct.Structure):
    _fields_ = [('wheel', CWheel*4), ('target_limit_cps', ct.c_float), ('feedback_limit_cps', ct.c_float),
                ('ramp_cps_per_s', ct.c_float), ('feedback_filter_ms', ct.c_float), ('duty_limit_permille', ct.c_int16),
                ('sample_period_ms', ct.c_uint32), ('max_sample_gap_ms', ct.c_uint32),
                ('command_timeout_ms', ct.c_uint32), ('run_limit_ms', ct.c_uint32)]


class CPlan(ct.Structure):
    _fields_ = [('wheel_m_s', ct.c_float*4), ('forward_cps', ct.c_float*4),
                ('expected_raw_encoder_cps', ct.c_float*4), ('motor_direction_reference', ct.c_int8*4),
                ('applied_body', CBody), ('common_scale', ct.c_float)]


@pytest.fixture(scope='module')
def native(tmp_path_factory):
    library = tmp_path_factory.mktemp('native_drive_plan') / 'drive_plan.so'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-shared', '-fPIC',
                    '-I'+str(MCU/'Inc'), str(MCU/'Src/mecanum_drive_plan.c'),
                    str(MCU/'Src/mecanum_kinematics.c'), str(MCU/'Src/four_wheel_speed.c'),
                    '-lm', '-o', str(library)], check=True, capture_output=True, timeout=30)
    lib = ct.CDLL(str(library))
    lib.four_speed_defaults.argtypes = [ct.POINTER(CConfig)]
    lib.four_speed_defaults.restype = None
    lib.mecanum_drive_plan.argtypes = [ct.POINTER(CGeometry), ct.POINTER(CConfig), ct.POINTER(CBody),
                                      ct.c_float, ct.POINTER(CPlan)]
    lib.mecanum_drive_plan.restype = ct.c_bool
    lib.mecanum_drive_body_from_raw_cps.argtypes = [ct.POINTER(CGeometry), ct.POINTER(CConfig),
                                                  ct.POINTER(ct.c_float), ct.POINTER(CBody)]
    lib.mecanum_drive_body_from_raw_cps.restype = ct.c_bool
    return lib


def c_inputs(native, wheels=WHEELS):
    config = CConfig()
    native.four_speed_defaults(ct.byref(config))
    config.target_limit_cps = 300
    for i, wheel in enumerate(wheels):
        config.wheel[i].counts_per_rev = wheel.counts_per_rev
        config.wheel[i].diameter_m = wheel.diameter_m
        config.wheel[i].motor_sign = wheel.motor_sign
        config.wheel[i].encoder_sign = wheel.encoder_sign
        config.wheel[i].calibration_confirmed = wheel.adopted
    geometry = CGeometry(CKinematics(0.03, 0.061, 0.085), True, True)
    return geometry, config


def test_2000_seeded_c_python_plan_and_feedback_parity(native):
    rng = random.Random(20261010)
    for i in range(2000):
        wheels = WHEELS if i % 2 else (replace(WHEELS[0], counts_per_rev=5000),) + WHEELS[1:]
        request = BodyVelocity(rng.uniform(-0.75, 0.75), rng.uniform(-0.75, 0.75), rng.uniform(-2, 2))
        expected = plan_body_velocity(GEOMETRY, wheels, request)
        geometry, config = c_inputs(native, wheels)
        body, result = CBody(*astuple(request)), CPlan()
        assert native.mecanum_drive_plan(ct.byref(geometry), ct.byref(config), ct.byref(body), 0.05, ct.byref(result)), request
        assert tuple(result.wheel_m_s) == pytest.approx(expected.wheel_m_s, abs=1e-7)
        assert tuple(result.forward_cps) == pytest.approx(expected.forward_cps, abs=0.002)
        assert tuple(result.expected_raw_encoder_cps) == pytest.approx(expected.expected_raw_encoder_cps, abs=0.002)
        assert tuple(result.motor_direction_reference) == expected.motor_direction_reference
        assert result.common_scale == pytest.approx(expected.common_scale, abs=1e-6)
        recovered = CBody()
        assert native.mecanum_drive_body_from_raw_cps(ct.byref(geometry), ct.byref(config),
                                                      result.expected_raw_encoder_cps, ct.byref(recovered))
        assert (recovered.vx_m_s, recovered.vy_m_s, recovered.wz_rad_s) == pytest.approx(astuple(expected.applied_body), abs=2e-6)


def test_c_failures_clear_all_output_fields(native):
    geometry, config = c_inputs(native)
    body, result = CBody(0.05, 0, 0), CPlan()
    ct.memset(ct.byref(result), 0x55, ct.sizeof(result))
    geometry.x_layout_confirmed = False
    assert not native.mecanum_drive_plan(ct.byref(geometry), ct.byref(config), ct.byref(body), 0.05, ct.byref(result))
    assert bytes(result) == bytes(ct.sizeof(result))
