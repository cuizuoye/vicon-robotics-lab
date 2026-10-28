"""New boundary tests; all encoders and cars are software-only."""
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys

import pytest

OUT = Path(os.environ['SYNC_OUTPUT'])
ROOT = Path(os.environ['PUBLIC_CAR_ROOT'])
sys.path.insert(0, str(OUT))
from test_reused_sync import Harness, config, arm, ZERO, Pose
from mecanum_wand_4wsa_hardware.stage0_contract import decode_status, SIZE
from mecanum_wand_4wsa_control.demo_policy import POLICY


@pytest.fixture(autouse=True)
def no_io(monkeypatch):
    import socket
    import subprocess
    import rclpy
    def denied(*a, **kw):
        pytest.fail('Long-session offline test attempted hardware or ROS process IO')
    monkeypatch.setattr(socket, 'socket', denied)
    monkeypatch.setattr(subprocess, 'Popen', denied)
    monkeypatch.setattr(rclpy, 'init', denied)


@pytest.mark.parametrize('stage', range(7))
def test_duration_and_yaw_profiles_are_consistent(stage):
    c = config(stage)
    assert c.max_session_s == POLICY['max_active_s'] == 600.
    assert c.feedback_timeout_s == .30
    assert c.pose_timeout_s == .20 and c.command_timeout_s == .10 and c.lease_timeout_s == .15
    assert c.follower_config().workspace_radius_enabled is False
    assert c.goal_boundary_policy == 'tracking'
    for field, value in [('max_session_s', 599.), ('max_session_s', 601.),
                         ('feedback_timeout_s', .31), ('command_timeout_s', .2),
                         ('pose_timeout_s', .3), ('lease_timeout_s', .2),
                         ('max_control_gap_s', .2), ('tilt_limit_rad', .5)]:
        with pytest.raises(ValueError):
            replace(c, **{field: value})


def test_full600s_virtual_time_and_no_automatic_rearm():
    h = Harness(stage=4)
    h.start()
    h.run(151.)
    assert h.sup.state == 'ACTIVE' and not h.gate.fault
    h.run(448.90)
    assert h.sup.state == 'ACTIVE' and not h.gate.fault
    assert h.sup.pilot_status(h.now)['remaining_s'] == pytest.approx(.1, abs=.021)
    h.run(.14)
    assert h.sup.state != 'ACTIVE' and h.output == ZERO
    h.held = False
    h.run(.8)
    assert h.output == ZERO and not h.sup.request_start(h.now, h.ros)[0]


def test_independent_final_gate600s_budget_not_reset_by_supervisor():
    h = Harness()
    h.start()
    h.gate.pilot.started = h.now-600.
    h.step()
    assert h.output == ZERO
    assert '600S' in h.gate.fault or '600S' in h.gate.pilot.reason


@pytest.mark.parametrize('stage', range(7))
def test_actual_firmware_reports630s_and_rejects_mismatched_budget(stage):
    oracle, session = arm(stage)
    payload = oracle.packet().payload
    status = decode_status(payload)
    assert SIZE == len(payload) == 204
    assert status['tag'] == '4WSA' and status['absolute_run_limit_ms'] == 630000
    assert status['output_authority_version'] == 6
    for budget in (0, 150000, 600000, 630001):
        raw = bytearray(payload)
        struct.pack_into('<I', raw, 200, budget)
        with pytest.raises(ValueError):
            decode_status(raw)
    with pytest.raises(ValueError):
        decode_status(payload[:200])


def test_no_fixed_arena_does_not_disable_pose_validation():
    c = config(4)
    origin = Pose(0, 0, c.ground_z_m, 0)
    far = Pose(50, -50, c.ground_z_m, 2.)
    assert c.project_goal(far, origin, c.max_goal_radius_m) == far
    assert c.inside(far) and not c.inside(Pose(math.nan, 0))
    assert c.goal_polygon() == c.fence_polygon() == ()


def test_600s_is_not_a_per_sample_deadline_relaxation():
    h = Harness()
    h.start()
    h.run(151.)
    h.pose_on['wand'] = False
    h.run(.24)
    assert h.output == ZERO and h.sup.state != 'ACTIVE'


def test_old4ws9_reader_rejects_new_wire_and_new_reader_rejects_old():
    parent = ROOT/'docs/wand_sync_4ws9_2026-10-27/prepared/runtime/src'
    sys.path.insert(0, str(parent/'mecanum_wand_4ws9_hardware'))
    from mecanum_wand_4ws9_hardware.stage0_contract import decode_status as old_decode
    oracle, _ = arm(4)
    raw = oracle.packet().payload
    with pytest.raises(ValueError):
        old_decode(raw)
    old = bytearray(raw[:200])
    old[:4] = b'4WS9'
    old[153] = 5
    with pytest.raises(ValueError):
        decode_status(old)


def test_all_pinned_current_entries_and_parent_sources_unchanged():
    m = json.loads((OUT/'build_manifest.json').read_text())
    for name, digest in m['protected_input_hashes'].items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == digest, name


def test_templates_grant_no_flash_or_motion_authority():
    from mecanum_wand_4wsa_hardware.authorization import verify_runtime_profile, verify_flash_record
    for stage in range(7):
        p = OUT/f'runtime/src/mecanum_wand_4wsa_hardware/config/stage{stage}.NOT_APPROVED.json'
        profile = json.loads(p.read_text())
        assert profile['trial_review']['human_authorized'] is False
        assert profile['long_session_qualified'] is False
        with pytest.raises((ValueError, KeyError)):
            verify_runtime_profile(profile, 'sign')
    with pytest.raises(ValueError):
        verify_flash_record(None, None)


def test_runtime_constructor_has_no_dormant120s_gate(tmp_path, monkeypatch):
    # Constructor-only IO-free test. The test-local approval stub is NEVER
    # installed into generated source and produces no real admission record.
    from mecanum_wand_4wsa_hardware import commissioning
    profile = json.loads((OUT/'runtime/src/mecanum_wand_4wsa_hardware/config/stage4.NOT_APPROVED.json').read_text())
    profile['confirmations'] = {k: True for k in profile['confirmations']}
    profile['tracking_confirmations'] = {k: True for k in profile['tracking_confirmations']}
    path = tmp_path/'constructor_fixture_not_hardware_approval.json'
    path.write_text(json.dumps(profile))
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    monkeypatch.setattr(commissioning, 'verify_runtime_profile', lambda *a, **kw: True)
    runtime = commissioning.Runtime('hardware', str(path), digest, commissioning.CONSENT)
    assert runtime.config.max_session_s == 600
    assert runtime.config.max_yaw_rate_radps == 1.5
    assert runtime.motion_status_size == 204
