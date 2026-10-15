import copy
from pathlib import Path
import sys
import subprocess

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import usb_height_fusion_policy as h
import usb_fusion_policy as original
from mavlink_readonly_probe import decode_parameter
from uav_coord_bridge.mavlink_vision import odometry_message


def enc():
    return m.MAVLink(None, srcSystem=250, srcComponent=190)


def test_independent_explicit_xyz_write_and_unchanged_original_boundary():
    msg = h.control_message(enc(), 3)
    assert decode_parameter(msg) == 3
    assert not original.allowed(msg, authorized=True, safe=True, backup_verified=True, clock_verified=True)
    assert not h.allowed(msg, safe=True, backup_verified=True, clock_verified=True)
    assert not h.allowed(msg, authorized=True, safe=True, backup_verified=True)
    assert h.allowed(msg, authorized=True, safe=True, backup_verified=True, clock_verified=True)
    for value in (1, 2, 7, 15, True):
        with pytest.raises(ValueError):
            h.control_message(enc(), value)
    msg.param_value = 3.0
    assert not h.allowed(msg, authorized=True, safe=True, backup_verified=True, clock_verified=True)


@pytest.mark.parametrize('name', ['EKF2_HGT_REF', 'EKF2_BARO_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_POS_Z'])
def test_reference_delay_barometer_and_offsets_are_never_written(name):
    msg = h.control_message(enc(), 3)
    msg.param_id = name
    assert not h.allowed(msg, authorized=True, safe=True, backup_verified=True, clock_verified=True)


def test_restore_independent_of_clock_source_and_no_pose_without_explicit_enable():
    msg = h.control_message(enc(), 0)
    assert h.allowed(msg, authorized=True, restore_safe=True, backup_verified=True)
    assert not h.allowed(msg, authorized=True, restore_safe=False, backup_verified=True)
    pose = odometry_message(enc(), 123, [0, 0, 0], [1, 0, 0, 0])
    assert not h.allowed(pose, authorized=True, safe=True, backup_verified=True)
    assert h.allowed(pose, authorized=True, safe=True, position_authorized=True)


def test_closed_scope_fixed_listener_and_no_control_commands():
    p = dict.fromkeys(h.READ_PARAMS, 0)
    p['EKF2_BARO_CTRL'] = 1
    assert h.fusion_scope_ok(p) and h.fusion_scope_ok({**p, 'EKF2_EV_CTRL': 3})
    for change in ({'EKF2_HGT_REF': 3}, {'EKF2_BARO_CTRL': 0}, {'EKF2_EV_CTRL': 15}):
        assert not h.fusion_scope_ok({**p, **change})
    assert h.allowed(h.fusion_shell_message(enc(), 'height_aid_0'), safe=True)
    assert not h.allowed(h.fusion_shell_message(enc(), 'height_aid_0'))
    for cmd in (400, 176, 185, 241, 246, 511):
        assert not h.allowed(enc().command_long_encode(1, 1, cmd, 0, 1, 0, 0, 0, 0, 0, 0),
                             authorized=True, safe=True, backup_verified=True, clock_verified=True)


def rows():
    flags = dict.fromkeys(('cs_ev_yaw', 'cs_ev_vel', 'cs_in_air', 'fs_bad_pos_n', 'fs_bad_pos_e',
                           'fs_bad_pos_d', 'reject_hor_pos', 'reject_ver_pos'), False)
    flags.update(cs_tilt_align=True, cs_ev_pos=True, cs_ev_hgt=True, cs_baro_hgt=True)
    aid = dict(estimator_instance=0, fusion_enabled=True, fused=True, innovation_rejected=False,
               timestamp=1000, timestamp_sample=900, time_last_fuse=950, test_ratio=[.1, .1])
    row = dict(flags=flags, aid=aid, height_aid={**aid, 'test_ratio': .1,
                                               'observation_variance': .01, 'innovation_variance': .1},
               local={'timestamp': 1050, 'xy_valid': True, 'z_valid': True})
    later = copy.deepcopy(row)
    for key in ('aid', 'height_aid'):
        later[key]['timestamp_sample'] += 100
        later[key]['time_last_fuse'] += 100
    return [row, later]


def test_xyz_proof_requires_both_aids_primary_freshness_and_recursive_progress():
    assert h.fusion_evidence_ok(rows(), 0)
    assert not h.fusion_evidence_ok(rows(), 1)
    for source in ('aid', 'height_aid'):
        r = rows()
        r[0][source]['fused'] = False
        assert not h.fusion_evidence_ok(r, 0)
        r = rows()
        r[0][source]['timestamp'] -= 3000000
        assert not h.fusion_evidence_ok(r, 0)
        r = rows()
        r[1][source] = r[0][source]
        assert not h.fusion_evidence_ok(r, 0)
    for key in ('cs_baro_hgt', 'cs_ev_hgt'):
        r = rows()
        r[0]['flags'][key] = False
        assert not h.fusion_evidence_ok(r, 0)
    r = rows()
    r[0]['local']['z_valid'] = False
    assert not h.fusion_evidence_ok(r, 0)


@pytest.mark.parametrize('extra', [[], ['--horizontal-fusion-bench'], ['--manual-translation-bench']])
def test_cli_height_requires_explicit_physical_confirmation_and_rejects_mixed_modes(tmp_path, extra):
    script = Path(__file__).resolve().parents[1] / 'usb_vicon_receive_bench.py'
    cmd = [sys.executable, str(script), '--port', '/dev/nonexistent_height_test', '--uid2', 'test',
           '--heading-candidate', '/nonexistent_heading', '--board-center-candidate', '/nonexistent_board',
           '--output', str(tmp_path / 'new.json'), '--height-fusion-bench', '--bench-send', *extra]
    if '--manual-translation-bench' in extra:
        cmd.append('--fusion-physical-confirmed')
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 2
    assert not (tmp_path / 'new.json').exists()
    assert any(text in result.stderr for text in ('explicit physical bench confirmation', 'Select one fusion mode',
                                                 'Manual bench requires matching explicit fusion mode'))
