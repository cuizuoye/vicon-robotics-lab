import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'ros2_ws/src/uav_coord_bridge'))
import vision_reference_xyz_policy as xyz
import vision_reference_fusion_permit as permit
from uav_coord_bridge.mavlink_vision import odometry_message


def enc():
    return m.MAVLink(None, srcSystem=250, srcComponent=190)


def test_only_two_to_three_to_two_no_zero_or_heading_or_reference_writes():
    gates = dict(authorized=True, safe=True, restore_safe=True, backup_verified=True, clock_verified=True)
    for value in (2, 3):
        assert xyz.allowed(xyz.control_message(enc(), value), **gates)
    for value in (0, 1, 8, 11, True):
        with pytest.raises(ValueError):
            xyz.control_message(enc(), value)
    for name in ('EKF2_HGT_REF', 'EKF2_RNG_CTRL', 'COM_ARM_HFLT_CHK'):
        msg = enc().param_set_encode(1, 1, name.encode(), 0, m.MAV_PARAM_TYPE_INT32)
        assert not xyz.allowed(msg, **gates)
    assert not xyz.allowed(xyz.control_message(enc(), 3), **{**gates, 'clock_verified': False})
    assert xyz.allowed(xyz.control_message(enc(), 2), **{**gates, 'safe': False, 'clock_verified': False})
    for key in ('authorized', 'backup_verified', 'restore_safe'):
        assert not xyz.allowed(xyz.control_message(enc(), 2), **{**gates, key: False})


def test_strict_reference_and_source_baseline():
    p = dict.fromkeys(xyz.READ_PARAMS, 0)
    p.update(EKF2_HGT_REF=3, EKF2_EV_CTRL=2, EKF2_BARO_CTRL=1, COM_RC_IN_MODE=4, COM_OBL_RC_ACT=4)
    assert xyz.fusion_scope_ok(p)
    assert xyz.fusion_scope_ok({**p, 'EKF2_EV_CTRL':3})
    for key, value in [('EKF2_HGT_REF', 2), ('EKF2_EV_CTRL', 0), ('EKF2_RNG_CTRL', 1),
                       ('EKF2_OF_CTRL', 1), ('COM_RC_IN_MODE', 0), ('EKF2_EV_POS_Z', .01)]:
        assert not xyz.fusion_scope_ok({**p, key:value})


@pytest.mark.parametrize('command', [176, 185, 241, 246, 400, 511])
def test_never_arm_mode_unlock_or_reboot(command):
    msg = enc().command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0)
    assert not xyz.allowed(msg, authorized=True, safe=True, restore_safe=True, backup_verified=True, clock_verified=True)


def test_pose_requires_explicit_safe_position_permission():
    msg = odometry_message(enc(), 123, [0, 0, 0], [1, 0, 0, 0])
    assert not xyz.allowed(msg, authorized=True, safe=True)
    assert xyz.allowed(msg, authorized=True, safe=True, position_authorized=True)


def request(**changes):
    r = dict(bench_send=True, height_fusion_bench=True, fusion_backup=Path('/fixture'),
             seconds=10, clock_settle_seconds=180, uid2=permit.UID)
    return SimpleNamespace(**{**r, **changes})


@pytest.mark.parametrize('change', [dict(seconds=11), dict(bench_send=False), dict(fusion_backup=None),
    dict(horizontal_fusion_bench=True), dict(yaw_fusion_bench=True), dict(combined_fusion_bench=True),
    dict(restore_height_only=True), dict(clock_probe_only=True), dict(manual_vertical_bench=True),
    dict(fusion_physical_confirmed=True), dict(horizontal_fusion_permit='old')])
def test_permit_only_exact_static_request(change):
    permit.validate_request(request())
    with pytest.raises(ValueError):
        permit.validate_request(request(**change))


def fixture(tmp_path, monkeypatch):
    monkeypatch.setattr(permit, 'TOOLS', tmp_path)
    (tmp_path/'known_incidents').mkdir()
    for name, value in [('micoair743_v1143_shell_crash.json', {'block_active_benches':True}),
                         ('physical_safety_hold.json', {'block_props_off_benches':True})]:
        (tmp_path/'known_incidents'/name).write_text(json.dumps(value))
    for name in ('usb_vicon_receive_bench.py', 'vision_reference_xyz_policy.py',
        'usb_height_fusion_policy.py', 'usb_fusion_policy.py', 'usb_vision_policy.py',
        'receive_only_permit.py', 'horizontal_fusion_permit.py', 'heading', 'board'):
        (tmp_path/name).touch()
    backup = tmp_path/'backup'; backup.mkdir()
    values = {'EKF2_HGT_REF':3, 'EKF2_EV_CTRL':2, 'EKF2_RNG_CTRL':0, 'COM_RC_IN_MODE':4, 'COM_OBL_RC_ACT':4}
    rows = [{'name':k,'value':v,'type':6} for k,v in values.items()]
    rows += [{'name':f'FIXTURE_{i}','value':0,'type':6} for i in range(921)]
    f = backup/'parameters_verified.json'
    f.write_text(json.dumps(dict(complete=True, expected_count=926, received_count=926,
                                firmware={'version':'1.14.3'}, records=rows)))
    (backup/'parameters_manifest.json').write_text(json.dumps({'sha256':{'parameters_verified.json':permit.digest(f)}}))
    now = f.stat().st_mtime
    output = tmp_path/'result.json'
    record = {**permit.fields(output, backup), 'issued_unix':now-1, 'expires_unix':now+600,
              **{k:permit.digest(p) for k,p in permit.paths(tmp_path/'heading', tmp_path/'board', backup).items()}}
    file = tmp_path/'permit.json'; file.write_text(json.dumps(record))
    return file, output, backup, record, now


def test_single_use_keeps_old_holds(tmp_path, monkeypatch):
    file, output, backup, record, now = fixture(tmp_path, monkeypatch)
    holds = {f:f.read_bytes() for f in (tmp_path/'known_incidents').iterdir()}
    permit.consume(file, output, tmp_path/'heading', tmp_path/'board', backup, now=now)
    with pytest.raises(FileExistsError):
        permit.consume(file, output, tmp_path/'heading', tmp_path/'board', backup, now=now)
    assert all(f.read_bytes() == content for f,content in holds.items())


@pytest.mark.parametrize('key,value', [('restore_value',0), ('height_reference',0),
    ('allowed_parameter_values',{'EKF2_EV_CTRL':[0,3]}), ('scope','flight'), ('user_authorized',False),
    ('max_pose_packets',1000), ('runner_sha256','bad')])
def test_scope_and_hash_fail_closed(tmp_path, monkeypatch, key, value):
    file, output, backup, record, now = fixture(tmp_path, monkeypatch)
    file.write_text(json.dumps({**record, key:value}))
    with pytest.raises(ValueError):
        permit.consume(file, output, tmp_path/'heading', tmp_path/'board', backup, now=now)
    assert not Path(str(file)+'.consumed.json').exists()


def test_expired_permit(tmp_path, monkeypatch):
    file, output, backup, record, now = fixture(tmp_path, monkeypatch)
    with pytest.raises(ValueError):
        permit.consume(file, output, tmp_path/'heading', tmp_path/'board', backup, now=now+1000)
