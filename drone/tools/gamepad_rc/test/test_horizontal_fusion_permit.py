import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import horizontal_fusion_permit as p


def request(**kwargs):
    r = dict(bench_send=True, horizontal_fusion_bench=True, fusion_backup=Path('/fixture'),
             seconds=10, clock_settle_seconds=180, uid2=p.UID)
    r.update(kwargs)
    return SimpleNamespace(**r)


@pytest.mark.parametrize('change', [dict(height_fusion_bench=True), dict(yaw_fusion_bench=True),
    dict(combined_fusion_bench=True), dict(restore_horizontal_only=True), dict(clock_probe_only=True),
    dict(fusion_physical_confirmed=True), dict(manual_translation_bench=True), dict(seconds=11),
    dict(bench_send=False), dict(fusion_backup=None), dict(receive_only_permit='another')])
def test_only_exact_horizontal_request_permitted(change):
    p.validate_request(request())
    with pytest.raises(ValueError):
        p.validate_request(request(**change))


def test_deadline_is_from_enable_attempt_and_does_not_block_restore():
    w = p.FusionWindow()
    w.check(100)
    w.start(101)
    w.check(110.999)
    with pytest.raises(p.WindowExpired):
        w.check(111)
    w.check(120, restoring=True)
    w.restored()
    w.check(121)
    with pytest.raises(ValueError):
        w.start(122)


def make_fixture(tmp_path, monkeypatch):
    monkeypatch.setattr(p, 'TOOLS', tmp_path)
    (tmp_path/'known_incidents').mkdir()
    for name, value in [('micoair743_v1143_shell_crash.json', {'block_active_benches':True}),
                        ('physical_safety_hold.json', {'block_props_off_benches':True})]:
        (tmp_path/'known_incidents'/name).write_text(json.dumps(value))
    for name in ('usb_vicon_receive_bench.py','receive_only_permit.py','usb_fusion_policy.py',
                 'usb_vision_policy.py','heading','board'):
        (tmp_path/name).touch()
    backup = tmp_path/'backup'; backup.mkdir()
    values = {'EKF2_EV_CTRL':0, 'EKF2_HGT_REF':2, 'COM_RC_IN_MODE':4, 'COM_OBL_RC_ACT':4}
    records = [{'name':k,'value':v} for k,v in values.items()]
    records += [{'name':f'FIXTURE_{i}', 'value':0} for i in range(922)]
    f = backup/'parameters_verified.json'
    f.write_text(json.dumps(dict(complete=True,received_count=926,expected_count=926,
                                firmware={'version':'1.14.3'},records=records)))
    manifest = backup/'parameters_manifest.json'
    manifest.write_text(json.dumps({'sha256':{'parameters_verified.json':p.digest(f)}}))
    now = f.stat().st_mtime
    output = tmp_path/'output.json'
    r = dict(schema=1,scope=p.SCOPE,uid2=p.UID,user_authorized=True,issued_unix=now-1,expires_unix=now+600,
        output_path=str(output),backup_path=str(backup),rate_hz=30,max_pose_packets=301,
        max_fusion_seconds=10,max_clock_stability_wait_seconds=180,height_reference=2,
        allowed_parameter_values={'EKF2_EV_CTRL':[0,1]},
        physical_conditions=['battery_disconnected','usb_only','props_installed','stationary','output_locked'])
    paths = {'runner_sha256':tmp_path/'usb_vicon_receive_bench.py',
        'permit_policy_sha256':Path(p.__file__),'receive_helper_sha256':tmp_path/'receive_only_permit.py',
        'fusion_policy_sha256':tmp_path/'usb_fusion_policy.py','message_policy_sha256':tmp_path/'usb_vision_policy.py',
        'incident_sha256':tmp_path/'known_incidents/micoair743_v1143_shell_crash.json',
        'physical_hold_sha256':tmp_path/'known_incidents/physical_safety_hold.json',
        'heading_sha256':tmp_path/'heading','board_sha256':tmp_path/'board',
        'backup_sha256':f,'backup_manifest_sha256':manifest}
    r.update({k:p.digest(v) for k,v in paths.items()})
    permit = tmp_path/'permit.json'; permit.write_text(json.dumps(r))
    return permit, output, backup, r, now


def test_once_only_and_holds_retained(tmp_path, monkeypatch):
    permit, output, backup, _, now = make_fixture(tmp_path, monkeypatch)
    before = {f:f.read_bytes() for f in (tmp_path/'known_incidents').iterdir()}
    p.consume(permit,output,tmp_path/'heading',tmp_path/'board',backup,now=now)
    with pytest.raises(FileExistsError):
        p.consume(permit,output,tmp_path/'heading',tmp_path/'board',backup,now=now)
    assert all(f.read_bytes()==b for f,b in before.items())


@pytest.mark.parametrize('key,value', [('max_fusion_seconds',11),('height_reference',0),
    ('allowed_parameter_values',{'EKF2_EV_CTRL':[0,11]}),('scope','flight'),('user_authorized',False),
    ('backup_sha256','changed'),('max_pose_packets',1000)])
def test_scope_and_hash_fail_closed(tmp_path,monkeypatch,key,value):
    permit,output,backup,r,now = make_fixture(tmp_path,monkeypatch)
    r[key]=value; permit.write_text(json.dumps(r))
    with pytest.raises(ValueError):
        p.consume(permit,output,tmp_path/'heading',tmp_path/'board',backup,now=now)
    assert not Path(str(permit)+'.consumed.json').exists()
