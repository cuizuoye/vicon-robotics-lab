import datetime
import json
from pathlib import Path
import sys
import time
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rigid_revision_static_permit as p
import rigid_body_revision as revision
import trs_vision_reference_permit as old
from test_trs_vision_reference import fixture as legacy_fixture


def fixture(tmp_path, monkeypatch):
    file, backup, output, _ = legacy_fixture(tmp_path, monkeypatch)
    h, b, proof = [tmp_path/n for n in ('heading', 'board', 'acceptance.json')]
    h.write_text(json.dumps({'rigid_body_revision_id':'new',
        'physical_forward_definition_inherited':True,
        'orientation_candidate_quality':'provisional_geometric_transfer'}))
    b.write_text(json.dumps({'rigid_body_revision_id':'new',
        'old_elevated_marker_height_assumption_reused':False,'imu_offset_verified':False,
        'source_heading':'heading','source_registration':'registration.json'}))
    (tmp_path/'registration.json').write_text(json.dumps({'rigid_body_revision_id':'new',
        'fixed_marker_fit_rms_mm':1.9,'automatic_live_adoption_allowed':False,'source_files':{}}))
    registry = tmp_path/'revision.json'
    registry.write_text(json.dumps({'active_revision_id':'new','heading_candidate_sha256':p.digest(h),
        'board_candidate_sha256':p.digest(b),'static_validation_completed':False}))
    monkeypatch.setattr(revision, 'REGISTRY', registry)
    proof.write_text(json.dumps({'captured_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'static_observation_passed':True,'error':None,'identity':{'uid2':p.UID,'firmware':'1.14.3'},
        'heading_candidate':str(h),'actual_sent_to_fc':0,'median_gravity_axis_difference_deg':1.1,
        'checks':{'all_heartbeats_disarmed':True,'tilt_difference_within_5_deg':True}}))
    fields, paths = p.validate(backup, output, h, b, proof)
    record = {**fields,'issued_unix':time.time()-1,'expires_unix':time.time()+600,
              **{k:p.digest(v) for k,v in paths.items()}}
    file.write_text(json.dumps(record))
    return file,backup,output,h,b,record


def test_one_use_static_permission_never_approves_registry_or_ground_check(tmp_path, monkeypatch):
    f,backup,out,h,b,r = fixture(tmp_path,monkeypatch)
    assert p.consume(f,backup,out,h,b)['max_fusion_seconds'] == 10
    assert not r['ground_command_scope'] and not r['disarmed_offboard_ground_check']
    assert r['allowed_parameter_values'] == {'EKF2_EV_CTRL':[2,11],'EKF2_EV_DELAY':[0,65]}
    assert json.loads(revision.REGISTRY.read_text())['static_validation_completed'] is False
    with pytest.raises(FileExistsError):p.consume(f,backup,out,h,b)


@pytest.mark.parametrize('key,value', [('scope','flight'),('disarmed_offboard_ground_check',True),
    ('max_fusion_seconds',20),('flight_authorized',True),('permits_revision_registry_update',True),
    ('rigid_body_revision_id','old'),('revision_registry_sha256','bad')])
def test_mutations_rejected_before_permission_consumed(tmp_path, monkeypatch,key,value):
    f,backup,out,h,b,r = fixture(tmp_path,monkeypatch)
    f.write_text(json.dumps({**r,key:value}))
    with pytest.raises(ValueError):p.consume(f,backup,out,h,b)
    assert not Path(str(f)+'.consumed.json').exists()


def test_existing_permit_does_not_authorize_pending_revision_validation(tmp_path,monkeypatch):
    f,backup,out,h,b,r = fixture(tmp_path,monkeypatch)
    f.write_text(json.dumps({**r,'scope':old.SCOPE}))
    with pytest.raises(ValueError,match='Wrong revision-static scope'):p.consume(f,backup,out,h,b)
    assert not Path(str(f)+'.consumed.json').exists()


def test_failed_attitude_proof_is_not_a_bypass_even_if_permit_hash_is_updated(tmp_path,monkeypatch):
    f,backup,out,h,b,r = fixture(tmp_path,monkeypatch)
    proof=Path(r['acceptance_path']);a=json.loads(proof.read_text());a['static_observation_passed']=False
    proof.write_text(json.dumps(a));r['acceptance_sha256']=p.digest(proof);f.write_text(json.dumps(r))
    with pytest.raises(ValueError,match='attitude proof'):p.consume(f,backup,out,h,b)


def test_incomplete_backup_rejected(tmp_path,monkeypatch):
    f,backup,out,h,b,r = fixture(tmp_path,monkeypatch)
    path=backup/'parameters_verified.json';a=json.loads(path.read_text());a['records'].pop()
    path.write_text(json.dumps(a))
    with pytest.raises(ValueError,match='parameter backup'):p.consume(f,backup,out,h,b)


def test_cli_rejects_revision_mode_expansion_before_any_devices(tmp_path):
    import subprocess
    runner=Path(__file__).resolve().parents[1]/'trs_static_fusion_bench.py'
    cmd=[sys.executable,str(runner),'--backup','/missing','--heading-candidate','/missing',
        '--board-center-candidate','/missing','--output',str(tmp_path/'never.json'),
        '--vision-reference-permit','/missing','--usb-only-props-installed-vision-reference-confirmed',
        '--rigid-revision-static-validation','--disarmed-offboard-ground-check']
    r=subprocess.run(cmd,capture_output=True,text=True,timeout=5)
    assert r.returncode==2 and 'forbids ground/mode' in r.stderr
    assert not (tmp_path/'never.json').exists()
