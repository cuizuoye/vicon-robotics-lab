import hashlib
import json
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import incident_guard
import static_fusion_permit as permit
import trs_range_static_policy as p
import trs_static_fusion_policy as legacy
from trs_static_fusion_bench import load_backup
from trs_fusion_restore_only import allowed as restore_allowed


def setup_permit(tmp_path, monkeypatch):
    tools = tmp_path / 'tools'
    tools.mkdir()
    for name in ('trs_static_fusion_bench.py', 'trs_range_static_policy.py', 'trs_static_fusion_policy.py', 'combined_input_loss.py'):
        (tools / name).write_text('test-code')
    incident = tmp_path / 'incident.json'
    incident.write_text(json.dumps({'block_active_benches': True}))
    monkeypatch.setattr(permit, 'TOOLS', tools)
    monkeypatch.setattr(permit, 'INCIDENT', incident)
    monkeypatch.setattr(incident_guard, 'INCIDENT', incident)
    backup = tmp_path / 'backup'
    backup.mkdir()
    (backup / 'parameters_verified.json').write_text('fresh backup fixture')
    acceptance = tmp_path / 'acceptance.json'
    acceptance.write_text(json.dumps({'basic_maintenance_evidence_passed': True}))
    heading, board = tmp_path / 'heading.json', tmp_path / 'board.json'
    heading.write_text('{}')
    board.write_text('{}')
    output = tmp_path / 'result.json'
    record = {
        'schema': 1, 'scope': permit.SCOPE, 'uid2': permit.UID, 'user_authorized': True,
        'issued_unix': 1000, 'expires_unix': 1600,
        'backup_path': str(backup), 'output_path': str(output), 'height_reference': 2,
        'max_pose_stream_seconds': 15, 'max_pose_packets': 480, 'max_fusion_seconds': 10,
        'allowed_parameter_values': {'EKF2_EV_CTRL': [0, 11], 'EKF2_EV_DELAY': [0, 65]},
        'physical_conditions': ['battery_disconnected', 'props_removed', 'usb_only', 'stationary'],
        'heading_path': str(heading), 'board_center_path': str(board), 'acceptance_path': str(acceptance),
    }
    for key, path in {
        'incident_sha256': incident, 'backup_sha256': backup / 'parameters_verified.json',
        'bench_sha256': tools / 'trs_static_fusion_bench.py',
        'policy_sha256': tools / 'trs_range_static_policy.py',
        'message_policy_sha256': tools / 'trs_static_fusion_policy.py',
        'heading_sha256': heading, 'board_center_sha256': board, 'acceptance_sha256': acceptance,
    }.items():
        record[key] = permit.digest(path)
    path = tmp_path / 'permit.json'
    path.write_text(json.dumps(record))
    return path, backup, output, record


def test_single_use_and_general_guard_unchanged(tmp_path, monkeypatch):
    path, backup, output, _ = setup_permit(tmp_path, monkeypatch)
    before = permit.digest(permit.INCIDENT)
    permit.consume(path, backup, output, physical_confirmed=True, now=1100)
    with pytest.raises(FileExistsError):
        permit.consume(path, backup, output, physical_confirmed=True, now=1101)
    with pytest.raises(RuntimeError):
        incident_guard.require_active_bench_clear()
    assert permit.digest(permit.INCIDENT) == before


@pytest.mark.parametrize('content', [json.dumps({'block_props_off_benches': True}), '{}', '{bad json'])
def test_physical_hold_fails_closed_before_permit_consumption(tmp_path, monkeypatch, content):
    path, backup, output, _ = setup_permit(tmp_path, monkeypatch)
    folder = permit.TOOLS / 'known_incidents'
    folder.mkdir()
    (folder / 'physical_safety_hold.json').write_text(content)
    with pytest.raises(ValueError):
        permit.consume(path, backup, output, physical_confirmed=True, now=1100)
    assert not path.with_name(path.name + '.consumed.json').exists()


@pytest.mark.parametrize('key,value', [
    ('uid2', 'another-aircraft'), ('scope', 'flight'), ('height_reference', 0),
    ('max_fusion_seconds', 60), ('expires_unix', 900), ('expires_unix', 100000),
    ('bench_sha256', 'changed'), ('user_authorized', False),
    ('allowed_parameter_values', {'EKF2_HGT_REF': [0, 2]}),
])
def test_bad_permit_rejected_before_consumption(tmp_path, monkeypatch, key, value):
    path, backup, output, record = setup_permit(tmp_path, monkeypatch)
    record[key] = value
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError):
        permit.consume(path, backup, output, physical_confirmed=True, now=1100)
    assert not path.with_name(path.name + '.consumed.json').exists()


def test_no_physical_confirmation_or_wrong_output(tmp_path, monkeypatch):
    path, backup, output, _ = setup_permit(tmp_path, monkeypatch)
    with pytest.raises(ValueError):
        permit.consume(path, backup, output, physical_confirmed=False, now=1100)
    with pytest.raises(ValueError):
        permit.consume(path, backup, tmp_path / 'other.json', physical_confirmed=True, now=1100)


def test_source_loss_requires_separate_prior_verified_fusion_permit(tmp_path, monkeypatch):
    path, backup, output, record = setup_permit(tmp_path, monkeypatch)
    with pytest.raises(ValueError):
        permit.consume(path, backup, output, physical_confirmed=True, source_loss=True, now=1100)
    prior = tmp_path / 'prior_static.json'
    prior.write_text(json.dumps({'static_bench_passed': True}))
    record.update(scope=permit.LOSS_SCOPE, source_loss_test=True,
                  prior_static_verification_path=str(prior),
                  prior_static_verification_sha256=permit.digest(prior),
                  loss_policy_sha256=permit.digest(permit.TOOLS / 'combined_input_loss.py'))
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError):
        permit.consume(path, backup, output, physical_confirmed=True, source_loss=False, now=1100)
    permit.consume(path, backup, output, physical_confirmed=True, source_loss=True, now=1100)
    with pytest.raises(FileExistsError):
        permit.consume(path, backup, output, physical_confirmed=True, source_loss=True, now=1101)


def test_source_loss_rejects_failed_static_prior(tmp_path, monkeypatch):
    path, backup, output, record = setup_permit(tmp_path, monkeypatch)
    prior = tmp_path / 'prior_static.json'
    prior.write_text(json.dumps({'static_bench_passed': False}))
    record.update(scope=permit.LOSS_SCOPE, source_loss_test=True,
                  prior_static_verification_path=str(prior),
                  prior_static_verification_sha256=permit.digest(prior),
                  loss_policy_sha256=permit.digest(permit.TOOLS / 'combined_input_loss.py'))
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='Prior static fusion'):
        permit.consume(path, backup, output, physical_confirmed=True, source_loss=True, now=1100)


def range_baseline():
    values = {name: 0 for name in p.READ_PARAMS}
    values.update(EKF2_HGT_REF=2, EKF2_BARO_CTRL=1)
    return values


def test_new_range_profile_does_not_relax_legacy():
    baseline = range_baseline()
    assert p.scope_ok(baseline, baseline)
    assert not legacy.scope_ok(baseline, baseline)
    assert p.scope_ok({**baseline, 'EKF2_EV_CTRL': 11, 'EKF2_EV_DELAY': 65}, baseline)
    for name, value in [('EKF2_HGT_REF', 0), ('EKF2_BARO_CTRL', 0), ('EKF2_GPS_CTRL', 7),
                        ('EKF2_EV_CTRL', 15), ('EKF2_EV_DELAY', 100), ('EKF2_EV_POS_X', 0.1)]:
        assert not p.scope_ok({**baseline, name: value}, baseline)


def test_range_restore_still_zero_only_and_no_control():
    enc = m.MAVLink(None)
    for name in ('EKF2_EV_CTRL', 'EKF2_EV_DELAY'):
        assert restore_allowed(p.parameter_message(enc, name, 0), safe=True, backup_verified=True)
    for name, value in [('EKF2_EV_CTRL', 11), ('EKF2_EV_DELAY', 65)]:
        assert not restore_allowed(p.parameter_message(enc, name, value), safe=True, backup_verified=True)
    with pytest.raises(ValueError):
        p.parameter_message(enc, 'EKF2_HGT_REF', 0)


def test_range_backup_is_opt_in(tmp_path):
    values = range_baseline()
    values.update({f'P{i}': 0 for i in range(926 - len(values))})
    record = {'complete': True, 'received_count': 926, 'expected_count': 926,
              'missing_indices': [], 'firmware': {'version': '1.14.3'},
              'system_id': 1, 'component_id': 1,
              'records': [{'name': n, 'index': i, 'value': v} for i, (n, v) in enumerate(values.items())]}
    raw = json.dumps(record).encode()
    (tmp_path / 'parameters_verified.json').write_bytes(raw)
    (tmp_path / 'parameters_manifest.json').write_text(json.dumps({'sha256': {
        'parameters_verified.json': hashlib.sha256(raw).hexdigest()}}))
    with pytest.raises(ValueError):
        load_backup(tmp_path)
    assert load_backup(tmp_path, range_reference_static=True)['EKF2_HGT_REF'] == 2
