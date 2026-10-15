"""One-use pending-revision validation: battery OFF, locked, no modes/targets.

This is NOT a global calibration approval. It binds the new provisional frame
to its independent read-only attitude observation and complete parameter
backup, then permits only the existing 10-second static XYZ/yaw policy.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import time

import trs_vision_reference_permit as base
import trs_vision_reference_policy as policy
import rigid_body_revision as revision
from receive_only_permit import digest, UID

SCOPE = 'battery-off-usb-observed-locked-rebuilt-rigid-frame-static-validation-once'


def files(backup, heading, board, acceptance):
    board_doc = json.loads(Path(board).read_text())
    registration = (Path(board).resolve().parent / board_doc['source_registration']).resolve()
    return {**base.paths(backup, heading, board, acceptance),
        'revision_registry_sha256': revision.REGISTRY,
        'revision_helper_sha256': Path(revision.__file__),
        'revision_permit_sha256': Path(__file__), 'registration_sha256': registration}


def fields(backup, output, heading, board, acceptance):
    registry = revision.inspect_current_calibration(heading, board, registry_path=revision.REGISTRY)
    return {**base.fields(output, backup, heading, board, acceptance),
            'scope': SCOPE, 'rigid_body_revision_id': registry['active_revision_id'],
            'rigid_revision_static_validation': True,
            'permits_revision_registry_update': False, 'flight_authorized': False}


def validate(backup, output, heading, board, acceptance):
    expected = fields(backup, output, heading, board, acceptance)
    h, b, proof = [json.loads(Path(path).read_text()) for path in (heading, board, acceptance)]
    if (not h.get('physical_forward_definition_inherited')
            or h.get('orientation_candidate_quality') != 'provisional_geometric_transfer'
            or b.get('old_elevated_marker_height_assumption_reused') is not False
            or b.get('imu_offset_verified') is not False
            or (Path(board).resolve().parent / b.get('source_heading', '')).resolve() != Path(heading).resolve()):
        raise ValueError('Explicit inherited-forward/new-body approximate reference required')
    captured = datetime.datetime.fromisoformat(proof['captured_at_utc'])
    if (captured.utcoffset() is None or not 0 <= time.time()-captured.timestamp() <= 1800
            or proof.get('static_observation_passed') is not True or proof.get('error')
            or proof.get('identity', {}).get('uid2') != UID
            or proof.get('identity', {}).get('firmware') != '1.14.3'
            or Path(proof['heading_candidate']).resolve() != Path(heading).resolve()
            or proof.get('actual_sent_to_fc') != 0
            or not 0 <= proof.get('median_gravity_axis_difference_deg', 100) <= 5
            or not proof.get('checks') or not all(proof['checks'].values())):
        raise ValueError('Fresh passing read-only same-candidate attitude proof required')
    checks = files(backup, heading, board, acceptance)
    registration = json.loads(checks['registration_sha256'].read_text())
    if (registration.get('rigid_body_revision_id') != expected['rigid_body_revision_id']
            or not 0 <= registration.get('fixed_marker_fit_rms_mm', 100) <= 3
            or registration.get('automatic_live_adoption_allowed') is not False):
        raise ValueError('Provisional registered geometry proof required')
    for source in registration.get('source_files', {}).values():
        if digest(source['path']) != source['sha256']:
            raise ValueError('Registration input evidence changed')
    capture = json.loads(checks['backup_sha256'].read_text())
    manifest = json.loads(checks['manifest_sha256'].read_text())
    rows = capture.get('records', [])
    values = {row['name']: row['value'] for row in rows}
    if (not capture.get('complete') or capture.get('received_count') != 926
            or capture.get('expected_count') != 926 or capture.get('missing_indices')
            or capture.get('system_id') != 1 or capture.get('component_id') != 1
            or capture.get('firmware', {}).get('version') != '1.14.3'
            or len(rows) != 926 or len(values) != 926
            or {r['index'] for r in rows} != set(range(926))
            or not 0 <= time.time()-checks['backup_sha256'].stat().st_mtime < 3600
            or manifest['sha256']['parameters_verified.json'] != digest(checks['backup_sha256'])
            or values.get('EKF2_EV_CTRL') != 2 or values.get('EKF2_EV_DELAY') != 0
            or not all(n in values for n in policy.READ_PARAMS) or not policy.scope_ok(values, values)):
        raise ValueError('Fresh complete verified Vision-reference parameter backup required')
    if (json.loads(checks['incident_sha256'].read_text()).get('block_active_benches') is not True
            or json.loads(checks['physical_hold_sha256'].read_text()).get('block_props_off_benches') is not True):
        raise ValueError('Original incident/physical holds must remain intact')
    return expected, checks


def consume(path, backup, output, heading, board):
    path = Path(path).resolve()
    r = json.loads(path.read_text())
    now = time.time()
    if (type(r.get('issued_unix')) not in (int, float)
            or type(r.get('expires_unix')) not in (int, float)
            or not r['issued_unix'] <= now < r['expires_unix']
            or not 0 < r['expires_unix']-r['issued_unix'] <= 900 or Path(output).exists()):
        raise ValueError('Expired/invalid one-use revision static permit or existing output')
    expected, checks = validate(backup, output, heading, board, Path(r['acceptance_path']))
    if any(r.get(k) != v for k, v in expected.items()):
        raise ValueError('Wrong revision-static scope; no modes, targets, flight or ground-check expansion')
    if any(r.get(k) != digest(p) for k, p in checks.items()):
        raise ValueError('Revision static code/evidence hash mismatch')
    fd = os.open(str(path)+'.consumed.json', os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump({'scope': SCOPE, 'consumed_unix': now, 'permit_sha256': digest(path),
                   'revision_registry_modified': False, 'flight_authorized': False}, stream, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--operator-authorized', action='store_true')
    for name in ('permit', 'backup', 'output', 'heading', 'board', 'acceptance'):
        ap.add_argument('--'+name, type=Path, required=True)
    a = ap.parse_args()
    if not a.operator_authorized or a.permit.exists() or a.output.exists():
        ap.error('Operator authorization and new output/permit required')
    try:
        expected, checks = validate(a.backup, a.output, a.heading, a.board, a.acceptance)
        now = time.time()
        r = {**expected, 'issued_unix': now, 'expires_unix': now+600,
             **{k: digest(p) for k, p in checks.items()}}
        with a.permit.open('x') as stream:
            json.dump(r, stream, indent=2); stream.write('\n')
    except (OSError, ValueError, KeyError) as exc:
        ap.error(str(exc))
    print('One-use locked/static revision validation permit created; no device commands')


if __name__ == '__main__':
    main()
