"""One-use, evidence-bound exception for one battery-off static bench only.

No changes to the general incident guard. A permit is an audit/consumption
record, not a cryptographic authorization boundary against local file owners.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import time

from incident_guard import INCIDENT

SCOPE = 'battery-off-props-off-locked-static-trs-fusion-once'
LOSS_SCOPE = 'battery-off-props-off-locked-static-trs-source-loss-once'
UID = '000000000000000000000000000000000000'
TOOLS = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def consume(path, backup, output, *, physical_confirmed, source_loss=False, now=None):
    hold = TOOLS / 'known_incidents/physical_safety_hold.json'
    if hold.exists() and json.loads(hold.read_text()).get('block_props_off_benches') is not False:
        raise ValueError('Physical safety hold: propellers are not confirmed removed; no static bench authorized')
    if not physical_confirmed:
        raise ValueError('Fresh battery-off/props-off/USB-only confirmation required')
    path, backup, output = Path(path).resolve(), Path(backup).resolve(), Path(output).resolve()
    record = json.loads(path.read_text())
    now = time.time() if now is None else now
    issued, expires = record.get('issued_unix'), record.get('expires_unix')
    if (type(issued) not in (int, float) or type(expires) not in (int, float)
            or not math.isfinite(issued) or not math.isfinite(expires)
            or not issued <= now < expires or not 0 < expires - issued <= 900):
        raise ValueError('Static permit expired or has an invalid lifetime')
    expected = {
        'schema': 1, 'scope': LOSS_SCOPE if source_loss else SCOPE, 'uid2': UID, 'user_authorized': True,
        'backup_path': str(backup), 'output_path': str(output),
        'height_reference': 2, 'max_pose_stream_seconds': 15,
        'max_pose_packets': 480, 'max_fusion_seconds': 10,
        'allowed_parameter_values': {'EKF2_EV_CTRL': [0, 11], 'EKF2_EV_DELAY': [0, 65]},
        'physical_conditions': ['battery_disconnected', 'props_removed', 'usb_only', 'stationary'],
    }
    if any(record.get(k) != v for k, v in expected.items()):
        raise ValueError('Static permit scope/path/limits mismatch')
    if output.exists():
        raise ValueError('Static result already exists')
    checks = {
        'incident_sha256': INCIDENT,
        'backup_sha256': backup / 'parameters_verified.json',
        'bench_sha256': TOOLS / 'trs_static_fusion_bench.py',
        'policy_sha256': TOOLS / 'trs_range_static_policy.py',
        'message_policy_sha256': TOOLS / 'trs_static_fusion_policy.py',
        'heading_sha256': Path(record['heading_path']),
        'board_center_sha256': Path(record['board_center_path']),
        'acceptance_sha256': Path(record['acceptance_path']),
    }
    if source_loss:
        if record.get('source_loss_test') is not True:
            raise ValueError('Source-loss test requires its own explicit permit')
        checks['prior_static_verification_sha256'] = Path(record['prior_static_verification_path'])
        checks['loss_policy_sha256'] = TOOLS / 'combined_input_loss.py'
    if any(record.get(k) != digest(p) for k, p in checks.items()):
        raise ValueError('Static permit evidence/code hash mismatch')
    acceptance = json.loads(checks['acceptance_sha256'].read_text())
    if acceptance.get('basic_maintenance_evidence_passed') is not True:
        raise ValueError('Basic maintenance audit did not pass')
    if source_loss:
        prior = json.loads(checks['prior_static_verification_sha256'].read_text())
        if prior.get('static_bench_passed') is not True:
            raise ValueError('Prior static fusion verification did not pass')
    if json.loads(INCIDENT.read_text()).get('block_active_benches') is not True:
        raise ValueError('Expected general incident guard is not retained')
    # Consume before any device is opened; failed attempts cannot silently retry.
    consumed = path.with_name(path.name + '.consumed.json')
    fd = os.open(consumed, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump({'scope': record['scope'], 'consumed_unix': now, 'permit_sha256': digest(path),
                   'output_path': str(output), 'general_guard_modified': False}, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    return record
