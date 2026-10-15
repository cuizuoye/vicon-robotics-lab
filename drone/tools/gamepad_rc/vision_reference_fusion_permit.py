"""Single-use operator-authorized USB-only Vision reference static XYZ exception."""
import argparse
import json
import math
import os
from pathlib import Path
import time
from receive_only_permit import digest, UID
from horizontal_fusion_permit import FusionWindow, WindowExpired

TOOLS = Path(__file__).resolve().parent
SCOPE = 'battery-off-usb-only-props-installed-vision-reference-static-xyz-once'


def validate_request(args):
    forbidden = ('clock_loss_check', 'clock_status_only', 'clock_characterize', 'clock_probe_only',
        'fusion_preflight', 'horizontal_fusion_bench', 'yaw_fusion_bench', 'combined_fusion_bench',
        'combined_source_loss_bench', 'restore_horizontal_only', 'restore_height_only',
        'restore_yaw_only', 'restore_combined_only', 'manual_translation_bench', 'manual_vertical_bench',
        'fusion_physical_confirmed', 'manual_control_file', 'receive_only_permit',
        'clock_sample_permit', 'horizontal_fusion_permit')
    if (not args.bench_send or not args.height_fusion_bench or not args.fusion_backup
            or args.seconds != 10 or args.clock_settle_seconds != 180 or args.uid2 != UID
            or any(getattr(args, k, False) for k in forbidden)):
        raise ValueError('Only 10s static Vision-reference XYZ with backup, 180s bounded clock wait')


def fields(output, backup):
    return {'schema': 1, 'scope': SCOPE, 'uid2': UID, 'user_authorized': True,
        'output_path': str(Path(output).resolve()), 'backup_path': str(Path(backup).resolve()),
        'rate_hz': 30, 'max_pose_packets': 301, 'max_fusion_seconds': 10,
        'max_clock_stability_wait_seconds': 180, 'height_reference': 3,
        'allowed_parameter_values': {'EKF2_EV_CTRL': [2, 3]}, 'restore_value': 2,
        'physical_conditions': ['battery_disconnected', 'usb_only', 'props_installed', 'stationary', 'output_locked']}


def paths(heading, board, backup):
    backup = Path(backup)
    return {'runner_sha256': TOOLS/'usb_vicon_receive_bench.py', 'permit_policy_sha256': Path(__file__),
        'fusion_policy_sha256': TOOLS/'vision_reference_xyz_policy.py',
        'height_helper_sha256': TOOLS/'usb_height_fusion_policy.py',
        'horizontal_helper_sha256': TOOLS/'usb_fusion_policy.py',
        'message_policy_sha256': TOOLS/'usb_vision_policy.py',
        'receive_helper_sha256': TOOLS/'receive_only_permit.py',
        'window_helper_sha256': TOOLS/'horizontal_fusion_permit.py',
        'incident_sha256': TOOLS/'known_incidents/micoair743_v1143_shell_crash.json',
        'physical_hold_sha256': TOOLS/'known_incidents/physical_safety_hold.json',
        'heading_sha256': Path(heading), 'board_sha256': Path(board),
        'backup_sha256': backup/'parameters_verified.json',
        'backup_manifest_sha256': backup/'parameters_manifest.json'}


def consume(path, output, heading, board, backup, *, now=None):
    path, backup = Path(path).resolve(), Path(backup).resolve()
    r = json.loads(path.read_text())
    now = time.time() if now is None else now
    issued, expires = r.get('issued_unix'), r.get('expires_unix')
    if (type(issued) not in (int, float) or type(expires) not in (int, float)
            or not all(math.isfinite(x) for x in (issued, expires, now))
            or not issued <= now < expires or not 0 < expires-issued <= 900):
        raise ValueError('Expired/invalid Vision-reference permit')
    if any(r.get(k) != v for k, v in fields(output, backup).items()) or Path(output).exists():
        raise ValueError('Vision-reference permit scope/output mismatch')
    checks = paths(heading, board, backup)
    if any(r.get(k) != digest(p) for k, p in checks.items()):
        raise ValueError('Evidence/code hash mismatch')
    if (json.loads(checks['incident_sha256'].read_text()).get('block_active_benches') is not True
            or json.loads(checks['physical_hold_sha256'].read_text()).get('block_props_off_benches') is not True):
        raise ValueError('General holds must remain intact')
    b = json.loads(checks['backup_sha256'].read_text())
    manifest = json.loads(checks['backup_manifest_sha256'].read_text())
    records = {x['name']: x for x in b.get('records', [])}
    required = {'EKF2_EV_CTRL': 2, 'EKF2_HGT_REF': 3, 'EKF2_RNG_CTRL': 0,
                'COM_RC_IN_MODE': 4, 'COM_OBL_RC_ACT': 4}
    if (b.get('complete') is not True or b.get('received_count') != 926 or b.get('expected_count') != 926
            or len(records) != 926 or b.get('firmware', {}).get('version') != '1.14.3'
            or manifest['sha256']['parameters_verified.json'] != digest(checks['backup_sha256'])
            or not 0 <= now-checks['backup_sha256'].stat().st_mtime < 3600
            or any(records.get(k, {}).get('value') != v or records[k].get('type') != 6
                   for k, v in required.items())):
        raise ValueError('Fresh complete Vision-reference baseline required')
    fd = os.open(str(path)+'.consumed.json', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump({'scope': SCOPE, 'consumed_unix': now, 'permit_sha256': digest(path),
                   'general_guards_modified': False, 'output_path': str(Path(output).resolve())}, stream, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--operator-authorized', action='store_true')
    ap.add_argument('--permit', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--backup', type=Path, required=True)
    ap.add_argument('--heading', type=Path, required=True)
    ap.add_argument('--board', type=Path, required=True)
    args = ap.parse_args()
    if not args.operator_authorized or args.permit.exists() or args.output.exists():
        ap.error('Explicit operator authorization and new paths required')
    now = time.time()
    record = {**fields(args.output, args.backup), 'issued_unix': now, 'expires_unix': now+600,
              **{k: digest(p) for k,p in paths(args.heading, args.board, args.backup).items()}}
    with args.permit.open('x') as stream:
        json.dump(record, stream, indent=2); stream.write('\n')
    print('One-use static XYZ permit created; no serial device opened')


if __name__ == '__main__':
    main()
