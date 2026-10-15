"""One explicitly authorized USB-only static horizontal fusion exception.

General incident/physical holds remain intact; no flight/arm/unlock permission.
"""
import json
import math
import os
from pathlib import Path
import time

from receive_only_permit import digest, UID

TOOLS = Path(__file__).resolve().parent
SCOPE = 'battery-off-usb-only-props-installed-locked-horizontal-fusion-once'


class WindowExpired(RuntimeError):
    pass


class FusionWindow:
    """Host-side deadline only, not an independent real-flight watchdog."""
    def __init__(self):
        self.started_at = None
        self.active = False

    def start(self, now):
        if not math.isfinite(now) or self.started_at is not None:
            raise ValueError('Only one finite horizontal enable attempt allowed')
        self.started_at, self.active = now, True

    def check(self, now, *, restoring=False):
        if not math.isfinite(now):
            raise ValueError('Invalid window clock')
        if self.active and now-self.started_at >= 10 and not restoring:
            raise WindowExpired('Horizontal window ended; restore fusion off')

    def restored(self):
        self.active = False


def validate_request(args):
    forbidden = ('clock_loss_check', 'clock_status_only', 'clock_characterize', 'clock_probe_only',
        'fusion_preflight', 'height_fusion_bench', 'yaw_fusion_bench', 'combined_fusion_bench',
        'combined_source_loss_bench', 'restore_horizontal_only', 'restore_height_only',
        'restore_yaw_only', 'restore_combined_only', 'manual_translation_bench',
        'manual_vertical_bench', 'fusion_physical_confirmed', 'manual_control_file',
        'receive_only_permit', 'clock_sample_permit', 'vision_reference_permit')
    if (not args.bench_send or not args.horizontal_fusion_bench or not args.fusion_backup
            or args.seconds != 10 or args.clock_settle_seconds != 180 or args.uid2 != UID
            or any(getattr(args, x, False) for x in forbidden)):
        raise ValueError('Only 10s horizontal static fusion with backup and 180s clock wait permitted')


def consume(path, output, heading, board, backup, *, now=None):
    path, output, backup = Path(path).resolve(), Path(output).resolve(), Path(backup).resolve()
    r = json.loads(path.read_text())
    now = time.time() if now is None else now
    issued, expires = r.get('issued_unix'), r.get('expires_unix')
    if (type(issued) not in (int,float) or type(expires) not in (int,float)
            or not all(math.isfinite(x) for x in (issued, expires, now))
            or not issued <= now < expires or not 0 < expires-issued <= 900):
        raise ValueError('Expired/invalid horizontal permit')
    expected = {'schema':1, 'scope':SCOPE, 'uid2':UID, 'user_authorized':True,
        'output_path':str(output), 'backup_path':str(backup), 'rate_hz':30,
        'max_pose_packets':301, 'max_fusion_seconds':10, 'max_clock_stability_wait_seconds':180,
        'allowed_parameter_values':{'EKF2_EV_CTRL':[0,1]}, 'height_reference':2,
        'physical_conditions':['battery_disconnected','usb_only','props_installed','stationary','output_locked']}
    if any(r.get(k) != v for k,v in expected.items()) or output.exists():
        raise ValueError('Horizontal permit scope/output mismatch')
    checks = {'runner_sha256':TOOLS/'usb_vicon_receive_bench.py',
        'permit_policy_sha256':Path(__file__), 'receive_helper_sha256':TOOLS/'receive_only_permit.py',
        'fusion_policy_sha256':TOOLS/'usb_fusion_policy.py',
        'message_policy_sha256':TOOLS/'usb_vision_policy.py',
        'incident_sha256':TOOLS/'known_incidents/micoair743_v1143_shell_crash.json',
        'physical_hold_sha256':TOOLS/'known_incidents/physical_safety_hold.json',
        'heading_sha256':Path(heading), 'board_sha256':Path(board),
        'backup_sha256':backup/'parameters_verified.json',
        'backup_manifest_sha256':backup/'parameters_manifest.json'}
    if any(r.get(k) != digest(p) for k,p in checks.items()):
        raise ValueError('Horizontal permit evidence/code hash mismatch')
    if (json.loads(checks['incident_sha256'].read_text()).get('block_active_benches') is not True
            or json.loads(checks['physical_hold_sha256'].read_text()).get('block_props_off_benches') is not True):
        raise ValueError('General holds must remain intact')
    b = json.loads(checks['backup_sha256'].read_text())
    manifest = json.loads(checks['backup_manifest_sha256'].read_text())
    values = {x['name']:x['value'] for x in b.get('records',[])}
    if (b.get('complete') is not True or b.get('received_count') != 926 or b.get('expected_count') != 926
            or b.get('firmware',{}).get('version') != '1.14.3'
            or manifest['sha256']['parameters_verified.json'] != digest(checks['backup_sha256'])
            or not 0 <= now-checks['backup_sha256'].stat().st_mtime < 3600
            or any(values.get(k) != v for k,v in {'EKF2_EV_CTRL':0, 'EKF2_HGT_REF':2,
                'COM_RC_IN_MODE':4, 'COM_OBL_RC_ACT':4}.items())):
        raise ValueError('Fresh complete no-RC/fusion-off backup required')
    fd = os.open(str(path)+'.consumed.json', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd,'w') as stream:
        json.dump({'scope':SCOPE, 'consumed_unix':now, 'permit_sha256':digest(path),
                   'general_guards_modified':False, 'output_path':str(output)}, stream, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    return r
