"""One-use battery-OFF, props-installed, USB-observed static TRS integration."""
import argparse
import json
import os
from pathlib import Path
import time
from receive_only_permit import digest, UID

TOOLS = Path(__file__).resolve().parent
SCOPE = 'battery-off-usb-observed-props-installed-vision-reference-trs-xyz-yaw-once'


def fields(output, backup, heading, board, acceptance, ground_check=False):
    return {'schema':1, 'scope':SCOPE+'-disarmed-offboard-ground-check' if ground_check else SCOPE,
        'disarmed_offboard_ground_check':ground_check, 'uid2':UID, 'user_authorized':True,
        'output_path':str(Path(output).resolve()), 'backup_path':str(Path(backup).resolve()),
        'heading_path':str(Path(heading).resolve()), 'board_path':str(Path(board).resolve()),
        'acceptance_path':str(Path(acceptance).resolve()), 'height_reference':3,
        'max_pose_stream_seconds':25 if ground_check else 15,
        'max_pose_packets':800 if ground_check else 480, 'max_fusion_seconds':20 if ground_check else 10,
        'allowed_parameter_values':{'EKF2_EV_CTRL':[2,11], 'EKF2_EV_DELAY':[0,65]},
        'ground_command_scope': ['frozen_current_position_yaw','one_disarmed_offboard_request','restore_initial_disarmed_mode'] if ground_check else [],
        'physical_conditions':['battery_disconnected','usb_only','props_installed','stationary','output_locked']}


def paths(backup, heading, board, acceptance):
    return {'runner_sha256':TOOLS/'trs_static_fusion_bench.py', 'permit_policy_sha256':Path(__file__),
        'fusion_policy_sha256':TOOLS/'trs_vision_reference_policy.py',
        'legacy_policy_sha256':TOOLS/'trs_static_fusion_policy.py',
        'xyz_policy_sha256':TOOLS/'vision_reference_xyz_policy.py',
        'compact_helper_sha256':TOOLS/'compact_vision.py', 'combined_helper_sha256':TOOLS/'usb_combined_fusion_policy.py',
        'horizontal_helper_sha256':TOOLS/'usb_fusion_policy.py',
        'ground_policy_sha256':TOOLS/'disarmed_offboard_policy.py',
        'incident_sha256':TOOLS/'known_incidents/micoair743_v1143_shell_crash.json',
        'physical_hold_sha256':TOOLS/'known_incidents/physical_safety_hold.json',
        'backup_sha256':Path(backup)/'parameters_verified.json',
        'manifest_sha256':Path(backup)/'parameters_manifest.json', 'heading_sha256':Path(heading),
        'board_sha256':Path(board), 'acceptance_sha256':Path(acceptance)}


def consume(path, backup, output, heading, board, *, ground_check=False):
    path = Path(path).resolve()
    r = json.loads(path.read_text())
    now = time.time()
    if (type(r.get('issued_unix')) not in (int,float) or type(r.get('expires_unix')) not in (int,float)
            or not r['issued_unix'] <= now < r['expires_unix']
            or not 0 < r['expires_unix']-r['issued_unix'] <= 900):
        raise ValueError('Expired/invalid one-use TRS integration permit')
    acceptance = Path(r['acceptance_path'])
    if any(r.get(k) != v for k,v in fields(output, backup, heading, board, acceptance,ground_check).items()) or Path(output).exists():
        raise ValueError('Wrong scope or output')
    checks = paths(backup, heading, board, acceptance)
    if any(r.get(k) != digest(p) for k,p in checks.items()):
        raise ValueError('Code/evidence hash mismatch')
    if (json.loads(checks['incident_sha256'].read_text()).get('block_active_benches') is not True
            or json.loads(checks['physical_hold_sha256'].read_text()).get('block_props_off_benches') is not True):
        raise ValueError('General holds must remain intact')
    proof = json.loads(acceptance.read_text())
    if (not proof.get('vision_reference_static_xyz_verified') or not proof.get('rollback_verified')
            or proof.get('error') or proof.get('identity',{}).get('uid2') != UID):
        raise ValueError('Prior Vision-reference XYZ fusion proof required')
    b = json.loads(checks['backup_sha256'].read_text())
    manifest = json.loads(checks['manifest_sha256'].read_text())
    values = {row['name']:row['value'] for row in b.get('records',[])}
    if (not b.get('complete') or b.get('received_count') != 926 or b.get('expected_count') != 926
            or b.get('firmware',{}).get('version') != '1.14.3'
            or len(values) != 926 or not 0 <= now-checks['backup_sha256'].stat().st_mtime < 3600
            or manifest['sha256']['parameters_verified.json'] != digest(checks['backup_sha256'])
            or any(values.get(k) != v for k,v in {'EKF2_EV_CTRL':2,'EKF2_HGT_REF':3,'EKF2_EV_DELAY':0,'EKF2_RNG_CTRL':0}.items())):
        raise ValueError('Fresh complete Vision-reference backup required')
    fd = os.open(str(path)+'.consumed.json', os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
    with os.fdopen(fd,'w') as stream:
        json.dump({'scope':SCOPE,'consumed_unix':now,'permit_sha256':digest(path),'general_guards_modified':False},stream,indent=2)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    return r


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--operator-authorized',action='store_true')
    ap.add_argument('--disarmed-offboard-ground-check',action='store_true')
    for name in ('permit','output','backup','heading','board','acceptance'):
        ap.add_argument('--'+name,type=Path,required=True)
    a=ap.parse_args()
    if not a.operator_authorized or a.permit.exists() or a.output.exists():
        ap.error('Operator authorization and new paths required')
    now=time.time()
    r={**fields(a.output,a.backup,a.heading,a.board,a.acceptance,a.disarmed_offboard_ground_check),'issued_unix':now,'expires_unix':now+600,
       **{k:digest(p) for k,p in paths(a.backup,a.heading,a.board,a.acceptance).items()}}
    with a.permit.open('x') as stream:
        json.dump(r,stream,indent=2);stream.write('\n')
    print('One-use TRS integration permit created; zero device commands')


if __name__=='__main__':
    main()
