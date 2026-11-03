#!/usr/bin/env python3
"""Seal explicitly requested launcher migration AFTER apply_patch promotion.

No source edits, ROS init, network or hardware I/O. Original launcher bytes
retained; exactly one named current-file hash changes, all other pins verified.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();plan=json.loads((out/'LAUNCHER_MIGRATION_PLAN.json').read_text())
    changed=plan['only_authorized_source_path']
    if changed!='/home/PRIMARY_USER/vicon_mecanum_ws/tools/start_car_demo_120s_toggle_eventio.sh':raise ValueError('unexpected source target')
    if sha(changed)!=plan['new_sha256'] or sha(plan['proposal'])!=plan['new_sha256'] or sha(plan['backup'])!=plan['old_sha256']:
        raise ValueError('exact proposed launcher or backup differs')
    path=out/'build_manifest.json';m=json.loads(path.read_text())
    old_sha=sha(path)
    seen=0
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):
        for p,value in m[group].items():
            if p==changed:
                if value!=plan['old_sha256']:raise ValueError('unexpected previous named hash')
                m[group][p]=plan['new_sha256'];seen+=1
            elif sha(p)!=value:raise ValueError('unrelated drift: '+p)
    if not seen:raise ValueError('authorized launcher not pinned')
    m['launcher_migration']=plan
    archive=out/'build_manifest_before_launcher_promotion.json'
    if archive.exists():raise ValueError('promotion already attempted')
    archive.write_bytes(path.read_bytes())
    path.write_text(json.dumps(m,indent=2)+'\n')
    sys.path.insert(0,'/home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407/candidates/servo_4ws8')
    from launch_cli import check
    check(path)
    prep_path=out/'PREPARATION_RESULT.json';prep=json.loads(prep_path.read_text())
    prep.update(launcher_promoted=True,launcher_before_sha256=plan['old_sha256'],launcher_after_sha256=plan['new_sha256'])
    prep_path.write_text(json.dumps(prep,indent=2)+'\n')
    print(json.dumps(dict(passed=True,only_authorized_source_changed=changed,
        old_launcher_sha256=plan['old_sha256'],new_launcher_sha256=plan['new_sha256'],
        original_manifest_before_promotion_sha256=old_sha,original_launcher_backup=plan['backup'],
        hardware_started=False,network_changed=False),indent=2))

if __name__=='__main__':main()
