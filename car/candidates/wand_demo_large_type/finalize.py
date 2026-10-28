#!/usr/bin/env python3
"""Seal only the explicitly requested shortcut promotion after apply_patch."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();plan=json.loads((out/'SHORTCUT_MIGRATION.json').read_text())
    changed='/home/PRIMARY_USER/.local/bin/小车演示'
    if plan['path']!=changed or plan['allowed_source_change']!=changed:raise ValueError('unexpected promotion target')
    if sha(changed)!=plan['new_sha256'] or sha(plan['proposal'])!=plan['new_sha256'] or sha(plan['backup'])!=plan['old_sha256']:
        raise ValueError('Exact new shortcut or old backup mismatch')
    p=out/'build_manifest.json';m=json.loads(p.read_text());seen=0
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):
        for path,digest in m[group].items():
            if path==changed:
                if digest!=plan['old_sha256']:raise ValueError('old shortcut hash mismatch')
                m[group][path]=plan['new_sha256'];seen+=1
            elif sha(path)!=digest:raise ValueError('unrelated drift: '+path)
    if not seen:raise ValueError('shortcut was not pinned')
    backup=out/'build_manifest_before_shortcut_promotion.json'
    if backup.exists():raise ValueError('already promoted')
    backup.write_bytes(p.read_bytes());m['shortcut_migration']=plan
    p.write_text(json.dumps(m,indent=2)+'\n')
    sys.path.insert(0,'/home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407/candidates/servo_4ws8')
    from launch_cli import check
    check(p)
    prep=out/'PREPARATION_RESULT.json';v=json.loads(prep.read_text());v['shortcut_promoted']=True
    prep.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=True,only_existing_source_changed=changed,backup=plan['backup'],
        hardware_started=False,network_changed=False),ensure_ascii=False))


if __name__=='__main__':main()
