#!/usr/bin/env python3
"""Portable export adaptations only; never execute primary-PC generators."""
import hashlib
import argparse
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
CAR=ROOT/'car'
NEW=CAR/'docs/wand_sync_600s_2026-10-28/prepared'


def edit(p,old,new):
    text=p.read_text()
    if text.count(old)!=1:raise ValueError('Unexpected export layout: '+str(p))
    p.write_text(text.replace(old,new))


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--reseal-only',action='store_true')
    args=ap.parse_args()
    if not args.reseal_only:
        adapt()
    seal()


def adapt():
    cm=NEW/'firmware/CMakeLists.txt'
    edit(cm,'set(BASE /home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407)\nset(WORKSPACE /home/PRIMARY_USER/vicon_mecanum_ws)',
         'get_filename_component(WORKSPACE "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)\nset(BASE ${WORKSPACE}/stm32/mecanum_f407)')
    edit(cm,'file(SHA256 ${file} actual)','file(SHA256 ${WORKSPACE}/${file} actual)')
    # No original hash is silently treated as a hash of normalized public code.
    for rel in ('candidates/wand_sync_600s/test_long_session.py',
                'docs/wand_sync_600s_2026-10-28/prepared/test_reused_sync.py',
                'candidates/wand_rotation_hardware/test_migration.py'):
        p=CAR/rel
        edit(p,"Path('/home/PRIMARY_USER/vicon_mecanum_ws')", "Path(os.environ['PUBLIC_CAR_ROOT'])")
    # Exported profiles intentionally revoke old onsite confirmations. These
    # snapshots remain useful for pure software geometry tests only.
    p=CAR/'stm32/mecanum_f407/candidates/continuous_4wgc/baseline.json'
    v=json.loads(p.read_text())
    for name in v:v[name]=hashlib.sha256((CAR/'stm32/mecanum_f407'/name).read_bytes()).hexdigest()
    p.write_text(json.dumps(v,indent=2)+'\n')


def seal():
    pins={}
    for folder in (NEW/'firmware',NEW/'runtime/src',CAR/'stm32/mecanum_f407/Inc',
                   CAR/'stm32/mecanum_f407/Src',CAR/'stm32/mecanum_f407/candidates/servo_4ws7',CAR/'protocol'):
        for p in sorted(folder.rglob('*')):
            if p.is_file() and not any(k in ('__pycache__','.pytest_cache') for k in p.parts):
                pins[str(p.relative_to(CAR))]=hashlib.sha256(p.read_bytes()).hexdigest()
    (NEW/'protected_inputs.json').write_text(json.dumps(pins,indent=2)+'\n')
    # Standalone public test manifest, NOT an original build/flash admission.
    (NEW/'public_offline_manifest.json').write_text(json.dumps(dict(
        public_source_snapshot=True,hardware_authorized=False,firmware_flashed=False,
        protected_input_hashes=pins),indent=2)+'\n')
    print(json.dumps(dict(public_pins=len(pins),source_tree_written=False)))


if __name__=='__main__':main()
