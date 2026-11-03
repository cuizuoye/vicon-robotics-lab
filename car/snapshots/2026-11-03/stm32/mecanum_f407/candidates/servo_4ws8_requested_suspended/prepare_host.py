#!/usr/bin/env python3
"""Independent host-only preparation; exact existing firmware, no IO/flash."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-23/ready'
ORIGINAL=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0,str(ORIGINAL))
from launch_cli import check

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts and
        '.pytest_cache' not in p.parts and not any(v.endswith('.egg-info') for v in p.parts)}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    out=parser.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);(out/'COLCON_IGNORE').write_text('')
    runtime=out/'runtime/src';runtime.parent.mkdir()
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','*.egg-info','.pytest_cache'))
    os.symlink(OLD/'firmware',out/'firmware',target_is_directory=True)
    hw=runtime/m['hardware_package']/m['hardware_package']
    shutil.copyfile(HERE/'requested_transition.py',hw/'requested_transition.py')
    auth=hw/'authorization.py';text=auth.read_text()
    changes={
        'from .stages import get_stage':'from .stages import get_stage\nfrom .requested_transition import predecessor',
        '    if stage:\n':'    expected_previous=predecessor(review,stage,purpose,firmware_sha)\n    if stage:\n',
        "review.get('previous_stage')!=stage-1":"review.get('previous_stage')!=expected_previous",
        "previous['stage']!=stage-1":"previous['stage']!=expected_previous",
    }
    for old,new in changes.items():
        if text.count(old)!=1:raise ValueError('authorization input drift: '+old)
        text=text.replace(old,new)
    auth.write_text(text)
    # Preserve the complete old candidate, its manifest, and the actual stage0
    # result/raw log/events; never manufacture a skipped stage1 result.
    pins=dict(m['protected_input_hashes'])
    for category in ('source_hashes','runtime_hashes','generated_firmware_hashes'):pins.update(m[category])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    baseline=OLD/'stage0_suspended_yjp1gyc2/bench'
    for name in ('SESSION_RESULT.json','raw_link.jsonl','events.json'):pins[str(baseline/name)]=sha(baseline/name)
    pins[m['firmware']['path']]=m['firmware']['sha256']
    new=dict(m);new.update(preparation_passed=False,runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        protected_input_hashes=pins,draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        source_hashes={**m['source_hashes'],**hashes(HERE)},
        host_change='Only explicit suspended 0-to-2 predecessor exception; all other transitions and ground reviews unchanged',
        firmware_reused_unchanged=True,firmware_flashed_this_preparation=False)
    env=os.environ.copy();env.update(STAGED_GENERATED=str(runtime),STAGED_NATIVE=str(OLD/'native'),
        PYTHONPATH=str(ORIGINAL)+os.pathsep+env.get('PYTHONPATH',''))
    cmd=[sys.executable,'-m','pytest','-q',str(ORIGINAL/'test_staged.py'),str(OLD/'tests'),
        str(HERE/'test_requested_transition.py'),'--junitxml='+str(out/'host_tests.xml')]
    r=subprocess.run(cmd,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
    (out/'host_tests.log').write_text(r.stdout);print(r.stdout,flush=True)
    if r.returncode:raise RuntimeError('host-only regression failed; no release or hardware IO')
    for p,v in pins.items():
        if sha(p)!=v:raise ValueError('original source/evidence drift: '+p)
    new['preparation_passed']=True
    (out/'build_manifest.json').write_text(json.dumps(new,indent=2)+'\n')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,host_tests=r.stdout.strip().splitlines()[-1],
        firmware_sha256=m['firmware']['sha256'],firmware_reused=True,flashed=False,hardware_started=False,
        original_files_and_evidence_unchanged=len(pins)),indent=2)+'\n')
    print('PASS: independent host-only suspended-sequence release; no hardware started')

if __name__=='__main__':main()
