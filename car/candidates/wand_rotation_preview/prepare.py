#!/usr/bin/env python3
"""Check/protect accepted inputs and run offline preview tests. No ROS init."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
BASELINE=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_autoprep/ready_v4/build_manifest.json'
sys.path.insert(0,str(ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'))
from launch_cli import check


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inputs():
    paths=[p for p in HERE.iterdir() if p.is_file() and p.suffix in ('.py','.md','.json')]
    paths+=[ROOT/'tools/start_sign_rotation_preview.sh',
        Path('/home/PRIMARY_USER/vicon_receiver_ws/drone_sim/tools/gamepad_rc/vicon_receiver_provenance.py'),
        Path('/home/PRIMARY_USER/vicon_receiver_ws/drone_sim/tools/gamepad_rc/plan_b/phase2/vicon_receiver_provenance.json')]
    return {str(p):sha(p) for p in paths}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    out=parser.parse_args().output.resolve();m=check(BASELINE)
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700)
    preserved={}
    for group in ('runtime_hashes','source_hashes','protected_input_hashes','generated_firmware_hashes'):
        preserved.update(m[group])
    preserved[str(BASELINE)]=sha(BASELINE);preserved[m['firmware']['path']]=m['firmware']['sha256']
    before=inputs();snapshot=ROOT/'docs/wand_rotation_preview_2026-10-26/initial_geometry.json'
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(HERE)+os.pathsep+os.environ.get('PYTHONPATH',''),
        DISPLAY=os.environ.get('DISPLAY',':1'),XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    p=subprocess.run([sys.executable,'-m','pytest','-q','-p','no:cacheprovider',str(HERE/'test_preview.py'),
        '--junitxml='+str(out/'tests.xml')],env=env,capture_output=True,text=True,timeout=60)
    (out/'tests.log').write_text(p.stdout+p.stderr);print(p.stdout+p.stderr,flush=True)
    if p.returncode:raise ValueError('offline preview regression failed')
    if inputs()!=before:raise ValueError('preview source changed during regression')
    check(BASELINE)
    manifest=dict(kind='READONLY_SIGN_ROTATION_PREVIEW_V1',offline_passed=True,actuator_outputs=False,
        source_hashes=before,baseline_hashes=preserved,initial_snapshot=str(snapshot),initial_snapshot_sha256=sha(snapshot),
        tests=p.stdout.strip().splitlines()[-1],default_window=[1280,720],
        hardware_started=False,firmware_changed=False,network_changed=False,real_rotation_accepted=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(passed=True,protected_files=len(preserved),actuator_outputs=False,output=str(out))))


if __name__=='__main__':main()
