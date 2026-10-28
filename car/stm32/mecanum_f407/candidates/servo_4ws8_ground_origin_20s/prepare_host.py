#!/usr/bin/env python3
"""Isolated 20s host entry; no hardware connection or firmware changes."""
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
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/ground_origin_sync_v3/ready'
BASE=HERE.parent/'servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
        and not any(v in ('__pycache__','.pytest_cache') or v.endswith('.egg-info') for v in p.parts)}


def edit(path,old,new):
    value=path.read_text()
    if value.count(old)!=1:raise ValueError('input drift: '+str(path)+' '+old)
    path.write_text(value.replace(old,new))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    out=parser.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);(out/'COLCON_IGNORE').write_text('')
    runtime=out/'runtime/src';runtime.parent.mkdir()
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','*.egg-info','.pytest_cache'))
    os.symlink(OLD/'firmware',out/'firmware',target_is_directory=True)
    control=runtime/m['control_package']/m['control_package'];hw=runtime/m['hardware_package']/m['hardware_package']
    edit(control/'pilot_core.py','max_active_s=5.0','max_active_s=20.0')
    edit(control/'pilot_core.py',"return 'PILOT_TIME_LIMIT_5S'","return 'PILOT_TIME_LIMIT_20S'")
    edit(control/'pilot_ui.py','from .operator_model import OperatorModel',
         'from .operator_model import OperatorModel\nfrom .pilot_core import POLICY')
    edit(control/'pilot_ui.py',"p.get('max_active_s') == 5.0","p.get('max_active_s') == POLICY['max_active_s']")
    for file in ('pilot_ui.py','operator_console.py'):
        path=control/file;value=path.read_text()
        value=value.replace('最长5秒','最长20秒').replace('单次≤5秒','单次≤20秒')
        value=value.replace('本轮最多5秒','本轮最多20秒').replace('本轮5秒已结束','本轮20秒已结束')
        value=value.replace('PILOT_TIME_LIMIT_5S','PILOT_TIME_LIMIT_20S')
        path.write_text(value)
    edit(hw/'stages.py',"SCOPE='FINITE_GROUND_PILOT_V3_ACTIVE_ORIGIN_4WS8_V4_STAGE_'",
         "SCOPE='FINITE_GROUND_PILOT_V3_20S_ACTIVE_ORIGIN_4WS8_V4_STAGE_'")
    edit(hw/'commissioning.py',"CONSENT = '4WS8_V3_ACTIVE_ORIGIN_5S_CLEAR_POWER_CUT_READY'",
         "CONSENT = '4WS8_V3_ACTIVE_ORIGIN_20S_CLEAR_POWER_CUT_READY'")
    launcher=(OLD/'launch_pilot.py').read_text().replace('at most5s','at most20s')
    launcher=launcher.replace('开始5秒计时','开始20秒计时').replace('4WS8_V3_ACTIVE_ORIGIN_5S_CLEAR_POWER_CUT_READY','4WS8_V3_ACTIVE_ORIGIN_20S_CLEAR_POWER_CUT_READY')
    (out/'launch_pilot.py').write_text(launcher)
    tests=out/'tests';shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    # Change duration expectations only; never alter 5cm trajectories, spatial
    # thresholds, motion caps, freshness windows, or first-neutral-frame checks.
    replacements={
        'PILOT_TIME_LIMIT_5S':'PILOT_TIME_LIMIT_20S',
        'h.run(5.0)':'h.run(20.0)',
        'm.started+5.001':'m.started+20.001',
        'h.now-5.01':'h.now-20.01',
        'frame.issued_steady+5.001':'frame.issued_steady+20.001',
        'max_active_s=5.0':'max_active_s=20.0',
        "POLICY['max_active_s']==5.":"POLICY['max_active_s']==20.",
        "status['remaining_s']==5.":"status['remaining_s']==20.",
        'h.run(7.)':'h.run(22.)',
        'h.run(8.)':'h.run(23.)',
        'h.run(4.8)':'h.run(19.8)',
        '本轮最多5秒':'本轮最多20秒',
        '单次≤5秒':'单次≤20秒',
    }
    for path in tests.glob('test_*.py'):
        value=path.read_text()
        for old,new in replacements.items():value=value.replace(old,new)
        path.write_text(value)
    shutil.copyfile(HERE/'test_duration_20s.py',tests/'test_duration_20s.py')
    env=os.environ.copy();env.update(STAGED_GENERATED=str(runtime),
        PYTHONPATH=str(BASE)+os.pathsep+env.get('PYTHONPATH',''),
        DISPLAY=env.get('DISPLAY',':1'),XAUTHORITY=env.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    result=subprocess.run([sys.executable,'-m','pytest','-q',str(tests),'--junitxml='+str(out/'host_tests.xml')],
        cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
    (out/'host_tests.log').write_text(result.stdout);print(result.stdout,flush=True)
    if result.returncode:raise ValueError('20s regression failed; hardware not opened')
    test_summary=result.stdout.strip().splitlines()[-1]
    pins=dict(m['protected_input_hashes'])
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[group])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    manifest=dict(m);manifest.update(runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        source_hashes={**m['source_hashes'],**hashes(HERE)},protected_input_hashes=pins,
        host_change='V3 single observation extended from5s to20s; capture timing, geometry, caps, spatial limits and watchdogs unchanged',
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    manifest['source_hashes'][str(out/'launch_pilot.py')]=sha(out/'launch_pilot.py')
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');check(out/'build_manifest.json')
    result=subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
        '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),
        '--symlink-install','--packages-select',m['control_package'],m['hardware_package']],cwd=ROOT,env=env,
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
    (out/'colcon_build.log').write_text(result.stdout);print(result.stdout,flush=True)
    if result.returncode:raise ValueError('private build failed')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,max_active_s=20.,
        flashed=False,hardware_started=False,firmware_reused_unchanged=True,
        prior_versions_and_original_rectangle_preserved=True,protected_files_unchanged=len(pins),
        tests=test_summary),indent=2)+'\n')


if __name__=='__main__':main()
