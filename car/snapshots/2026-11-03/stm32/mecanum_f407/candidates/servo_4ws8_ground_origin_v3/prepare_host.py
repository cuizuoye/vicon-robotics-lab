#!/usr/bin/env python3
"""Offline-only isolated origin-timing repair; no firmware writer/ROS launch."""
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
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-23/ground_pilot_070_080/ui_fix_v2_ready'
BASE=HERE.parent/'servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
        and not any(v in ('__pycache__','.pytest_cache') or v.endswith('.egg-info') for v in p.parts)}


def edit(path,old,new):
    text=path.read_text()
    if text.count(old)!=1:raise ValueError('input drift: '+str(path)+' '+old)
    path.write_text(text.replace(old,new))


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    out=p.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);(out/'COLCON_IGNORE').write_text('')
    runtime=out/'runtime/src';runtime.parent.mkdir()
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','*.egg-info','.pytest_cache'))
    os.symlink(OLD/'firmware',out/'firmware',target_is_directory=True)
    control=runtime/m['control_package']/m['control_package'];hw=runtime/m['hardware_package']/m['hardware_package']
    shutil.copyfile(HERE/'pilot_core.py',control/'pilot_core.py')
    shutil.copyfile(HERE/'README.md',out/'README.md')
    edit(hw/'stages.py',"SCOPE='FINITE_GROUND_PILOT_V2_5S_4WS8_V4_STAGE_'",
        "SCOPE='FINITE_GROUND_PILOT_V3_ACTIVE_ORIGIN_4WS8_V4_STAGE_'")
    edit(hw/'commissioning.py',"CONSENT = '4WS8_FINITE_GROUND_PILOT_5S_CLEAR_POWER_CUT_READY'",
        "CONSENT = '4WS8_V3_ACTIVE_ORIGIN_5S_CLEAR_POWER_CUT_READY'")
    edit(control/'pilot_ui.py',"return '准备就绪：按住空格并立即前移Sign约5cm；本轮最长5秒、仅一次'",
        "return '准备就绪：先拿稳Sign；按住空格时才记录原点并计时，再前移约5cm；最长5秒、仅一次'")
    launcher=(OLD/'launch_pilot.py').read_text()
    launcher=launcher.replace('4WS8_FINITE_GROUND_PILOT_5S_CLEAR_POWER_CUT_READY','4WS8_V3_ACTIVE_ORIGIN_5S_CLEAR_POWER_CUT_READY')
    launcher=launcher.replace("    print('仅一次按住使能，最长5秒；杆只前移约5cm，不做返回或连续跟随。松键/定位/反馈故障立即撤能。')",
        "    print('准备期间可拿起/调整Sign，保持松键；按住使能时记录当前原点并开始5秒计时。随后仅前移约5cm，不做返回或连续跟随。')")
    (out/'launch_pilot.py').write_text(launcher)
    tests=out/'tests';shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copyfile(HERE/'test_origin_capture.py',tests/'test_origin_capture.py')
    shutil.copyfile(HERE/'replay_trial.py',tests/'replay_trial.py')
    env=os.environ.copy();env.update(STAGED_GENERATED=str(runtime),
        STAGED_NATIVE=str(ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/native'),
        PYTHONPATH=str(BASE)+os.pathsep+env.get('PYTHONPATH',''),
        DISPLAY=env.get('DISPLAY',':1'),XAUTHORITY=env.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    reports={}
    cases=[('baseline',[str(BASE/'test_staged.py'),str(ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/tests')],
            dict(env,STAGED_GENERATED=str(ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/runtime/src'))),
           ('origin_v3',[str(tests)],env)]
    for name,paths,test_env in cases:
        result=subprocess.run([sys.executable,'-m','pytest','-q',*paths,'--junitxml='+str(out/(name+'_tests.xml'))],
            cwd=ROOT,env=test_env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
        (out/(name+'_tests.log')).write_text(result.stdout);print(result.stdout,flush=True)
        if result.returncode:raise ValueError(name+' regression failed; hardware not opened')
        reports[name]=result.stdout.strip().splitlines()[-1]
    replay={}
    for label,folder in (('old_v2',Path(m['runtime_src'])),('new_v3',runtime)):
        result=subprocess.run([sys.executable,str(HERE/'replay_trial.py'),'--runtime',str(folder)],
            cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=15,check=True)
        replay[label]=json.loads(result.stdout)
    if (replay['old_v2']['enable_reason']!='PILOT_HAND_LIMIT' or replay['new_v3']['enable_state']!='ACTIVE'
            or not replay['new_v3']['enable_command_zero'] or not replay['new_v3']['post_enable_5cm_command_nonzero']):
        raise ValueError('historical preparation-flow regression not resolved')
    (out/'RECORDED_FLOW_REPLAY.json').write_text(json.dumps(replay,indent=2,ensure_ascii=False)+'\n')
    pins=dict(m['protected_input_hashes'])
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[group])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    for name in ('site_confirmed.json','observation.jsonl','mcu_receive.jsonl'):
        path=OLD/'ground_stage4_single_nn_at6cn'/name;pins[str(path)]=sha(path)
    manifest=dict(m);manifest.update(runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        source_hashes={**m['source_hashes'],**hashes(HERE)},protected_input_hashes=pins,
        host_change='V3: no ARMED origin capture; supervisor reuses engagement mapping; gate independently captures on first neutral ACTIVE; shared activation clock, one shot, unchanged 1:1/3x3/50cm/5s limits',
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    manifest['source_hashes'][str(out/'launch_pilot.py')]=sha(out/'launch_pilot.py')
    manifest['source_hashes'][str(out/'README.md')]=sha(out/'README.md')
    manifest['source_hashes'][str(out/'RECORDED_FLOW_REPLAY.json')]=sha(out/'RECORDED_FLOW_REPLAY.json')
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');check(out/'build_manifest.json')
    build=subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
        '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),
        '--symlink-install','--packages-select',m['control_package'],m['hardware_package']],cwd=ROOT,env=env,
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
    (out/'colcon_build.log').write_text(build.stdout);print(build.stdout,flush=True)
    if build.returncode:raise ValueError('private build failed')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,flashed=False,
        hardware_started=False,live_vicon_opened=False,udp_link_opened=False,
        ground_qualified=False,previous_trial_preserved=True,firmware_reused_unchanged=True,
        protected_files_unchanged=len(pins),tests=reports),indent=2)+'\n')


if __name__=='__main__':main()
