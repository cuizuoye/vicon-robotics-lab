#!/usr/bin/env python3
"""Generate independent fixed host; preserve all old entry/provenance bytes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_autoprep/ready_v4'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file() and not any(
        k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    before=hashes(HERE)
    for p in (ROOT/'tools/start_car_demo_startup_fixed.sh',Path('/home/PRIMARY_USER/.local/bin/小车演示')):before[str(p)]=sha(p)
    pins={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):pins.update(m[group])
    # Preserve the accepted rotation simulations too, including nested exact pins.
    for prior in (ROOT/'docs/wand_rotation_accel_sim_2026-10-26/prepared/manifest.json',):
        p=json.loads(prior.read_text());pins[str(prior)]=sha(prior)
        for group in ('source_hashes','protected_hashes','output_hashes'):pins.update(p[group])
    trace=OLD/'oneclick_session_9rsmnc5n/observation.jsonl';pins[str(trace)]=sha(trace)
    for p,digest in pins.items():
        if sha(p)!=digest:raise ValueError('pre-existing drift: '+p)
    runtime=out/'runtime/src';shutil.copytree(m['runtime_src'],runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    target=runtime/m['control_package']/m['control_package']/'auto_prepare.py'
    shutil.copyfile(HERE/'auto_prepare.py',target)
    app=out/'oneclick_app.py';text=(OLD/'oneclick_app.py').read_text()
    fragment='READY=Path('+repr(str(OLD))+')'
    if text.count(fragment)!=1:raise ValueError('exact app READY drift')
    app.write_text(text.replace(fragment,'READY=Path('+repr(str(out))+')').replace(
        '小车单击 · 自动准备 · 1280×720 · 120秒','小车演示 · 启动门控修复 · 1280×720 · 120秒'))
    shutil.copyfile(OLD/'auto_open.py',out/'auto_open.py')
    tests=out/'tests';shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copyfile(HERE/'test_startup_fixed.py',tests/'test_startup_fixed.py')
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',STAGED_GENERATED=str(runtime),
        ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
        EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
        AUTOPREP_OUTPUT=str(out),PYTHONPATH=str(out)+os.pathsep+str(BASE)+os.pathsep+
            str(ROOT/'src/mecanum_vicon_control')+os.pathsep+os.environ.get('PYTHONPATH',''))
    def run(label,args,environment=env,timeout=90):
        p=subprocess.run(list(map(str,args)),cwd=ROOT,env=environment,capture_output=True,text=True,timeout=timeout)
        (out/(label+'.log')).write_text(p.stdout+p.stderr)
        print(label+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode:raise ValueError((p.stdout+p.stderr)[-18000:])
        return p.stdout.strip().splitlines()[-1]
    with tempfile.TemporaryDirectory(prefix='startup_fix_x_') as task_dir:
        display_file=Path(task_dir)/'display'
        with display_file.open('w+') as display,(out/'private_x_server.log').open('w') as log:
            server=subprocess.Popen(['Xephyr','-displayfd',str(display.fileno()),'-screen','1280x720','-nolisten','tcp','-ac'],
                env=dict(os.environ,DISPLAY=os.environ.get('DISPLAY',':1'),
                    XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority')),
                pass_fds=(display.fileno(),),stdout=log,stderr=log)
            try:
                deadline=time.monotonic()+5
                while not display_file.read_text().strip():
                    if server.poll() is not None or time.monotonic()>deadline:raise ValueError('private X unavailable')
                    time.sleep(.02)
                testenv=dict(env,DISPLAY=':'+display_file.read_text().strip())
                result=run('host_tests',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',tests,
                    '--junitxml='+str(out/'host_tests.xml')],testenv)
            finally:
                server.terminate()
                try:server.wait(timeout=3)
                except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
    run('host_build',['colcon','--log-base',out/'colcon_log','build','--base-paths',runtime,
        '--build-base',out/'runtime/build','--install-base',out/'runtime/install','--symlink-install',
        '--packages-select',m['control_package'],m['hardware_package']])
    private_logs=out/'rehearsal_ros_logs';private_logs.mkdir(mode=0o700)
    run('real_ros_startup_rehearsal',[sys.executable,HERE/'ros_rehearsal.py','--runtime',runtime,
        '--output',out/'REAL_ROS_REHEARSAL.json'],dict(env,ROS_DOMAIN_ID='187',
            ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(private_logs)))
    if hashes(HERE)!={p:v for p,v in before.items() if str(HERE) in p}:raise ValueError('candidate source drift')
    for p,digest in pins.items():
        if sha(p)!=digest:raise ValueError('unrelated file drift: '+p)
    manifest=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        source_hashes={**m['source_hashes'],**before,str(app):sha(app),str(out/'auto_open.py'):sha(out/'auto_open.py')},
        protected_input_hashes=pins,draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        host_change='Bounded virgin zero-output COMMAND_TIMEOUT handoff; no fault clearing, motion or automatic retry; new independent entry')
    write(out/'build_manifest.json',manifest);check(out/'build_manifest.json')
    write(out/'PREPARATION_RESULT.json',dict(passed=True,tests=result,protected_files_unchanged=len(pins),
        hardware_started=False,flashed=False,motion_started=False,network_changed=False,
        feedback_protection_ms=200,hello_cap_ms=40,automatic_prepare_attempts_max=1,
        real_ros_rehearsal=json.loads((out/'REAL_ROS_REHEARSAL.json').read_text()),
        command='小车演示',old_entries_unchanged=True))
    print(json.dumps(dict(passed=True,tests=result,protected_files=len(pins),output=str(out)),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
