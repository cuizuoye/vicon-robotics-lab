#!/usr/bin/env python3
"""Generate presentation overlay and test it before promoting one shortcut."""
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

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_demo_startup_fixed_2026-10-26/prepared'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
SHORT=Path('/home/PRIMARY_USER/.local/bin/小车演示')
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
        and not any(k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    before=hashes(HERE);before[str(ROOT/'tools/start_car_demo_large_type.sh')]=sha(ROOT/'tools/start_car_demo_large_type.sh')
    protected={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','protected_input_hashes','generated_firmware_hashes'):protected.update(m[group])
    for p,v in protected.items():
        if sha(p)!=v:raise ValueError('parent drift: '+p)
    backup=out/'previous_shortcut.sh';shutil.copyfile(SHORT,backup);protected[str(backup)]=sha(backup)
    proposal=out/'PROPOSED_SHORTCUT.sh'
    proposal.write_text('#!/usr/bin/env bash\nexec bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_car_demo_large_type.sh "$@"\n')
    write(out/'SHORTCUT_MIGRATION.json',dict(path=str(SHORT),old_sha256=sha(SHORT),new_sha256=sha(proposal),
        backup=str(backup),proposal=str(proposal),allowed_source_change=str(SHORT),
        request=str(HERE/'USER_REQUEST.json')))
    runtime=out/'runtime/src';shutil.copytree(m['runtime_src'],runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    control=runtime/m['control_package']/m['control_package']
    shutil.copyfile(HERE/'large_type.py',control/'large_type.py')
    ui=control/'operator_console.py';text=ui.read_text();fragment='Console = AutomaticPrepareConsole'
    if text.count(fragment)!=1:raise ValueError('UI class alias drift')
    ui.write_text(text.replace(fragment,"""from .large_type import LargeTypeConsoleMixin

class LargeTypeConsole(LargeTypeConsoleMixin, AutomaticPrepareConsole):
    pass

Console = LargeTypeConsole"""))
    app=out/'oneclick_app.py';text=(OLD/'oneclick_app.py').read_text()
    for old,new in (('READY=Path('+repr(str(OLD))+')','READY=Path('+repr(str(out))+')'),
        ('from auto_open import AutomaticOpenMixin','from auto_open import AutomaticOpenMixin\nfrom large_type import LargeTypeLauncherMixin'),
        ('class App(AutomaticOpenMixin, ManualApp):','class App(LargeTypeLauncherMixin, AutomaticOpenMixin, ManualApp):')):
        if text.count(old)!=1:raise ValueError('app exact display hook drift: '+old)
        text=text.replace(old,new)
    app.write_text(text);shutil.copyfile(HERE/'large_type.py',out/'large_type.py')
    shutil.copyfile(OLD/'auto_open.py',out/'auto_open.py')
    tests=out/'tests';shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copyfile(HERE/'test_large_type.py',tests/'test_large_type.py')
    inventory=tests/'test_startup_fixed.py';text=inventory.read_text()
    old="assert changed==['mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/auto_prepare.py']"
    new="assert sorted(changed)==sorted(['mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/auto_prepare.py', 'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/operator_console.py'])"
    if text.count(old)!=1:raise ValueError('old inventory assertion drift')
    inventory.write_text(text.replace(old,new))
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',STAGED_GENERATED=str(runtime),
        ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),AUTOPREP_OUTPUT=str(out),
        EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
        PYTHONPATH=str(tests)+os.pathsep+str(out)+os.pathsep+str(BASE)+os.pathsep+
            str(ROOT/'src/mecanum_vicon_control')+os.pathsep+os.environ.get('PYTHONPATH',''))
    def run(label,args,environment=env):
        p=subprocess.run(list(map(str,args)),cwd=ROOT,env=environment,text=True,capture_output=True,timeout=90)
        (out/(label+'.log')).write_text(p.stdout+p.stderr)
        print(label+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode:raise ValueError((p.stdout+p.stderr)[-18000:])
        return p.stdout.strip().splitlines()[-1]
    with tempfile.TemporaryDirectory(prefix='large_type_x_') as task_dir:
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
                private=':'+display_file.read_text().strip();testenv=dict(env,DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private)
                result=run('host_tests',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',tests,
                    '--junitxml='+str(out/'host_tests.xml')],testenv)
                run('ui_render',[sys.executable,HERE/'render.py','--output',out/'renders'],testenv)
            finally:
                server.terminate()
                try:server.wait(timeout=3)
                except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
    run('host_build',['colcon','--log-base',out/'colcon_log','build','--base-paths',runtime,
        '--build-base',out/'runtime/build','--install-base',out/'runtime/install','--symlink-install',
        '--packages-select',m['control_package'],m['hardware_package']])
    # Replay the startup integration again with real isolated ROS nodes. The
    # safety/model files in this UI build are byte-identical to the fixed base.
    logs=out/'rehearsal_ros_logs';logs.mkdir(mode=0o700)
    run('real_ros_startup_rehearsal',[sys.executable,ROOT/'candidates/wand_demo_startup_fixed/ros_rehearsal.py',
        '--runtime',runtime,'--output',out/'REAL_ROS_REHEARSAL.json'],dict(env,ROS_DOMAIN_ID='187',
            ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(logs)))
    for p,v in protected.items():
        if sha(p)!=v:raise ValueError('unrelated original changed: '+p)
    for p,v in before.items():
        if sha(p)!=v:raise ValueError('new display source changed during tests: '+p)
    manifest=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        source_hashes={**m['source_hashes'],**before,str(app):sha(app),str(out/'large_type.py'):sha(out/'large_type.py'),
            str(out/'auto_open.py'):sha(out/'auto_open.py')},protected_input_hashes=protected,
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        host_change='Display-only mixins for 22px body / 24px status and stop / 1280x720; startup fix and safety behavior unchanged')
    write(out/'build_manifest.json',manifest);check(out/'build_manifest.json')
    write(out/'PREPARATION_RESULT.json',dict(passed=True,tests=result,protected_files_unchanged=len(protected),
        body_px=22,secondary_px=20,status_stop_px=24,title_px=30,window=[1280,720],
        hardware_started=False,motion_started=False,flashed=False,network_changed=False,
        shortcut_promoted=False,command='小车演示',private_render_path=str(out/'renders')))
    print(json.dumps(dict(passed=True,tests=result,protected_files=len(protected),output=str(out)),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
