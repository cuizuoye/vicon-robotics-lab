#!/usr/bin/env python3
"""Full offline verification and evidence sealing; no real-car launch."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from prepare import generate,ROOT,HERE,OLD,BASE,sha,write,hashes,check


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    out=a.output.resolve();m=generate(out);runtime=Path(m['runtime_src'])
    snapshot=hashes(HERE)
    def run(label,command,env,timeout=90):
        p=subprocess.run(list(map(str,command)),cwd=ROOT,env=env,capture_output=True,text=True,timeout=timeout)
        (out/(label+'.log')).write_text(p.stdout+p.stderr)
        print(label+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode:raise ValueError((p.stdout+p.stderr)[-17000:])
        return p.stdout.strip().splitlines()[-1] if p.stdout.strip() else 'PASS'
    base_env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
    new_env=dict(base_env,ROTATION_HW_OUTPUT=str(out),STAGED_GENERATED=str(runtime),AUTOPREP_OUTPUT=str(out),
        ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
        EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
        PYTHONPATH=os.pathsep.join((str(HERE),str(out),str(OLD/'tests'),str(BASE),
            str(ROOT/'src/mecanum_vicon_control'),str(runtime/m['control_package']),str(runtime/m['hardware_package']),os.environ.get('PYTHONPATH',''))))
    old_env=dict(base_env,STAGED_GENERATED=str(OLD/'runtime/src'),AUTOPREP_OUTPUT=str(OLD),
        ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
        EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
        PYTHONPATH=os.pathsep.join((str(OLD/'tests'),str(OLD),str(BASE),str(ROOT/'src/mecanum_vicon_control'),os.environ.get('PYTHONPATH',''))))
    with tempfile.TemporaryDirectory(prefix='rotation_hw_private_x_') as task_dir:
        display_file=Path(task_dir)/'display'
        with display_file.open('w+') as display,(out/'private_x.log').open('w') as log:
            server=subprocess.Popen(['Xephyr','-displayfd',str(display.fileno()),'-screen','1280x720','-nolisten','tcp','-ac'],
                env=dict(base_env,DISPLAY=os.environ.get('DISPLAY',':1'),XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority')),
                pass_fds=(display.fileno(),),stdout=log,stderr=log)
            try:
                deadline=time.monotonic()+5
                while not display_file.read_text().strip():
                    if server.poll() is not None or time.monotonic()>deadline:raise ValueError('Private X unavailable')
                    time.sleep(.02)
                private=':'+display_file.read_text().strip()
                ne=dict(new_env,DISPLAY=private,ROTATION_HW_PRIVATE_DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private)
                oe=dict(old_env,DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private)
                new_tests=run('migration_tests',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    HERE/'test_migration.py',HERE/'test_ui.py','--junitxml='+str(out/'migration_tests.xml')],ne)
                old_tests=run('accepted_translation_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    OLD/'tests','--junitxml='+str(out/'accepted_tests.xml')],oe)
                run('ui_render',[sys.executable,HERE/'render.py','--output',out/'renders'],ne)
            finally:
                server.terminate()
                try:server.wait(timeout=3)
                except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
    run('host_build',['colcon','--log-base',out/'colcon_log','build','--base-paths',runtime,
        '--build-base',out/'runtime/build','--install-base',out/'runtime/install','--symlink-install',
        '--packages-select',m['control_package'],m['hardware_package']],base_env)
    logs=out/'private_ros_logs';logs.mkdir(mode=0o700)
    run('real_ros_rehearsal',[sys.executable,HERE/'ros_rehearsal.py','--runtime',runtime,
        '--output',out/'REAL_ROS_REHEARSAL.json'],dict(base_env,ROS_DOMAIN_ID='187',
            ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(logs)))
    for p,v in m['protected_input_hashes'].items():
        if sha(p)!=v:raise ValueError('Parent changed: '+p)
    if hashes(HERE)!=snapshot:raise ValueError('Candidate source changed during validation')
    check(out/'build_manifest.json')
    m['rotation_migration_passed']=True
    m['rotation_qualification']='OFFLINE_AND_PRIVATE_ROS_ONLY_NOT_PHYSICAL_ACCEPTANCE'
    m['verification_hashes']={str(p):sha(p) for p in (out/'migration_tests.xml',out/'accepted_tests.xml',
        out/'REAL_ROS_REHEARSAL.json',out/'renders/geometry.json',out/'PRIVATE_CHANGE_INVENTORY.json')}
    # Include evidence in the ordinary launch guard's already-enforced hash
    # group rather than inventing a new ignored verification whitelist.
    m['protected_input_hashes'].update(m['verification_hashes'])
    write(out/'build_manifest.json',m);check(out/'build_manifest.json')
    run('candidate_file_check',[sys.executable,out/'oneclick_app.py','--check'],new_env)
    run('original_entry_check',['bash',ROOT/'tools/start_car_demo_large_type.sh','--check'],base_env)
    result=dict(passed=True,new_tests=new_tests,accepted_translation_tests=old_tests,
        protected_parent_files_unchanged=len(m['protected_input_hashes'])-len(m['verification_hashes']),
        host_packages_built=True,real_ros_software_cases=6,hardware_started=False,flashed=False,
        network_changed=False,original_entry_changed=False,physical_acceptance_pending=True,
        yaw_limits=dict(rate_radps=.1,accel_radps2=.4),xy_limits=dict(speed_mps=.7,accel_mps2=.8),
        file_check='bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_car_xy_yaw_demo.sh --check')
    write(out/'PREPARATION_RESULT.json',result)
    print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
