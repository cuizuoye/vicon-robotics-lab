#!/usr/bin/env python3
"""Offline cores, private Tk, private ROS sources/Plant, parent regressions."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from prepare import generate,ROOT,HERE,OLD,BASE,sha,write,hashes,check

XY=ROOT/'docs/wand_demo_large_type_2026-10-26/prepared'


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    out=a.output.resolve();m=generate(out);runtime=Path(m['runtime_src']);snapshot=hashes(HERE)
    def run(label,command,env,timeout=100):
        p=subprocess.run(list(map(str,command)),cwd=ROOT,env=env,capture_output=True,text=True,timeout=timeout)
        (out/(label+'.log')).write_text(p.stdout+p.stderr)
        print(label+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode:raise ValueError((p.stdout+p.stderr)[-18000:])
        return p.stdout.strip().splitlines()[-1] if p.stdout.strip() else 'PASS'
    base_env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
    def env_for(folder):
        return dict(base_env,TRACKING_OUTPUT=str(folder),ROTATION_HW_OUTPUT=str(folder),
            STAGED_GENERATED=str(folder/'runtime/src'),AUTOPREP_OUTPUT=str(folder),
            ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
            EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
            PYTHONPATH=os.pathsep.join((str(HERE),str(folder),str(XY/'tests'),str(BASE),
                str(ROOT/'src/mecanum_vicon_control'),str(folder/'runtime/src'/m['control_package']),
                str(folder/'runtime/src'/m['hardware_package']),os.environ.get('PYTHONPATH',''))))
    with tempfile.TemporaryDirectory(prefix='tracking_private_x_') as task_dir:
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
                ne=dict(env_for(out),DISPLAY=private,TRACKING_PRIVATE_DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private)
                new_tests=run('tracking_tests',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    HERE/'test_tracking.py',HERE/'test_ui.py','--junitxml='+str(out/'tracking_tests.xml')],ne)
                old_tests=run('accepted_translation_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    XY/'tests','--junitxml='+str(out/'accepted_tests.xml')],dict(env_for(XY),DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private))
                rotation_tests=run('accepted_rotation_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    ROOT/'candidates/wand_rotation_hardware/test_migration.py',
                    ROOT/'candidates/wand_rotation_hardware/test_ui.py','--junitxml='+str(out/'rotation_tests.xml')],
                    dict(env_for(OLD),DISPLAY=private,ROTATION_HW_PRIVATE_DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private))
                run('ui_render',[sys.executable,HERE/'render.py','--output',out/'renders'],ne)
            finally:
                server.terminate()
                try:server.wait(timeout=3)
                except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
    run('host_build',['colcon','--log-base',out/'colcon_log','build','--base-paths',runtime,
        '--build-base',out/'runtime/build','--install-base',out/'runtime/install','--symlink-install',
        '--packages-select',m['control_package'],m['hardware_package']],base_env)
    logs=out/'private_ros_logs';logs.mkdir(mode=0o700)
    run('real_ros_rehearsal',[sys.executable,out/'ros_rehearsal.py','--runtime',runtime,
        '--output',out/'REAL_ROS_REHEARSAL.json'],dict(env_for(out),ROS_DOMAIN_ID='187',
            ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(logs)),timeout=140)
    for p,v in m['protected_input_hashes'].items():
        if sha(p)!=v:raise ValueError('Parent changed: '+p)
    if hashes(HERE)!=snapshot:raise ValueError('Source changed during validation')
    check(out/'build_manifest.json')
    m['tracking_workspace_passed']=True
    m['verification_hashes']={str(p):sha(p) for p in (out/'tracking_tests.xml',out/'accepted_tests.xml',
        out/'rotation_tests.xml',out/'REAL_ROS_REHEARSAL.json',out/'renders/geometry.json')}
    m['protected_input_hashes'].update(m['verification_hashes']);write(out/'build_manifest.json',m)
    run('candidate_file_check',['bash',ROOT/'tools/start_car_tracking_workspace.sh','--check'],env_for(out))
    run('original_entry_check',['bash',ROOT/'tools/start_car_demo_large_type.sh','--check'],base_env)
    run('original_rotation_entry_check',['bash',ROOT/'tools/start_car_xy_yaw_demo.sh','--check'],base_env)
    result=dict(passed=True,new_tests=new_tests,translation_regression=old_tests,rotation_regression=rotation_tests,
        real_ros_software_cases=8,protected_parent_files_unchanged=len(m['protected_input_hashes'])-len(m['verification_hashes']),
        fixed_fence_enabled=False,target_radius_limit_enabled=False,mapping_scale=1.,max_active_s=120.,
        pose_timeout_s=.20,xy_speed_mps=.70,xy_accel_mps2=.80,yaw_rate_radps=.10,yaw_accel_radps2=.40,
        hardware_started=False,flashed=False,network_changed=False,original_entry_changed=False,
        full_vicon_coverage_measured=False,obstacle_detection_available=False)
    write(out/'PREPARATION_RESULT.json',result);print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
