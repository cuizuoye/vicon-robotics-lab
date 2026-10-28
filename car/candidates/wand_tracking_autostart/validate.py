#!/usr/bin/env python3
"""No real devices: private GUI, software ROS plant and parent regressions."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from prepare import generate,ROOT,HERE,OLD,BASE,sha,write,hashes,check

XY=ROOT/'docs/wand_demo_large_type_2026-10-26/prepared'
ROT=ROOT/'docs/wand_rotation_hardware_2026-10-26/prepared'


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    out=a.output.resolve();m=generate(out);runtime=Path(m['runtime_src']);snapshot=hashes(HERE)
    def run(label,command,env,timeout=100):
        p=subprocess.run(list(map(str,command)),cwd=ROOT,env=env,capture_output=True,text=True,timeout=timeout)
        (out/(label+'.log')).write_text(p.stdout+p.stderr)
        print(label+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode:raise ValueError((p.stdout+p.stderr)[-16000:])
        return p.stdout.strip().splitlines()[-1] if p.stdout.strip() else 'PASS'
    base_env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
    def env_for(folder):
        return dict(base_env,AUTOSTART_OUTPUT=str(folder),TRACKING_OUTPUT=str(folder),ROTATION_HW_OUTPUT=str(folder),
            STAGED_GENERATED=str(folder/'runtime/src'),AUTOPREP_OUTPUT=str(folder),
            ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
            EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
            PYTHONPATH=os.pathsep.join((str(HERE),str(folder),str(ROOT/'candidates/wand_tracking_workspace'),
                str(XY/'tests'),str(BASE),str(ROOT/'src/mecanum_vicon_control'),
                str(folder/'runtime/src'/m['control_package']),str(folder/'runtime/src'/m['hardware_package']),
                os.environ.get('PYTHONPATH',''))))
    with tempfile.TemporaryDirectory(prefix='tracking_auto_private_x_') as task_dir:
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
                ne=dict(env_for(out),DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private,TRACKING_PRIVATE_DISPLAY=private)
                new_tests=run('autostart_tests',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    HERE/'test_autostart.py',HERE/'test_display.py','--junitxml='+str(out/'autostart_tests.xml')],ne)
                tracking_tests=run('parent_tracking_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    ROOT/'candidates/wand_tracking_workspace/test_tracking.py',ROOT/'candidates/wand_tracking_workspace/test_ui.py',
                    '--junitxml='+str(out/'parent_tracking_tests.xml')],dict(env_for(OLD),DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private))
                xy_tests=run('parent_translation_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    XY/'tests','--junitxml='+str(out/'parent_xy_tests.xml')],dict(env_for(XY),DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private))
                rotation_tests=run('parent_rotation_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    ROOT/'candidates/wand_rotation_hardware/test_migration.py',ROOT/'candidates/wand_rotation_hardware/test_ui.py',
                    '--junitxml='+str(out/'parent_rotation_tests.xml')],dict(env_for(ROT),DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private))
                # Same motion-console renderer, new launcher class/layout.
                run('ui_render',[sys.executable,ROOT/'candidates/wand_tracking_workspace/render.py','--output',out/'renders'],ne)
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
        if sha(p)!=v:raise ValueError('Protected parent changed: '+p)
    if hashes(HERE)!=snapshot:raise ValueError('Source changed during validation')
    check(out/'build_manifest.json');m['autostart_passed']=True
    m['verification_hashes']={str(p):sha(p) for p in (out/'autostart_tests.xml',out/'parent_tracking_tests.xml',
        out/'parent_xy_tests.xml',out/'parent_rotation_tests.xml',out/'REAL_ROS_REHEARSAL.json',
        out/'renders/geometry.json',out/'PRIVATE_CHANGE_INVENTORY.json')}
    m['protected_input_hashes'].update(m['verification_hashes']);write(out/'build_manifest.json',m)
    run('new_file_check',['bash',ROOT/'tools/start_car_tracking_auto.sh','--check'],base_env)
    run('short_command_file_check',[Path('/home/PRIMARY_USER/.local/bin/小车全场演示'),'--check'],base_env)
    run('parent_entry_check',['bash',ROOT/'tools/start_car_tracking_workspace.sh','--check'],base_env)
    result=dict(passed=True,new_tests=new_tests,parent_tracking_tests=tracking_tests,
        parent_xy_tests=xy_tests,parent_rotation_tests=rotation_tests,real_ros_software_cases=8,
        manual_checkboxes=0,automatic_open=True,automatic_prepare=True,automatic_motion=False,
        fresh_space_click_required=True,site_conditions_machine_verified=False,
        site_preconditions_basis='EXPLICIT_USER_STANDING_INSTRUCTION',max_active_s=120.,
        controller_bridge_console_bytes_unchanged=True,
        protected_parent_files_unchanged=len(m['protected_input_hashes'])-len(m['verification_hashes']),
        firmware_flashed=False,physical_car_started=False,network_settings_changed=False,
        remote_display_environment_tested=True,actual_windows_x11_acceptance_pending=True)
    write(out/'PREPARATION_RESULT.json',result);print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
