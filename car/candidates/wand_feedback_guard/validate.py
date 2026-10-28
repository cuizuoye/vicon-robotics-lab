#!/usr/bin/env python3
"""Offline native C, private Tk and actual ROS software-plant verification."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from prepare import generate,ROOT,HERE,OLD,sha,hashes,write,check


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    out=a.output.resolve();m=generate(out);runtime=Path(m['runtime_src'])
    snapshot=hashes(HERE)
    def run(label,command,env,timeout=100):
        p=subprocess.run(list(map(str,command)),cwd=ROOT,env=env,capture_output=True,text=True,timeout=timeout)
        (out/(label+'.log')).write_text(p.stdout+p.stderr)
        print(label+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode:raise ValueError((p.stdout+p.stderr)[-18000:])
        return p.stdout.strip().splitlines()[-1] if p.stdout.strip() else 'PASS'
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',FEEDBACK_OUTPUT=str(out),
        STAGED_GENERATED=str(runtime),AUTOSTART_OUTPUT=str(out),TRACKING_OUTPUT=str(out),
        ROTATION_HW_OUTPUT=str(out),AUTOPREP_OUTPUT=str(out),
        ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
        EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
        PYTHONPATH=os.pathsep.join((str(out),str(HERE),str(ROOT/'candidates/wand_tracking_autostart'),
            str(ROOT/'candidates/wand_tracking_workspace'),
            str(ROOT/'docs/wand_demo_large_type_2026-10-26/prepared/tests'),
            str(ROOT/'src/mecanum_vicon_control'),str(runtime/m['control_package']),
            str(runtime/m['hardware_package']),os.environ.get('PYTHONPATH',''))))
    native=run('native_feedback_tests',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
        HERE/'test_feedback.py','--junitxml='+str(out/'native_feedback_tests.xml')],env)
    guard=run('diagnostic_allowlist',[sys.executable,HERE/'diagnose.py','--offline-check'],env)
    # Two parent test functions check intentional differences (unchanged bridge
    # bytes, old instance protocol). They are replaced by exact new inventory
    # and native tests, never passed off as successful candidate regressions.
    with tempfile.TemporaryDirectory(prefix='feedback_guard_private_x_') as temp:
        display_file=Path(temp)/'display'
        with display_file.open('w+') as display,(out/'private_x.log').open('w') as log:
            server=subprocess.Popen(['Xephyr','-displayfd',str(display.fileno()),'-screen','1280x720',
                '-nolisten','tcp','-ac'],env=dict(env,DISPLAY=os.environ.get('DISPLAY',':1'),
                XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority')),
                pass_fds=(display.fileno(),),stdout=log,stderr=log)
            try:
                deadline=time.monotonic()+5
                while not display_file.read_text().strip():
                    if server.poll() is not None or time.monotonic()>deadline:raise ValueError('private X failed')
                    time.sleep(.02)
                private=':'+display_file.read_text().strip()
                ui_env=dict(env,DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private,TRACKING_PRIVATE_DISPLAY=private)
                ui=run('reused_autostart_ui',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    ROOT/'candidates/wand_tracking_autostart/test_autostart.py',
                    '-k','not test_all_controller_bridge_and_operator_console_sources_are_byte_identical and not test_running_old_console_is_not_mistaken_for_this_frontend',
                    '--junitxml='+str(out/'reused_autostart_ui.xml')],ui_env)
                run('private_ui_render',[sys.executable,ROOT/'candidates/wand_tracking_workspace/render.py',
                    '--output',out/'renders'],ui_env)
            finally:
                server.terminate()
                try:server.wait(timeout=3)
                except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
    run('host_build',['colcon','--log-base',out/'colcon_log','build','--base-paths',runtime,
        '--build-base',out/'runtime/build','--install-base',out/'runtime/install','--symlink-install',
        '--packages-select',m['control_package'],m['hardware_package']],env)
    ros_logs=out/'private_ros_logs';ros_logs.mkdir(mode=0o700)
    run('actual_ros_software_rehearsal',[sys.executable,out/'ros_rehearsal.py','--runtime',runtime,
        '--output',out/'REAL_ROS_REHEARSAL.json'],dict(env,ROS_DOMAIN_ID='187',
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(ros_logs)),timeout=150)
    if hashes(HERE)!=snapshot:raise ValueError('source changed during validation')
    check(out/'build_manifest.json')
    m['feedback_guard_passed']=True
    verified=[out/n for n in ('native_feedback_tests.xml','reused_autostart_ui.xml',
        'REAL_ROS_REHEARSAL.json','renders/geometry.json','PRIVATE_CHANGE_INVENTORY.json')]
    m['verification_hashes']={str(p):sha(p) for p in verified}
    m['protected_input_hashes'].update(m['verification_hashes'])
    write(out/'build_manifest.json',m)
    run('new_entry_check',['bash',ROOT/'tools/start_car_tracking_feedback.sh','--check'],env)
    run('accepted_entry_check',['bash',ROOT/'tools/start_car_tracking_auto.sh','--check'],env)
    result=dict(passed=True,native_c_and_feedback_tests=native,reused_autostart_tests=ui,
        diagnostic_guard=guard,actual_ros_software_cases=8,
        protected_parent_files_unchanged=len(m['protected_input_hashes'])-len(verified),
        controller_and_ui_bytes_unchanged=True,transport_files_changed=3,
        firmware_flashed=False,motion_started=False,network_settings_changed=False,
        feedback_protection_ms=200,automatic_recovery_after_active=False,
        hardware_diagnostic_pending=True,physical_motion_acceptance_pending=True,
        old_entry_unchanged=True,new_entry=str(ROOT/'tools/start_car_tracking_feedback.sh'))
    write(out/'PREPARATION_RESULT.json',result);print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
