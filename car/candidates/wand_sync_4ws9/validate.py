#!/usr/bin/env python3
"""Native actual C + ARM + private ROS/UI; never connect to hardware."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from prepare import generate,ROOT,HERE,PARENT,BASE,CONTROL,HARDWARE,sha,hashes,write

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);out=p.parse_args().output.resolve()
    m=generate(out);snapshot=hashes(HERE);env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
    def run(name,command,environment=env,timeout=90):
        r=subprocess.run(list(map(str,command)),cwd=ROOT,env=environment,capture_output=True,text=True,timeout=timeout)
        (out/(name+'.log')).write_text(r.stdout+r.stderr)
        print(name+': '+('PASS' if r.returncode==0 else 'FAIL'),flush=True)
        if r.returncode:raise ValueError((r.stdout+r.stderr)[-12000:])
        return r.stdout.strip().splitlines()[-1] if r.stdout.strip() else 'PASS'
    fw=out/'firmware';native=out/'native'
    run('native_configure',['cmake','-S',fw,'-B',native,'-G','Ninja','-DCMAKE_BUILD_TYPE=Debug'])
    run('native_build',['cmake','--build',native,'-j2'])
    run('native_actual_controller_faults',['ctest','--test-dir',native,'--output-on-failure'])
    run('native_angular_ramps',[native/'test_rotation',native])
    esp=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready/esp_project/main'
    run('esp_protocol_build',['cc','-std=c11','-Wall','-Wextra','-Wpedantic','-Werror','-g',
        '-fsanitize=address,undefined','-fno-pie','-no-pie','-I'+str(esp),'-I'+str(ROOT/'protocol'),
        HERE/'test_esp_payload.c',esp/'mc_protocol.c','-o',out/'test_esp_payload'])
    run('esp_protocol_200byte',[out/'test_esp_payload'])
    arm=out/'arm'
    run('arm_configure',['cmake','-S',fw,'-B',arm,'-G','Ninja','-DSERVO_ARM=ON','-DCMAKE_BUILD_TYPE=Debug'])
    run('arm_build',['cmake','--build',arm,'--target','mecanum_f407_4ws9_staged_NOT_FLASHED','-j2'])
    image=arm/'mecanum_f407_4ws9_staged_NOT_FLASHED.bin'
    m['firmware']=dict(path=str(image),sha256=sha(image),bytes=image.stat().st_size,identity='4WS9',
        authority_version=5,diagnostic_payload_bytes=200,flashed=False,physical_qualified=False,
        supported_stages=list(range(7)),boot_stage=0,bench_limit_ms=5000)
    runtime=Path(m['runtime_src'])
    ne=dict(env,SYNC_OUTPUT=str(out),PYTHONPATH=os.pathsep.join((str(HERE),str(runtime/CONTROL),str(runtime/HARDWARE),
        str(ROOT/'src/mecanum_vicon_control'),os.environ.get('PYTHONPATH',''))))
    tests=run('new_tier_wire_and_follow_tests',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',HERE/'test_sync.py',
        '--junitxml='+str(out/'new_tests.xml')],ne)
    run('same_time_comparison',[sys.executable,'-c',
        'import json;from pathlib import Path;from test_sync import comparison;'
        'p=Path('+repr(str(out/'SAME_TIME_COMPARISON.json'))+');r=comparison();p.write_text(json.dumps(r,indent=2)+"\\n");print(json.dumps(r))'],ne)
    # Full original UI/regressions run against the ORIGINAL runtime in separate
    # processes; intentional new limits are not hidden by modifying old tests.
    old=ROOT/'docs/wand_demo_large_type_2026-10-26/prepared'
    common=dict(env,STAGED_GENERATED=str(old/'runtime/src'),AUTOPREP_OUTPUT=str(old),
        ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
        EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
        PYTHONPATH=os.pathsep.join((str(old/'tests'),str(old),str(BASE/'candidates/servo_4ws8'),
            str(ROOT/'src/mecanum_vicon_control'),os.environ.get('PYTHONPATH',''))))
    with tempfile.TemporaryDirectory(prefix='4ws9_private_x_') as td:
        display_file=Path(td)/'display'
        with display_file.open('w+') as display,(out/'private_x.log').open('w') as log:
            server=subprocess.Popen(['Xephyr','-displayfd',str(display.fileno()),'-screen','1280x720','-nolisten','tcp','-ac'],
                env=dict(env,DISPLAY=os.environ.get('DISPLAY',':1'),XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority')),
                pass_fds=(display.fileno(),),stdout=log,stderr=log)
            try:
                deadline=time.monotonic()+5
                while not display_file.read_text().strip():
                    if server.poll() is not None or time.monotonic()>deadline:raise ValueError('Private X failed')
                    time.sleep(.02)
                private=':'+display_file.read_text().strip()
                old_tests=run('accepted_xy_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',old/'tests',
                    '--junitxml='+str(out/'accepted_tests.xml')],dict(common,DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private))
                ro=dict(common,ROTATION_HW_OUTPUT=str(PARENT),STAGED_GENERATED=str(PARENT/'runtime/src'),AUTOPREP_OUTPUT=str(PARENT),
                    DISPLAY=private,UI_PRIVATE_TEST_DISPLAY=private,PYTHONPATH=str(PARENT)+os.pathsep+common['PYTHONPATH'])
                rotation_tests=run('accepted_rotation_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    ROOT/'candidates/wand_rotation_hardware/test_migration.py',ROOT/'candidates/wand_rotation_hardware/test_ui.py',
                    '--junitxml='+str(out/'accepted_rotation_tests.xml')],ro)
                run('large_ui_preview',[sys.executable,HERE/'preview.py','--output',out/'renders'],
                    dict(ne,DISPLAY=private,SYNC_PRIVATE_DISPLAY=private))
            finally:
                server.terminate()
                try:server.wait(timeout=3)
                except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
    run('private_ros_packages',['colcon','--log-base',out/'colcon_log','build','--base-paths',runtime,
        '--build-base',out/'runtime/build','--install-base',out/'runtime/install','--symlink-install',
        '--packages-select',CONTROL,HARDWARE])
    logs=out/'private_ros_logs';logs.mkdir(mode=0o700)
    run('real_ros_high_sync_and_faults',[sys.executable,out/'ros_rehearsal.py','--runtime',runtime,
        '--output',out/'REAL_ROS_REHEARSAL.json'],dict(ne,ROS_DOMAIN_ID='187',
            ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(logs)))
    # Original rectangle algorithms with sanitizers, not a modified mock.
    flags=['cc','-std=c11','-Wall','-Wextra','-Wpedantic','-Werror','-g','-fsanitize=address,undefined',
        '-fno-omit-frame-pointer','-fno-pie','-no-pie','-DDRIVE_STARTUP_OBSERVE=1','-DAUTONOMOUS_RECTANGLE=1',
        '-I'+str(BASE/'Inc'),'-I'+str(ROOT/'protocol')]
    sources=[BASE/'Src'/n for n in ('rectangle_demo.c','four_drive_group.c','four_wheel_speed.c','mecanum_drive_plan.c','mecanum_kinematics.c')]
    for label,test,extra in (('rectangle_body','test_rectangle_demo.c',[]),
                           ('rectangle_trace','test_rectangle_startup_trace.c',[BASE/'Src/rectangle_startup_trace.c'])):
        run(label+'_build',flags+[BASE/'tests'/test,*sources,*extra,'-lm','-o',out/label]);run(label,[out/label])
    for f,v in m['protected_input_hashes'].items():
        if sha(f)!=v:raise ValueError('Protected parent changed: '+f)
    if hashes(HERE)!=snapshot:raise ValueError('Candidate sources changed during validation')
    m['offline_passed']=True;m['runtime_hashes']=hashes(runtime);m['firmware_source_hashes']=hashes(fw)
    m['evidence_hashes']={str(p):sha(p) for p in (out/'new_tests.xml',out/'accepted_tests.xml',out/'accepted_rotation_tests.xml',
        out/'SAME_TIME_COMPARISON.json',out/'REAL_ROS_REHEARSAL.json',out/'renders/geometry.json')}
    write(out/'build_manifest.json',m)
    # Explicit draft templates cannot authorize a programmer or a transport.
    write(out/'FLASH_RECORD.NOT_APPROVED.json',dict(identity='4WS9',image_sha256=m['firmware']['sha256'],flashed=False,
        independent_readback_verified=False,pre_flash_full_backup_path=None,pre_flash_full_backup_sha256=None,
        full_readback_path=None,full_readback_sha256=None))
    for stage in range(7):
        for purpose in ('sign','suspended'):
            write(out/f'TRIAL_STAGE_{stage}_{purpose}.NOT_APPROVED.json',dict(identity='4WS9',
                kind='4WS9_V5_CONTROLLED_TRIAL_NOT_QUALIFICATION',image_sha256=m['firmware']['sha256'],authorized_stage=stage,
                purpose=purpose,human_authorized=False,previous_stage_evidence=None,previous_stage_evidence_sha256=None,
                measured_stop_envelope_m=None,electrical_thermal_trial_reviewed=False))
            run(f'file_check_{purpose}_{stage}',[sys.executable,HERE/'cli.py','--prepared',out,'--stage',stage,'--purpose',purpose,'--check'])
    result=dict(passed=True,new_tests=tests,accepted_xy_tests=old_tests,accepted_rotation_tests=rotation_tests,
        protected_parent_files_unchanged=len(m['protected_input_hashes']),real_ros_software_cases=6,
        native_c_sanitizers=True,arm_built=True,firmware=m['firmware'],
        hardware_connected=False,flashed=False,physical_motion=False,network_changed=False,old_entries_changed=False,
        stopping_electrical_thermal_qualified=False,next_required='New-image full backup, authorized STM32 flash/readback, disabled handshake, stage0 suspended first')
    write(out/'PREPARATION_RESULT.json',result);print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
