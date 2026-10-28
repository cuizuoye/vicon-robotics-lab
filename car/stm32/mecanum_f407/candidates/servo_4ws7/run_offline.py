#!/usr/bin/env python3
"""Independent offline builds/unit/model checks. No ROS nodes/device/flash IO.

Never accepts a runtime mode or executable command from JSON. Commands below
are only compilers, native tests, read-only hashes and Python unit checks.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

HERE=Path(__file__).resolve().parent
BASE=HERE.parents[1]
ROOT=BASE.parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--arm',action='store_true')
    parser.add_argument('--regression',action='store_true')
    args=parser.parse_args();out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    report=dict(passed=False,flashed=False,hardware_started=False,network_changed=False,
        accepted_sources_edited=False,electrical_acceptance=False,loaded_performance_acceptance=False,
        hardware_released=False,commands=[])
    before={}
    def run(name,command,*,env=None,expected_failure=None,input_text=None):
        started=time.monotonic()
        p=subprocess.run(list(map(str,command)),cwd=ROOT,env=env,text=True,input=input_text,
                         stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (out/(name+'.log')).write_text(p.stdout)
        passed=p.returncode==0 if expected_failure is None else p.returncode!=0 and expected_failure in p.stdout
        report['commands'].append(dict(name=name,returncode=p.returncode,
            expected_failure=expected_failure,passed=passed,seconds=time.monotonic()-started))
        print(('PASS ' if passed else 'FAIL ')+name,flush=True)
        if not passed:raise RuntimeError(str(out/(name+'.log')))
        return p.stdout
    try:
        expected=json.loads((HERE/'upstream.json').read_text())
        previous=json.loads((ROOT/'docs/wand_070_migration_review_2026-10-23/analysis_with_driver_spec/assessment.json').read_text())
        for path,value in previous['retained_hashes'].items():
            expected[str(Path(path).relative_to(ROOT))]=value
        before={path:sha(ROOT/path) for path in expected}
        if before!=expected:raise ValueError('protected source/image drift before test')
        (out/'preserved_before.json').write_text(json.dumps(before,indent=2)+'\n')
        for name in ('docs/wand_offline_2026-10-22/existing_control.sha256',
                     'docs/wand_hardware_2026-10-22_firmware_source.sha256'):
            run('before_'+Path(name).stem,['sha256sum','-c','--quiet',name])
        native=out/'native'
        run('native_configure',['cmake','-S',HERE,'-B',native,'-G','Ninja','-DCMAKE_BUILD_TYPE=Debug'])
        run('native_build',['cmake','--build',native,'-j2'])
        run('native_ctest',['ctest','--test-dir',native,'--output-on-failure'])
        run('native_synthetic_fixtures',[native/'test_servo_4ws7',out/'active.bin',out/'reversing.bin'])
        env=os.environ.copy();env['SERVO_FIXTURES']=str(out)
        host=run('host_strict_contract',['python3','-m','pytest','-q',HERE/'host/test_servo_contract.py',
                 '--junitxml='+str(out/'host_contract.xml')],env=env)
        report['host_summary']=host.strip().splitlines()[-1]
        defs=['-DFOUR_SERVO_CANDIDATE=1','-DFOUR_LINK_MOTION=1','-DFOUR_LINK_GROUNDB=1','-DDRIVE_STARTUP_OBSERVE=1']
        run('unreleased_policy_cannot_be_toggled',['cc','-E','-x','c',*defs,'-DSERVO_EXECUTION_RELEASED=1',
            '-I'+str(HERE/'Inc'),'-'],expected_failure='Hardware execution is NOT released',
            input_text='#include "four_speed_pwm_profile.h"\n')
        run('native_test_bypass_forbidden_on_arm',['cc','-E','-x','c',*defs,'-D__arm__=1','-DSERVO_NATIVE_TEST=1',
            '-I'+str(HERE/'Inc'),'-'],expected_failure='Native test bypass must never enter an ARM image',
            input_text='#include "four_speed_pwm_profile.h"\n')
        if args.regression:
            run('legacy_motion_12_profiles',['bash',BASE/'tests/run_motion_tests.sh'])
            flags=['cc','-std=c11','-Wall','-Wextra','-Wpedantic','-Werror','-g',
                '-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie',
                '-DDRIVE_STARTUP_OBSERVE=1','-DAUTONOMOUS_RECTANGLE=1',
                '-I'+str(BASE/'Inc'),'-I'+str(ROOT/'protocol')]
            algorithms=[BASE/'Src'/p for p in ('rectangle_demo.c','four_drive_group.c',
                'four_wheel_speed.c','mecanum_drive_plan.c','mecanum_kinematics.c')]
            for name,source,extra in (
                ('rectangle_body','test_rectangle_demo.c',[]),
                ('rectangle_trace','test_rectangle_startup_trace.c',[BASE/'Src/rectangle_startup_trace.c'])):
                run(name+'_build',flags+[BASE/'tests'/source,*algorithms,*extra,'-lm','-o',out/name])
                run(name,[out/name])
            legacy_env=os.environ.copy()
            legacy_env['SERVO_GENERATED_ROOT']=str(ROOT/'docs/wand_low_latency_servo_2026-10-23/verified_runtime/src')
            py=run('python_legacy_regression',[sys.executable,'-m','pytest','-q',
                ROOT/'src/mecanum_wand_control/test',ROOT/'src/mecanum_wand_hardware/test',
                ROOT/'src/mecanum_vicon_control/test',ROOT/'candidates/wand_low_latency_servo/test_continuous_servo.py'],env=legacy_env)
            report['legacy_python_summary']=py.strip().splitlines()[-1]
        if args.arm:
            arm=out/'arm'
            run('arm_configure',['cmake','-S',HERE,'-B',arm,'-G','Ninja','-DSERVO_ARM=ON','-DCMAKE_BUILD_TYPE=Debug'])
            run('arm_build',['cmake','--build',arm,'--target','mecanum_f407_servo_4ws7_LOCKED_OFFLINE','-j2'])
            image=arm/'mecanum_f407_servo_4ws7_LOCKED_OFFLINE.bin'
            report['arm_image']=dict(path=str(image),bytes=image.stat().st_size,sha256=sha(image),
                                     hardware_execution_locked=True,flashed=False)
        report['passed']=True
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc())
    finally:
        if before:
            after={path:sha(ROOT/path) for path in before}
            report['preservation_hashes_match']=before==after
            report['protected_file_count']=len(before)
            report['passed'] &= before==after
            (out/'preserved_after.json').write_text(json.dumps(after,indent=2)+'\n')
        report['candidate_sources']={str(p.relative_to(HERE)):sha(p) for p in HERE.rglob('*')
            if p.is_file() and '__pycache__' not in p.parts and p.suffix in ('.c','.h','.py','.json','.txt','.md')}
        (out/'summary.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
