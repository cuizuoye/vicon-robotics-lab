#!/usr/bin/env python3
"""Reproducible OFFLINE candidate validation. No flasher, serial, UDP or ROS nodes.

All generated products are in a fresh --output directory. Existing accepted
sources and the saved 4WGB firmware are checked before and after. Source this
workspace's ROS environment only when using --python-tests.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time
import traceback

HERE=Path(__file__).resolve().parent
BASE=HERE.parents[1]
WORKSPACE=BASE.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def disable_core():
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--arm',action='store_true')
    parser.add_argument('--python-tests',action='store_true')
    args=parser.parse_args()
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    result=dict(candidate_validation_passed=False,all_suites_passed=False,
        verification='NATIVE C + SYNTHETIC PWM-DRIVEN PLANT ONLY; NO HARDWARE',
        actions=dict(flashed=False,hardware_started=False,network_changed=False,legacy_files_edited=False),
        commands=[],baseline_warnings=[])
    def run(name,cmd,required=True):
        started=time.monotonic()
        proc=subprocess.run(cmd,cwd=WORKSPACE,text=True,stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,preexec_fn=disable_core)
        (output/(name+'.log')).write_text(proc.stdout)
        result['commands'].append(dict(name=name,command=list(map(str,cmd)),returncode=proc.returncode,
            elapsed_s=time.monotonic()-started,log=name+'.log'))
        print(('PASS ' if proc.returncode==0 else 'FAIL ')+name,flush=True)
        if proc.returncode and required: raise RuntimeError(name+' failed; see '+str(output/(name+'.log')))
        return proc
    before={}
    try:
        for file,expected in json.loads((HERE/'baseline.json').read_text()).items():
            assert sha(BASE/file)==expected,('baseline overlay drift',file)
        protected=[]
        for manifest in ('docs/wand_hardware_2026-10-22_firmware_source.sha256',
                         'docs/wand_offline_2026-10-22/existing_control.sha256'):
            run('before_'+Path(manifest).stem,['sha256sum','-c','--quiet',manifest])
            for line in (WORKSPACE/manifest).read_text().splitlines():
                if not line.strip():continue
                _,file=line.split(maxsplit=1)
                protected.append(WORKSPACE/file.lstrip('*'))
        for suffix in ('bin','elf'):
            saved=BASE/('build/GroundMotionB_20261021/mecanum_f407_four_link_groundb.'+suffix)
            if saved.exists():protected.append(saved)
        before={str(p.relative_to(WORKSPACE)):sha(p) for p in protected}
        (output/'preserved_before.json').write_text(json.dumps(before,indent=2)+'\n')
        native=output/'native'
        run('native_configure',['cmake','-S',str(HERE),'-B',str(native),'-G','Ninja','-DCMAKE_BUILD_TYPE=Debug'])
        run('native_build',['cmake','--build',str(native),'-j2'])
        capture=output/'candidate_fault_status.bin'
        tested=run('native_tests',[str(native/'test_continuous_4wgc'),'--dump-status',str(capture)])
        result['native']=json.loads(tested.stdout.strip().splitlines()[-1])
        assert result['native']['passed'] and result['native']['hardware_access'] is False
        # Inspect actual C-produced bytes, while keeping production host code
        # unchanged. The old hardware admission must NOT accept this ABI.
        import struct
        sys.path.insert(0,str(WORKSPACE/'src/mecanum_vicon_control'))
        from mecanum_vicon_control.motion_link_status import decode_motion_status
        from mecanum_vicon_control.transport_protocol import ProtocolError
        payload=capture.read_bytes()
        assert len(payload)==156 and payload[:4]==b'4WGC'
        detail=struct.unpack('<IIiiiiHHhH',payload[100:132])
        assert detail[1]==10 and detail[2]==101 and detail[6:8]==(2,3)
        try: decode_motion_status(payload)
        except ProtocolError as error:
            result['legacy_host_rejects_candidate']=str(error)
        else: raise AssertionError('legacy host unexpectedly accepted candidate diagnostics')
        run('native_ctest',['ctest','--test-dir',str(native),'--output-on-failure'])
        run('legacy_motion_12_profiles',['bash',str(BASE/'tests/run_motion_tests.sh')])
        # Capture the historical full runner faithfully; never alter a test
        # or raise a limit to conceal its pre-existing 400>300 mismatch.
        legacy=run('legacy_rectangle_full_runner',['bash',str(BASE/'tests/run_rectangle_tests.sh')],False)
        if legacy.returncode:
            assert 'test_ramp_pi_and_signs' in legacy.stdout and 'four_speed_set_target' in legacy.stdout, legacy.stdout
            result['baseline_warnings'].append(dict(test='legacy standalone speed unit',
                reason='unchanged test requests 400 count/s while default target cap is 300 count/s',
                log='legacy_rectangle_full_runner.log',not_modified=True))
        flags=['cc','-std=c11','-Wall','-Wextra','-Wpedantic','-Werror','-g',
               '-fsanitize=address,undefined','-fno-omit-frame-pointer',
               '-DDRIVE_STARTUP_OBSERVE=1','-DAUTONOMOUS_RECTANGLE=1',
               '-I'+str(BASE/'Inc'),'-I'+str(WORKSPACE/'protocol')]
        algorithms=[str(BASE/'Src'/p) for p in ('rectangle_demo.c','four_drive_group.c',
                     'four_wheel_speed.c','mecanum_drive_plan.c','mecanum_kinematics.c')]
        for name,test,extra in (
            ('legacy_rectangle_body','test_rectangle_demo.c',[]),
            ('legacy_rectangle_trace','test_rectangle_startup_trace.c',[str(BASE/'Src/rectangle_startup_trace.c')])):
            exe=output/name
            run(name+'_build',flags+[str(BASE/'tests'/test)]+algorithms+extra+['-lm','-o',str(exe)])
            run(name,[str(exe)])
        if args.python_tests:
            py=run('python_regression',[sys.executable,'-m','pytest','-q',
                'src/mecanum_wand_control/test','src/mecanum_wand_hardware/test','src/mecanum_vicon_control/test'])
            result['python_regression_summary']=py.stdout.strip().splitlines()[-1]
        if args.arm:
            arm=output/'arm'
            run('arm_configure',['cmake','-S',str(HERE),'-B',str(arm),'-G','Ninja',
                    '-DCONTINUOUS_ARM=ON','-DCMAKE_BUILD_TYPE=Debug'])
            run('arm_build',['cmake','--build',str(arm),'--target','mecanum_f407_continuous_4wgc_OFFLINE','-j2'])
            image=arm/'mecanum_f407_continuous_4wgc_OFFLINE.bin'
            result['offline_arm_image']=dict(path=str(image),bytes=image.stat().st_size,sha256=sha(image),flashed=False)
        after={file:sha(WORKSPACE/file) for file in before}
        assert after==before,'legacy file or accepted image changed'
        (output/'preserved_after.json').write_text(json.dumps(after,indent=2)+'\n')
        result.update(candidate_validation_passed=True,all_suites_passed=not result['baseline_warnings'],
            preserved_file_count=len(before),preservation_hashes_match=True,
            runtime_compatibility='4WGC v1/156-byte status is rejected by the unchanged rectangle decoder; use the reviewed dedicated Sign profile',
            source_tree=str(HERE))
    except Exception as error:
        result.update(error=repr(error),traceback=traceback.format_exc())
    finally:
        if before:
            result['preservation_hashes_match']=all((WORKSPACE/file).exists() and sha(WORKSPACE/file)==digest
                for file,digest in before.items())
        (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['candidate_validation_passed'] and result['preservation_hashes_match'] else 1


if __name__=='__main__':raise SystemExit(main())
