#!/usr/bin/env python3
"""Reproducible OFFLINE preparation. No programmer, real ROS nodes or UDP IO."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
from prepare_sources import generate,pinned_inputs,ROOT,BASE,CONTROL,HARDWARE
from port_regressions import generate as port_tests
from stages import STAGES

HERE=Path(__file__).resolve().parent
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in Path(folder).rglob('*') if p.is_file() and
        not any(v in p.parts for v in ('__pycache__','.pytest_cache')) and
        not any(v.endswith('.egg-info') for v in p.parts) and p.suffix not in ('.pyc',)}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    out=p.parse_args().output.resolve();before=pinned_inputs()
    result=dict(passed=False,flashed=False,hardware_started=False,network_changed=False,
        real_ros_nodes_started=False,real_vicon_started=False,real_udp_sent=False,commands=[])
    def run(name,args,env=None,timeout=60):
        start=time.monotonic();r=subprocess.run(list(map(str,args)),cwd=ROOT,env=env,
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout)
        (out/(name+'.log')).write_text(r.stdout)
        result['commands'].append(dict(name=name,returncode=r.returncode,seconds=time.monotonic()-start))
        print(('PASS ' if r.returncode==0 else 'FAIL ')+name,flush=True)
        if r.returncode:raise RuntimeError(str(out/(name+'.log')))
        return r.stdout
    try:
        result['generation']=generate(out)
        native=out/'native';source=out/'firmware';runtime=out/'runtime/src'
        run('native_configure',['cmake','-S',source,'-B',native,'-G','Ninja','-DCMAKE_BUILD_TYPE=Debug'])
        run('native_build',['cmake','--build',native,'-j2'])
        run('native_ctest',['ctest','--test-dir',native,'--output-on-failure'])
        run('native_stage_fixtures',[native/'test_stages',native])
        run('native_controller',[native/'test_controller'])
        tests=out/'tests';tests.mkdir();port_tests(tests)
        env=os.environ.copy();env.update(STAGED_GENERATED=str(runtime),STAGED_NATIVE=str(native),
            PYTHONPATH=str(HERE)+os.pathsep+env.get('PYTHONPATH',''))
        result['new_tests_summary']=run('new_protocol_gate_fault_tests',[
            sys.executable,'-m','pytest','-q',HERE/'test_staged.py',tests,
            '--junitxml='+str(out/'new_tests.xml')],env).strip().splitlines()[-1]
        run('private_ros_packages_build',['colcon','--log-base',out/'runtime/log','build','--base-paths',
            runtime/CONTROL,runtime/HARDWARE,'--packages-select',CONTROL,HARDWARE,
            '--build-base',out/'runtime/build','--install-base',out/'runtime/install',
            '--symlink-install','--event-handlers','console_direct+'],env)
        env.setdefault('DISPLAY',':1');env.setdefault('XAUTHORITY','/run/user/1000/gdm/Xauthority')
        run('private_ui_fake_io',[sys.executable,tests/'ui_smoke.py','--output',out/'ui'],env)
        result['ui_checks']=json.loads((out/'ui/ui_summary.json').read_text())['checks']
        arm=out/'arm'
        run('arm_configure',['cmake','-S',source,'-B',arm,'-G','Ninja','-DSERVO_ARM=ON','-DCMAKE_BUILD_TYPE=Debug'])
        run('arm_build',['cmake','--build',arm,'--target','mecanum_f407_4ws8_staged_NOT_FLASHED','-j2'])
        image=arm/'mecanum_f407_4ws8_staged_NOT_FLASHED.bin'
        # Actual original rectangle sources, with sanitizers. No mutation.
        flags=['cc','-std=c11','-Wall','-Wextra','-Wpedantic','-Werror','-g','-fsanitize=address,undefined',
            '-fno-omit-frame-pointer','-fno-pie','-no-pie','-DDRIVE_STARTUP_OBSERVE=1','-DAUTONOMOUS_RECTANGLE=1',
            '-I'+str(BASE/'Inc'),'-I'+str(ROOT/'protocol')]
        algorithms=[BASE/'Src'/name for name in ('rectangle_demo.c','four_drive_group.c','four_wheel_speed.c',
            'mecanum_drive_plan.c','mecanum_kinematics.c')]
        for name,test,extra in (('rectangle_body','test_rectangle_demo.c',[]),
            ('rectangle_trace','test_rectangle_startup_trace.c',[BASE/'Src/rectangle_startup_trace.c'])):
            run(name+'_build',flags+[BASE/'tests'/test,*algorithms,*extra,'-lm','-o',out/name]);run(name,[out/name])
        result['legacy_tests_summary']=run('legacy_python_regression',[sys.executable,'-m','pytest','-q',
            ROOT/'src/mecanum_wand_control/test',ROOT/'src/mecanum_wand_hardware/test',ROOT/'src/mecanum_vicon_control/test'],env).strip().splitlines()[-1]
        fw=dict(path=str(image),sha256=sha(image),bytes=image.stat().st_size,identity='4WS8',authority_version=4,
            supported_stages=list(range(7)),boot_stage=0,bench_limit_ms=5000,flashed=False,
            hardware_qualified=False)
        tiers=[dict(stage=i,**asdict(s),target_rpm=s.target_rpm,feedback_rpm=s.feedback_rpm) for i,s in enumerate(STAGES)]
        manifest=dict(preparation_passed=True,firmware=fw,stages=tiers,
            control_package=CONTROL,hardware_package=HARDWARE,runtime_src=str(runtime),
            domain=119,namespace='/wand_4ws8_staged',runtime_hashes=hashes(runtime),
            source_hashes=hashes(HERE),generated_firmware_hashes=hashes(source),protected_input_hashes=before,
            draft_profiles=[str(runtime/HARDWARE/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
        manifest['source_hashes'].update({str(ROOT/'tools'/name):sha(ROOT/'tools'/name)
            for name in ('start_4ws8_staged_console.sh','start_4ws8_staged_suspended.sh')})
        (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        (out/'FLASH_RECORD_TEMPLATE.NOT_APPROVED.json').write_text(json.dumps(dict(identity='4WS8',flashed=False,
            independent_readback_verified=False,image_sha256=fw['sha256'],full_readback_path=None,
            full_readback_sha256=None,pre_flash_full_backup_path=None,pre_flash_full_backup_sha256=None),indent=2)+'\n')
        for stage in range(7):
            for purpose in ('sign','suspended'):
                review=dict(kind='BOUNDED_TRIAL_NOT_QUALIFICATION',identity='4WS8',image_sha256=fw['sha256'],
                    authorized_stage=stage,purpose=purpose,human_authorized=False,
                    previous_stage=stage-1 if stage else None,previous_stage_action_and_stop_passed=False,
                    previous_stage_evidence=None,previous_stage_evidence_sha256=None,
                    measured_trial_stop_distance_m=None,stop_test_speed_mps=None,
                    stop_envelope_includes_fallback_and_footprint=False,stop_envelope_fits_original_buffer=False,
                    electrical_thermal_trial_reviewed=False)
                (out/f'TRIAL_STAGE_{stage}_{purpose}.NOT_APPROVED.json').write_text(json.dumps(review,indent=2)+'\n')
                run(f'file_check_{purpose}_{stage}',[sys.executable,HERE/'launch_cli.py','--manifest',out/'build_manifest.json',
                    '--purpose',purpose,'--stage',stage,'--check'])
        result['firmware']=fw;result['passed']=True
    except Exception as e:
        result.update(error=repr(e),traceback=traceback.format_exc())
    finally:
        if out.exists():
            after={p:sha(p) for p in before};result['protected_files']=len(before)
            result['protected_hashes_match']=before==after;result['passed'] &= before==after
            (out/'protected_after.json').write_text(json.dumps(after,indent=2)+'\n')
            (out/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='commands'},indent=2,ensure_ascii=False))
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
