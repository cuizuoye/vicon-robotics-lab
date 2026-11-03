#!/usr/bin/env python3
"""Offline-only prepare/build/tests. Never calls a programmer or real nodes.

UI smoke is private Xephyr + native C fake IO; all Python UDP sockets are
forbidden there. Source environment should contain ROS Jazzy for imports/build.
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

HERE=Path(__file__).resolve().parent;BASE=HERE.parents[1];ROOT=BASE.parents[1]

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reuse-firmware-manifest',type=Path,
        help='Host-only repair: pin existing ARM inputs/image and DO NOT build a new firmware')
    a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    result=dict(passed=False,flashed=False,hardware_started=False,network_changed=False,
        real_vicon_started=False,real_ros_control_nodes_started=False,real_udp_sent=False,commands=[])
    expected=json.loads((HERE/'upstream.json').read_text());before={p:sha(ROOT/p) for p in expected}
    def run(name,command,env=None):
        start=time.monotonic();r=subprocess.run(list(map(str,command)),cwd=ROOT,env=env,
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        (out/(name+'.log')).write_text(r.stdout)
        result['commands'].append(dict(name=name,returncode=r.returncode,seconds=time.monotonic()-start))
        print(('PASS ' if r.returncode==0 else 'FAIL ')+name,flush=True)
        if r.returncode:raise RuntimeError(str(out/(name+'.log')))
        return r.stdout
    try:
        if before!=expected:raise ValueError('protected source or prior locked candidate drift')
        (out/'preserved_before.json').write_text(json.dumps(before,indent=2)+'\n')
        native=out/'native'
        run('native_configure',['cmake','-S',HERE,'-B',native,'-G','Ninja','-DCMAKE_BUILD_TYPE=Debug'])
        run('native_build',['cmake','--build',native,'-j2'])
        run('native_ctest',['ctest','--test-dir',native,'--output-on-failure'])
        run('native_v3_fixtures',[native/'test_stage0',native/'idle.bin',native/'bench_active.bin',native/'bench_completed.bin'])
        src=out/'runtime/src'
        generated=run('private_runtime_generate',[sys.executable,HERE/'host/build_runtime.py','--output',src])
        runtime=json.loads(generated)
        env=os.environ.copy();env['STAGE0_GENERATED']=str(src);env['STAGE0_NATIVE']=str(native)
        py=run('new_unit_protocol_and_authority',[sys.executable,'-m','pytest','-q',
            HERE/'host/test_stage0.py',HERE/'host/test_sign_adapter.py',HERE/'host/test_authorization.py',
            HERE/'host/test_wire_regression.py',
            '--junitxml='+str(out/'new_tests.xml')],env)
        result['new_tests_summary']=py.strip().splitlines()[-1]
        packages=(runtime['control_package'],runtime['hardware_package'])
        run('private_ros_packages_build',['colcon','--log-base',out/'runtime/log','build','--base-paths',
            *(src/n for n in packages),'--packages-select',*packages,'--build-base',out/'runtime/build',
            '--install-base',out/'runtime/install','--symlink-install','--event-handlers','console_direct+'])
        env.setdefault('DISPLAY',':1');env.setdefault('XAUTHORITY','/run/user/1000/gdm/Xauthority')
        run('private_ui_fake_io',[sys.executable,HERE/'host/ui_smoke.py','--output',out/'ui'],env)
        result['ui_checks']=json.loads((out/'ui/ui_summary.json').read_text())
        if a.reuse_firmware_manifest:
            previous=json.loads(a.reuse_firmware_manifest.read_text())
            for path,expected_sha in previous['source_hashes'].items():
                relative=Path(path).relative_to(HERE)
                if relative.parts[0] in ('Src','Inc') or relative.name=='CMakeLists.txt':
                    if sha(path)!=expected_sha:raise ValueError('ARM input drift: '+path)
            image=Path(previous['firmware']['path'])
            if sha(image)!=previous['firmware']['sha256']:raise ValueError('existing ARM image drift')
            result['firmware_reused_from']=str(a.reuse_firmware_manifest.resolve())
            result['firmware_image_unchanged']=True
        else:
            arm=out/'arm'
            run('arm_configure',['cmake','-S',HERE,'-B',arm,'-G','Ninja','-DSERVO_ARM=ON','-DCMAKE_BUILD_TYPE=Debug'])
            run('arm_build',['cmake','--build',arm,'--target','mecanum_f407_4ws7_stage0_NOT_FLASHED','-j2'])
            image=arm/'mecanum_f407_4ws7_stage0_NOT_FLASHED.bin'
        # Original rectangle regression, not new hardware/field testing.
        flags=['cc','-std=c11','-Wall','-Wextra','-Wpedantic','-Werror','-g','-fsanitize=address,undefined',
            '-fno-omit-frame-pointer','-fno-pie','-no-pie','-DDRIVE_STARTUP_OBSERVE=1','-DAUTONOMOUS_RECTANGLE=1',
            '-I'+str(BASE/'Inc'),'-I'+str(ROOT/'protocol')]
        algorithms=[BASE/'Src'/p for p in ('rectangle_demo.c','four_drive_group.c','four_wheel_speed.c',
                                          'mecanum_drive_plan.c','mecanum_kinematics.c')]
        for name,test,extra in (('rectangle_body','test_rectangle_demo.c',[]),
            ('rectangle_trace','test_rectangle_startup_trace.c',[BASE/'Src/rectangle_startup_trace.c'])):
            run(name+'_build',flags+[BASE/'tests'/test,*algorithms,*extra,'-lm','-o',out/name]);run(name,[out/name])
        regression=run('legacy_python_regression',[sys.executable,'-m','pytest','-q',ROOT/'src/mecanum_wand_control/test',
            ROOT/'src/mecanum_wand_hardware/test',ROOT/'src/mecanum_vicon_control/test'])
        result['legacy_tests_summary']=regression.strip().splitlines()[-1]
        result['firmware']=dict(path=str(image),sha256=sha(image),bytes=image.stat().st_size,identity='4WS7',
            authority_version=3,stage=0,pwm_limit=350,bench_limit_ms=5000,flashed=False)
        hashes=lambda folder:{str(p):sha(p) for p in folder.rglob('*') if p.is_file() and
            '__pycache__' not in p.parts and '.pytest_cache' not in p.parts and p.suffix in ('.py','.json','.xml','.txt','.sh','.c','.h','.md')}
        manifest=dict(preparation_passed=True,firmware=result['firmware'],runtime_src=str(src),
            runtime_hashes=hashes(src),source_hashes=hashes(HERE),**runtime)
        # No flash record, no approved runtime profile, and no motion authority are generated.
        (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        (out/'FLASH_RECORD_TEMPLATE.NOT_APPROVED.json').write_text(json.dumps(dict(flashed=False,
            independent_readback_verified=False,full_readback_path=None,full_readback_sha256=None,
            expected_new_image_sha256=sha(image)),indent=2)+'\n')
        run('default_check_no_hardware',[sys.executable,HERE/'host/launch_cli.py','--manifest',out/'build_manifest.json','--purpose','suspended'])
        result['passed']=True
    except Exception as error:
        result.update(error=repr(error),traceback=traceback.format_exc())
    finally:
        after={p:sha(ROOT/p) for p in before};result['protected_files']=len(before)
        result['protected_hashes_match']=before==after;result['passed'] &= before==after
        (out/'preserved_after.json').write_text(json.dumps(after,indent=2)+'\n')
        (out/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='ui_checks'},indent=2,ensure_ascii=False))
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
