#!/usr/bin/env python3
"""Static high-speed preparation only. No hardware, ROS executor or vehicle model.

Builds a separate native checker and optional ARM artifact, verifies immutable
accepted sources/images, and tests the isolated host draft/strict decoder.
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
WORKSPACE=BASE.parents[1]
ACCEPTED=BASE/'candidates/continuous_4wgc'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--arm',action='store_true')
    parser.add_argument('--runtime',action='store_true',help='Generate/build private 0.20m/s packages; NEVER launch nodes')
    a=parser.parse_args();output=a.output.resolve();output.mkdir(parents=True,exist_ok=False)
    report=dict(passed=False,verification='STATIC C + PRESCRIBED STATE/INPUT UNIT CHECKS ONLY; NO VEHICLE MODEL',
        actions=dict(flashed=False,hardware_started=False,network_changed=False,accepted_sources_edited=False),
        hardware_entry_implemented=False,commands=[])
    def run(name,command):
        started=time.monotonic()
        proc=subprocess.run(list(map(str,command)),cwd=WORKSPACE,stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,text=True)
        (output/(name+'.log')).write_text(proc.stdout)
        report['commands'].append(dict(name=name,returncode=proc.returncode,seconds=time.monotonic()-started))
        print(('PASS ' if proc.returncode==0 else 'FAIL ')+name,flush=True)
        if proc.returncode:raise RuntimeError(name+'; see '+str(output/(name+'.log')))
        return proc.stdout
    before={}
    try:
        protected=[]
        for directory in (ACCEPTED,WORKSPACE/'src/mecanum_wand_control',WORKSPACE/'src/mecanum_wand_hardware'):
            protected.extend(p for p in directory.rglob('*') if p.is_file()
                and '__pycache__' not in p.parts and '.egg-info' not in str(p)
                and p.suffix in ('.py','.c','.h','.json','.yaml','.txt'))
        protected.append(WORKSPACE/'docs/wand_4wgc_migration_2026-10-23/arm/mecanum_f407_continuous_4wgc_OFFLINE.bin')
        for manifest in ('docs/wand_offline_2026-10-22/existing_control.sha256',
                         'docs/wand_hardware_2026-10-22_firmware_source.sha256'):
            run('before_'+Path(manifest).stem,['sha256sum','-c','--quiet',manifest])
            for line in (WORKSPACE/manifest).read_text().splitlines():
                if line.strip():protected.append(WORKSPACE/line.split(maxsplit=1)[1].lstrip('*'))
        before={str(p.relative_to(WORKSPACE)):sha(p) for p in protected}
        (output/'preserved_before.json').write_text(json.dumps(before,indent=2)+'\n')
        native=output/'native'
        run('native_configure',['cmake','-S',HERE,'-B',native,'-G','Ninja','-DCMAKE_BUILD_TYPE=Debug'])
        run('native_build',['cmake','--build',native,'-j2'])
        result=run('native_static_checks',[native/'test_high_speed_4wgd',output/'idle_status.bin',output/'fast_status.bin'])
        report['native']=json.loads(result.strip().splitlines()[-1])
        old_environment=os.environ.get('HIGH_SPEED_STATIC_FIXTURES')
        os.environ['HIGH_SPEED_STATIC_FIXTURES']=str(output)
        try:
            host_result=run('host_unit_checks',[sys.executable,'-m','pytest','-q',HERE/'host/test_high_speed_4wgd.py'])
        finally:
            if old_environment is None:os.environ.pop('HIGH_SPEED_STATIC_FIXTURES',None)
            else:os.environ['HIGH_SPEED_STATIC_FIXTURES']=old_environment
        report['host_unit_summary']=host_result.strip().splitlines()[-1]
        sys.path.insert(0,str(HERE/'host'))
        from high_speed_4wgd import HighSpeedPlan,provisional_stopping_reserve
        report['speed_calculation']=HighSpeedPlan().speed_table()
        report['planning_only_stop_reserve_at_02_mps_assuming_02_mps2']=provisional_stopping_reserve(.2,.2)
        report['stopping_deceleration_physically_measured']=False
        if a.arm:
            arm=output/'arm'
            run('arm_configure',['cmake','-S',HERE,'-B',arm,'-G','Ninja','-DHIGH_SPEED_ARM=ON','-DCMAKE_BUILD_TYPE=Debug'])
            run('arm_build',['cmake','--build',arm,'--target','mecanum_f407_high_speed_4wgd_OFFLINE','-j2'])
            binary=arm/'mecanum_f407_high_speed_4wgd_OFFLINE.bin'
            report['arm_artifact']=dict(path=str(binary),bytes=binary.stat().st_size,sha256=sha(binary),flashed=False)
        if a.runtime:
            runtime=output/'private_runtime';source=runtime/'src'
            run('private_runtime_generate',[sys.executable,HERE/'host/build_isolated_runtime.py',
                '--output',source,'--accepted-snapshot',output/'preserved_before.json'])
            old_root=os.environ.get('HIGH_SPEED_GENERATED_ROOT')
            old_fixture=os.environ.get('HIGH_SPEED_STATIC_FIXTURES')
            os.environ['HIGH_SPEED_GENERATED_ROOT']=str(source)
            os.environ['HIGH_SPEED_STATIC_FIXTURES']=str(output)
            try:
                runtime_checks=run('private_runtime_contract_tests',[sys.executable,'-m','pytest','-q',
                    HERE/'host/test_generated_runtime.py'])
            finally:
                for key,value in (('HIGH_SPEED_GENERATED_ROOT',old_root),('HIGH_SPEED_STATIC_FIXTURES',old_fixture)):
                    if value is None:os.environ.pop(key,None)
                    else:os.environ[key]=value
            packages=('mecanum_wand_highspeed_control','mecanum_wand_highspeed_hardware')
            run('private_runtime_build',['colcon','--log-base',runtime/'log','build','--base-paths',
                *(source/name for name in packages),'--packages-select',*packages,
                '--build-base',runtime/'build','--install-base',runtime/'install','--symlink-install',
                '--event-handlers','console_direct+'])
            report['hardware_entry_implemented']=True
            report['runtime']=dict(source=str(source),private_install=str(runtime/'install'),
                domain=95,namespace='/wand_highspeed_acceptance',first_stage_mps=.20,
                hardware_admitted=False,contract_summary=runtime_checks.strip().splitlines()[-1])
        report['passed']=True
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc())
    finally:
        if before:
            after={path:sha(WORKSPACE/path) for path in before}
            report['preservation_hashes_match']=before==after
            report['protected_file_count']=len(before)
            (output/'preserved_after.json').write_text(json.dumps(after,indent=2)+'\n')
            report['passed'] &= report['preservation_hashes_match']
        (output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
