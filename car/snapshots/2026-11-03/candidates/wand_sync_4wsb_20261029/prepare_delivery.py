"""Package verified MCU artifacts, compatibility evidence and a no-IO checker."""
import argparse
import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

from prepare import ROOT,HERE,PARENT,TARGET,sha,save,hashes,edit
from stages import table
from oracle_compare import compare
from verify_bundle import verify


def make_contract(out):
    folder=out/'fourwsb_wire';folder.mkdir()
    (folder/'__init__.py').write_text('"""Offline protocol verification, NOT a hardware runtime."""\n')
    shutil.copyfile(HERE/'stages.py',folder/'stages.py')
    source=PARENT/'runtime/src/mecanum_wand_4wsa_hardware/mecanum_wand_4wsa_hardware/stage0_contract.py'
    value=source.read_text().replace('4WSA','4WSB').replace('v6','v7').replace('0x7A00','0x7B00')
    value=value.replace("DURATION=struct.Struct('<I')", "DURATION=struct.Struct('<I')\nSLEW=struct.Struct('<f')")
    value=value.replace('+ROTATION.size+DURATION.size','+ROTATION.size+DURATION.size+SLEW.size')
    value=value.replace('exactly 204 bytes','exactly 208 bytes').replace('stage 0..6','stage 0..7')
    value=value.replace('version!=6','version!=7').replace('output_authority_version=6','output_authority_version=7')
    value=value.replace("status.get('output_authority_version')==6","status.get('output_authority_version')==7")
    value=value.replace('    yr,ya,wr,revision=ROTATION.unpack_from(payload,184)',
        "    wheel_guard,=SLEW.unpack_from(payload,204)\n"
        "    if not math.isfinite(wheel_guard) or wheel_guard!=stage.wheel_slew_guard:\n"
        "        raise ValueError('stage-specific wheel slew guard mismatch')\n"
        '    yr,ya,wr,revision=ROTATION.unpack_from(payload,184)')
    value=value.replace('absolute_run_limit_ms=budget,',
        'wheel_target_slew_guard_cps_s=wheel_guard,absolute_run_limit_ms=budget,')
    value=value.replace('from mecanum_vicon_control.transport_protocol import ProtocolError',
        'class ProtocolError(ValueError):\n    """Offline decoder only; no live transport is constructed."""\n    pass')
    (folder/'contract.py').write_text(value)
    return folder


def run(out):
    m=json.loads((out/'build_manifest.json').read_text())
    assert m['identity']=='4WSB' and m['offline_controller_passed'] and not m['flashed']
    def command(name,args):
        r=subprocess.run(list(map(str,args)),capture_output=True,text=True,cwd=ROOT,timeout=120,
                         env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
        (out/(name+'.log')).write_text(r.stdout+r.stderr)
        print(name+': '+('PASS' if not r.returncode else 'FAIL'),flush=True)
        if r.returncode: raise RuntimeError((r.stdout+r.stderr)[-7000:])
    command('baseline_oracle_build',['cmake','--build',out/'baseline_native','--target','stage0_oracle','-j2'])
    equivalent=compare(out/'baseline_native/libstage0_oracle.so',out/'native/libstage0_oracle.so')
    save(out/'OLD_STAGE_EQUIVALENCE.json',equivalent)
    wire=make_contract(out)
    sys.path.insert(0,str(out))
    contract=importlib.import_module('fourwsb_wire.contract')
    values=[]
    for stage in range(8):
        decoded=contract.decode_status((out/f'native/active{stage}.bin').read_bytes())
        assert decoded['stage']==stage and decoded['output_authority_version']==7
        assert decoded['wheel_target_slew_guard_cps_s']==(33000 if stage==7 else 26000)
        values.append(decoded)
    save(out/'C_TO_HOST_DECODE.json',dict(passed=True,actual_c_snapshots=8,stages=values,
        hardware_runtime_constructed=False))
    esp=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready/esp_project/main'
    test=(ROOT/'candidates/wand_sync_4ws9/test_esp_payload.c').read_text().replace('200','208')
    (out/'test_esp208.c').write_text(test)
    command('esp208_build',['cc','-std=c11','-Wall','-Wextra','-Wpedantic','-Werror','-g',
        '-fsanitize=address,undefined','-fno-pie','-no-pie','-I'+str(esp),'-I'+str(ROOT/'protocol'),
        out/'test_esp208.c',esp/'mc_protocol.c','-o',out/'test_esp208'])
    command('preserved_esp208_crc_cobs',[out/'test_esp208'])
    command('host_contract_and_package_tests',[sys.executable,HERE/'test_delivery.py','--prepared',out])
    bundle=out/'flash_bundle';bundle.mkdir()
    artifacts={}
    for extension,record in m['firmware'].items():
        path=Path(record['path']);assert sha(path)==record['sha256']
        shutil.copyfile(path,bundle/path.name);artifacts[extension]=path.name
    shutil.copyfile(HERE/'verify_bundle.py',bundle/'verify_bundle.py')
    shutil.copyfile(HERE/'FLASH_GUIDE.md',bundle/'FLASH_GUIDE.md')
    shutil.copyfile(out/'stages.json',bundle/'stages.json')
    shutil.copytree(wire,bundle/'fourwsb_wire',ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(out/'firmware',bundle/'firmware_source')
    evidence=bundle/'evidence';evidence.mkdir()
    for name in ('native_tests.log','native_all_snapshots.log','native_yaw_snapshots.log',
        'preserved_esp208_crc_cobs.log','host_contract_and_package_tests.log',
        'OLD_STAGE_EQUIVALENCE.json','C_TO_HOST_DECODE.json'):
        shutil.copyfile(out/name,evidence/name)
    record=ROOT/'stm32/mecanum_f407/backups/pre_4ws8_staged_20261023_1q2wgy/POST_FLASH_RECORD.json'
    old=json.loads(record.read_text());rollback=Path(old['full_readback_path'])
    assert rollback.stat().st_size==1048576 and sha(rollback)==old['full_readback_sha256']
    (bundle/'historical_rollback').mkdir()
    shutil.copyfile(rollback,bundle/'historical_rollback/HISTORICAL_4WS8_20261023_1MiB.bin')
    shutil.copyfile(record,bundle/'historical_rollback/POST_FLASH_RECORD.json')
    bm=dict(identity='4WSB',authority_version=7,diagnostic_payload_bytes=208,
        **artifacts,image_bytes=m['firmware']['bin']['bytes'],flash_address='0x08000000',
        flash_capacity_bytes=1048576,application_erase_sectors=[0,1,2,3],
        default_boot_enabled=False,default_boot_selected=False,first_test_stage=0,
        experimental_stage=7,stage_table=table(),host_session_s=600,
        independent_firmware_limit_ms=630000,suspended_test_limit_ms=5000,
        flashed=False,physical_qualified=False,host_migration_complete=False,
        legacy_demo_compatible=False,latest_ros_scheduler_is_pc_only=True,
        full_screen_os_freeze_not_fixed_by_firmware=True,
        fresh_full_backup_required_before_flash=True,historical_rollback_not_current_backup=True,
        native_sanitized_groups=5,old_stages_equal_ticks=equivalent['compared_control_ticks'],
        real_sign_simulation_reference=str(ROOT/'docs/wand_sync_tuning_2026-10-29/real_sign_vx7co3fk/RESULT.json'))
    save(bundle/'BUNDLE.json',bm)
    sums=[sha(p)+'  '+p.relative_to(bundle).as_posix() for p in sorted(bundle.rglob('*')) if p.is_file()]
    (bundle/'SHA256SUMS').write_text('\n'.join(sums)+'\n')
    result=verify(bundle)
    save(out/'BUNDLE_VERIFICATION.json',result)
    for p,digest in m['protected_input_hashes'].items():
        assert sha(p)==digest,p
    m.update(source_hashes=hashes(HERE),wire_contract_hashes=hashes(wire),
        evidence_hashes={str(p):sha(p) for p in sorted(out.glob('*.log'))},
        delivery_complete=True,protected_unchanged=True)
    save(out/'build_manifest.json',m)
    archive=out.parent/'4WSB_stage0-7_NOT_FLASHED.zip'
    if archive.exists(): raise ValueError('do not overwrite existing release archive')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(bundle.rglob('*')):
            if p.is_file(): z.write(p,Path('4WSB_flash_bundle')/p.relative_to(bundle))
    save(out/'DELIVERY_RESULT.json',dict(passed=True,firmware_only=True,archive=str(archive),
        archive_sha256=sha(archive),bundle=str(bundle),bin=m['firmware']['bin'],
        original_files_unchanged=len(m['protected_input_hashes']),
        flashed=False,hardware_output=False,host_migration_complete=False,
        physical_qualified=False,full_screen_issue_fixed=False))
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepared',type=Path,required=True)
    run(p.parse_args().prepared.resolve())
