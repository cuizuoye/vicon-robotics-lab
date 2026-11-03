"""Independent offline 4WSC firmware and handoff. No programmer/device IO."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile
from stages import table

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');HERE=Path(__file__).resolve().parent
PARENT=ROOT/'docs/wand_firmware_4wsb_2026-10-29/ready'
OUT=ROOT/'docs/wand_firmware_4wsc_2026-10-30/prepared_v4'
BASELINE=ROOT/'docs/wand_browser_direct_2026-10-30/prepared_v1'
TARGET='mecanum_f407_4wsc_staged_NOT_FLASHED'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def edit(p,old,new,count=1):
    s=p.read_text()
    if s.count(old)!=count:raise ValueError('pinned source layout: '+str(p)+' '+old[:80])
    p.write_text(s.replace(old,new))
def hashes(path):return {str(p):sha(p) for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts}

def build():
    if OUT.exists():raise ValueError('prepare a new folder; never overwrite evidence')
    OUT.mkdir(parents=True)
    def command(name,args):
        r=subprocess.run(list(map(str,args)),capture_output=True,text=True,cwd=ROOT,timeout=240,
            env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
        (OUT/(name+'.log')).write_text(r.stdout+r.stderr)
        print(name+': '+('PASS' if not r.returncode else 'FAIL'),flush=True)
        if r.returncode:raise ValueError((r.stdout+r.stderr)[-9000:])
    command('old_firmware_check',['bash',ROOT/'tools/check_4wsb_firmware.sh','--check'])
    command('baseline_browser_check',['bash',ROOT/'tools/start_sign_browser_direct.sh','--check'])
    parent=json.loads((PARENT/'build_manifest.json').read_text())
    pins=dict(parent['protected_input_hashes']);pins.update(parent['firmware_source_hashes'])
    pins[str(PARENT/'build_manifest.json')]=sha(PARENT/'build_manifest.json')
    baseline=json.loads((BASELINE/'MANIFEST.json').read_text())
    pins.update(baseline['source_hashes']);pins.update(baseline['artifact_hashes'])
    pins[str(BASELINE/'MANIFEST.json')]=sha(BASELINE/'MANIFEST.json')
    pins[str(BASELINE/'RELEASE.json')]=sha(BASELINE/'RELEASE.json')
    save(OUT/'protected_inputs.json',pins)
    fw=OUT/'firmware';shutil.copytree(PARENT/'firmware',fw)
    for p in fw.rglob('*'):
        if p.is_file():p.write_text(p.read_text().replace('4WSB','4WSC').replace('4wsb','4wsc')
            .replace('0x42535734','0x43535734').replace('0x7B00','0x7C00').replace('0x7B80','0x7C80')
            .replace('STAGED_V7','STAGED_V8').replace('v7','v8'))
    # Old shared planner has an independent 1m/s input ceiling. Replace only
    # in this copied image, keeping finite checks and per-stage vector caps.
    shutil.copyfile(ROOT/'stm32/mecanum_f407/Src/mecanum_drive_plan.c',fw/'Src/mecanum_drive_plan.c')
    edit(fw/'Src/mecanum_drive_plan.c','fabsf(requested->vx_m_s) > 1.0f','fabsf(requested->vx_m_s) > FOUR_DRIVE_MAX_LINEAR_M_S')
    edit(fw/'Src/mecanum_drive_plan.c','fabsf(requested->vy_m_s) > 1.0f','fabsf(requested->vy_m_s) > FOUR_DRIVE_MAX_LINEAR_M_S')
    p=fw/'Src/servo_policy.c'
    edit(p,'{.70f,4.f,800,4000,.10f,.40f}};',
        '{.70f,4.f,800,4000,.10f,.40f},\n        {.70f,4.f,800,4000,2.f,12.f},{1.20f,4.f,800,4000,2.f,12.f}};')
    edit(p,'i==2?250.f:400.f','i==2?250.f:i==9?381.97186f:400.f')
    edit(p,'i==2?280.f:440.f','i==2?280.f:i==9?420.f:440.f')
    edit(p,'return i==7?33000.f:26000.f;','return i>=8?43000.f:i==7?33000.f:26000.f;')
    edit(fw/'Inc/servo_policy.h','SERVO_STAGE_COUNT=8','SERVO_STAGE_COUNT=10')
    p=fw/'Inc/four_speed_pwm_profile.h'
    edit(p,'#define FOUR_DRIVE_MAX_LINEAR_M_S 0.700f','#define FOUR_DRIVE_MAX_LINEAR_M_S 1.200f')
    edit(p,'#define FOUR_DRIVE_MAX_BODY_MM_S 700','#define FOUR_DRIVE_MAX_BODY_MM_S 1200')
    edit(p,'#define SERVO_WHEEL_SLEW_GUARD_CPS_S 33000.0f',
        '#define SERVO_PREVIOUS_WHEEL_SLEW_GUARD_CPS_S 33000.0f\n#define SERVO_WHEEL_SLEW_GUARD_CPS_S 43000.0f')
    edit(p,'Absolute bound for stage7 4m/s^2 plus .4rad/s^2 yaw; old stages retain26000,',
        'Absolute bound for new4m/s^2 plus12rad/s^2 yaw; old stages retain26000/33000,')
    edit(fw/'Src/four_wheel_speed.c',
        '(c->ramp_cps_per_s != SERVO_BASE_WHEEL_SLEW_GUARD_CPS_S &&',
        '(c->ramp_cps_per_s != SERVO_PREVIOUS_WHEEL_SLEW_GUARD_CPS_S &&\n         c->ramp_cps_per_s != SERVO_BASE_WHEEL_SLEW_GUARD_CPS_S &&')
    # At the new rim ceiling, using wheel0's calibrated CPS ceiling for all
    # wheels wrongly rescales the other CPRs and rejects a coherent plan.
    # New tiers use the original shared planner ceiling; stage_plan_valid's
    # independent per-wheel RPM test and every loop's own CPS bound remain.
    # Restrict to tiers8/9 so old output equivalence remains measurable.
    p=fw/'Src/four_drive_group.c'
    helper='''static bool coordinated_plan(const four_drive_group_t *g,
    const mecanum_body_velocity_t *body,float wheel_limit,mecanum_drive_plan_t *out)
{
    four_speed_config_t planner=g->loop[0].config;
    if(g->servo_stage>=8U) planner.target_limit_cps=FOUR_DRIVE_TARGET_LIMIT_CPS;
    if(!mecanum_drive_plan(&g->geometry,&planner,body,wheel_limit,out)) return false;
    if(g->servo_stage>=8U) {
        float ratio=1.f;
        for(unsigned i=0;i<4;i++) {
            const float magnitude=fabsf(out->forward_cps[i]);
            const float limit=g->loop[i].config.target_limit_cps;
            if(magnitude>limit) ratio=fminf(ratio,limit/magnitude);
        }
        if(ratio<1.f) {
            ratio=nextafterf(ratio,0.f);
            out->common_scale*=ratio;
            out->applied_body.vx_m_s*=ratio;
            out->applied_body.vy_m_s*=ratio;
            out->applied_body.wz_rad_s*=ratio;
            for(unsigned i=0;i<4;i++) {
                out->forward_cps[i]*=ratio;out->wheel_m_s[i]*=ratio;
                out->expected_raw_encoder_cps[i]*=ratio;
                if(fabsf(out->forward_cps[i])>g->loop[i].config.target_limit_cps)return false;
            }
        }
    }
    return true;
}

'''
    edit(p,'static bool stage_plan_valid(',helper+'static bool stage_plan_valid(')
    edit(p,'mecanum_drive_plan(&g->geometry,&g->loop[0].config,','coordinated_plan(g,',count=2)
    edit(p,'mecanum_drive_plan(&g->geometry, &g->loop[0].config, request,','coordinated_plan(g, request,')
    edit(fw/'Src/four_link_motion.c','s->output_authority_version = 7U;','s->output_authority_version = 8U;')
    for name in ('test_stages.c','test_controller.c'):
        p=fw/'tests'/name
        p.write_text(p.read_text().replace('output_authority_version==7','output_authority_version==8'))
    shutil.copyfile(HERE/'test_new_stages.c',fw/'tests/test_new_stages.c')
    cm=fw/'CMakeLists.txt'
    edit(cm,'${BASE}/Src/mecanum_drive_plan.c','Src/mecanum_drive_plan.c')
    cm.write_text(cm.read_text()+'''
if(NOT SERVO_ARM)
  add_executable(test_new_stages tests/test_new_stages.c ${ALGORITHMS})
  foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(values test_stages ${property})
    if(values)
      set_property(TARGET test_new_stages PROPERTY ${property} "${values}")
    endif()
  endforeach()
  target_link_libraries(test_new_stages PRIVATE m)
  add_test(NAME direct_sign_high_xy_yaw_and_faults COMMAND test_new_stages ${CMAKE_CURRENT_BINARY_DIR})
endif()
''')
    save(OUT/'stages.json',table())
    command('native_configure',['cmake','-S',fw,'-B',OUT/'native','-G','Ninja','-DCMAKE_BUILD_TYPE=Debug'])
    command('native_build',['cmake','--build',OUT/'native','-j2'])
    command('native_tests',['ctest','--test-dir',OUT/'native','--output-on-failure'])
    command('all_snapshots',[OUT/'native/test_stages',OUT/'native'])
    command('arm_configure',['cmake','-S',fw,'-B',OUT/'arm','-G','Ninja','-DSERVO_ARM=ON','-DCMAKE_BUILD_TYPE=Debug'])
    command('arm_build',['cmake','--build',OUT/'arm','--target',TARGET,'-j2'])
    command('hex_export',['arm-none-eabi-objcopy','-O','ihex',OUT/'arm'/(TARGET+'.elf'),OUT/'arm'/(TARGET+'.hex')])
    package(command,pins,fw)

def package(command,pins,fw):
    # Same old8 tiers, byte-for-byte outputs; identity/version differ only.
    old=ROOT/'candidates/wand_sync_4wsb_20261029'
    shutil.copyfile(HERE/'stages.py',OUT/'stages.py')
    sys.path.insert(0,str(old))
    src=(old/'oracle_compare.py').read_text().replace('old,new=Oracle(old_library,0x7A00),Oracle(new_library,0x7B00)',
        'old,new=Oracle(old_library,0x7B00),Oracle(new_library,0x7C00)')
    src=src.replace('STAGES[:7]','STAGES[:8]').replace('b[:204]==a','b==a').replace('list(range(7))','list(range(8))')
    helper=OUT/'old_stage_compare.py';helper.write_text(src)
    command('old_stage_equivalence',[sys.executable,helper,'--old',PARENT/'native/libstage0_oracle.so',
        '--new',OUT/'native/libstage0_oracle.so','--output',OUT/'OLD_STAGE_EQUIVALENCE.json'])
    bundle=OUT/'flash_bundle';bundle.mkdir()
    wire=bundle/'fourwsc_wire';wire.mkdir();(wire/'__init__.py').write_text('"""FILE ONLY: no socket, ROS or motion authority."""\n')
    shutil.copyfile(HERE/'stages.py',wire/'stages.py')
    source=(PARENT/'fourwsb_wire/contract.py').read_text().replace('4WSB','4WSC').replace('v7','v8').replace('0x7B00','0x7C00')
    source=source.replace('version!=7','version!=8').replace('output_authority_version=7','output_authority_version=8').replace('0..7','0..9')
    source=source.replace('wr!=stage.target_rpm','not math.isclose(wr,stage.target_rpm,rel_tol=1e-6)')
    source=source.replace("status.get('output_authority_version')==7","status.get('output_authority_version')==8")
    (wire/'contract.py').write_text(source)
    spec=importlib.util.spec_from_file_location('fourwsc_wire',wire/'__init__.py',submodule_search_locations=[str(wire)])
    mod=importlib.util.module_from_spec(spec);sys.modules['fourwsc_wire']=mod;spec.loader.exec_module(mod)
    from fourwsc_wire.contract import decode_status
    snapshots=[decode_status((OUT/f'native/active{i}.bin').read_bytes()) for i in range(10)]
    save(OUT/'C_TO_HOST.json',dict(passed=True,snapshots=snapshots,hardware_output=False))
    for ext in ('bin','hex','elf','map'):shutil.copyfile(OUT/'arm'/(TARGET+'.'+ext),bundle/(TARGET+'.'+ext))
    shutil.copyfile(HERE/'FLASH_GUIDE.md',bundle/'FLASH_GUIDE.md')
    shutil.copyfile(OUT/'stages.json',bundle/'stages.json')
    src=(old/'verify_bundle.py').read_text().replace("'4WSB'","'4WSC'").replace("m['authority_version']!=7","m['authority_version']!=8")
    (bundle/'verify_bundle.py').write_text(src)
    shutil.copytree(fw,bundle/'firmware_source')
    evidence=bundle/'evidence';evidence.mkdir()
    for name in ('native_tests.log','OLD_STAGE_EQUIVALENCE.json','C_TO_HOST.json','baseline_browser_check.log'):
        shutil.copyfile(OUT/name,evidence/name)
    m=dict(identity='4WSC',authority_version=8,diagnostic_payload_bytes=208,
        **{ext:TARGET+'.'+ext for ext in ('bin','hex','elf','map')},image_bytes=(bundle/(TARGET+'.bin')).stat().st_size,
        flash_address='0x08000000',flash_capacity_bytes=1048576,application_erase_sectors=[0,1,2,3],
        default_boot_enabled=False,default_boot_selected=False,first_test_stage=0,stage_table=table(),
        host_session_s=600,independent_firmware_limit_ms=630000,suspended_test_limit_ms=5000,
        flashed=False,physical_qualified=False,host_migration_complete=False,
        browser_baseline=str(BASELINE),browser_baseline_hash=sha(BASELINE/'MANIFEST.json'),
        algorithm_location='Vicon mapping and adaptive/direct servo stay on PC; STM32 wheel control and protective caps',
        esp32_firmware_changed=False,network_changed=False,fresh_full_backup_required_before_flash=True)
    save(bundle/'BUNDLE.json',m)
    (bundle/'SHA256SUMS').write_text('\n'.join(sha(p)+'  '+p.relative_to(bundle).as_posix() for p in sorted(bundle.rglob('*')) if p.is_file())+'\n')
    command('bundle_file_check',[sys.executable,bundle/'verify_bundle.py','--check','--folder',bundle])
    for p,digest in pins.items():
        if sha(p)!=digest:raise ValueError('protected input drift: '+p)
    archive=OUT.parent/'4WSC_direct_sync_NOT_FLASHED.zip'
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for p in bundle.rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts:z.write(p,Path('4WSC_flash_bundle')/p.relative_to(bundle))
    save(OUT/'DELIVERY.json',dict(passed=True,firmware_prepared=True,flashed=False,hardware_output=False,
        host_migration_complete=False,physical_qualified=False,archive=str(archive),archive_sha256=sha(archive),
        bin=str(bundle/(TARGET+'.bin')),bin_sha256=sha(bundle/(TARGET+'.bin')),
        frozen_browser_baseline=str(BASELINE),protected_files=len(pins),protected_unchanged=True,
        source_hashes=hashes(HERE),firmware_source_hashes=hashes(fw)))
    print('4WSC prepared; no hardware opened, firmware not flashed',flush=True)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--resume-package',action='store_true');a=p.parse_args()
    if a.resume_package:
        pins=json.loads((OUT/'protected_inputs.json').read_text())
        for path,digest in pins.items():
            if sha(path)!=digest:raise ValueError('protected inputs drifted before packaging')
        def command(name,args):
            r=subprocess.run(list(map(str,args)),capture_output=True,text=True,cwd=ROOT,timeout=240,
                env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
            (OUT/(name+'.log')).write_text(r.stdout+r.stderr)
            print(name+': '+('PASS' if not r.returncode else 'FAIL'),flush=True)
            if r.returncode:raise ValueError((r.stdout+r.stderr)[-9000:])
        command('native_recheck',['ctest','--test-dir',OUT/'native','--output-on-failure'])
        package(command,pins,OUT/'firmware')
    else:build()
