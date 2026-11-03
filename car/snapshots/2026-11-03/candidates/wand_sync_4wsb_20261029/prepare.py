"""Independent MCU build only. No programmer, ROS initialization or device IO."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

from stages import table

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
PARENT=ROOT/'docs/wand_sync_600s_2026-10-28/prepared'
SIM=ROOT/'docs/wand_sync_tuning_2026-10-29/real_sign_vx7co3fk/RESULT.json'
TARGET='mecanum_f407_4wsb_staged_NOT_FLASHED'


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value): path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def hashes(folder):
    return {str(p):sha(p) for p in sorted(folder.rglob('*')) if p.is_file()
            and not any(v in ('__pycache__','.pytest_cache') for v in p.parts)}
def edit(path, old, new, count=1):
    value=path.read_text()
    if value.count(old)!=count: raise ValueError('pinned source layout drift: '+str(path)+' '+old[:70])
    path.write_text(value.replace(old,new))


def generate(out):
    sys.path.insert(0,str(ROOT/'candidates/wand_sync_600s'))
    from cli import check
    parent=check(PARENT)
    accepted=json.loads(SIM.read_text())
    if not accepted['sample_valid'] or not accepted['stability']['passed']:
        raise ValueError('latest real Sign simulation did not pass')
    pins={str(PARENT/'build_manifest.json'):sha(PARENT/'build_manifest.json'),str(SIM):sha(SIM)}
    for group in ('runtime_hashes','source_hashes','firmware_source_hashes','protected_input_hashes'):
        pins.update(parent[group])
    pins[parent['firmware']['path']]=parent['firmware']['sha256']
    # Preserve all original hardware/rectangle inputs and the accepted new SIM.
    sys.path.insert(0,str(ROOT/'candidates/wand_latency_fast_follow_20261029'))
    from common import protected_hashes
    pins.update(protected_hashes())
    if out.exists(): raise ValueError('use a NEW preparation folder, never overwrite an accepted release')
    out.mkdir(parents=True)
    (out/'COLCON_IGNORE').touch()
    save(out/'protected_inputs.json',pins)
    fw=out/'firmware'
    shutil.copytree(PARENT/'firmware',fw)
    # Only a private mechanical copy is edited. Parent files stay hash-pinned.
    for p in fw.rglob('*'):
        if p.is_file():
            p.write_text(p.read_text().replace('4WSA','4WSB').replace('4wsa','4wsb')
                .replace('0x41535734','0x42535734').replace('0x7A00','0x7B00')
                .replace('0x7A80','0x7B80').replace('v6','v7').replace('STAGED_V6','STAGED_V7'))
    for name in ('servo_policy.c','servo_policy.h'):
        shutil.copyfile(HERE/name,fw/('Src' if name.endswith('.c') else 'Inc')/name)
    profile=fw/'Inc/four_speed_pwm_profile.h'
    edit(profile,'#define SERVO_WHEEL_SLEW_GUARD_CPS_S 26000.0f',
         '#define SERVO_BASE_WHEEL_SLEW_GUARD_CPS_S 26000.0f\n#define SERVO_WHEEL_SLEW_GUARD_CPS_S 33000.0f')
    edit(profile,'Bound derived from 2m/s^2 vector acceleration plus 12rad/s^2 yaw,',
         'Absolute bound for stage7 4m/s^2 plus .4rad/s^2 yaw; old stages retain26000,')
    # Per-wheel defensive bound is stage-specific. Do not broaden old tiers.
    wheel=fw/'Src/four_wheel_speed.c'
    shutil.copyfile(ROOT/'stm32/mecanum_f407/candidates/servo_4ws7/Src/four_wheel_speed.c',wheel)
    edit(wheel,'c->ramp_cps_per_s != SERVO_WHEEL_SLEW_GUARD_CPS_S ||',
         '(c->ramp_cps_per_s != SERVO_BASE_WHEEL_SLEW_GUARD_CPS_S &&\n'
         '         c->ramp_cps_per_s != SERVO_WHEEL_SLEW_GUARD_CPS_S) ||')
    edit(wheel,'c->resume_cps_per_s>SERVO_WHEEL_SLEW_GUARD_CPS_S ||',
         'c->resume_cps_per_s>c->ramp_cps_per_s ||')
    edit(wheel,'const float max_change=SERVO_WHEEL_SLEW_GUARD_CPS_S*',
         'const float max_change=c->config.ramp_cps_per_s*')
    group=fw/'Src/four_drive_group.c'
    edit(group,'g->loop[i].config.launch_slew_permille_per_s=p.output_slew_permille_s;',
         'g->loop[i].config.launch_slew_permille_per_s=p.output_slew_permille_s;\n'
         '        g->loop[i].config.ramp_cps_per_s=servo_stage_wheel_slew_guard(stage);')
    edit(group,'g->loop[i].config.launch_slew_permille_per_s!=stage.output_slew_permille_s ||',
         'g->loop[i].config.launch_slew_permille_per_s!=stage.output_slew_permille_s ||\n'
         '                g->loop[i].config.ramp_cps_per_s!=servo_stage_wheel_slew_guard(g->servo_stage) ||')
    header=fw/'Inc/four_link_bench.h'
    edit(header,'    uint32_t absolute_run_limit_ms;',
         '    uint32_t absolute_run_limit_ms;\n    float wheel_target_slew_guard_cps_s;')
    edit(header,'==204','==208')
    edit(header,'204-byte','208-byte')
    link=fw/'Src/four_link_motion.c'
    edit(link,'s->output_authority_version = 6U;','s->output_authority_version = 7U;')
    edit(link,'s->absolute_run_limit_ms=FOUR_DRIVE_RUN_LIMIT_MS;',
         's->absolute_run_limit_ms=FOUR_DRIVE_RUN_LIMIT_MS;\n'
         '    s->wheel_target_slew_guard_cps_s=servo_stage_wheel_slew_guard(b->group.servo_stage);')
    for name in ('test_controller.c','test_stages.c'):
        p=fw/'tests'/name
        p.write_text(p.read_text().replace('output_authority_version==6','output_authority_version==7')
                     .replace('sizeof(s)==204','sizeof(s)==208').replace('(stage+1)%7','(stage+1)%SERVO_STAGE_COUNT')
                     .replace('(s+1)%7','(s+1)%SERVO_STAGE_COUNT'))
    edit(fw/'tests/test_stages.c','s<7;s++','s<SERVO_STAGE_COUNT;s++')
    edit(fw/'tests/test_stages.c','!servo_stage_get(7,&p)','!servo_stage_get(SERVO_STAGE_COUNT,&p)')
    # Preserve the physical micro-turn start threshold: .10rad/s alone may not
    # start this drive. Test that truth, not fabricate a lower dead zone.
    angular=fw/'tests/test_rotation.c'
    edit(angular,'        command[2]=(int)lroundf(s.yaw_rate_rad_s*1000.f);',
         '        if(stage==7) continue; /* stage7 pure micro-turn remains below inherited start threshold */\n'
         '        command[2]=(int)lroundf(s.yaw_rate_rad_s*1000.f);')
    shutil.copyfile(HERE/'test_stage7.c',fw/'tests/test_stage7.c')
    cm=fw/'CMakeLists.txt'
    edit(cm,'${PARENT}/Src/four_wheel_speed.c','Src/four_wheel_speed.c')
    cm.write_text(cm.read_text()+'''
if(NOT SERVO_ARM)
  add_executable(test_stage7 tests/test_stage7.c ${ALGORITHMS})
  foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(values test_stages ${property})
    if(values)
      set_property(TARGET test_stage7 PROPERTY ${property} "${values}")
    endif()
  endforeach()
  target_link_libraries(test_stage7 PRIVATE m)
  add_test(NAME experimental_stage7_4mps2_and_faults COMMAND test_stage7 ${CMAKE_CURRENT_BINARY_DIR})
endif()
''')
    save(out/'stages.json',table())
    return pins,fw


def run(out):
    pins,fw=generate(out)
    def command(name,args,timeout=180):
        p=subprocess.run(list(map(str,args)),capture_output=True,text=True,cwd=ROOT,timeout=timeout)
        (out/(name+'.log')).write_text(p.stdout+p.stderr)
        print(name+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode: raise RuntimeError((p.stdout+p.stderr)[-10000:])
    command('native_configure',['cmake','-S',fw,'-B',out/'native','-G','Ninja','-DCMAKE_BUILD_TYPE=Debug'])
    command('native_build',['cmake','--build',out/'native','-j2'])
    command('native_tests',['ctest','--test-dir',out/'native','--output-on-failure'])
    command('native_all_snapshots',[out/'native/test_stages',out/'native'])
    command('native_yaw_snapshots',[out/'native/test_rotation',out/'native'])
    command('arm_configure',['cmake','-S',fw,'-B',out/'arm','-G','Ninja','-DSERVO_ARM=ON','-DCMAKE_BUILD_TYPE=Debug'])
    command('arm_build',['cmake','--build',out/'arm','--target',TARGET,'-j2'])
    command('hex_export',['arm-none-eabi-objcopy','-O','ihex',out/'arm'/(TARGET+'.elf'),out/'arm'/(TARGET+'.hex')])
    for p,digest in pins.items():
        if sha(p)!=digest: raise ValueError('protected input changed: '+p)
    save(out/'build_manifest.json',dict(identity='4WSB',authority_version=7,diagnostic_payload_bytes=208,
        firmware_only=True,offline_controller_passed=True,arm_built=True,flashed=False,
        physical_motion_started=False,network_settings_changed=False,
        automatic_hardware_start=False,host_migration_complete=False,
        firmware={extension:dict(path=str(out/'arm'/(TARGET+'.'+extension)),
            sha256=sha(out/'arm'/(TARGET+'.'+extension)),bytes=(out/'arm'/(TARGET+'.'+extension)).stat().st_size)
            for extension in ('bin','hex','elf','map')},
        flash_address='0x08000000',flash_capacity_bytes=1048576,
        independent_run_limit_ms=630000,suspended_limit_ms=5000,
        boot_enabled=False,boot_tier_selected=False,first_hardware_stage=0,
        experimental_stage=7,hardware_qualified=False,source_hashes=hashes(HERE),
        firmware_source_hashes=hashes(fw),protected_input_hashes=pins,
        reference_simulation=str(SIM),stage_table=table()))
    print('4WSB ARM image built; offline only, no programmer or hardware opened',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    run(p.parse_args().output.resolve())
