#!/usr/bin/env python3
"""Generate separate pinned 4WS8 sources/runtime; no device/network access."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
BASE=ROOT/'stm32/mecanum_f407'
OLD=BASE/'candidates/servo_4ws7_stage0'
READY=ROOT/'docs/wand_4ws7_stage0_2026-10-23/host_fix_20261023/verified_v2'
CONTROL='mecanum_wand_4ws8_control';HARDWARE='mecanum_wand_4ws8_hardware'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def pinned_inputs():
    m=json.loads((READY/'build_manifest.json').read_text())
    pins={str(ROOT/p):v for p,v in json.loads((READY/'preserved_after.json').read_text()).items()}
    pins.update(m['source_hashes']);pins.update(m['runtime_hashes'])
    pins[m['firmware']['path']]=m['firmware']['sha256']
    pins[str(READY/'build_manifest.json')]=sha(READY/'build_manifest.json')
    for p,v in pins.items():
        if sha(p)!=v:raise ValueError('protected input drift: '+p)
    return pins

def change(path,old,new,count=1):
    s=path.read_text()
    if s.count(old)!=count:raise ValueError('source fragment drift '+str(path)+': '+old[:80])
    path.write_text(s.replace(old,new))

def copy(path,dest):
    dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,dest)

def generate(out):
    from stages import STAGES
    pins=pinned_inputs();out.mkdir(parents=True,exist_ok=False)
    (out/'COLCON_IGNORE').write_text('')
    (out/'protected_inputs.json').write_text(json.dumps(pins,indent=2)+'\n')
    src=out/'firmware';src.mkdir()
    # Source generation is mechanical; accepted inputs never change in place.
    for name in ('CMakeLists.txt','Inc/four_link_bench.h','Src/four_link_motion.c',
                 'Src/drive_bench_control.c','tests/oracle.c'):
        copy(OLD/name,src/name)
    copy(BASE/'candidates/servo_4ws7/Src/servo_policy.c',src/'Src/servo_policy.c')
    copy(BASE/'candidates/servo_4ws7/Inc/servo_policy.h',src/'Inc/servo_policy.h')
    change(src/'Inc/servo_policy.h','0x7000','0x7800')
    change(src/'Inc/servo_policy.h','#endif','float servo_stage_target_rpm(unsigned index);\nfloat servo_stage_feedback_rpm(unsigned index);\n#endif')
    with (src/'Src/servo_policy.c').open('a') as f:
        f.write('\nfloat servo_stage_target_rpm(unsigned i) { return i==0?100.0f:i==1?180.0f:i==2?250.0f:400.0f; }\n'
                'float servo_stage_feedback_rpm(unsigned i) { return i==0?120.0f:i==1?210.0f:i==2?280.0f:440.0f; }\n')
    copy(BASE/'candidates/servo_4ws7/Src/four_drive_group.c',src/'Src/four_drive_group.c')
    change(src/'Src/four_drive_group.c','        g->loop[i].config.duty_limit_permille=p.duty_permille;',
        '''        const float cpr=g->loop[i].config.wheel[i].counts_per_rev;
        g->loop[i].config.target_limit_cps=fminf(7100.0f,cpr*servo_stage_target_rpm(stage)/60.0f);
        g->loop[i].config.feedback_limit_cps=cpr*servo_stage_feedback_rpm(stage)/60.0f;
        g->loop[i].config.duty_limit_permille=p.duty_permille;''')
    profile=(BASE/'candidates/servo_4ws7/Inc/four_speed_pwm_profile.h').read_text()
    start=profile.index('/* This OFFLINE artifact')
    end=profile.index('_Static_assert',start)
    profile=profile[:start]+'''/* Separate staged commissioning image; no test bypass. Capability is not site approval. */
#ifndef SERVO_STAGED_COMMISSIONING
#error Independent staged commissioning target required.
#endif
#ifdef SERVO_NATIVE_TEST
#error Native execution bypass forbidden.
#endif
#define SERVO_EXECUTION_RELEASED 1
#define SERVO_BENCH_LIMIT_MS 5000U
#define SERVO_BENCH_MAGIC 0x7880U
#include "servo_policy.h"
'''+profile[end:]
    profile=profile.replace('#include "../../../Inc/four_speed_pwm_profile.h"',
        '#include "'+str(BASE/'Inc/four_speed_pwm_profile.h')+'"')
    (src/'Inc/four_speed_pwm_profile.h').write_text(profile)
    change(src/'Inc/four_link_bench.h','0x37535734','0x38535734')
    change(src/'Inc/four_link_bench.h','4WS7 stage0 commissioning v3.','4WS8 staged commissioning v4.')
    change(src/'Src/drive_bench_control.c','commissioning_4ws7_stage0_v3','commissioning_4ws8_staged_v4')
    c=src/'Src/four_link_motion.c'
    change(c,'{ return value==SERVO_TIER_MAGIC ? 1U : value==SERVO_BENCH_MAGIC ? 2U : 0U; }',
        '''{ return value>=SERVO_TIER_MAGIC && value<SERVO_TIER_MAGIC+SERVO_STAGE_COUNT ? 1U :
    value>=SERVO_BENCH_MAGIC && value<SERVO_BENCH_MAGIC+SERVO_STAGE_COUNT ? 2U : 0U; }
static unsigned requested_stage(uint16_t value)
{ return requested_mode(value)==1U ? (unsigned)value-(unsigned)SERVO_TIER_MAGIC : (unsigned)value-SERVO_BENCH_MAGIC; }''')
    change(c,'four_drive_select_stage(&b->group,0,now)','four_drive_select_stage(&b->group,requested_stage(e->reserved),now)')
    change(c,'mode!=c->action_mode || b->group.servo_stage!=0',
        'mode!=c->action_mode || b->group.servo_stage!=requested_stage(e->reserved)')
    change(c,'s->output_authority_version = 3U;','s->output_authority_version = 4U;')
    change(c,'/* Dedicated target is physically bounded to stage0; no full-speed unlock. */',
        '/* Fixed stage/mode must match this fresh session; never auto-promote. */')
    # Safety promotion is explicit and isolated; no original ARM source edited.
    cm=src/'CMakeLists.txt';s=cm.read_text()
    s=s.replace('get_filename_component(BASE "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)',
        'set(BASE '+str(BASE)+')')
    s=s.replace('get_filename_component(WORKSPACE "${BASE}/../.." ABSOLUTE)','set(WORKSPACE '+str(ROOT)+')')
    s=s.replace('file(READ ${CMAKE_CURRENT_LIST_DIR}/upstream.json manifest)',
        'file(READ ${CMAKE_CURRENT_LIST_DIR}/../protected_inputs.json manifest)')
    s=s.replace('${WORKSPACE}/${file}','${file}')
    s=s.replace('4ws7_stage0','4ws8_staged').replace('4WS7_STAGE0_V3','4WS8_STAGED_V4')
    s=s.replace('${PARENT}/Src/four_drive_group.c','Src/four_drive_group.c')
    s=s.replace('SERVO_STAGE0_COMMISSIONING','SERVO_STAGED_COMMISSIONING')
    s=s.replace('tests/test_stage0.c','tests/test_stages.c').replace('test_stage0','test_stages')
    s=s.replace('four_link_bench_main_4ws7','four_link_bench_main_4ws8')
    s=s.replace('bounded stage0 NOT-FLASHED','independent staged NOT-FLASHED').replace('STAGE0 0.20/35% BUILD ONLY','4WS8 FIXED-STAGE BUILD ONLY')
    cm.write_text(s)
    copy(HERE/'test_stages.c',src/'tests/test_stages.c')
    copy(BASE/'candidates/servo_4ws7/tests/test_servo.c',src/'tests/test_controller.c')
    change(src/'tests/test_controller.c','0x37535734','0x38535734')
    change(src/'tests/test_controller.c','s.output_authority_version==2 && s.stage_id==6 && !s.hardware_released',
        's.output_authority_version==4 && s.stage_id==6 && s.hardware_released')
    with cm.open('a') as f:
        f.write('''
if(NOT SERVO_ARM)
  add_executable(test_controller tests/test_controller.c ${ALGORITHMS})
  foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(values test_stages ${property})
    if(values)
      set_property(TARGET test_controller PROPERTY ${property} "${values}")
    endif()
  endforeach()
  target_link_libraries(test_controller PRIVATE m)
  add_test(NAME continuous_controller_fault_injection COMMAND test_controller)
endif()
''')
    runtime=out/'runtime/src';runtime.mkdir(parents=True)
    for old,new in (('mecanum_wand_4ws7_control',CONTROL),('mecanum_wand_4ws7_hardware',HARDWARE)):
        shutil.copytree(READY/'runtime/src'/old,runtime/new,
            ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    for p in runtime.rglob('*'):
        if p.is_file():
            p.write_text(p.read_text().replace('mecanum_wand_4ws7','mecanum_wand_4ws8')
                .replace('WAND_4WS7','WAND_4WS8').replace('wand_4ws7_stage0','wand_4ws8_staged')
                .replace('4WS7_STAGE0','4WS8_STAGED').replace('4WS7','4WS8'))
    for p in sorted(runtime.rglob('*'),key=lambda x:len(x.parts),reverse=True):
        new=p.name.replace('mecanum_wand_4ws7','mecanum_wand_4ws8').replace('wand_4ws7_stage0','wand_4ws8_staged')
        if new!=p.name:p.rename(p.with_name(new))
    control=runtime/CONTROL/CONTROL;hw=runtime/HARDWARE/HARDWARE
    (runtime/HARDWARE/'README.md').write_text((HERE/'README.md').read_text())
    for name in ('stages.py','staged_config.py'):copy(HERE/name,control/name)
    for name in ('stages.py','authorization.py'):copy(HERE/name,hw/name)
    # Generate a strict new decoder, not a widened old whitelist.
    contract=hw/'stage0_contract.py'
    change(contract,"TAG=int.from_bytes(b'4WS8','little')","TAG=int.from_bytes(b'4WS8','little')")
    change(contract,'from dataclasses import dataclass','from dataclasses import dataclass\nfrom .stages import STAGES')
    change(contract,'STAGES=(Stage(.20,.08,350,1200),)','')
    change(contract,'stage_id!=0','stage_id>=len(STAGES)')
    change(contract,'0x7000+stage','0x7800+stage')
    change(contract,'version!=3','version!=4')
    change(contract,'output_authority_version=3','output_authority_version=4')
    s=contract.read_text().replace('4WS8 v3 stage0-only','4WS8 v4 fixed-stage').replace('184 bytes of 4WS8 v3 stage0','184 bytes of 4WS8 v4')
    s=s.replace('explicit stage 0 only','explicit fixed stage 0..6')
    contract.write_text(s)
    change(contract,"status.get('output_authority_version')==3","status.get('output_authority_version')==4")
    change(contract,'>100.001','>stage.target_rpm+.001');change(contract,'>100.1','>stage.target_rpm+.1')
    change(contract,'abs(f[8])>350','abs(f[8])>stage.duty')
    change(contract,"MOTION_LINEAR_LIMITS={'4WS8':.20}","MOTION_LINEAR_LIMITS={'4WS8':.70}")
    # Preserve the known 10Hz/focus fixes; add a fixed, per-launch stage.
    session=hw/'stage0_session.py'
    change(session,'def __init__(self,*args,action_mode=1,**kwargs):',
        'def __init__(self,*args,action_mode=1,stage=0,**kwargs):\n        from .stages import get_stage\n        self.stage=stage;self.limits=get_stage(stage)')
    s=session.read_text().replace('0x7000 if self.action_mode==1 else 0x7080',
        '(0x7800 if self.action_mode==1 else 0x7880)+self.stage')
    s=s.replace('math.hypot(vx_mps,vy_mps)>.200000001','math.hypot(vx_mps,vy_mps)>self.limits.speed+1e-9')
    session.write_text(s)
    suspended=hw/'suspended_core.py'
    change(suspended,'def __init__(self,session_factory=None):',
        'def __init__(self,session_factory=None,stage=0):\n        from .stages import get_stage\n        self.stage=stage;self.limits=get_stage(stage)')
    change(suspended,'Stage0Session(session_factory,action_mode=2)','Stage0Session(session_factory,action_mode=2,stage=stage)')
    change(suspended,'abs(v)>120','abs(v)>self.limits.feedback_rpm')
    change(suspended,"if self.status['action_mode'] not in (0,2):raise ProtocolError('not suspended mode')",
        "if self.status['action_mode'] not in (0,2):raise ProtocolError('not suspended mode')\n                if self.status['selected'] and self.status['stage']!=self.stage:raise ProtocolError('wrong selected stage')")
    change(suspended,'x,y=DIRECTIONS[self.direction]','x,y=DIRECTIONS[self.direction]\n                x*=self.limits.speed/.20;y*=self.limits.speed/.20')
    gui=hw/'suspended_console.py'
    change(gui,'worker=Worker(SuspendedController(),io)',"worker=Worker(SuspendedController(stage=profile['stage']),io)")
    change(gui,"root.title('4WS8 · 悬空首档0.20m/s · 单次最长5秒')",
        "root.title(f'4WS8 · 悬空档{worker.core.stage} · {worker.core.limits.speed:.2f}m/s · 最长5秒')")
    change(gui,"text='0.20m/s合速度上限 · 0.08m/s² · 35% PWM · PC与MCU双重5秒上限'",
        "text=f'{worker.core.limits.speed:.2f}m/s · {worker.core.limits.accel:.2f}m/s² · {worker.core.limits.duty/10:g}% PWM · 双重5秒上限'")
    commission=hw/'commissioning.py'
    change(commission,'from '+CONTROL+'.servo_config import ServoSafetyConfig as SafetyConfig',
        'from '+CONTROL+'.staged_config import StagedSafetyConfig as SafetyConfig\nfrom .stages import STAGES,get_stage,SCOPE')
    change(commission,"118, '/wand_4ws8_staged'","119, '/wand_4ws8_staged'")
    change(commission,"if mode not in ('shadow', 'emulator', 'hardware'):","if mode != 'hardware':")
    change(commission,"'max_goal_radius_m', 'goal_boundary_policy',","'max_accel_mps2', 'max_goal_radius_m', 'goal_boundary_policy',")
    change(commission,"hw.get('output_authority_version') != 3","hw.get('output_authority_version') != 4")
    change(commission,'self.config.max_speed_mps > .20','self.config.max_speed_mps > .70')
    s=commission.read_text().replace('184-byte v3','184-byte v4').replace('fixed domain118','fixed domain119').replace('first hardware acceptance is capped at 0.20m/s and 0.10rad/s','fixed stage speed and 0.10rad/s limits required')
    commission.write_text(s)
    start=commission.read_text().index("        if mode == 'hardware':\n            confirmation = self.profile.get('stage020_confirmations'")
    end=commission.read_text().index('        self.wand_reference_margin_m = 0.',start)
    s=commission.read_text();s=s[:start]+s[end:];commission.write_text(s)
    # Per-tier signed scope prevents SIM, legacy and cross-tier authority reuse.
    change(commission,'def __init__(self, key):\n        self.key, self.owner, self.sequence = secret_bytes(key), uuid.uuid4().hex, 0',
        'def __init__(self, key, stage=0):\n        self.stage=stage;get_stage(stage)\n        self.key, self.owner, self.sequence = secret_bytes(key), uuid.uuid4().hex, 0')
    change(commission,"epoch=epoch, state=state, permit=permit, estop=estop, command=asdict(command))",
        "epoch=epoch, state=state, permit=permit, estop=estop, command=asdict(command),scope=SCOPE+str(self.stage))")
    change(commission,'def __init__(self, key):\n        self.key, self.owner, self.sequence = secret_bytes(key), None, -1',
        'def __init__(self, key, stage=0):\n        self.stage=stage;self.limits=get_stage(stage)\n        self.key, self.owner, self.sequence = secret_bytes(key), None, -1')
    change(commission,"'permit', 'estop', 'command'}:","'permit', 'estop', 'command', 'scope'} or p.get('scope')!=SCOPE+str(self.stage):")
    change(commission,'> .20+1e-9','> self.limits.speed+1e-9')
    change(commission,'def __init__(self, key):\n        self.verifier = AuthorityVerifier(key)',
        'def __init__(self, key, stage=0):\n        self.verifier = AuthorityVerifier(key,stage)')
    nodes=hw/'nodes.py'
    change(nodes,'from .continuous_status import disabled_motion_status_ready',
        'from .continuous_status import disabled_motion_status_ready\nfrom .stage0_session import Stage0Session')
    change(nodes,"AuthoritySigner(os.environ['WAND_4WS8_CHANNEL_KEY'])","AuthoritySigner(os.environ['WAND_4WS8_CHANNEL_KEY'],self.runtime.config.stage)")
    change(nodes,"TransportPolicy(os.environ['WAND_4WS8_CHANNEL_KEY'])","TransportPolicy(os.environ['WAND_4WS8_CHANNEL_KEY'],self.runtime.config.stage)")
    change(nodes,'        super().__init__()\n        self.runtime.guard(self)',
        '        super().__init__()\n        self.session=Stage0Session(stage=self.runtime.config.stage)\n        self.runtime.guard(self)')
    change(nodes,'self.max_vx_mps != .20 or self.max_vy_mps != .20',
        'self.max_vx_mps != self.runtime.config.max_speed_mps or self.max_vy_mps != self.runtime.config.max_speed_mps')
    change(nodes,"prepared and self.last_motion_status.get('selected')",
        "prepared and self.last_motion_status.get('selected')\n                and self.last_motion_status.get('stage')==self.runtime.config.stage")
    change(nodes,'        status = self.last_motion_status\n        if (self.feedback_is_fresh',
        "        status = self.last_motion_status\n        if status and status['selected'] and status['stage']!=self.runtime.config.stage:\n            self.policy.revoke('SELECTED_STAGE_MISMATCH',hard=True);self.stop_wire();return\n        if (self.feedback_is_fresh")
    launch=runtime/HARDWARE/'launch/wand_4ws8_staged.launch.py'
    change(launch,"'max_vx_mps': .20, 'max_vy_mps': .20,","'max_vx_mps': runtime.config.max_speed_mps, 'max_vy_mps': runtime.config.max_speed_mps,")
    # Current stage is immutable for the entire launch, not a live slider.
    console=control/'operator_console.py'
    change(console,"'hardware': '真实 Sign → 4WS8首档0.20m/s（35%PWM）'",
        "'hardware': f'真实 Sign → 4WS8档{self.config.stage} · {self.config.max_speed_mps:.2f}m/s · {self.config.max_accel_mps2:.2f}m/s²'")
    change(console,"'域97 独立0.20首档  ·  1:1映射  ·  ≤0.20 m/s  ·  '",
        "f'域119 · 固定档{self.config.stage} · 1:1 · ≤{self.config.max_speed_mps:.2f}m/s · '")
    change(console,'now-self.last_draw >= .10','now-self.last_draw >= .033')
    config=runtime/HARDWARE/'config'
    template=json.loads(next(config.glob('*.json')).read_text())
    for p in config.glob('*.json'):p.unlink()
    for i,stage in enumerate(STAGES):
        p=json.loads(json.dumps(template));p['stage']=i
        p['hardware'].update(output_authority_version=4,motor_target_rpm_cap=stage.target_rpm,pwm_cap_permille=stage.duty,
            wheel_rim_limit_mps=1.256636,target_limit_cps=7100)
        p['safety'].update(max_speed_mps=stage.speed,max_accel_mps2=stage.accel)
        p.pop('stage020_policy',None);p.pop('stage020_confirmations',None);p.pop('stage0_runtime_approval',None)
        p['staged_runtime_approval']={};p['trial_review']={}
        p['evidence']['change_scope']='Independent 4WS8 v4 fixed-stage commissioning; no flash or physical acceptance inferred'
        (config/f'stage{i}.NOT_APPROVED.json').write_text(json.dumps(p,indent=2,ensure_ascii=False)+'\n')
    gate=lambda path:ast.get_source_segment(path.read_text(),next(n for n in ast.parse(path.read_text()).body
        if isinstance(n,ast.ClassDef) and n.name=='FinalGate'))
    original=READY/'runtime/src/mecanum_wand_4ws7_control/mecanum_wand_4ws7_control/safety_core.py'
    if gate(control/'safety_core.py')!=gate(original):raise ValueError('FinalGate drift')
    return dict(source=str(src),runtime_src=str(runtime),protected_files=len(pins),
        independent_final_gate_unchanged=True,identity='4WS8',version=4,domain=119,
        namespace='/wand_4ws8_staged',flashed=False,hardware_started=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    print(json.dumps(generate(p.parse_args().output.resolve()),indent=2))
