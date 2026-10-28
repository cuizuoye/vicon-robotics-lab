#!/usr/bin/env python3
"""Pin/copy accepted inputs into a new 4WS9 candidate. No hardware access."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import sys
from stages import STAGES

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
PARENT=ROOT/'docs/wand_rotation_hardware_2026-10-26/prepared'
FW_PARENT=ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/firmware'
BASE=ROOT/'stm32/mecanum_f407'
CONTROL='mecanum_wand_4ws9_control';HARDWARE='mecanum_wand_4ws9_hardware'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def hashes(folder):
    return {str(p):sha(p) for p in Path(folder).rglob('*') if p.is_file() and
        not any(k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def edit(p,old,new,count=1):
    s=p.read_text()
    if s.count(old)!=count:raise ValueError('Source layout drift: '+str(p)+' / '+old[:100])
    p.write_text(s.replace(old,new))
def copy(a,b):b.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(a,b)

def protected_inputs():
    m=json.loads((PARENT/'build_manifest.json').read_text())
    if m.get('rotation_migration_passed') is not True:raise ValueError('Parent migration not verified')
    pins={str(PARENT/'build_manifest.json'):sha(PARENT/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):pins.update(m[group])
    pins.update(hashes(FW_PARENT))
    for p,v in pins.items():
        if sha(p)!=v:raise ValueError('Protected input drift: '+p)
    return m,pins

def firmware(out):
    fw=out/'firmware';shutil.copytree(FW_PARENT,fw)
    copy(HERE/'servo_policy.h',fw/'Inc/servo_policy.h');copy(HERE/'servo_policy.c',fw/'Src/servo_policy.c')
    profile=fw/'Inc/four_speed_pwm_profile.h'
    for a,b in [('0.100f','2.000f'),('YAW_MRAD_S 100','YAW_MRAD_S 2000'),
                ('16300.0f','26000.0f'),('0x7880U','0x7980U')]:
        edit(profile,a,b)
    # Keep the pinned parent's include guard: its quoted header must consume
    # this preincluded NEW profile rather than re-including the old profile.
    profile.write_text(profile.read_text().replace('Independent OFFLINE 4WS7 candidate only.',
        'Independent OFFLINE 4WS9 candidate only.'))
    edit(profile,'2m/s^2 vector acceleration plus .4rad/s^2 yaw,','2m/s^2 vector acceleration plus 12rad/s^2 yaw,')
    h=fw/'Inc/four_link_bench.h';edit(h,'0x38535734','0x39535734')
    edit(h,'4WS8 staged commissioning v4.','4WS9 XY+yaw staged commissioning v5.')
    edit(h,'    uint8_t rejoining_mask, servo_reserved;','''    uint8_t rejoining_mask, servo_reserved;
    float yaw_rate_limit, yaw_accel_limit, wheel_rpm_limit;
    uint32_t coupled_shape_revision;''')
    edit(h,'==184','==200');edit(h,'4WS7 v3 stage0 authority diagnostic','4WS9 v5 200-byte XY+yaw diagnostic')
    link=fw/'Src/four_link_motion.c'
    s=link.read_text();start=s.index('static bool plan_for(');end=s.index('\nbool four_link_bench_process',start)
    s=s[:start]+'''static bool plan_for(const drive_bench_t *b,const mc_cmd_vel_v1_t *v,mecanum_drive_plan_t *p)
{
    servo_stage_t stage;
    if (!servo_stage_get(b->group.servo_stage,&stage) ||
        abs(v->vx_mm_s)>FOUR_DRIVE_MAX_BODY_MM_S || abs(v->vy_mm_s)>FOUR_DRIVE_MAX_BODY_MM_S ||
        abs(v->wz_mrad_s)>FOUR_DRIVE_MAX_YAW_MRAD_S) return false;
    const mecanum_body_velocity_t body={v->vx_mm_s/1000.f,v->vy_mm_s/1000.f,v->wz_mrad_s/1000.f};
    if (hypotf(body.vx_m_s,body.vy_m_s)>stage.speed_m_s+.000001f ||
        fabsf(body.wz_rad_s)>stage.yaw_rate_rad_s+.000001f) return false;
    const float wheel_limit=fminf(FOUR_DRIVE_MAX_WHEEL_M_S,
        servo_stage_target_rpm(b->group.servo_stage)*3.14159265358979323846f*.060f/60.f);
    if (!mecanum_drive_plan(&b->group.geometry,&b->adopted,&body,wheel_limit,p)) return false;
    four_drive_normalize_plan(p);return true;
}
'''+s[end:]
    s=s.replace('#include <math.h>','#include <math.h>\n#include <stdlib.h>');link.write_text(s)
    edit(link,'s->output_authority_version = 4U;','s->output_authority_version = 5U;')
    edit(link,'s->output_slew_limit=stage.output_slew_permille_s;','''s->output_slew_limit=stage.output_slew_permille_s;
    s->yaw_rate_limit=stage.yaw_rate_rad_s; s->yaw_accel_limit=stage.yaw_accel_rad_s2;
    s->wheel_rpm_limit=servo_stage_target_rpm(b->group.servo_stage);
    s->coupled_shape_revision=1U;''')
    group=fw/'Src/four_drive_group.c'
    edit(group,'p.accel_m_s2+.146f*.40f','p.accel_m_s2+.146f*p.yaw_accel_rad_s2')
    edit(group,'stage.accel_m_s2+.146f*.40f','stage.accel_m_s2+.146f*stage.yaw_accel_rad_s2')
    edit(group,'fabsf(v->wz_rad_s)>FOUR_DRIVE_MAX_YAW_RAD_S','fabsf(v->wz_rad_s)>stage.yaw_rate_rad_s+.000001f')
    edit(group,'g->loop[i].config.wheel[i].counts_per_rev*SERVO_TARGET_RPM/60',
        'g->loop[i].config.wheel[i].counts_per_rev*servo_stage_target_rpm(g->servo_stage)/60+.001f')
    edit(group,'FOUR_DRIVE_MAX_WHEEL_M_S,&expected',
        'fminf(FOUR_DRIVE_MAX_WHEEL_M_S,servo_stage_target_rpm(g->servo_stage)*3.14159265358979323846f*.060f/60.f),&expected')
    start=group.read_text().index('    const float distance=hypotf(dx,dy),step=stage.accel_m_s2*dt;')
    end=group.read_text().index('    mecanum_drive_plan_t next;',start)
    s=group.read_text();s=s[:start]+'''    const float distance=hypotf(dx,dy),step=stage.accel_m_s2*dt;
    const float dz=v->wz_rad_s-g->shaped_body.wz_rad_s;
    float alpha=distance>step ? step/distance : 1.f;
    const float yaw_step=stage.yaw_accel_rad_s2*dt;
    if (fabsf(dz)>yaw_step) alpha=fminf(alpha,yaw_step/fabsf(dz));
    /* One convex SI interpolation: preserves both acceleration limits and
     * common wheel envelope; no independent wheel/axis clipping normal ramp. */
    g->shaped_body.vx_m_s+=alpha*dx;
    g->shaped_body.vy_m_s+=alpha*dy;
    g->shaped_body.wz_rad_s+=alpha*dz;
'''+s[end:];group.write_text(s)
    edit(group,'FOUR_DRIVE_MAX_WHEEL_M_S,&next',
        'fminf(FOUR_DRIVE_MAX_WHEEL_M_S,servo_stage_target_rpm(g->servo_stage)*3.14159265358979323846f*.060f/60.f),&next')
    edit(fw/'Src/drive_bench_control.c','commissioning_4ws8_staged_v4','commissioning_4ws9_xy_yaw_v5')
    cm=fw/'CMakeLists.txt'
    s=cm.read_text().replace('4ws8','4ws9').replace('4WS8','4WS9').replace('STAGED_V4','STAGED_V5')
    cm.write_text(s)
    # Retain the old full fault suite but make its assertions refer to the
    # NEW actual angular policy and identity, not a fabricated old fixture.
    t=fw/'tests/test_controller.c'
    s=t.read_text().replace('0x38535734','0x39535734').replace('output_authority_version==4','output_authority_version==5')
    s=s.replace('<=.4f*dt*.001f','<=p.yaw_accel_rad_s2*dt*.001f').replace('sizeof(s)==184','sizeof(s)==200')
    old='for(unsigned i=0;i<200;i++){tick(false,10);velocity(false);}'
    if s.count(old)!=1:raise ValueError('Measured standstill test layout drift')
    s=s.replace(old,'''/* Wait for ACTUAL encoder-stationary qualification, not an assumed
     * two-second coast. The firmware's 500ms guard is unchanged; host stop
     * timeout is also unchanged and may report STOP_UNCONFIRMED in hardware. */
    for(unsigned i=0;i<300 && b.group.stationary_ms<500;i++){tick(false,10);velocity(false);}''')
    t.write_text(s)
    t=fw/'tests/test_stages.c'
    s=t.read_text().replace('0x38535734','0x39535734').replace('output_authority_version==4','output_authority_version==5').replace('4WS8','4WS9')
    t.write_text(s)
    copy(HERE/'test_rotation.c',fw/'tests/test_rotation.c')
    cm.write_text(cm.read_text()+'''
if(NOT SERVO_ARM)
  add_executable(test_rotation tests/test_rotation.c ${ALGORITHMS})
  foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(values test_stages ${property})
    if(values)
      set_property(TARGET test_rotation PROPERTY ${property} "${values}")
    endif()
  endforeach()
  target_link_libraries(test_rotation PRIVATE m)
  add_test(NAME xy_yaw_rotation_and_faults COMMAND test_rotation ${CMAKE_CURRENT_BINARY_DIR})
endif()
''')
    return fw

def runtime(out,m):
    dest=out/'runtime/src';dest.mkdir(parents=True)
    for old,new in (('mecanum_wand_4ws8_control',CONTROL),('mecanum_wand_4ws8_hardware',HARDWARE)):
        shutil.copytree(Path(m['runtime_src'])/old,dest/new,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    for p in dest.rglob('*'):
        if p.is_file():p.write_text(p.read_text().replace('mecanum_wand_4ws8','mecanum_wand_4ws9')
            .replace('WAND_4WS8','WAND_4WS9').replace('4WS8','4WS9').replace('wand_4ws8','wand_4ws9'))
    for p in sorted(dest.rglob('*'),key=lambda x:len(x.parts),reverse=True):
        n=p.name.replace('mecanum_wand_4ws8','mecanum_wand_4ws9').replace('wand_4ws8','wand_4ws9')
        if n!=p.name:p.rename(p.with_name(n))
    c=dest/CONTROL/CONTROL;h=dest/HARDWARE/HARDWARE
    for folder in (c,h):copy(HERE/'stages.py',folder/'stages.py')
    copy(HERE/'staged_config.py',c/'staged_config.py');copy(HERE/'authorization.py',h/'authorization.py')
    # Config has a physical wheel cap as well as XY/yaw acceleration. This
    # applies before the SI ramp and is independently checked in the gate/MCU.
    edit(c/'core.py','    max_yaw_rate_radps: float = 0.50','    max_wheel_rpm: float = 400.0\n    max_yaw_rate_radps: float = 0.50')
    edit(c/'rotation_servo.py','MAX_RPM/peak if peak else 1.','c.max_wheel_rpm/peak if peak else 1.')
    edit(c/'rotation_core.py','>400.+1e-6','>self.c.limits.target_rpm+1e-6')
    edit(c/'pilot_core.py',"if config.stage != 4 or config.max_speed_mps != .70 or config.max_accel_mps2 != .80:\n        raise ValueError('finite first-ground pilot is fixed to stage4 .70/.80')",
        "from .staged_config import StagedSafetyConfig\n    if not isinstance(config,StagedSafetyConfig):raise ValueError('Exact 4WS9 tier config required')")
    p=c/'demo_policy.py';s=p.read_text().replace('stage=4,','fixed_stage_per_process=True,')
    s=s.replace("'SIGNED_XY_YAW_4WS9_PILOT_V1'","'SIGNED_XY_YAW_4WS9_V5_TIER_PILOT_V1'")
    s=s.replace('yaw_rate_limit_radps=.10, yaw_accel_limit_radps2=.40','maximum_candidate_yaw_rate_radps=2., maximum_candidate_yaw_accel_radps2=12.')
    s=s.replace('yaw_rate_limit_radps=.10,yaw_accel_limit_radps2=.40','yaw_rate_limit_radps=monitor.config.max_yaw_rate_radps,yaw_accel_limit_radps2=monitor.config.max_yaw_accel_radps2')
    p.write_text(s)
    commission=h/'commissioning.py'
    edit(commission,"120, '/wand_4ws9_ground_pilot'","121, '/wand_4ws9_sync'")
    edit(commission,"if candidate.get('trial_review', {}).get('kind') != KIND or candidate.get('stage') != 4:",
        "if candidate.get('trial_review', {}).get('kind') != KIND or type(candidate.get('stage')) is not int or candidate['stage'] not in range(7):")
    edit(commission,"from .pilot_authorization import KIND","from .authorization import KIND")
    edit(commission,"'max_accel_mps2', 'max_goal_radius_m'","'max_accel_mps2', 'max_yaw_rate_radps', 'max_yaw_accel_radps2', 'max_goal_radius_m'")
    s=commission.read_text().replace("!= 184","!= 200").replace("!= 4:","!= 5:").replace('184-byte v4','200-byte v5')
    s=s.replace('self.motion_status_size = 184','self.motion_status_size = 200')
    s=s.replace('self.config.max_yaw_rate_radps > .10','self.config.max_yaw_rate_radps > 2.')
    s=s.replace('abs(command.wz) > .10+1e-9','abs(command.wz) > self.limits.yaw_rate+1e-9')
    s=s.replace('fixed stage speed and 0.10rad/s limits required','fixed reviewed 4WS9 XY+yaw tier required')
    commission.write_text(s)
    p=h/'stage0_session.py'
    s=p.read_text().replace('0x7800','0x7900').replace('0x7880','0x7980').replace('abs(wz_radps)>.100000001','abs(wz_radps)>self.limits.yaw_rate+1e-9');p.write_text(s)
    decoder=h/'stage0_contract.py'
    s=decoder.read_text().replace('SIZE=STATUS.size+SNAPSHOT.size+AUTHORITY.size+SERVO.size',
        "ROTATION=struct.Struct('<3fI')\nSIZE=STATUS.size+SNAPSHOT.size+AUTHORITY.size+SERVO.size+ROTATION.size")
    s=s.replace('0x7800','0x7900').replace('version!=4','version!=5').replace('output_authority_version=4','output_authority_version=5')
    s=s.replace("status.get('output_authority_version')==4","status.get('output_authority_version')==5")
    s=s.replace('184 bytes of 4WS9 v4','200 bytes of 4WS9 v5').replace('4WS9 v4','4WS9 v5')
    s=s.replace('    stage=STAGES[stage_id]','''    stage=STAGES[stage_id]
    yr,ya,wr,revision=ROTATION.unpack_from(payload,184)
    if (not all(math.isfinite(v) for v in (yr,ya,wr)) or revision!=1
            or not math.isclose(yr,stage.yaw_rate,rel_tol=1e-6)
            or not math.isclose(ya,stage.yaw_accel,rel_tol=1e-6) or wr!=stage.target_rpm):
        raise ValueError('4WS9 independent angular tier/coupled revision mismatch')''')
    s=s.replace('abs(shape[2])>.100001','abs(shape[2])>stage.yaw_rate+1e-6')
    s=s.replace('abs(body[2])>100','abs(body[2])>stage.yaw_rate*1000+1')
    s=s.replace("body_si=shape,effective_cps=effective","yaw_rate_limit=yr,yaw_accel_limit=ya,wheel_rpm_limit=wr,coupled_shape_revision=revision,body_si=shape,effective_cps=effective")
    decoder.write_text(s)
    edit(h/'nodes.py','self.max_wz_radps != .10','self.max_wz_radps != self.runtime.config.max_yaw_rate_radps')
    edit(h/'continuous_bridge.py','self.max_wz_radps <= .20','self.max_wz_radps <= 2.')
    launch=dest/HARDWARE/'launch/wand_4ws9_staged.launch.py'
    edit(launch,"'max_wz_radps': .10","'max_wz_radps': runtime.config.max_yaw_rate_radps")
    edit(c/'rotation_receiver_guard.py',"('0','120')","('0','121')")
    # The old wrapper refers to old-image suspended results. It must never be
    # reachable as an alternate approval path in this new runtime.
    (h/'pilot_authorization.py').write_text('from .authorization import KIND,verify_trial_review as verify_pilot_review\n')
    # Add explicit left/right yaw options to the inherited SINGLE 5s bench.
    p=h/'suspended_core.py'
    edit(p,"PREPARING=('HELLO'","DIRECTIONS.update({'左旋':(0.,0.),'右旋':(0.,0.)})\nPREPARING=('HELLO'")
    edit(p,'self.pending.append(self.session.cmd_vel(x,y,0,600,True,self.stamp(now)))',
        "w=self.limits.yaw_rate if self.direction=='左旋' else -self.limits.yaw_rate if self.direction=='右旋' else 0.\n                self.pending.append(self.session.cmd_vel(x,y,w,600,True,self.stamp(now)))")
    # Generate explicit NOT_APPROVED profiles. No historical site/flash record
    # or automatically true ground qualifications migrate to the new image.
    config=dest/HARDWARE/'config'
    template=json.loads((config/'stage0.NOT_APPROVED.json').read_text())
    for i,stage in enumerate(STAGES):
        p=json.loads(json.dumps(template));p['stage']=i
        p['safety'].update(max_speed_mps=stage.speed,max_accel_mps2=stage.accel,
            max_yaw_rate_radps=stage.yaw_rate,max_yaw_accel_radps2=stage.yaw_accel)
        p['hardware'].update(required_motion_tag='4WS9',diagnostic_payload_bytes=200,output_authority_version=5,
            pwm_cap_permille=stage.duty,motor_target_rpm_cap=stage.target_rpm)
        p['trial_review']=dict(kind='4WS9_V5_CONTROLLED_TRIAL_NOT_QUALIFICATION',identity='4WS9',
            image_sha256=None,authorized_stage=i,human_authorized=False,purpose='sign')
        p['staged_runtime_approval']={};p['confirmations']={k:False for k in p['confirmations']}
        p['arena_confirmations']={k:False for k in p['arena_confirmations']}
        write(config/f'stage{i}.NOT_APPROVED.json',p)
    # New UI reports the selected actual tier, not the old low-speed label.
    p=c/'rotation_ui.py';s=p.read_text()
    s=s.replace("top[1].configure(text='位移/转角1:1 · 平移0.70m/s、0.80m/s² · 转速≤5.7°/秒 · 最长120秒')",
        "top[1].configure(text=f'档{self.config.stage} · 1:1 · {self.config.max_speed_mps:.2f}m/s / {self.config.max_accel_mps2:.2f}m/s² · {self.config.max_yaw_rate_radps:.2f}rad/s / {self.config.max_yaw_accel_radps2:.2f}rad/s² · 120秒')")
    s=s.replace("f'车相对角 {math.degrees(actual):+.1f}°\\n转角1:1；现固件转速≤5.7°/秒\\n'",
        "f'车相对角 {math.degrees(actual):+.1f}°\\n转角1:1；档{self.config.stage}转速≤{math.degrees(self.config.max_yaw_rate_radps):.1f}°/秒\\n'")
    p.write_text(s)
    return dest

def generate(out):
    m,pins=protected_inputs();out.mkdir(parents=True,exist_ok=False);out.chmod(0o700)
    (out/'COLCON_IGNORE').touch();write(out/'protected_inputs.json',pins)
    firmware(out);rt=runtime(out,m)
    write(out/'stages.json',[dict(stage=i,**asdict(s),target_rpm=s.target_rpm,feedback_rpm=s.feedback_rpm) for i,s in enumerate(STAGES)])
    manifest=dict(offline_passed=False,identity='4WS9',runtime_src=str(rt),control_package=CONTROL,hardware_package=HARDWARE,
        firmware={},source_hashes=hashes(HERE),runtime_hashes=hashes(rt),firmware_source_hashes=hashes(out/'firmware'),
        protected_input_hashes=pins,flashed=False,hardware_started=False,network_changed=False)
    # The controlled ROS rehearsal is ported mechanically into this NEW
    # package namespace; actual control adapters are used, never hardware IO.
    rehearsal=(ROOT/'candidates/wand_rotation_hardware/ros_rehearsal.py').read_text()
    rehearsal=rehearsal.replace('mecanum_wand_4ws8','mecanum_wand_4ws9').replace('WAND_4WS8','WAND_4WS9')
    rehearsal=rehearsal.replace('config/stage4.NOT_APPROVED.json','config/stage6.NOT_APPROVED.json')
    rehearsal=rehearsal.replace(".10*math.sin(t*1.6)",".55*math.sin(t*1.8)")
    rehearsal=rehearsal.replace('state.max_wz<=.1+1e-6','state.max_wz<=c.max_yaw_rate_radps+1e-6')
    (out/'ros_rehearsal.py').write_text(rehearsal)
    manifest['source_hashes'][str(out/'ros_rehearsal.py')]=sha(out/'ros_rehearsal.py')
    manifest['source_hashes'][str(ROOT/'tools/start_car_sync_4ws9.sh')]=sha(ROOT/'tools/start_car_sync_4ws9.sh')
    write(out/'build_manifest.json',manifest);return manifest

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();generate(a.output.resolve());print('Generated NOT_APPROVED offline candidate only')
