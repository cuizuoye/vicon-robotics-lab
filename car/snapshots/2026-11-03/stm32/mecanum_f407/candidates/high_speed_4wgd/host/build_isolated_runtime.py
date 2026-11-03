#!/usr/bin/env python3
"""Build-source generation ONLY; never runs ROS nodes or opens hardware.

Fork the hash-verified working implementation into differently named packages,
domain95 and a private install prefix. Original source/entry points are not
edited. All policy edits require exact expected source fragments/counts.
"""
import argparse
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
WORKSPACE=HERE.parents[4]
NAMES={'mecanum_wand_control':'mecanum_wand_highspeed_control',
       'mecanum_wand_hardware':'mecanum_wand_highspeed_hardware'}


def rename(text):
    for old,new in NAMES.items():text=text.replace(old,new)
    return text.replace('WAND_ACCEPTANCE_','WAND_HIGHSPEED_').replace(
        'WAND_SAFE_CHANNEL_KEY','WAND_HIGHSPEED_CHANNEL_KEY').replace(
        'WAND_RAW_RECEIVE_TRACE','WAND_HIGHSPEED_RAW_RECEIVE_TRACE')


def generate(destination,snapshot):
    expected=json.loads(snapshot.read_text())
    for relative,digest in expected.items():
        if relative.startswith(('src/mecanum_wand_control/','src/mecanum_wand_hardware/')):
            if hashlib.sha256((WORKSPACE/relative).read_bytes()).hexdigest()!=digest:
                raise RuntimeError('accepted host source drift: '+relative)
    destination.mkdir(parents=True,exist_ok=False)
    changes=[]
    for old,new in NAMES.items():
        source=WORKSPACE/'src'/old
        for p in source.rglob('*'):
            if not p.is_file():continue
            relative=p.relative_to(source)
            if relative.parts[0] in ('test','config') or '__pycache__' in relative.parts or '.egg-info' in str(relative):continue
            if p.suffix not in ('.py','.xml','.cfg','.md') and relative.parts[0]!='resource':continue
            if relative.parts[0]=='launch' and p.name!='wand_sign_acceptance.launch.py':continue
            target_relative=Path(rename(str(relative)))
            if p.name=='wand_sign_acceptance.launch.py':target_relative=target_relative.with_name('wand_highspeed_acceptance.launch.py')
            target=destination/new/target_relative
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text(rename(p.read_text()))
        (destination/new/'README.md').write_text(
            '# Independent 4WGD first stage (0.20 m/s)\n\n'
            'Generated from the verified low-speed implementation; original packages are unchanged.\n'
            'Domain95, /wand_highspeed_acceptance, strict 4WGD diagnostics only.\n'
            'The included REVIEW_REQUIRED profile deliberately rejects real hardware before Node/socket construction.\n'
            'Physical stopping margin, current/thermal review, explicit flash migration and operator approval remain required.\n'
            'Use the source candidate README for evidence and reproduction; do not copy old 4WGC commands into this entry.\n')
    control=destination/NAMES['mecanum_wand_control']/NAMES['mecanum_wand_control']
    hardware=destination/NAMES['mecanum_wand_hardware']/NAMES['mecanum_wand_hardware']
    def edit(path,old,new,count=1):
        text=path.read_text()
        if text.count(old)!=count:raise RuntimeError('unexpected patch count '+str(path)+': '+old)
        path.write_text(text.replace(old,new))
        changes.append(dict(file=str(path.relative_to(destination)),old=old,new=new,count=count))
    edit(control/'safety_core.py','self.max_speed_mps <= .15','self.max_speed_mps <= .20')
    edit(control/'safety_wire.py','> .05+1e-9','> .20+1e-9')
    edit(control/'operator_console.py',"'域94  ·  2倍映射  ·  ≤0.05 m/s  ·  '",
         "'域95 高速首档  ·  2倍映射  ·  ≤0.20 m/s  ·  '")
    edit(control/'operator_console.py',"'hardware': '真实 Sign → 实车低速验收'",
         "'hardware': '真实 Sign → 高速首档0.20m/s（审核后）'")
    commission=hardware/'commissioning.py'
    edit(commission,"94, '/wand_acceptance', 'map'","95, '/wand_highspeed_acceptance', 'map'")
    edit(commission,"CONSENT = 'CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY'",
         "CONSENT = 'HIGH_SPEED_020_REVIEWED_AND_PHYSICAL_POWER_CUT_READY'")
    edit(commission,"not in ('4WGB', '4WGC')","not in ('4WGD',)")
    edit(commission,"hw.get('required_motion_tag', '4WGB')","hw.get('required_motion_tag', '4WGD')")
    edit(commission,"self.motion_tag == '4WGC'","self.motion_tag == '4WGD'",2)
    edit(commission,"self.motion_tag != '4WGB'","self.motion_tag != '4WGD'")
    edit(commission,'self.config.max_speed_mps > .05','self.config.max_speed_mps > .20')
    edit(commission,'> .05+1e-9','> .20+1e-9')
    edit(commission,'        self.wand_reference_margin_m = 0.',
        '''        if mode == 'hardware':
            high = self.profile.get('high_speed_confirmations', {})
            required = ('fixed_arena_with_stopping_margin_checked', 'coast_stop_measured_at_020',
                        'motor_current_and_driver_temperature_checked', 'physical_power_cut_ready_now')
            if any(high.get(k) is not True for k in required):
                raise ValueError('HIGH_SPEED_REVIEW_INCOMPLETE: do not open hardware transport')
            review = self.profile.get('stopping_review', {})
            speed = finite_number(review.get('test_speed_mps'))
            coast = finite_number(review.get('measured_coast_distance_m'))
            if speed < .20 or coast < 0 or self.config.boundary_margin_m < .20+.20*.60+coast:
                raise ValueError('insufficient measured high-speed stopping margin')
            if (self.config.max_speed_mps != .20 or self.config.max_accel_mps2 != .08
                    or hw.get('motor_target_rpm_cap') != 400 or hw.get('pwm_cap_permille') != 800):
                raise ValueError('this independent entry is ONLY the reviewed 0.20m/s first stage')
        self.wand_reference_margin_m = 0.''')
    # Diagnostic and error strings only; no aliasing a 4WGD packet to 4WGC.
    text=commission.read_text().replace('4WGB/4WGC','4WGD').replace('4WGC requires','4WGD requires').replace(
        '0.05m/s and 0.10rad/s','0.20m/s and 0.10rad/s').replace('domain94','domain95')
    commission.write_text(text)
    edit(hardware/'nodes.py','self.max_vx_mps != .05 or self.max_vy_mps != .05',
         'self.max_vx_mps != .20 or self.max_vy_mps != .20')
    launch=destination/NAMES['mecanum_wand_hardware']/'launch/wand_highspeed_acceptance.launch.py'
    edit(launch,"'max_vx_mps': .05, 'max_vy_mps': .05","'max_vx_mps': .20, 'max_vy_mps': .20")
    # No public/global allowlist is changed. This decoder accepts D ONLY.
    (hardware/'high_speed_contract.py').write_text(rename((HERE/'high_speed_4wgd.py').read_text()))
    (hardware/'continuous_status.py').write_text('''"""Independent strict 4WGD-only adapter; never accept C/B aliases."""
from .high_speed_contract import decode_high_speed_status as decode_motion_status
MOTION_TAGS=frozenset(('4WGD',))
MOTION_LINEAR_LIMITS={'4WGD':.20}
MOTION_WIRE_TIMEOUTS={'4WGD':600}
def disabled_motion_status_ready(s):
    return bool(isinstance(s,dict) and s.get('tag')=='4WGD' and s.get('permitted')==0
        and s.get('stationary_ms',0)>=500 and s.get('output_authority_version')==1
        and s.get('reversing_mask')==0 and all(not any(s.get(k,[1])) for k in
        ('compares','raw_duty','target_cps','effective_cps','granted_direction','filtered_cps','body_milli')))
''')
    profile=json.loads((WORKSPACE/'src/mecanum_wand_hardware/config/hardware_acceptance.corner_2x2_4wgc_governed_2026-10-23.json').read_text())
    profile['hardware'].update(required_motion_tag='4WGD',motor_target_rpm_cap=400,pwm_cap_permille=800)
    profile['safety']['max_speed_mps']=.20
    profile['high_speed_confirmations']=dict.fromkeys(('fixed_arena_with_stopping_margin_checked',
        'coast_stop_measured_at_020','motor_current_and_driver_temperature_checked','physical_power_cut_ready_now'),False)
    profile['stopping_review']=dict(test_speed_mps=None,measured_coast_distance_m=None,
        note='Must be measured/reviewed; current fence/margin is copied for OFFLINE preview ONLY, not approved for high speed')
    profile['evidence']['change_scope']='Independent 4WGD 0.20m/s first-stage draft; NOT hardware approved'
    config=destination/NAMES['mecanum_wand_hardware']/'config'
    config.mkdir();(config/'high_speed_020.REVIEW_REQUIRED.json').write_text(json.dumps(profile,indent=2)+'\n')
    (destination/'generation_report.json').write_text(json.dumps(dict(
        source_snapshot=str(snapshot.resolve()),packages=NAMES,domain=95,
        namespace='/wand_highspeed_acceptance',hardware_approval=False,
        exact_policy_changes=changes,original_sources_edited=False),indent=2)+'\n')
    return destination


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--accepted-snapshot',type=Path,required=True)
    a=p.parse_args();generate(a.output.resolve(),a.accepted_snapshot)
