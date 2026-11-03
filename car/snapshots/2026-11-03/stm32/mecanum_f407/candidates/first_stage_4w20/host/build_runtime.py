#!/usr/bin/env python3
"""Generate private 4W20 packages. No ROS executor, socket, or flashing.

Reuse the verified generator but produce new packages/domain and a different
firmware contract. No physical measurements are filled in or marked passed.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
HIGH=HERE.parents[1]/'high_speed_4wgd'


def replace(path,old,new,count=1):
    s=path.read_text()
    if s.count(old)!=count: raise RuntimeError('unexpected source fragment: '+str(path)+' '+old)
    path.write_text(s.replace(old,new))


def generate(output,snapshot):
    spec=importlib.util.spec_from_file_location('verified_highspeed_generator',HIGH/'host/build_isolated_runtime.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.generate(output,snapshot)
    pairs=(('mecanum_wand_highspeed_control','mecanum_wand_stage020_control'),
           ('mecanum_wand_highspeed_hardware','mecanum_wand_stage020_hardware'),
           ('WAND_HIGHSPEED_','WAND_STAGE020_'),
           ('/wand_highspeed_acceptance','/wand_stage020_acceptance'),
           ('wand_highspeed_acceptance','wand_stage020_acceptance'),
           ('4WGD','4W20'),('domain95','domain96'),('Domain95','Domain96'),
           ("95, '/wand_stage020_acceptance'","96, '/wand_stage020_acceptance'"),
           ('域95 高速首档','域96 独立0.20首档'),
           ('HIGH_SPEED_020_REVIEWED_AND_PHYSICAL_POWER_CUT_READY',
            'STAGE020_35PWM_CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY'))
    for p in list(output.rglob('*')):
        if p.is_file():
            s=p.read_text()
            for old,new in pairs:s=s.replace(old,new)
            p.write_text(s)
    for p in sorted(output.rglob('*'),key=lambda p:len(p.parts),reverse=True):
        name=p.name
        for old,new in pairs:name=name.replace(old,new)
        if name!=p.name:p.rename(p.with_name(name))
    control=output/'mecanum_wand_stage020_control/mecanum_wand_stage020_control'
    hardware=output/'mecanum_wand_stage020_hardware/mecanum_wand_stage020_hardware'
    (hardware/'stage020_quantization.py').write_text((HERE/'stage020_quantization.py').read_text())
    replace(hardware/'continuous_bridge.py',
        'from mecanum_vicon_control.mc_link_session import McLinkSession, SessionMismatchError',
        'from mecanum_vicon_control.mc_link_session import SessionMismatchError\n'
        'from .stage020_quantization import FirstStageSession as McLinkSession')
    commission=hardware/'commissioning.py'
    text=commission.read_text()
    start=text.index("        if mode == 'hardware':\n            high =")
    end=text.index('        self.wand_reference_margin_m = 0.',start)
    text=text[:start]+'''        if mode == 'hardware':
            confirmation = self.profile.get('stage020_confirmations', {})
            required = ('first_stage_only', 'clear_area_and_power_cut_ready_now')
            if any(confirmation.get(k) is not True for k in required):
                raise ValueError('STAGE020_SITE_CONFIRMATION_REQUIRED')
            # Explicit restricted commissioning policy, NOT measured braking,
            # electrical or thermal approval. Full 4WGD remains blocked.
            policy = self.profile.get('stage020_policy', {})
            if (policy.get('kind') != 'restricted_020_existing_35_percent_envelope'
                    or self.config.max_speed_mps != .20 or self.config.max_accel_mps2 != .08
                    or hw.get('motor_target_rpm_cap') != 100 or hw.get('pwm_cap_permille') != 350
                    or hw.get('wheel_rim_limit_mps') != .30 or hw.get('target_limit_cps') != 1700):
                raise ValueError('4W20 first-stage envelope mismatch')
''' + text[end:]
    commission.write_text(text)
    contract=hardware/'high_speed_contract.py'
    replace(contract,'0x44475734','0x30325734')
    replace(contract,'TARGET_RPM = 400.','TARGET_RPM = 100.')
    replace(contract,'FEEDBACK_RPM = 440.','FEEDBACK_RPM = 120.')
    replace(contract,'max_pwm_permille: int = 800','max_pwm_permille: int = 350')
    replace(contract,'(.20,.08,2.,400.,440.,800,300,600,.20,150.,120.)',
            '(.20,.08,2.,100.,120.,350,300,600,.20,150.,120.)')
    replace(contract,'STAGES_MPS = (.20, .30, .50, .75, 1.0, 1.20)','STAGES_MPS = (.20,)')
    replace(contract,'target_rpm=400.','target_rpm=100.')
    replace(contract,'max(CPR)*400/60/800','max(CPR)*100/60/800')
    replace(contract,'zip(body,(1256,1256,200))','zip(body,(200,200,100))')
    replace(contract,'    for i,(requested,effective_cps,g) in enumerate(zip(target,effective,directions)):',
            "    if math.hypot(body[0],body[1])>201: raise ProtocolError('4W20 vector envelope')\n"
            '    for i,(requested,effective_cps,g) in enumerate(zip(target,effective,directions)):')
    replace(contract,'limit=CPR[i]*400/60','limit=min(1700.,CPR[i]*100/60)')
    replace(contract,'abs(d)>800','abs(d)>350')
    replace(contract,'abs(snapshot[8])>800','abs(snapshot[8])>350')
    # The generated file is a fixed first-stage contract, not a speed selector.
    replace(control/'operator_console.py','高速首档0.20m/s（审核后）','0.20m/s首档（35%PWM）')
    config=output/'mecanum_wand_stage020_hardware/config'
    old=config/'high_speed_020.REVIEW_REQUIRED.json'
    profile=json.loads(old.read_text())
    profile['hardware'].update(required_motion_tag='4W20',motor_target_rpm_cap=100,
        pwm_cap_permille=350,wheel_rim_limit_mps=.30,target_limit_cps=1700)
    profile.pop('high_speed_confirmations');profile.pop('stopping_review')
    profile['stage020_confirmations']=dict(first_stage_only=True,clear_area_and_power_cut_ready_now=False)
    profile['stage020_policy']=dict(kind='restricted_020_existing_35_percent_envelope',
        body_vector_limit_mps=.20,body_yaw_limit_radps=.10,pwm_cap_permille=350,
        braking_current_thermal_acceptance='DEFERRED; NOT MEASURED OR PASSED',
        note='Separate bounded first stage requested 2026-10-23; full 400rpm/80% firmware remains offline')
    profile['evidence']['change_scope']='4W20 limited 0.20m/s /35% PWM commissioning; no high-speed acceptance claim'
    old.unlink()
    (config/'stage020.SITE_CONFIRMATION_REQUIRED.json').write_text(json.dumps(profile,indent=2)+'\n')
    for package in ('mecanum_wand_stage020_control','mecanum_wand_stage020_hardware'):
        (output/package/'README.md').write_text(
            '# 4W20 bounded first stage\n\nDomain96, /wand_stage020_acceptance.\n'
            '0.20m/s VECTOR cap, 0.08m/s2, yaw0.10rad/s, 35% PWM, strict4W20-only.\n'
            'Generated from verified sources, never edits the working low-speed/rectangle packages.\n'
            'Profile defaults to blocking hardware until fresh site/power-cut confirmation.\n'
            'Braking/current/temperature acceptance is deferred, not passed.\n'
            'No ROS nodes, motor commands, firmware writes or network changes are run by this generator.\n')
    report=dict(source_snapshot=str(snapshot.resolve()),firmware_tag='4W20',domain=96,
        namespace='/wand_stage020_acceptance',speed_mps=.20,pwm_permille=350,
        hardware_started=False,profile_ready=False,full_highspeed_review_bypassed=False,
        deferred_tests=['measured_stop','current','temperature'])
    (output/'generation_report.json').write_text(json.dumps(report,indent=2)+'\n')
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--accepted-snapshot',required=True,type=Path)
    a=p.parse_args();generate(a.output.resolve(),a.accepted_snapshot)
