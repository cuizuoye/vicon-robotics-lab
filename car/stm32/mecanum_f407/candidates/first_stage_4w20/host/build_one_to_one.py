#!/usr/bin/env python3
"""Generate isolated 1:1 host software; never launch or change hardware.

The requested 3x3m/0.50m fence is a planning artifact only. The independently
generated 1:1 runtime retains the existing fence until the separate arena
extension is implemented/reviewed. An already running 2x console is untouched.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def edit(path,old,new,count=1):
    text=path.read_text()
    if text.count(old)!=count:raise RuntimeError('source drift: '+str(path)+' '+old)
    path.write_text(text.replace(old,new))


def generate(output,snapshot):
    spec=importlib.util.spec_from_file_location('stage020_generator',HERE/'build_runtime.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.generate(output,snapshot)
    pairs=(('mecanum_wand_stage020_control','mecanum_wand_one_to_one_control'),
           ('mecanum_wand_stage020_hardware','mecanum_wand_one_to_one_hardware'),
           ('WAND_STAGE020_','WAND_ONE_TO_ONE_'),
           ('wand_stage020_acceptance','wand_one_to_one_acceptance'),
           ("96, '/wand_one_to_one_acceptance'","97, '/wand_one_to_one_acceptance'"),
           ('domain96','domain97'),('Domain96','Domain97'),('域96','域97'),
           ('STAGE020_35PWM_CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY',
            'ONE_TO_ONE_020_CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY'))
    for p in list(output.rglob('*')):
        if p.is_file():
            text=p.read_text()
            for old,new in pairs:text=text.replace(old,new)
            p.write_text(text)
    for p in sorted(output.rglob('*'),key=lambda p:len(p.parts),reverse=True):
        name=p.name
        for old,new in pairs:name=name.replace(old,new)
        if name!=p.name:p.rename(p.with_name(name))
    control=output/'mecanum_wand_one_to_one_control/mecanum_wand_one_to_one_control'
    hardware=output/'mecanum_wand_one_to_one_hardware/mecanum_wand_one_to_one_hardware'
    edit(control/'core.py','scale: float = 2.0','scale: float = 1.0',2)
    edit(control/'safety_core.py','return Config(scale=2.,','return Config(scale=1.,')
    edit(control/'safety_nodes.py','scale=2.0, speed_limit_mps=',
         'scale=self.core.follower.config.scale, speed_limit_mps=')
    edit(control/'operator_guidance.py','car_origin.x+2*dx, car_origin.y+2*dy',
         'car_origin.x+dx, car_origin.y+dy')
    edit(control/'operator_guidance.py','car_origin.y-ymin, ymax-car_origin.y)/2)',
         'car_origin.y-ymin, ymax-car_origin.y))')
    edit(control/'operator_guidance.py','config.max_goal_radius_m/2','config.max_goal_radius_m')
    for filename in ('operator_console.py','operator_guidance.py'):
        path=control/filename
        path.write_text(path.read_text().replace('2倍','1:1').replace('映射2倍','映射1:1'))
    contract=hardware/'high_speed_contract.py'
    edit(contract,'scale: float = 2.','scale: float = 1.')
    edit(contract,'(.20,.08,2.,100.,120.,350,300,600,.20,150.,120.)',
         '(.20,.08,1.,100.,120.,350,300,600,.20,150.,120.)')
    edit(contract,'Config(scale=2.,','Config(scale=1.,')
    edit(hardware/'commissioning.py',"        self.motion_tag = hw.get('required_motion_tag', '4W20')",
         "        if self.profile and self.profile.get('mapping_scale') != 1.0:\n"
         "            raise ValueError('this independent entry requires an explicit 1:1 profile')\n"
         "        self.motion_tag = hw.get('required_motion_tag', '4W20')")
    config=output/'mecanum_wand_one_to_one_hardware/config'
    old=config/'stage020.SITE_CONFIRMATION_REQUIRED.json'
    profile=json.loads(old.read_text());old.unlink()
    profile['mapping_scale']=1.0
    profile['input_workspace']['mapping']='1:1 XY final target; existing fixed-boundary clamp and governed reference'
    profile['evidence']['change_scope']='Independent 1:1 mapping, displays and preview; existing physical arena retained'
    profile['stage020_confirmations']['clear_area_and_power_cut_ready_now']=False
    target=config/'one_to_one.CURRENT_AREA.SITE_CONFIRMATION_REQUIRED.json'
    target.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
    planning=dict(status='PLAN ONLY; NOT A HARDWARE PROFILE; NO ACTIVE CONFIG CHANGE',
        mapping_scale=1.0,origin='recorded original rectangle start and heading',
        center_workspace_local_m=dict(forward=[0,3],left=[0,3]),
        stopping_buffer_each_side_m=.50,buffer_is_not_proven_stop_distance=True,
        car_length_m=.190,car_width_m=.210,
        aligned_physical_clear_extent_m=[4.190,4.210],
        suggested_all_yaw_clear_extent_m=[4.30,4.30],
        far_corner_distance_m=18**.5,
        current_host_radius_limit_m=3.0,
        remaining=['world-fixed oriented arena implementation and enlarged radius review',
                   'separate goal boundary from hard-stop boundary and validate deceleration',
                   'fresh site/Vicon coverage confirmation; no automatic approval'],
        battery_operator_label=dict(cells=3,capacity_mah=1080,label_c=110,
            label_product_ah_times_c_a=118.8,continuous_or_burst='UNCONFIRMED',
            bms_current='UNCONFIRMED',actual_pack_current_capability='NOT MEASURED'))
    (output/'arena_3x3_buffer050.PLAN_ONLY.json').write_text(json.dumps(planning,indent=2)+'\n')
    for package in ('mecanum_wand_one_to_one_control','mecanum_wand_one_to_one_hardware'):
        (output/package/'README.md').write_text(
            '# Independent 1:1 Sign entry\n\n'
            'Domain97; /wand_one_to_one_acceptance. 4W20 firmware unchanged.\n'
            'Mapping/preview/status 1:1; only XY, engage-time origins, return on retract.\n'
            '0.20m/s vector, 35% PWM, all existing timing/output/source guards retained.\n'
            'Current fixed arena retained. 3x3m +0.50m buffer is PLAN ONLY, not enabled.\n'
            'Hardware blocked pending a new site confirmation; never launch beside another motor bridge.\n')
    report=dict(domain=97,namespace='/wand_one_to_one_acceptance',mapping_scale=1.0,
        firmware_changed=False,hardware_started=False,existing_runtime_modified=False,
        current_arena_retained=True,arena_3x3_implemented=False,
        profile=str(target.resolve()),sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    (output/'generation_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--accepted-snapshot',required=True,type=Path)
    a=parser.parse_args();generate(a.output.resolve(),a.accepted_snapshot)
