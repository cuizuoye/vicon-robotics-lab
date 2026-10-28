#!/usr/bin/env python3
"""Generate isolated first-stage test helpers and a fresh confirmed profile.

No ROS/network/keyboard/hardware actions. Requires explicit current human site
confirmation; never marks deferred braking/electrical/thermal tests passed.
Original tools and profiles are left byte-for-byte unchanged.
"""
import argparse
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
WORKSPACE=HERE.parents[4]


def prepare(output,profile,confirmation):
    if confirmation!='CLEAR_AREA_AND_IMMEDIATE_BATTERY_CUT_CONFIRMED':
        raise ValueError('fresh human confirmation required')
    value=json.loads(profile.read_text())
    if (value['hardware']['required_motion_tag']!='4W20'
            or value['hardware']['pwm_cap_permille']!=350
            or value['safety']['max_speed_mps']!=.20
            or value['stage020_confirmations']['first_stage_only'] is not True):
        raise ValueError('wrong first-stage profile')
    output.mkdir(parents=True,exist_ok=False)
    value['stage020_confirmations']['clear_area_and_power_cut_ready_now']=True
    value['evidence']['current_site_confirmation']={
        'date':'2026-10-23',
        'user':'Tenda reconnected, powered, original start/heading, clear area; immediate battery cut explicitly confirmed',
        'scope':'0.20m/s 35%PWM controlled test only; no higher-speed acceptance',
        'source_profile':str(profile.resolve()),
        'source_profile_sha256':hashlib.sha256(profile.read_bytes()).hexdigest()}
    approved=output/'stage020.site_confirmed.json'
    approved.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    records=[]
    for source_name,target_name in (
            ('wand_hardware_idle_preflight.py','stage020_idle_preflight.py'),
            ('wand_sign_acceptance_observer.py','stage020_observer.py')):
        source=WORKSPACE/'tools'/source_name
        text=source.read_text()
        for old,new in (
                ('mecanum_wand_control','mecanum_wand_stage020_control'),
                ('mecanum_wand_hardware','mecanum_wand_stage020_hardware'),
                ('wand_sign_acceptance.launch.py','wand_stage020_acceptance.launch.py'),
                ('domain94','domain96'),('/wand_acceptance','/wand_stage020_acceptance'),
                ("ROS_DOMAIN_ID='94'","ROS_DOMAIN_ID='96'"),
                ("os.environ['ROS_DOMAIN_ID'] = '94'","os.environ['ROS_DOMAIN_ID'] = '96'"),
                ('rclpy.init(domain_id=94)','rclpy.init(domain_id=96)'),
                ('self.max_speed <= .05+1e-9','self.max_speed <= .20+1e-9')):
            text=text.replace(old,new)
        if 'domain_id=94' in text or 'from mecanum_wand_hardware' in text:
            raise ValueError('incomplete first-stage adaptation')
        target=output/target_name
        target.write_text(text)
        records.append(dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                            generated=str(target),sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    result=dict(profile=str(approved.resolve()),
        approved_sha256=hashlib.sha256(approved.read_bytes()).hexdigest(),
        generated=records,domain=96,namespace='/wand_stage020_acceptance',
        hardware_actions_performed=False,original_files_modified=False,
        braking_current_thermal='DEFERRED; NOT MEASURED OR PASSED')
    (output/'preparation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--site-confirmation',required=True)
    args=parser.parse_args();prepare(args.output,args.profile,args.site_confirmation)
