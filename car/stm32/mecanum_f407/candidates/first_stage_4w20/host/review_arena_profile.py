#!/usr/bin/env python3
"""Validate pinned arena data; optionally record human site confirmation.

Never creates ROS nodes/sockets or enables hardware. Approval changes ONLY
current-site booleans, never geometry, speed, source policy or physical tests.
"""
import argparse
import hashlib
import json
from pathlib import Path

from mecanum_wand_one_to_one_hardware.commissioning import Runtime,CONSENT

TEMPLATE=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_one_to_one_3x3_2026-10-23/verified_runtime/src/'
              'mecanum_wand_one_to_one_hardware/config/one_to_one.3x3_BUFFER050.SITE_CONFIRMATION_REQUIRED.json')
DIGEST='cc05ce545b92ce4303a94bda73264869fa1e9573929089b03b8af285771b99c3'
PHRASE='3X3_CLEAR_TRACKED_POWER_CUT_READY'


def pinned_profile():
    raw=TEMPLATE.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=DIGEST:raise ValueError('pinned template changed; review before use')
    runtime=Runtime('shadow',str(TEMPLATE))
    return json.loads(raw),runtime


def prepare(output,confirmation):
    if confirmation!=PHRASE:raise ValueError('explicit current human site confirmation required')
    profile,runtime=pinned_profile()
    profile['confirmations']['fixed_arena_checked']=True
    profile['stage020_confirmations']['clear_area_and_power_cut_ready_now']=True
    profile['arena_confirmations']=dict.fromkeys(profile['arena_confirmations'],True)
    profile['evidence']['current_site_confirmation']='Human typed '+PHRASE+'; not an automated coverage or braking measurement'
    output.mkdir(parents=True,exist_ok=False)
    target=output/'site_confirmed.json'
    target.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
    digest=hashlib.sha256(target.read_bytes()).hexdigest()
    Runtime('hardware',str(target),digest,CONSENT)  # Pure data validator, no ROS/UDP.
    return target,digest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path)
    p.add_argument('--human-confirmation',default='')
    a=p.parse_args()
    if a.output:
        target,digest=prepare(a.output.resolve(),a.human_confirmation)
        print(json.dumps(dict(profile=str(target),sha256=digest,hardware_started=False),indent=2))
    else:
        profile,r=pinned_profile()
        print(json.dumps(dict(status='DATA CHECK ONLY; NO ROS OR HARDWARE',mapping_scale=1.,
            car_center_forward_m=[0,3],car_center_left_m=[0,3],buffer_each_side_m=.50,
            preserved_extra_margin_m=.20,max_speed_mps=r.config.max_speed_mps,
            physical_clear_extent_m=[4.19,4.21],site_approved=False,
            physical_stop_distance_measured=False,template_sha256=DIGEST),indent=2))
