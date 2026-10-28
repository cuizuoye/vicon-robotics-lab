#!/usr/bin/env python3
"""File-check default; explicit private stage4 small-displacement ground pilot."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

HERE=Path(__file__).resolve().parent
BASE=HERE.parent/'servo_4ws8'
ROOT=HERE.parents[3]
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--flash-record',type=Path)
    parser.add_argument('--launch',action='store_true')
    a=parser.parse_args();m=check(a.manifest)
    for package in (m['control_package'],m['hardware_package']):
        sys.path.insert(0,str(Path(m['runtime_src'])/package))
    from mecanum_wand_4ws8_control.pilot_core import POLICY
    from mecanum_wand_4ws8_hardware.pilot_authorization import KIND
    from mecanum_wand_4ws8_hardware.authorization import verify_flash_record,verify_trial_review,verify_runtime_profile
    from mecanum_wand_4ws8_hardware.commissioning import Runtime,CONSENT,DOMAIN
    if DOMAIN!=120:raise ValueError('private pilot runtime required')
    evidence=ROOT/'docs/wand_servo_4ws8_2026-10-23/requested_suspended_050_070/host_ready/stage4_suspended_l79694pa/bench/SESSION_RESULT.json'
    user=HERE/'USER_AUTHORIZATION.json'
    review=dict(kind=KIND,identity='4WS8',authorized_stage=4,purpose='sign',human_authorized=True,
        image_sha256=m['firmware']['sha256'],pilot_policy=POLICY,
        continuous_ground_qualified=False,electrical_thermal_trial_reviewed=False,
        suspended_evidence=str(evidence),suspended_evidence_sha256=sha(evidence),
        user_authorization=str(user),user_authorization_sha256=sha(user))
    verify_trial_review(review,4,'sign',m['firmware']['sha256'])
    if not a.launch:
        print('PASS: private stage4 .70/.80, one engagement, at most2s; hand/goal15cm, car20cm, path25cm; no hardware opened.')
        return 0
    if not sys.stdin.isatty() or not a.flash_record:
        raise ValueError('fresh human terminal and actual full-readback record required')
    verify_flash_record(a.flash_record,a.manifest)
    print('首轮有限落地观察，不是连续高速合格证。固定 .70m/s / .80m/s²，1:1、原3x3/50cm围栏保留。')
    print('仅一次按住使能，最长2秒；杆只前移约5cm，不做返回或连续跟随。松键/定位/反馈故障立即撤能。')
    print('确认原起点、原朝向、Sign/car01可见、场地空旷、有人可立即断电；其他小车控制程序已关。')
    phrase='4WS8_FINITE_GROUND_PILOT_CLEAR_POWER_CUT_READY'
    if input('现场确认后输入 '+phrase+'：').strip()!=phrase:raise ValueError('site not freshly confirmed')
    profile=json.loads(Path(m['draft_profiles'][4]).read_text())
    profile['trial_review']=review
    profile['staged_runtime_approval']=dict(purpose='sign',issued_utc_s=time.time(),
        clear_and_power_cut_ready_now=True,flash_record=str(a.flash_record.resolve()),build_manifest=str(a.manifest.resolve()))
    profile['confirmations']['fixed_arena_checked']=True
    for key in profile['arena_confirmations']:profile['arena_confirmations'][key]=True
    verify_runtime_profile(profile,'sign')
    folder=Path(tempfile.mkdtemp(prefix='ground_stage4_single_',dir=a.manifest.parent))
    approved=folder/'site_confirmed.json'
    approved.write_text(json.dumps(profile,indent=2,ensure_ascii=False)+'\n');approved.chmod(0o600)
    Runtime('hardware',str(approved),sha(approved),CONSENT)
    env=os.environ.copy()
    env.update(ROS_DOMAIN_ID='120',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        ROS_LOG_DIR=str(folder/'ros_logs'),WAND_PILOT_TRACE=str(folder/'observation.jsonl'),
        WAND_4WS8_RAW_RECEIVE_TRACE=str(folder/'mcu_receive.jsonl'))
    Path(env['ROS_LOG_DIR']).mkdir(mode=0o700)
    print('EVIDENCE_DIR='+str(folder),flush=True)
    os.execvpe('ros2',['ros2','launch',m['hardware_package'],'wand_4ws8_staged.launch.py',
        'mode:=hardware','gui:=true','profile:='+str(approved),'approved_sha256:='+sha(approved),
        'hardware_consent:='+CONSENT],env)


if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,KeyError,TypeError) as error:
        print('BLOCKED: '+str(error),file=sys.stderr);raise SystemExit(2)
