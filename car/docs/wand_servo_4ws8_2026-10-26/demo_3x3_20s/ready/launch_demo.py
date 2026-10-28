#!/usr/bin/env python3
"""File-check default; explicit private stage4 bounded 3x3m / 20s demonstration."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

HERE=Path('/home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407/candidates/servo_4ws8_ground_pilot')
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
        user_authorization=str(user),user_authorization_sha256=sha(user),
        range_request='/home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407/candidates/servo_4ws8_demo_3x3_20s/USER_REQUEST.json',
        range_request_sha256=sha(Path('/home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407/candidates/servo_4ws8_demo_3x3_20s/USER_REQUEST.json')))
    verify_trial_review(review,4,'sign',m['firmware']['sha256'])
    if not a.launch:
        print('PASS: private stage4 .70/.80, one engagement, at most20s; fixed 3x3m clamped goals and independent body fence; no hardware opened.')
        return 0
    if not sys.stdin.isatty() or not a.flash_record:
        raise ValueError('fresh human terminal and actual full-readback record required')
    verify_flash_record(a.flash_record,a.manifest)
    print('3×3m有界演示入口，不是连续高速或电气/温升合格证。固定 .70m/s / .80m/s²，1:1、原3x3/50cm围栏保留。')
    print('准备期间可拿起/调整Sign，保持松键；按住使能时记录当前原点并开始20秒计时。可在固定3×3m区内全向跟随，边界限幅；松键结束本轮，不自动恢复。')
    print('确认车在绿色区内且原朝向正确，Sign/car01可见；3×3m和四边50cm缓冲空旷、有人可立即断电；其他小车控制程序已关。停车距离尚未完成全速实测，不要贴边高速试停。')
    phrase='4WS8_DEMO_3X3_20S_CLEAR_POWER_CUT_READY'
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
