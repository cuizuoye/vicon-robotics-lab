#!/usr/bin/env python3
"""Default offline check. Actual launch requires future flash evidence + TTY.

Contains no programming command. Never reads devices or changes networking.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from authorization import verify_flash_record

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def check(manifest_path):
    path=Path(manifest_path).resolve();m=json.loads(path.read_text())
    if not m['preparation_passed'] or m['firmware']['stage']!=0 or m['firmware']['pwm_limit']!=350:
        raise ValueError('preparation did not pass bounded stage0 review')
    if sha(m['firmware']['path'])!=m['firmware']['sha256']:raise ValueError('image drift')
    for p,v in m['runtime_hashes'].items():
        if sha(p)!=v:raise ValueError('private runtime drift: '+p)
    for p,v in m['source_hashes'].items():
        if sha(p)!=v:raise ValueError('candidate source drift: '+p)
    return m

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--purpose',choices=('sign','suspended'),required=True)
    p.add_argument('--launch',action='store_true');p.add_argument('--flash-record',type=Path)
    a=p.parse_args()
    m=check(a.manifest)
    if not a.launch:
        print('PASS: 4WS7 v3首档0.20m/s、35%PWM软件准备完整；未连接设备或打开控制入口。')
        print('实机启动仍需传入实际刷后读回记录并作当次现场确认；本次只检查文件。')
        return 0
    if not sys.stdin.isatty():raise ValueError('hardware launch requires a human terminal')
    if a.flash_record is None:raise ValueError('POST_FLASH_RECORD_REQUIRED; no hardware opened')
    verify_flash_record(a.flash_record,a.manifest)
    if a.purpose=='suspended':
        phrase='4WS7_STAGE0_SUSPENDED_SECURED_POWER_CUT_READY'
        print('确认：车架支撑固定、四轮完全悬空且不碰线；其他小车控制程序已关闭；有人能立即断电。')
        print('本入口不使用Sign；0.20m/s上限，每次最长5秒；准备不转轮，必须按住空格。')
    else:
        phrase='4WS7_STAGE0_SIGN_CLEAR_POWER_CUT_READY'
        print('确认：已完成悬空初筛后才使用此落地入口；原起点/朝向匹配，3×3m及四边50cm缓冲在Vicon内且空旷。')
        print('Sign/car01无遮挡，有人能立即断电，其他小车控制程序已关闭；首档0.20m/s/35%PWM，不自动使能。')
    if input('全部满足后输入 '+phrase+'：').strip()!=phrase:raise ValueError('not confirmed; no hardware opened')
    profile=json.loads(Path(m['draft_profile']).read_text())
    profile['stage0_runtime_approval']=dict(purpose=a.purpose,issued_utc_s=time.time(),
        clear_and_power_cut_ready_now=True,four_wheels_secured_clear_now=a.purpose=='suspended',
        flash_record=str(a.flash_record.resolve()),build_manifest=str(a.manifest.resolve()))
    if a.purpose=='sign':
        profile['confirmations']['fixed_arena_checked']=True
        profile['stage020_confirmations']['clear_area_and_power_cut_ready_now']=True
        for k in profile['arena_confirmations']:profile['arena_confirmations'][k]=True
    folder=Path(tempfile.mkdtemp(prefix='session_',dir=Path(a.manifest).parent))
    approved=folder/'site_confirmed.json';approved.write_text(json.dumps(profile,indent=2,ensure_ascii=False)+'\n')
    os.chmod(approved,0o600)
    for package in (m['control_package'],m['hardware_package']):sys.path.insert(0,str(Path(m['runtime_src'])/package))
    if a.purpose=='suspended':
        from mecanum_wand_4ws7_hardware.suspended_console import run
        run(approved,folder/'bench')
    else:
        from mecanum_wand_4ws7_hardware.commissioning import Runtime,CONSENT
        Runtime('hardware',str(approved),sha(approved),CONSENT)
        env=os.environ.copy();env['ROS_DOMAIN_ID']='118';env['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
        env['ROS_LOG_DIR']=str(folder/'ros_logs')
        Path(env['ROS_LOG_DIR']).mkdir(mode=0o700)
        os.execvpe('ros2',['ros2','launch',m['hardware_package'],'wand_4ws7_stage0.launch.py',
            'mode:=hardware','gui:=true','profile:='+str(approved),'approved_sha256:='+sha(approved),
            'hardware_consent:='+CONSENT],env)
    return 0

if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,KeyError,TypeError) as error:
        print('BLOCKED: '+str(error),file=sys.stderr);raise SystemExit(2)
