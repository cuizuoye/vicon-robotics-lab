#!/usr/bin/env python3
"""File-check by default. No programmer. Launch needs readback + human trial/site review."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from stages import STAGES,get_stage

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def check(manifest_path):
    m=json.loads(Path(manifest_path).read_text())
    if (m.get('preparation_passed') is not True or m['firmware']['identity']!='4WS8' or
        m['firmware']['authority_version']!=4 or m['firmware']['supported_stages']!=list(range(7))):
        raise ValueError('independent staged preparation incomplete')
    if sha(m['firmware']['path'])!=m['firmware']['sha256']:raise ValueError('image drift')
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):
        for p,v in m[group].items():
            if sha(p)!=v:raise ValueError(group+' drift: '+p)
    return m

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--purpose',choices=('sign','suspended'),required=True)
    p.add_argument('--stage',type=int,default=0)
    action=p.add_mutually_exclusive_group()
    action.add_argument('--launch',action='store_true');action.add_argument('--check',action='store_true')
    p.add_argument('--flash-record',type=Path);p.add_argument('--trial-review',type=Path)
    a=p.parse_args();limits=get_stage(a.stage);m=check(a.manifest)
    if not a.launch:
        print(f'PASS: 4WS8 v4 档{a.stage}: {limits.speed:.2f}m/s, {limits.accel:.2f}m/s², PWM≤{limits.duty/10:g}%。')
        print('仅文件核对；未打开设备、网络或控制节点。尚无本候选刷写/实测验收。')
        return 0
    if not sys.stdin.isatty():raise ValueError('human terminal required; nothing opened')
    if not a.flash_record or not a.trial_review:raise ValueError('actual flash readback and per-stage trial review required')
    for package in (m['control_package'],m['hardware_package']):
        sys.path.insert(0,str(Path(m['runtime_src'])/package))
    from mecanum_wand_4ws8_hardware.authorization import verify_flash_record,verify_trial_review,verify_runtime_profile
    verify_flash_record(a.flash_record,a.manifest)
    review=json.loads(a.trial_review.read_text())
    verify_trial_review(review,a.stage,a.purpose,m['firmware']['sha256'])
    print(f'固定档{a.stage}: {limits.speed:.2f}m/s / {limits.accel:.2f}m/s² / PWM≤{limits.duty/10:g}%；不会自动使能或升档。')
    if a.purpose=='suspended':
        print('确认四轮完全悬空、支撑固定、不碰线，其他小车控制程序已关，有人能立即断电；每次最多5秒。')
    else:
        print('确认原起点及朝向，固定3×3m和四边50cm缓冲空旷且在Vicon内；Sign/car01可见，其他控制程序已关，有人能立即断电。')
        print('高档地面入口必须先有本档停车包络与电气/温升复核记录；悬空通过不等于落地合格。')
    phrase=f'4WS8_STAGE_{a.stage}_{a.purpose.upper()}_CLEAR_POWER_CUT_READY'
    if input('全部满足后输入 '+phrase+'：').strip()!=phrase:raise ValueError('site not confirmed')
    profile=json.loads(Path(m['draft_profiles'][a.stage]).read_text())
    profile['trial_review']=review
    profile['staged_runtime_approval']=dict(purpose=a.purpose,issued_utc_s=time.time(),
        clear_and_power_cut_ready_now=True,four_wheels_secured_clear_now=a.purpose=='suspended',
        flash_record=str(a.flash_record.resolve()),build_manifest=str(a.manifest.resolve()))
    if a.purpose=='sign':
        profile['confirmations']['fixed_arena_checked']=True
        for k in profile['arena_confirmations']:profile['arena_confirmations'][k]=True
    verify_runtime_profile(profile,a.purpose) # Still before Node/UDP construction.
    folder=Path(tempfile.mkdtemp(prefix=f'stage{a.stage}_{a.purpose}_',dir=Path(a.manifest).parent))
    approved=folder/'site_confirmed.json';approved.write_text(json.dumps(profile,indent=2,ensure_ascii=False)+'\n')
    os.chmod(approved,0o600)
    if a.purpose=='suspended':
        from mecanum_wand_4ws8_hardware.suspended_console import run
        run(approved,folder/'bench')
    else:
        from mecanum_wand_4ws8_hardware.commissioning import Runtime,CONSENT
        Runtime('hardware',str(approved),sha(approved),CONSENT)
        env=os.environ.copy();env['ROS_DOMAIN_ID']='119';env['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
        env['ROS_LOG_DIR']=str(folder/'ros_logs');Path(env['ROS_LOG_DIR']).mkdir(mode=0o700)
        os.execvpe('ros2',['ros2','launch',m['hardware_package'],'wand_4ws8_staged.launch.py',
            'mode:=hardware','gui:=true','profile:='+str(approved),'approved_sha256:='+sha(approved),
            'hardware_consent:='+CONSENT],env)
    return 0

if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,KeyError,TypeError) as error:
        print('BLOCKED: '+str(error),file=sys.stderr);raise SystemExit(2)
