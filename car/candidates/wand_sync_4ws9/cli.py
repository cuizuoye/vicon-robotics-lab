#!/usr/bin/env python3
"""4WS9 FILE CHECK by default; actual execution needs new-image evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from stages import get_stage

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
DEFAULT=ROOT/'docs/wand_sync_4ws9_2026-10-27/prepared'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def check(folder):
    m=json.loads((folder/'build_manifest.json').read_text())
    if m.get('offline_passed') is not True or m.get('identity')!='4WS9':raise ValueError('Offline 4WS9 preparation incomplete')
    for group in ('runtime_hashes','source_hashes','firmware_source_hashes','protected_input_hashes','evidence_hashes'):
        for p,v in m[group].items():
            if sha(p)!=v:raise ValueError(group+' drift: '+p)
    f=m['firmware']
    if (f['identity']!='4WS9' or f['authority_version']!=5 or f['diagnostic_payload_bytes']!=200
            or sha(f['path'])!=f['sha256']):raise ValueError('New firmware identity/hash mismatch')
    return m

def main():
    p=argparse.ArgumentParser();p.add_argument('--prepared',type=Path,default=DEFAULT)
    p.add_argument('--check',action='store_true');p.add_argument('--launch',action='store_true')
    p.add_argument('--stage',type=int,default=0);p.add_argument('--purpose',choices=('sign','suspended'),default='suspended')
    p.add_argument('--profile',type=Path)
    a=p.parse_args();s=get_stage(a.stage);m=check(a.prepared)
    if not a.launch:
        print(f'PASS: 4WS9 v5 / 200字节；档{a.stage}: XY {s.speed:.2f}m/s、{s.accel:.2f}m/s²，旋转{s.yaw_rate:.2f}rad/s、{s.yaw_accel:.2f}rad/s²。')
        print('仅文件核验，未初始化ROS/连接硬件/刷写/启动小车；实测与电气资格仍未通过。');return
    if a.check:raise ValueError('Choose --check OR --launch')
    # No reuse of historical 4WS8 readback, suspended/ground acceptance or an
    # invented hardware approval. This is intentionally blocked until hardware.
    if not a.profile or not sys.stdin.isatty():raise ValueError('New-image approved profile and human terminal required; nothing opened')
    for name in (m['control_package'],m['hardware_package']):sys.path.insert(0,str(Path(m['runtime_src'])/name))
    from mecanum_wand_4ws9_hardware.authorization import verify_runtime_profile
    profile=json.loads(a.profile.read_text())
    if profile['stage']!=a.stage:raise ValueError('Immutable requested stage/profile mismatch')
    if Path(profile['staged_runtime_approval']['build_manifest']).resolve()!=(a.prepared/'build_manifest.json').resolve():
        raise ValueError('Exact prepared manifest required')
    verify_runtime_profile(profile,a.purpose)
    if a.purpose=='suspended':print('保持四轮悬空固定，有人能立即断电；先准备，按住空格仅单次最多5秒，松键停车。')
    else:print('新固件本档停车、电气和逐档复核均须通过；固定区域与缓冲空旷，有人能立即断电。单击开始，再次空格/Esc停。')
    phrase=f'4WS9_STAGE_{a.stage}_{a.purpose.upper()}_CLEAR_POWER_CUT_READY'
    if input('当次现场确认后输入 '+phrase+'：').strip()!=phrase:raise ValueError('Site not confirmed')
    approved=a.profile.resolve()
    if a.purpose=='suspended':
        from mecanum_wand_4ws9_hardware.suspended_console import run
        run(approved,a.prepared/'physical_bench_pending');return
    from mecanum_wand_4ws9_hardware.commissioning import Runtime,CONSENT
    Runtime('hardware',str(approved),sha(approved),CONSENT)
    env=dict(os.environ,ROS_DOMAIN_ID='121',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1')
    os.execvpe('ros2',['ros2','launch',m['hardware_package'],'wand_4ws9_staged.launch.py','mode:=hardware',
        'gui:=true','profile:='+str(approved),'approved_sha256:='+sha(approved),'hardware_consent:='+CONSENT],env)

if __name__=='__main__':
    try:main()
    except (ValueError,OSError,KeyError,TypeError) as e:print('BLOCKED: '+str(e),file=sys.stderr);raise SystemExit(2)
