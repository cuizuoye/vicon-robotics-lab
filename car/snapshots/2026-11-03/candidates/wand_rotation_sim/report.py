#!/usr/bin/env python3
"""Read saved offline results only. No launch or device options."""
import argparse
import hashlib
import json
from pathlib import Path

READY=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_rotation_sim_2026-10-26/offline_ready')


def verify():
    m=json.loads((READY/'manifest.json').read_text())
    if m.get('offline_passed') is not True or m.get('hardware_qualified') is not False:
        raise ValueError('offline preparation incomplete')
    for group in ('source_hashes','protected_hashes','output_hashes'):
        for path,value in m[group].items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=value:raise ValueError('file drift: '+path)
    return m


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');p.parse_args()
    m=verify();r=json.loads((READY/'recommended_software_only.json').read_text())
    print('PASS：独立旋转仿真文件/保留基线校验通过；未初始化ROS或连接设备。')
    print(json.dumps(dict(tests=m['tests'],recommended_sim_only=r['limits'],
        hardware_qualified=False,real_sign_sim_accepted=False,report=str(READY/'summary.json')),ensure_ascii=False))


if __name__=='__main__':
    try:main()
    except Exception as e:raise SystemExit('离线结果未就绪：'+str(e))
