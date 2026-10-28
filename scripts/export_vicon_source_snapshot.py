#!/usr/bin/env python3
"""Read-only-source supplement; no SDK, binary install or private provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from export_car_snapshot import ROOT, normalize, SKIP


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--receiver-source',type=Path,required=True)
    ap.add_argument('--car-source',type=Path,required=True)
    args=ap.parse_args()
    source=args.receiver_source.resolve()
    destination=ROOT/'shared/vicon_receiver_snapshot_20261028'
    if destination.exists():raise ValueError('Do not overwrite a curated snapshot')
    rows=[]

    def copy(p, target):
        raw=p.read_bytes()
        if b'\0' in raw:raise ValueError('Binary source excluded')
        text=normalize(raw.decode('utf-8'),args.car_source.resolve())
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(text)
        rows.append(dict(path=str(target.relative_to(ROOT)),source_sha256=hashlib.sha256(raw).hexdigest(),
                         exported_sha256=hashlib.sha256(text.encode()).hexdigest()))
    for p in sorted(source.rglob('*')):
        if not p.is_file() or p.is_symlink() or any(k in SKIP for k in p.relative_to(source).parts):continue
        if p.suffix in {'.cpp','.hpp','.h','.py','.msg','.xml','.txt','.md'} or p.name=='LICENSE':
            copy(p,destination/p.relative_to(source))
    # Two read-only integration sources used by the car, kept outside drone.
    workspace=source.parents[1]
    for rel in ('drone_sim/tools/gamepad_rc/vicon_receiver_provenance.py',
                'drone_sim/ros2_ws/src/uav_coord_bridge/uav_coord_bridge/vicon_to_poses.py'):
        p=workspace/rel
        copy(p,destination/'integration'/p.name)
    for p in (args.car_source/'esp32/mecanum_wifi_bridge').rglob('Kconfig.projbuild'):
        if not any(k in SKIP for k in p.relative_to(args.car_source).parts):
            copy(p,ROOT/'car'/p.relative_to(args.car_source))
    event=args.car_source/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready/esp_project/main/Kconfig.projbuild'
    copy(event,ROOT/'car'/event.relative_to(args.car_source))
    commit=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
    (ROOT/'handoff/VICON_SOURCE_EXPORT_2026-10-28.json').write_text(json.dumps(dict(
        upstream_commit=commit,includes_uncommitted_development_sources=True,
        installed_receiver_equivalence_verified=False,hardware_qualification=False,
        source_tree_written=False,proprietary_sdk_included=False,files=rows),indent=2)+'\n')
    print(json.dumps(dict(files=len(rows),source_tree_written=False)))


if __name__=='__main__':main()
