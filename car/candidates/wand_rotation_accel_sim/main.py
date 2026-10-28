#!/usr/bin/env python3
"""A2 acceleration SIM launcher; passive Sign subscriptions only."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import tempfile
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
READY=ROOT/'docs/wand_rotation_accel_sim_2026-10-26/prepared'


def check():
    m=json.loads((READY/'manifest.json').read_text())
    if m.get('offline_passed') is not True or m.get('actuator_outputs') is not False:
        raise ValueError('A2 simulation preparation missing')
    for group in ('source_hashes','protected_hashes','output_hashes'):
        for path,digest in m[group].items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:
                raise ValueError('File drift: '+path)
    return m


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--check',action='store_true')
    args=ap.parse_args();check()
    recommendation=json.loads((READY/'RECOMMENDED_SIM_ONLY.json').read_text())
    if args.check:
        print('PASS：独立加速度仿真文件和保留基线校验通过；未初始化ROS、未连接硬件。')
        print(json.dumps(recommendation,ensure_ascii=False));return
    if os.environ.get('ROS_DOMAIN_ID','0')!='0': raise ValueError('ROS domain 0 required')
    from accel_core import AccelerationEngine,AccelerationLimits,load_definition,preview_model
    from live_input import LiveInput
    # Same lock as the old real-Sign SIM; do not change/stop an existing owner.
    lock=(ROOT/'docs/wand_rotation_live_sim_2026-10-26/console.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise ValueError('已有旋转仿真窗口，请先在窗口内停止并关闭；不会抢占')
    c=AccelerationLimits(recommendation['xy_accel_mps2'],recommendation['yaw_accel_radps2'])
    definition=load_definition();model=preview_model(definition)
    folder=Path(tempfile.mkdtemp(prefix='real_sign_A2_',dir=READY.parent));folder.chmod(0o700)
    logs=folder/'ros_logs';logs.mkdir(mode=0o700);os.environ['ROS_LOG_DIR']=str(logs)
    reader=LiveInput(model);engine=AccelerationEngine(definition,time.monotonic(),reader.ros_now(),c)
    (folder/'RECEIVER_PROVENANCE.json').write_text(json.dumps(reader.provenance,indent=2)+'\n')
    contract=dict(kind='PASSIVE_SIGN_TO_ACCELERATION_A2_SOFTWARE_CAR',geometry=definition,
        **recommendation,command_publishers=[],service_clients=[],real_car_feedback=False,
        source_binding='original same-frame Sign+five-marker packet to both local safety cores',
        multi_process_wire_protocol_qualified=False)
    (folder/'SESSION_CONTRACT.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n')
    print('A2仿真未使能；实车保持断电。记录目录：'+str(folder),flush=True)
    try:
        import tkinter as tk
        from ui_accel import AccelerationWindow
        root=tk.Tk();window=AccelerationWindow(root,model,reader,engine,folder)
        signal.signal(signal.SIGINT,lambda *_:root.after(0,window.close))
        signal.signal(signal.SIGTERM,lambda *_:root.after(0,window.close))
        root.mainloop()
    except Exception:
        reader.close();raise


if __name__=='__main__':
    try:main()
    except Exception as exc:raise SystemExit('A2软件仿真未启动：'+str(exc))
