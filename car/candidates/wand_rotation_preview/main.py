#!/usr/bin/env python3
"""Independent passive Sign preview. Never imports a hardware controller."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import tempfile
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
READY=ROOT/'docs/wand_rotation_preview_2026-10-26/prepared'


def verify_manifest():
    manifest=json.loads((READY/'manifest.json').read_text())
    if manifest.get('offline_passed') is not True or manifest.get('actuator_outputs') is not False:
        raise ValueError('只读预览尚未通过离线准备')
    for group in ('source_hashes','baseline_hashes'):
        for path,sha in manifest[group].items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=sha:raise ValueError('输入文件变化：'+path)
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true');parser.add_argument('--headless',action='store_true')
    parser.add_argument('--seconds',type=float,default=600.)
    args=parser.parse_args();verify_manifest()
    if args.check:
        print('PASS：只读旋转预览文件和保留基线校验通过；未初始化ROS、未连接设备。');return
    if not 1<=args.seconds<=600 or (args.headless and args.seconds>30):raise ValueError('只读GUI限1..600秒；无界面诊断限1..30秒')
    if os.environ.get('ROS_DOMAIN_ID','0')!='0':raise ValueError('只读入口固定ROS域0')
    from preview_core import PreviewModel
    from live_input import LiveInput
    folder=Path(tempfile.mkdtemp(prefix='readonly_',dir=READY.parent));folder.chmod(0o700)
    logdir=folder/'ros_logs';logdir.mkdir(mode=0o700)
    os.environ['ROS_LOG_DIR']=str(logdir) # This process and its child ROS context only.
    model=PreviewModel();reader=LiveInput(model)
    (folder/'RECEIVER_PROVENANCE.json').write_text(json.dumps(reader.provenance,indent=2)+'\n')
    print('只读记录目录：'+str(folder),flush=True)
    if args.headless:
        started=time.monotonic()
        try:
            while time.monotonic()-started<args.seconds:reader.pump();time.sleep(.005)
            result=model.report();result.update(output=str(folder),source_ok=reader.flags,raw=reader.raw)
            (folder/'SESSION_RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            print(json.dumps(result,ensure_ascii=False),flush=True)
        finally:reader.close()
        return
    import tkinter as tk
    from preview_ui import PreviewWindow
    try:
        root=tk.Tk();window=PreviewWindow(root,model,reader,folder,args.seconds)
        signal.signal(signal.SIGINT,lambda *_:root.after(0,window.close))
        signal.signal(signal.SIGTERM,lambda *_:root.after(0,window.close))
        root.mainloop()
    except Exception:
        reader.close();raise


if __name__=='__main__':
    try:main()
    except Exception as exc:raise SystemExit('只读预览未启动：'+str(exc))
