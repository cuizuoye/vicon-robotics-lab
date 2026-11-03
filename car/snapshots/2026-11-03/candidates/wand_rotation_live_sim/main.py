#!/usr/bin/env python3
"""Passive real Sign subscriptions -> in-memory car. Never a hardware entry."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import tempfile
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
READY = ROOT/'docs/wand_rotation_live_sim_2026-10-26/prepared'


def check():
    m = json.loads((READY/'manifest.json').read_text())
    if m.get('offline_passed') is not True or m.get('actuator_outputs') is not False:
        raise ValueError('simulation-only offline preparation missing')
    for group in ('source_hashes', 'protected_hashes', 'output_hashes'):
        for path, digest in m[group].items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
                raise ValueError('file drift: '+path)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args(); check()
    if args.check:
        print('PASS：真实Sign→软件车仿真文件及保留基线校验通过；未初始化ROS、未连接硬件。'); return
    if os.environ.get('ROS_DOMAIN_ID', '0') != '0': raise ValueError('ROS domain 0 required')
    # Verify all immutable dependencies BEFORE ROS initialization.
    from engine import Engine, load_definition, preview_model
    from live_input import LiveInput
    lock = (READY.parent/'console.lock').open('a')
    try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: raise ValueError('真实Sign旋转仿真窗口已打开；不会重复运行')
    definition = load_definition(); model = preview_model(definition)
    folder = Path(tempfile.mkdtemp(prefix='real_sign_', dir=READY.parent)); folder.chmod(0o700)
    logs = folder/'ros_logs'; logs.mkdir(mode=0o700)
    os.environ['ROS_LOG_DIR'] = str(logs)  # Process-local, never receiver/global environment.
    reader = LiveInput(model)
    engine = Engine(definition, time.monotonic(), reader.ros_now())
    (folder/'RECEIVER_PROVENANCE.json').write_text(json.dumps(reader.provenance, indent=2)+'\n')
    (folder/'SESSION_CONTRACT.json').write_text(json.dumps(dict(
        kind='PASSIVE_SIGN_TO_LOCAL_SOFTWARE_CAR', geometry=definition,
        command_publishers=[], service_clients=[], actuator_outputs=False,
        real_car_feedback=False, new_session_required_after_stop=True,
        xy_speed_mps=.7, xy_accel_mps2=.8, yaw_rate_radps=2., yaw_accel_radps2=6.,
        xy_scale=1., yaw_scale=1., max_active_s=120.,
        source_identity='same complete raw Sign+five-marker frame to both local safety cores',
        independent_process_protocol_qualified=False), ensure_ascii=False, indent=2)+'\n')
    print('仅仿真，未使能。记录目录：'+str(folder), flush=True)
    try:
        import tkinter as tk
        from ui import Window
        root = tk.Tk(); window = Window(root, model, reader, engine, folder)
        signal.signal(signal.SIGINT, lambda *_: root.after(0, window.close))
        signal.signal(signal.SIGTERM, lambda *_: root.after(0, window.close))
        root.mainloop()
    except Exception:
        reader.close(); raise


if __name__ == '__main__':
    try: main()
    except Exception as exc: raise SystemExit('真实Sign软件仿真未启动：'+str(exc))
