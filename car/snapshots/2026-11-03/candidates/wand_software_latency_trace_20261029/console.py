"""Independent, timed real Sign -> signed SOFTWARE car console. No device IO."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import tempfile
import threading
import time

from instrument import PARENT, install
from trace_log import TraceLog
import app as baseline_app
from bootstrap import ROOT, save, protected_hashes

HERE = Path(__file__).resolve().parent
REPORT = ROOT/'docs/wand_software_latency_trace_2026-10-29'


def hashes():
    paths = list(PARENT.glob('*.py')) + list(HERE.glob('*.py'))
    paths += [ROOT/'tools/start_fast_follow_ros_sim.sh', ROOT/'tools/start_wand_software_latency_trace.sh']
    return {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def check():
    baseline_app.check()
    manifest = json.loads((REPORT/'PREPARED.json').read_text())
    if not manifest['passed'] or hashes() != manifest['sources']:
        raise ValueError('profiling source changed or regression evidence missing')
    for name, sha in manifest['evidence'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != sha:
            raise ValueError('profiling regression evidence changed')
    return manifest


class Worker(baseline_app.Worker):
    def __init__(self, folder, trace):
        self.trace = trace
        super().__init__(folder)

    def run(self):
        from live_reader import RealReader
        import rclpy
        module = install(self.trace)
        reader = chain = None
        before = protected_hashes()
        save(self.folder/'PROTECTED_BEFORE.json', before)
        try:
            reader = RealReader()
            save(self.folder/'RECEIVER_PROVENANCE.json', reader.provenance)
            reader.start()
            module.init_private()
            # Existing synchronous observer disk output is disabled ONLY in
            # this independent profiler; the observer callbacks stay present.
            chain = module.Chain(reader, None)
            save(self.folder/'CONTRACT.json', dict(real_sign=True, domain=189,
                domain0_subscriptions_only=True, xy_scale=1., fixed_fence=False,
                speed=.7, accel=.8, yaw=.1, yaw_accel=.4, session_s=120.,
                pose_timeout_s=.2, command_timeout_s=.1, lease_timeout_s=.15,
                feedback_timeout_s=.3, ui_timeout_s=.1, hardware_bridge_constructed=False,
                firmware_changed=False, network_changed=False, physical_car_started=False,
                logger='bounded asynchronous sidecar; parent synchronous disk observers disabled'))
            self.started.set()
            last = 0.
            last_status = 0.
            while not self.done.is_set():
                chain.ui_seen = self.ui_stamp
                while not self.commands.empty():
                    action, issued = self.commands.get()
                    self.trace.emit('ui_action', action=action, issued_ns=round(issued*1e9))
                    self.dispatch(chain, action, issued)
                chain.step()
                now = time.monotonic()
                if now-last > .025:
                    snapshot = chain.snapshot()
                    snapshot['reader'] = reader.status()
                    self.snapshot = snapshot
                    last = now
                    if now-last_status > 1.:
                        self.trace.emit('runtime_status', state=snapshot['state'],
                            sign_valid=snapshot['sign_valid'], ready=snapshot['ready'],
                            blocker=snapshot['prepare_blocker'], reader=snapshot['reader'],
                            reason=snapshot['reason'], gate_fault=snapshot['gate_fault'])
                        last_status = now
            chain.close()
            save(self.folder/'SESSION_RESULT.json', dict(snapshot=self.snapshot, events=chain.events,
                max_pending=chain.max_pending, real_car_started=False, firmware_changed=False))
            chain = None
        except Exception as error:
            self.snapshot = dict(error=repr(error), state='ERROR', message='软件采样撤销：'+str(error))
            self.trace.emit('worker_error', error=repr(error))
            save(self.folder/'ERROR.json', self.snapshot)
            self.started.set()
        finally:
            if chain:
                try:
                    chain.close()
                except Exception as error:
                    self.trace.emit('shutdown_error', error=repr(error))
            if reader:
                try:
                    reader.close()
                except Exception as error:
                    self.trace.emit('reader_close_error', error=repr(error))
            if rclpy.ok():
                rclpy.shutdown()
            save(self.folder/'PROTECTED_AFTER.json', protected_hashes())


class Window(baseline_app.Window):
    draw_sequence = 0

    def __init__(self, root, worker):
        # Independent window only: host high-DPI point scaling otherwise clips
        # preparation controls inside the fixed 1280x720 accepted layout.
        root.tk.call('tk', 'scaling', 4./3.)
        super().__init__(root, worker)
        root.title('真实Sign → 软件全链路计时 · 原参数不变 · 实车断电')
        root.after(200, root.lift)

    def draw(self, snapshot):
        begin = time.monotonic_ns()
        super().draw(snapshot)
        end = time.monotonic_ns()
        self.draw_sequence += 1
        meta = snapshot.get('trace')
        if meta:
            self.worker.trace.emit('draw', snapshot_seq=meta['snapshot_seq'],
                model_seq=meta['model_seq'], oid=meta['oid'], cid=meta['cid'],
                draw_seq=self.draw_sequence, begin_ns=begin, end_ns=end)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--launch', action='store_true')
    args = parser.parse_args()
    manifest = check()
    if not args.launch:
        print(json.dumps(dict(passed=True, software_only=True, parameters_unchanged=True,
                              tests=manifest['tests']), ensure_ascii=False))
        return
    folder = Path(tempfile.mkdtemp(prefix='real_sign_', dir=REPORT))
    os.environ['ROS_LOG_DIR'] = str(folder/'ros_logs')
    trace = TraceLog(folder)
    worker = Worker(folder, trace)
    import tkinter as tk
    root = tk.Tk()
    window = Window(root, worker)
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: root.after(0, window.close))
    print('SOFTWARE_LATENCY_SESSION '+str(folder), flush=True)
    try:
        root.mainloop()
    finally:
        worker.done.set()
        worker.thread.join(timeout=5.)
        if worker.thread.is_alive():
            trace.emit('shutdown_error', error='ROS worker did not stop within 5s')
        trace.close()
        from analyze import run
        result = run(folder)
        print('LATENCY_RESULT '+json.dumps(dict(sample_valid=result['sample_valid'],
              coverage=result['coverage'], folder=str(folder)), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
