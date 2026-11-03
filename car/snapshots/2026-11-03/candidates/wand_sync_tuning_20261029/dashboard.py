"""New SIM-only console; original demonstration and profiling entries unchanged."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import tempfile
import time

from model import ROOT,Limits,save,protected_hashes
from ros_tuning import install,TraceLog,TRACE
import console as trace_console
from isolated_reader import IsolatedReader as Reader

HERE=Path(__file__).resolve().parent
REPORT=ROOT/'docs/wand_sync_tuning_2026-10-29'


def hashes():
    paths=list(HERE.glob('*.py'))+[ROOT/'tools/start_wand_sync_tuning_sim.sh']
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def check():
    trace_console.check()
    manifest=json.loads((REPORT/'PREPARED.json').read_text())
    if not manifest['passed'] or manifest['sources']!=hashes():
        raise ValueError('candidate/evidence changed; regression and preparation required')
    for path,sha in manifest['evidence'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=sha:raise ValueError('evidence changed: '+path)
    return manifest


class Worker(trace_console.Worker):
    def __init__(self,folder,trace,profile):
        self.profile=profile
        super().__init__(folder,trace)

    def run(self):
        import rclpy
        reader=chain=None
        save(self.folder/'PROTECTED_BEFORE.json',protected_hashes())
        try:
            module=install(self.trace,self.profile,True)
            reader=Reader();reader.start()
            save(self.folder/'RECEIVER_PROVENANCE.json',reader.provenance)
            module.init_private();chain=module.Chain(reader,None)
            save(self.folder/'CONTRACT.json',dict(real_sign=True,domain=190,xy_scale=1.,fixed_fence=False,
                speed=self.profile.max_speed_mps,accel=self.profile.max_accel_mps2,
                rpm=self.profile.sim_max_rpm,yaw=.1,yaw_accel=.4,session_s=120.,
                optimized_scheduler=True,source_period_s=.005,control_period_s=.01,
                independent_gate_watchdog_s=.01,snapshot_period_s=.025,
                pose_timeout_s=.2,command_timeout_s=.1,
                lease_timeout_s=.15,feedback_timeout_s=.3,ui_timeout_s=.1,
                car01='read-only reference; NEVER software car feedback',
                reader_isolated_process=True,ipc_latest_capacity=1,ipc_timeout_s=.2,
                real_car_started=False,hardware_bridge_constructed=False,firmware_changed=False,
                hardware_qualification=False,model_tau_s=.12))
            self.started.set();last=last_status=0.
            while not self.done.is_set():
                chain.ui_seen=self.ui_stamp
                while not self.commands.empty():
                    action,issued=self.commands.get()
                    self.trace.emit('ui_action',action=action,issued_ns=round(issued*1e9))
                    self.dispatch(chain,action,issued)
                chain.step();now=time.monotonic()
                if now-last>.025:
                    snapshot=chain.snapshot();snapshot['reader']=reader.status()
                    self.snapshot=snapshot;last=now
                    if now-last_status>1.:
                        self.trace.emit('runtime_status',state=snapshot['state'],ready=snapshot['ready'],
                            sign_valid=snapshot['sign_valid'],blocker=snapshot['prepare_blocker'],
                            reader=snapshot['reader'],reason=snapshot['reason'],gate_fault=snapshot['gate_fault'])
                        last_status=now
            chain.close()
            save(self.folder/'SESSION_RESULT.json',dict(snapshot=self.snapshot,events=chain.events,
                max_pending=chain.max_pending,real_car_started=False))
            chain=None
        except Exception as error:
            self.snapshot=dict(error=repr(error),state='ERROR',message='仿真已撤销：'+str(error))
            self.trace.emit('worker_error',error=repr(error));save(self.folder/'ERROR.json',self.snapshot)
            self.started.set()
        finally:
            if chain:
                try:chain.close()
                except Exception as error:self.trace.emit('shutdown_error',error=repr(error))
            if reader:
                try:reader.close()
                except Exception as error:self.trace.emit('reader_close_error',error=repr(error))
            if rclpy.ok():rclpy.shutdown()
            save(self.folder/'PROTECTED_AFTER.json',protected_hashes())


class Window(trace_console.Window):
    def __init__(self,root,worker):
        self.items=None
        super().__init__(root,worker)
        root.title('真实Sign → 高同步参数仿真 · %.2fm/s / %.2fm/s² · 无硬件输出'%(
            worker.profile.max_speed_mps,worker.profile.max_accel_mps2))
        for child in root.winfo_children():
            try:text=child.cget('text')
            except Exception:continue
            if text.startswith('真实 Sign → ROS软件车'):
                child.configure(text='真实 Sign → 高同步参数 / 低等待ROS软件车')
            elif text.startswith('1:1 ·'):
                child.configure(text='1:1 · %.2fm/s · %.2fm/s² · %dRPM仿真上限 · 实车无输出'%(
                    worker.profile.max_speed_mps,worker.profile.max_accel_mps2,worker.profile.sim_max_rpm))

    def draw(self,s):
        begin=time.monotonic_ns();c=self.canvas
        if self.items is None:
            self.items=dict(trail=c.create_line(0,0,0,0,fill='#a6bec8',width=2),
                goal=c.create_oval(0,0,0,0,fill='#e18b20',outline=''),
                car=c.create_oval(0,0,0,0,fill='#207bab',outline=''),
                ga=c.create_line(0,0,0,0,fill='#e18b20',width=4,arrow='last'),
                ca=c.create_line(0,0,0,0,fill='#207bab',width=4,arrow='last'),
                title=c.create_text(14,16,anchor='nw',text='蓝：软件车  橙：1:1目标；car01仅作只读参照',font=('Sans',15),fill='#3b4b58'),
                footer=c.create_text(14,40,anchor='nw',text='',font=('Sans',14),fill='#3b4b58'))
        p=s['car'];g=s['goal'] or p;now=time.monotonic()
        if now-self.last_trail>.05:
            self.trail.append((p['x'],p['y']));self.trail=self.trail[-400:];self.last_trail=now
        points=self.trail+[(p['x'],p['y']),(g['x'],g['y'])]
        xmin=min(v[0] for v in points)-.4;xmax=max(v[0] for v in points)+.4
        ymin=min(v[1] for v in points)-.4;ymax=max(v[1] for v in points)+.4
        w=max(10,c.winfo_width());h=max(10,c.winfo_height())
        scale=min((w-50)/max(1.5,xmax-xmin),(h-60)/max(1.2,ymax-ymin))
        cx=(xmin+xmax)/2;cy=(ymin+ymax)/2
        def xy(x,y):return w/2+(x-cx)*scale,h/2-(y-cy)*scale
        trail=[value for point in self.trail for value in xy(*point)]
        if len(trail)>=4:c.coords(self.items['trail'],*trail)
        for pose,body,arrow,radius in ((g,'goal','ga',10),(p,'car','ca',13)):
            x,y=xy(pose['x'],pose['y'])
            c.coords(self.items[body],x-radius,y-radius,x+radius,y+radius)
            c.coords(self.items[arrow],x,y,x+28*math.cos(pose['yaw']),y-28*math.sin(pose['yaw']))
        error=s.get('error_m')
        c.coords(self.items['footer'],14,h-28)
        c.itemconfigure(self.items['footer'],text='目标间距：%s；参考点超前约束：%s；Z不映射'%(
            '—' if error is None else '%.1fcm'%(error*100),'生效' if s.get('reference_limited') else '未触发'))
        self.detail.set('签名输出已验：%d\n\n当前档：%s\n\ncar01：%s（非运动反馈）\n\n空格开始/再次停止，Esc停车。\n遮挡后须手动重新准备。'%(
            s['accepted_signed'],s['profile'],
            '可见' if s.get('reader',{}).get('car01_reference_only',{}).get('valid') else '暂不可见'))
        end=time.monotonic_ns();self.draw_sequence+=1;meta=s['trace']
        self.worker.trace.emit('draw',snapshot_seq=meta['snapshot_seq'],model_seq=meta['model_seq'],
            oid=meta['oid'],cid=meta['cid'],draw_seq=self.draw_sequence,begin_ns=begin,end_ns=end)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true');parser.add_argument('--launch',action='store_true')
    parser.add_argument('--speed',type=float,default=.70);parser.add_argument('--accel',type=float,default=4.)
    parser.add_argument('--rpm',type=float,default=400.)
    args=parser.parse_args();manifest=check();profile=Limits(args.speed,args.accel,args.rpm)
    if not args.launch:
        print(json.dumps(dict(passed=True,profile=profile.name,software_only=True,tests=manifest['tests'])));return
    folder=Path(tempfile.mkdtemp(prefix='real_sign_',dir=REPORT))
    os.environ['ROS_LOG_DIR']=str(folder/'ros_logs')
    trace=TraceLog(folder);worker=Worker(folder,trace,profile)
    import tkinter as tk
    root=tk.Tk();window=Window(root,worker)
    for number in (signal.SIGTERM,signal.SIGINT):signal.signal(number,lambda *_:root.after(0,window.close))
    print('SYNC_TUNING_SESSION '+str(folder),flush=True)
    try:root.mainloop()
    finally:
        worker.done.set();worker.thread.join(timeout=5.)
        if worker.thread.is_alive():trace.emit('shutdown_error',error='worker timeout')
        trace.close()
        if (folder/'CONTRACT.json').exists():
            from live_analysis import analyze
            result=analyze(folder)
            print('SYNC_TUNING_RESULT '+json.dumps(dict(valid=result['sample_valid'],folder=str(folder))),flush=True)


if __name__=='__main__':main()
