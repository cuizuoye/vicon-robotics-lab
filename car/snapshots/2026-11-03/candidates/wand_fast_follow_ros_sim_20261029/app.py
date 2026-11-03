"""Real Sign -> private-domain ROS -> SOFTWARE car; standalone Tk console."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import threading
import time

from bootstrap import ROOT, save, protected_hashes, input_hashes
HERE=Path(__file__).resolve().parent
REPORT=ROOT/'docs/wand_fast_follow_ros_sim_2026-10-29'


def check():
    manifest=json.loads((REPORT/'PREPARED.json').read_text())
    if not manifest.get('actual_ros_tests_passed'):raise ValueError('ROS回归尚未通过')
    for group in ('inputs','sources','protected'):
        for path,sha in manifest[group].items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=sha:raise ValueError('文件变化：'+path)
    evidence=Path(manifest['evidence'])/'RESULT.json'
    if hashlib.sha256(evidence.read_bytes()).hexdigest()!=manifest['evidence_sha256']:
        raise ValueError('ROS验收记录变化')
    return manifest


class Worker:
    def __init__(self,folder):
        self.folder=folder;self.commands=queue.SimpleQueue();self.snapshot={};self.ui_stamp=time.monotonic()
        self.done=threading.Event();self.started=threading.Event()
        self.thread=threading.Thread(target=self.run,name='private-ros-software-chain',daemon=True)
        self.thread.start()

    def run(self):
        reader=chain=None
        snapshots=None
        from ros_chain import Chain,init_private
        from live_reader import RealReader
        import rclpy
        try:
            snapshots=(self.folder/'snapshots.jsonl').open('w')
            reader=RealReader();save(self.folder/'RECEIVER_PROVENANCE.json',reader.provenance);reader.start()
            init_private();chain=Chain(reader,self.folder);self.started.set()
            last=0.
            while not self.done.is_set():
                # Domain0 subscriptions have their own bounded executor thread;
                # GUI draws and input callbacks cannot block the final gate.
                chain.ui_seen=self.ui_stamp
                while not self.commands.empty():
                    action,issued=self.commands.get()
                    self.dispatch(chain,action,issued)
                chain.step()
                now=time.monotonic()
                if now-last>.025:
                    self.snapshot=chain.snapshot();self.snapshot['reader']=reader.status();last=now
                    snapshots.write(json.dumps(dict(steady=now,**self.snapshot),ensure_ascii=False)+'\n')
            chain.close();chain=None
        except Exception as error:
            self.snapshot=dict(error=repr(error),state='ERROR',message='仿真已撤销：'+str(error))
            save(self.folder/'ERROR.json',self.snapshot);self.started.set()
        finally:
            if chain:
                try:chain.close()
                except Exception:pass
            if reader:reader.close()
            if rclpy.ok():rclpy.shutdown()
            if snapshots:snapshots.close()
            save(self.folder/'PROTECTED_AFTER.json',protected_hashes())

    def post(self,action):self.commands.put((action,time.monotonic()))

    @staticmethod
    def dispatch(chain,action,issued):
        now=time.monotonic();age=now-issued
        chain.events.append(dict(at=now,event='UI_ACTION',action=action,age_s=age))
        if action=='stop':chain.stop('操作员Esc/停止');return
        if action=='focus_lost':chain.focus_lost();return
        if not 0<=age<=.10:
            chain.message='操作请求已过期（100ms保护），未执行；请等界面稳定后重试'
            chain.events.append(dict(at=now,event='UI_ACTION_EXPIRED',action=action,age_s=age));return
        if action=='prepare':chain.prepare()
        elif action=='toggle':
            if chain.requested:chain.stop('再次空格停止')
            else:chain.engage()


class Window:
    def __init__(self,root,worker):
        import tkinter as tk
        from tkinter import ttk
        self.tk=tk;self.root=root;self.worker=worker;self.down=False;self.release_after=None;self.closed=False
        self.trail=[];self.last_trail=self.last_draw=0.
        root.title('真实Sign → 快速跟随ROS仿真 · 实车断电 · 无硬件输出')
        root.geometry('1280x720');root.minsize(1100,660)
        root.option_add('*Font',('Sans',16))
        style=ttk.Style(root);style.configure('TButton',font=('Sans',17),padding=10)
        ttk.Label(root,text='真实 Sign → ROS软件车 | 快速手势候选',font=('Sans',23,'bold')).pack(anchor='w',padx=16,pady=(10,0))
        ttk.Label(root,text='1:1 · ≤0.70m/s · ≤0.80m/s² · 旋转沿用原限速 · 最长120秒 · 实车无输出',font=('Sans',16)).pack(anchor='w',padx=16,pady=8)
        body=ttk.Frame(root);body.pack(fill='both',expand=True,padx=12)
        self.canvas=tk.Canvas(body,bg='#f3f7fa',highlightthickness=0);self.canvas.pack(side='left',fill='both',expand=True)
        panel=ttk.Frame(body,width=390);panel.pack(side='right',fill='y',padx=12);panel.pack_propagate(False)
        self.state=tk.StringVar(value='等待只读Sign输入…')
        ttk.Label(panel,textvariable=self.state,wraplength=380,font=('Sans',17),justify='left').pack(anchor='w',pady=12)
        self.prepare_button=ttk.Button(panel,text='准备 / 确认并重新准备',command=lambda:worker.post('prepare'))
        self.prepare_button.pack(fill='x',pady=8)
        self.toggle_button=ttk.Button(panel,text='空格：开始 / 再次停止',command=lambda:worker.post('toggle'))
        self.toggle_button.pack(fill='x',pady=8)
        self.prepare_button.state(['disabled']);self.toggle_button.state(['disabled'])
        ttk.Button(panel,text='Esc：停止软件车',command=lambda:worker.post('stop')).pack(fill='x',pady=8)
        self.detail=tk.StringVar()
        ttk.Label(panel,textvariable=self.detail,wraplength=380,font=('Sans',16),justify='left').pack(anchor='w',pady=12)
        self.message=tk.StringVar(value='本窗口只控制软件模型；原演示不变。')
        ttk.Label(root,textvariable=self.message,wraplength=1240,font=('Sans',17),foreground='#9a4b0c').pack(anchor='w',padx=16,pady=10)
        # Own bindtag consumes space BEFORE ttk button defaults. Auto-repeat
        # releases are deferred/cancelled, avoiding accidental double toggles.
        root.bind_class('FastSimKeys','<KeyPress-space>',self.press)
        root.bind_class('FastSimKeys','<KeyRelease-space>',self.release)
        root.bind_class('FastSimKeys','<Escape>',lambda e:(worker.post('stop'),'break')[-1])
        def tags(widget):
            widget.bindtags(('FastSimKeys',*widget.bindtags()))
            for child in widget.winfo_children():tags(child)
        tags(root)
        root.bind('<FocusOut>',lambda e:root.after(30,self.check_focus))
        root.protocol('WM_DELETE_WINDOW',self.close)
        root.after(100,root.focus_force);self.tick()

    def press(self,event=None):
        if self.release_after:self.root.after_cancel(self.release_after);self.release_after=None
        if not self.down:self.down=True;self.worker.post('toggle')
        return 'break'

    def release(self,event=None):
        if self.release_after:self.root.after_cancel(self.release_after)
        self.release_after=self.root.after(60,self.clear_key);return 'break'

    def clear_key(self):self.down=False;self.release_after=None

    def check_focus(self):
        if not self.closed and self.root.focus_displayof() is None:
            self.clear_key();self.worker.post('focus_lost')

    def controls(self,s):
        ready=s.get('ready',False);active=s.get('state')=='ACTIVE'
        pending=s.get('prepare_pending',False)
        self.prepare_button.state(['!disabled'] if s.get('can_prepare',False) else ['disabled'])
        self.prepare_button.configure(text='已准备完成（READY）' if ready else
            '准备中，请等待…' if pending else '准备 / 确认并重新准备')
        self.toggle_button.state(['!disabled'] if ready or active else ['disabled'])

    def tick(self):
        if self.closed:return
        self.worker.ui_stamp=time.monotonic()
        s=self.worker.snapshot
        if s:
            self.message.set(s.get('message','')+'\n'+s.get('reason',''))
            if 'error' in s:self.state.set(s['message'])
            else:
                self.controls(s)
                gate_label=s['gate_fault'] or '正常'
                if s['state'] in ('STANDBY','PAUSED') and gate_label=='COMMAND_TIMEOUT':gate_label='未准备，等待零速握手'
                self.state.set('状态：'+s['state']+(' · READY' if s['ready'] else '')+
                    '\nSign：'+('有效' if s['sign_valid'] else '不可用/遮挡')+
                    '\n剩余：%.1f秒'%s['remaining']+'\n末级门：'+gate_label+
                    '\n停稳：'+('是' if s['stop_confirmed'] else '等待'))
                blocker='' if s['ready'] or s['state']=='ACTIVE' else s.get('prepare_blocker','')
                self.detail.set('蓝色：仿真车 · 橙色：目标\n\n签名输出已验：%d\n追赶样本：%d\n快速前馈：%d\n\n%s\n\n松键不停；再次空格/Esc停。\n遮挡后不会自动恢复。'%(s['accepted_signed'],s['catchup_samples'],s['feedforward_samples'],blocker))
                if time.monotonic()-self.last_draw>.033:
                    self.draw(s);self.last_draw=time.monotonic()
        self.root.after(20,self.tick)

    def draw(self,s):
        c=self.canvas;c.delete('all');w=max(10,c.winfo_width());h=max(10,c.winfo_height())
        p=s['car'];g=s['goal'] or p
        now=time.monotonic()
        if now-self.last_trail>.05:
            self.trail.append((p['x'],p['y']));self.trail=self.trail[-1200:];self.last_trail=now
        points=self.trail+[(p['x'],p['y']),(g['x'],g['y'])]
        xmin=min(v[0] for v in points)-.4;xmax=max(v[0] for v in points)+.4
        ymin=min(v[1] for v in points)-.4;ymax=max(v[1] for v in points)+.4
        scale=min((w-50)/max(1.5,xmax-xmin),(h-60)/max(1.2,ymax-ymin))
        cx=(xmin+xmax)/2;cy=(ymin+ymax)/2
        def xy(x,y):return w/2+(x-cx)*scale,h/2-(y-cy)*scale
        trail=[v for pt in self.trail for v in xy(*pt)]
        if len(trail)>=4:c.create_line(*trail,fill='#a6bec8',width=2)
        for v,color in ((g,'#e18b20'),(p,'#207bab')):
            x,y=xy(v['x'],v['y']);r=10 if color=='#e18b20' else 13
            c.create_oval(x-r,y-r,x+r,y+r,fill=color,outline='')
            c.create_line(x,y,x+28*math.cos(v['yaw']),y-28*math.sin(v['yaw']),fill=color,width=4,arrow='last')
        c.create_text(14,16,anchor='nw',text='只控制软件车 · 视图自动缩放，非运动限位',font=('Sans',16),fill='#3b4b58')
        c.create_text(14,h-28,anchor='nw',text='X前向 / Y左向，世界平面1:1；Z不映射',font=('Sans',15),fill='#3b4b58')

    def close(self):
        if self.closed:return
        self.closed=True;self.worker.post('stop');self.worker.done.set();self.root.destroy()


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--check',action='store_true')
    ap.add_argument('--launch',action='store_true');args=ap.parse_args();m=check()
    if not args.launch:
        print(json.dumps(dict(file_check_passed=True,actual_ros_tests=m['actual_ros_case_count'],
            software_only=True,hardware_output=False),ensure_ascii=False));return
    import tempfile
    folder=Path(tempfile.mkdtemp(prefix='real_sign_',dir=REPORT))
    save(folder/'PROTECTED_BEFORE.json',protected_hashes())
    os.environ['ROS_LOG_DIR']=str(folder/'ros_logs')
    worker=Worker(folder)
    import tkinter as tk
    root=tk.Tk();w=Window(root,worker)
    signal.signal(signal.SIGTERM,lambda *_:root.after(0,w.close))
    signal.signal(signal.SIGINT,lambda *_:root.after(0,w.close))
    print('SIM_ONLY_SESSION '+str(folder),flush=True)
    try:root.mainloop()
    finally:worker.done.set();worker.thread.join(timeout=4.)


if __name__=='__main__':main()
