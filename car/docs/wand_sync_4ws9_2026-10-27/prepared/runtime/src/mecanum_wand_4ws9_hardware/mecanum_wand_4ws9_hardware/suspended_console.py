"""Opt-in suspended GUI. Hardware creation requires post-flash/site approval.

Importing or testing widgets opens no socket. Worker accepts an explicit IO
object so tests use the native C oracle, never the real network interface.
"""
import fcntl
import json
import os
from pathlib import Path
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk
from .authorization import verify_runtime_profile
from .suspended_core import SuspendedController,DIRECTIONS
from mecanum_wand_4ws9_control.x11_key_state import X11KeyState

class UdpIO:
    def __init__(self,profile,log_path):
        verify_runtime_profile(profile,'suspended') # BEFORE lock/socket/device access
        hw=profile['hardware']
        self.ip=hw['esp_ip'];self.destination=(self.ip,46000)
        self.port_lock=open('/tmp/wand_mc_link_46001.lock','a')
        fcntl.flock(self.port_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.log=open(log_path,'x',buffering=1)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,hw['network_interface'].encode()+b'\0')
            self.sock.bind(('0.0.0.0',46001));self.sock.setblocking(False)
        except Exception:
            self.sock.close();self.log.close();self.port_lock.close();raise

    def send(self,data,now):
        self.sock.sendto(data,self.destination)
        self.log.write(json.dumps(dict(steady=now,kind='tx',hex=data.hex()))+'\n')

    def poll(self,now):
        output=[]
        for _ in range(32):
            try:data,sender=self.sock.recvfrom(1024)
            except BlockingIOError:break
            if sender[0]!=self.ip:continue
            self.log.write(json.dumps(dict(steady=now,kind='rx',hex=data.hex()))+'\n');output.append(data)
        return output

    def close(self):
        self.sock.close();self.log.close();self.port_lock.close()

class Worker:
    def __init__(self,controller,io):
        self.core=controller;self.io=io;self.lock=threading.RLock();self.done=threading.Event()
        self.thread=threading.Thread(target=self.run,name='stage0-suspended-safety',daemon=True)

    def run(self):
        try:
            while not self.done.is_set():
                now=time.monotonic()
                with self.lock:
                    for packet in self.io.poll(now):self.core.receive(packet,now)
                    frames=self.core.step(now)
                    # Send under the same short safety-state lock. A GUI stop
                    # cannot interleave after validation but before old output.
                    for frame in frames:self.io.send(frame,now)
                self.done.wait(.01)
        except Exception as error:
            with self.lock:self.core.stop(time.monotonic(),'执行线程异常：'+str(error),fault=True)
        finally:
            with self.lock:
                self.core.stop(time.monotonic(),'执行线程关闭')
                frames=self.core.step(time.monotonic())
            for packet in frames:
                try:self.io.send(packet,time.monotonic())
                except Exception:break
            self.io.close()

    def close(self):
        with self.lock:self.core.stop(time.monotonic(),'窗口关闭：撤能')
        self.done.set()
        if self.thread.is_alive():self.thread.join(timeout=1.)

class SuspendedConsole:
    def __init__(self,root,worker,key_state=None):
        self.root=root;self.worker=worker;self.closed=False
        self.key_state=key_state if key_state is not None else X11KeyState(os.environ.get('DISPLAY',':1'))
        root.title(f'4WS9 · 悬空档{worker.core.stage} · {worker.core.limits.speed:.2f}m/s · 最长5秒')
        root.geometry('900x480')
        self.status=tk.StringVar();self.detail=tk.StringVar();self.direction=tk.StringVar(value='前进')
        ttk.Label(root,text='悬空测试，不使用Sign位置闭环；实车不得落地运行此入口',font=('',15)).pack(pady=12)
        ttk.Label(root,text=f'{worker.core.limits.speed:.2f}m/s · {worker.core.limits.accel:.2f}m/s² · {worker.core.limits.duty/10:g}% PWM · 双重5秒上限').pack()
        ttk.Label(root,text='轻按松开空格解锁 → 准备（不转轮）→ 就绪后按住空格；松键/失焦停止').pack(pady=8)
        row=ttk.Frame(root);row.pack(pady=12)
        self.combo=ttk.Combobox(row,textvariable=self.direction,values=tuple(DIRECTIONS),state='readonly',width=10)
        self.combo.pack(side='left',padx=8)
        self.prepare_button=ttk.Button(row,text='准备（不转轮）',command=self.prepare);self.prepare_button.pack(side='left',padx=8)
        ttk.Button(row,text='停止/撤能',command=self.stop).pack(side='left',padx=8)
        ttk.Button(row,text='急停锁存',command=lambda:self.stop(True)).pack(side='left',padx=8)
        ttk.Label(root,textvariable=self.status,font=('',16)).pack(pady=12)
        ttk.Label(root,textvariable=self.detail,wraplength=850,justify='left').pack(pady=8)
        ttk.Label(root,text='注意：零PWM不等于轮子已停。异常或停车未确认时，准备手动断电；万用表不能证明峰值电流/温升合格。',wraplength=850).pack(pady=15)
        # Consume Space before ttk's button/combobox class bindings. Actual
        # physical bitmap is authoritative, so X11 repeat Release/Press pairs
        # cannot activate a focused button or fabricate a key release.
        root.bind_class('Stage0BenchKeys','<KeyPress-space>',lambda e:'break')
        root.bind_class('Stage0BenchKeys','<KeyRelease-space>',lambda e:'break')
        def tag(w):
            w.bindtags(('Stage0BenchKeys',*w.bindtags()))
            for child in w.winfo_children():tag(child)
        tag(root)
        root.bind('<FocusOut>',lambda e:root.after_idle(self.poll))
        root.protocol('WM_DELETE_WINDOW',self.close)
        self.timer=root.after(20,self.poll)

    def focused(self):
        # ttk creates its popdown in Tcl, not Python's widget children map.
        # Resolve raw Tcl paths instead of focus_displayof/_nametowidget.
        # A popup/other toplevel is NOT an authority to operate the vehicle.
        try:
            path=self.root.tk.call('focus','-displayof',self.root._w)
            return bool(path) and str(self.root.tk.call('winfo','toplevel',path))==str(self.root)
        except tk.TclError:
            return False

    def poll(self):
        if self.closed:return
        try:
            self.poll_once()
        except Exception as error:
            with self.worker.lock:
                self.worker.core.stop(time.monotonic(),'操作台输入/显示异常：'+str(error),fault=True)
            try:self.status.set('FAULT ｜ 操作台异常，已撤能；请勿重试')
            except tk.TclError:pass
        finally:
            if not self.closed:
                # Even a failed UI callback must not silently kill the polling loop.
                try:self.root.after_cancel(self.timer)
                except (AttributeError,tk.TclError):pass
                self.timer=self.root.after(20,self.poll)

    def poll_once(self):
        now=time.monotonic()
        try:down=self.key_state.down()
        except Exception:
            self.stop();down=False
        with self.worker.lock:
            core=self.worker.core
            core.operator(now,self.focused(),down)
            self.status.set(core.state+' ｜ '+core.reason)
            rpm=core.wheel.wheel_rpm if core.wheel else ('--',)*4
            left=max(0.,5-(now-core.action_at)) if core.action_at is not None and core.state in ('ACTIVE','WAIT_ENABLE') else 5.
            self.detail.set(f'解锁：{core.unlocked}；方向：{core.direction}；单次剩余：{left:.1f}s\n'
                f'实际编码器RPM FL/FR/RL/RR：{rpm}\n'
                f'PWM：{core.status["raw_duty"] if core.status else "--"}；故障：{hex(core.wheel.fault_bits) if core.wheel else "--"}；'
                f'首故障轮/原因：{core.status["first_feedback_fault"]["wheel"] if core.status else "--"}/'
                f'{core.status["first_feedback_fault"]["reason"] if core.status else "--"}\n'
                f'反馈新鲜：{core.fresh(now)}；任何重新准备都需要松键、停稳和新的人工流程。')
            idle=core.state in ('IDLE','COMPLETE','FAULT')
            self.prepare_button.configure(state='normal' if idle and core.unlocked and not down else 'disabled')
            self.combo.configure(state='readonly' if idle else 'disabled')
    def prepare(self):
        self.poll()
        with self.worker.lock:self.worker.core.prepare(time.monotonic(),self.direction.get())

    def stop(self,estop=False):
        with self.worker.lock:self.worker.core.stop(time.monotonic(),'人工急停' if estop else '人工停止',estop=estop,fault=estop)

    def close(self):
        if self.closed:return
        self.closed=True;self.worker.close();self.key_state.close();self.root.destroy()

def run(profile_path,evidence_dir):
    profile=json.loads(Path(profile_path).read_text())
    verify_runtime_profile(profile,'suspended')
    folder=Path(evidence_dir);folder.mkdir(parents=True,exist_ok=False)
    io=UdpIO(profile,folder/'raw_link.jsonl')
    worker=Worker(SuspendedController(stage=profile['stage']),io)
    root=None;app=None
    try:
        root=tk.Tk();app=SuspendedConsole(root,worker);worker.thread.start();root.mainloop()
    finally:
        if app and not app.closed:app.close()
        elif not app:io.close()
        (folder/'events.json').write_text(json.dumps(worker.core.events,ensure_ascii=False,indent=2)+'\n')
