"""1280x720 read-only direction/angle display, not a car-control console."""
import json
import math
import time
import tkinter as tk
from tkinter import ttk
import numpy as np

from preview_core import IDS,vector,rotation


class PreviewWindow:
    def __init__(self,root,model,reader,folder,duration=600.):
        self.root,self.model,self.reader,self.folder=root,model,reader,folder
        self.started=time.monotonic();self.duration=duration;self.closed=False;self.job=None
        self.last_log=-1e9;self.last_candidates=[];self.result=None
        self.log=(folder/'observation.jsonl').open('x',buffering=65536)
        root.title('Sign旋转方向核对 · 只读 / 无电机输出');root.geometry('1280x720');root.minsize(1000,640)
        style=ttk.Style(root)
        for name in ('TLabel','TButton','TCheckbutton','TCombobox'):
            style.configure(name,font=('Noto Sans CJK SC',-16))
        top=tk.Frame(root,bg='#15334b');top.pack(fill='x')
        tk.Label(top,text='真实Sign → 方向与相对转角预览（不控制小车）',font=('Noto Sans CJK SC',-26,'bold'),
            fg='white',bg='#15334b',padx=16,pady=8).pack(anchor='w')
        tk.Label(top,text='实车保持断电 · 1:1相对旋转 · 横杆外侧/背离长柄为前方 · 无速度、使能或UDP输出',
            font=('Noto Sans CJK SC',-16),fg='#bfe8f5',bg='#15334b',padx=16,pady=5).pack(anchor='w')
        body=ttk.Frame(root,padding=12);body.pack(fill='both',expand=True)
        left=ttk.Frame(body);left.pack(side='left',fill='both',expand=True,padx=(0,14))
        ttk.Label(left,text='Vicon地面XY俯视图：蓝色=横杆三点；灰色=长柄；绿色箭头=正前方').pack(anchor='w')
        self.canvas=tk.Canvas(left,width=760,height=390,bg='white',highlightthickness=1,highlightbackground='#b6c8d4')
        self.canvas.pack(fill='both',expand=True,pady=8)
        self.status=tk.StringVar(value='等待同源、同帧的Sign与五点')
        tk.Label(left,textvariable=self.status,font=('Noto Sans CJK SC',-17,'bold'),anchor='w',justify='left',
            wraplength=760,bg='#fff0cf',fg='#5b4215',padx=10,pady=10).pack(fill='x')
        right=ttk.Frame(body,width=370);right.pack(side='right',fill='y')
        ttk.Label(right,text='① 核对横杆与前方箭头',font=('Noto Sans CJK SC',-19,'bold')).pack(anchor='w')
        self.choice=tk.StringVar();self.combo=ttk.Combobox(right,textvariable=self.choice,state='readonly',width=30)
        self.combo.pack(fill='x',pady=4)
        self.ack=tk.BooleanVar(value=False)
        ttk.Checkbutton(right,text='确认蓝色三点为短横杆，\n箭头朝外、背离长柄',variable=self.ack).pack(anchor='w')
        self.confirm=ttk.Button(right,text='确认这组三点（仅校准）',command=self.confirm_geometry)
        self.confirm.pack(fill='x',pady=4)
        ttk.Label(right,text='② 拿稳杆，记录相对零角',font=('Noto Sans CJK SC',-17,'bold')).pack(anchor='w',pady=4)
        actions=ttk.Frame(right);actions.pack(fill='x',pady=4)
        self.zero=ttk.Button(actions,text='记录零角（只读）',command=self.zero_angle);self.zero.pack(side='left',expand=True,fill='x')
        ttk.Button(actions,text='暂停 / 清除零角',command=self.clear).pack(side='left',expand=True,fill='x',padx=(6,0))
        self.angle=tk.StringVar(value='相对转角：未记录零角')
        tk.Label(right,textvariable=self.angle,font=('Noto Sans CJK SC',-20,'bold'),justify='left',
            bg='#e7f2ff',padx=10,pady=6).pack(fill='x',pady=6)
        self.car=tk.Canvas(right,width=340,height=110,bg='#f1f6fa',highlightthickness=0);self.car.pack(fill='x')
        ttk.Label(right,text='下图仅显示“目标车头”角度，\n不是实车反馈或动力学仿真。\n抬杆不映射；接近竖直或失跟踪时暂停。',
            font=('Noto Sans CJK SC',-13),wraplength=350).pack(anchor='w',pady=4)
        ttk.Button(right,text='关闭只读预览',command=self.close).pack(fill='x')
        root.protocol('WM_DELETE_WINDOW',self.close);root.bind('<Escape>',lambda _:self.clear())
        self.job=root.after(20,self.tick)

    def selected(self):
        index=self.combo.current()
        return index if 0<=index<len(self.model.candidates) else None

    def confirm_geometry(self):
        try:
            if not self.ack.get():raise ValueError('请先核对画面并勾选方向确认')
            index=self.selected()
            if index is None:raise ValueError('请明确选择横杆候选')
            d=self.model.confirm_geometry(time.monotonic(),self.reader.ros_now(),index)
            (self.folder/'geometry_confirmed.json').write_text(json.dumps(d.summary(),indent=2)+'\n')
        except ValueError as e:self.status.set(str(e))

    def zero_angle(self):
        try:
            self.model.zero(time.monotonic(),self.reader.ros_now())
            with (self.folder/'reference_events.jsonl').open('a') as stream:
                stream.write(json.dumps(self.model.report(),ensure_ascii=False)+'\n')
        except ValueError as e:self.status.set(str(e))

    def clear(self):self.model.clear_zero()

    def draw(self):
        m=self.model;r=m.report();names=['—'.join(d.bar)+' | 长柄 '+','.join(d.stem) for d in m.candidates]
        if names!=self.last_candidates:
            self.combo.configure(values=names);self.last_candidates=names;self.ack.set(False)
            self.combo.set('')
            if len(names)==1:self.combo.current(0)
        self.confirm.configure(state='normal' if names else 'disabled')
        self.zero.configure(state='normal' if m.valid else 'disabled')
        self.status.set(m.fault or m.reason)
        if r['relative_deg'] is None:self.angle.set('相对转角：未记录零角')
        else:self.angle.set(f'相对转角：{r["relative_deg"]:+.1f}°\n1:1目标：{r["relative_deg"]:+.1f}°')
        c=self.canvas;c.delete('all');w=max(100,c.winfo_width());h=max(100,c.winfo_height())
        c.create_text(12,12,anchor='nw',text='上=Vicon +Y；右=+X；不是摄像机画面',fill='#46647a',font=('Noto Sans CJK SC',-14))
        packet=m.packet
        if packet is not None:
            points={k:vector(packet[k].position) for k in IDS};center=np.mean(list(points.values()),axis=0)
            radius=max(.25,max(float(np.linalg.norm(v[:2]-center[:2])) for v in points.values())+.10)
            scale=min(w-70,h-90)/(2*radius)
            def xy(v):return w/2+(v[0]-center[0])*scale,h/2-(v[1]-center[1])*scale
            d=m.definition
            index=self.selected()
            if d is None and index is not None:d=m.candidates[index]
            if d is not None:
                for a,b in zip(d.bar,d.bar[1:]):c.create_line(*xy(points[a]),*xy(points[b]),width=5,fill='#237ccc')
                stem=sorted(d.stem,key=lambda k:np.linalg.norm(points[k]-points[d.bar[1]]))
                for a,b in zip((d.bar[1],*stem),(d.bar[1],*stem)[1:]):
                    c.create_line(*xy(points[a]),*xy(points[b]),width=4,fill='#81919d')
                forward=rotation(packet['sign'].quaternion)@vector(d.forward)
                start=points[d.bar[1]];end=start+.16*forward
                color='#1c9c62' if m.valid else '#bd8d18'
                c.create_line(*xy(start),*xy(end),width=5,arrow='last',fill=color)
                c.create_text(*xy(end),text='前方（背离长柄）',anchor='s',fill=color,font=('Noto Sans CJK SC',-16,'bold'))
            for k,p in points.items():
                x,y=xy(p);blue=d is not None and k in d.bar
                c.create_oval(x-7,y-7,x+7,y+7,fill='#237ccc' if blue else '#687c8b',outline='')
                c.create_text(x+11,y+10,anchor='nw',text=k,font=('Noto Sans CJK SC',-17,'bold'))
            if not m.valid:c.create_text(w/2,h-22,text='候选/失效画面：不能据此控制车辆',fill='#a36c08',font=('Noto Sans CJK SC',-16))
        else:c.create_text(w/2,h/2,text='等待五点同帧坐标',fill='#648097',font=('Noto Sans CJK SC',-22))
        self.car.delete('all');self.car.create_text(170,12,text='只读：目标车头示意',font=('Noto Sans CJK SC',-14))
        if r['target_car_heading_deg'] is not None:
            angle=math.radians(r['target_car_heading_deg']);x,y=170,66
            self.car.create_oval(x-28,y-28,x+28,y+28,fill='#c4dffa',outline='#237ccc',width=2)
            self.car.create_line(x,y,x+52*math.cos(angle),y-52*math.sin(angle),arrow='last',width=5,fill='#237ccc')
        else:self.car.create_text(170,80,text='尚无有效目标角度',font=('Noto Sans CJK SC',-16))

    def tick(self):
        self.job=None
        if self.closed:return
        try:
            self.reader.pump();self.draw();now=time.monotonic()
            if now-self.last_log>=.05:
                self.log.write(json.dumps(dict(steady=now,ros=self.reader.ros_now(),report=self.model.report(),
                    raw=self.reader.raw,source_ok=self.reader.flags),ensure_ascii=False)+'\n');self.last_log=now
            if now-self.started>=self.duration:self.close();return
        except Exception as exc:
            self.model.invalidate('只读处理错误：'+str(exc));self.draw()
            self.status.set('只读预览错误，已停止处理：'+str(exc));return
        self.job=self.root.after(20,self.tick)

    def close(self):
        if self.closed:return
        self.closed=True
        if self.job is not None:self.root.after_cancel(self.job)
        self.result=self.model.report();self.result['kind']='READONLY_SESSION_RESULT'
        self.log.flush();self.log.close()
        (self.folder/'SESSION_RESULT.json').write_text(json.dumps(self.result,ensure_ascii=False,indent=2)+'\n')
        self.reader.close();self.root.destroy()
