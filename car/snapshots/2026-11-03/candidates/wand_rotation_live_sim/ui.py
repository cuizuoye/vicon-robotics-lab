"""Independent 1280x720 SIM console; only reader is ROS, never actuator IO."""
import json
import math
import os
import time
import tkinter as tk
from tkinter import ttk

from engine import Clicks, select_source, ZERO
from mecanum_wand_4ws8_control.x11_key_state import X11KeyState


class Window:
    def __init__(self, root, model, reader, engine, folder, *, key_state=None, schedule=True):
        self.root, self.model, self.reader, self.e, self.folder = root, model, reader, engine, folder
        self.keys = Clicks(engine)
        self.key_state = key_state if key_state is not None else X11KeyState(os.environ.get('DISPLAY', ''))
        self.closed = False; self.job = self.focus_job = None
        self.last_draw = self.last_log = self.last_flush = -1e9
        self.logged_events = 0
        self.log = (folder/'observation.jsonl').open('x', buffering=65536)
        root.title('真实Sign → 仿真车 · 平移＋旋转 · 实车无输出')
        root.geometry('1280x720'); root.minsize(1000, 640)
        style = ttk.Style(root)
        for name in ('TLabel', 'TButton'): style.configure(name, font=('Noto Sans CJK SC', -16))
        top = tk.Frame(root, bg='#16384b'); top.pack(fill='x')
        tk.Label(top, text='真实Sign → 软件小车：1:1平移＋相对旋转', bg='#16384b', fg='white',
            font=('Noto Sans CJK SC', -25, 'bold'), padx=16, pady=10).pack(anchor='w')
        tk.Label(top, text='实车保持断电 / 无电机输出   ·   横杆外侧为前方   ·   抬杆不映射',
            bg='#16384b', fg='#c0ecf3', font=('Noto Sans CJK SC', -16), padx=16).pack(anchor='w', pady=(0, 8))
        body = ttk.Frame(root, padding=12); body.pack(fill='both', expand=True)
        left = ttk.Frame(body); left.pack(side='left', fill='both', expand=True, padx=(0, 14))
        ttk.Label(left, text='固定启动车头坐标：上=前，左=左；蓝=软件车，橙=1:1目标').pack(anchor='w')
        self.canvas = tk.Canvas(left, width=785, height=440, bg='#f6fafc', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True, pady=8)
        self.status = tk.StringVar()
        tk.Label(left, textvariable=self.status, anchor='w', justify='left', wraplength=780,
            font=('Noto Sans CJK SC', -16), bg='#fff0cd', fg='#664b19', padx=10, pady=8).pack(fill='x')
        right = ttk.Frame(body, width=350); right.pack(side='right', fill='y')
        ttk.Label(right, text='仅软件仿真验收', font=('Noto Sans CJK SC', -22, 'bold')).pack(anchor='w')
        self.numbers = tk.StringVar()
        tk.Label(right, textvariable=self.numbers, anchor='w', justify='left', bg='#e7f1fc',
            font=('Noto Sans CJK SC', -18), padx=12, pady=12).pack(fill='x', pady=12)
        self.prepare_button = ttk.Button(right, text='准备 / 确认并重新准备', command=self.prepare)
        self.prepare_button.pack(fill='x', pady=5)
        ttk.Button(right, text='停止（Esc）', command=self.stop).pack(fill='x', pady=5)
        ttk.Label(right, text='① 拿稳杆，准备→就绪→单击空格\n② 再次空格或Esc停止（松键不停）\n失焦、遮挡、跳变或超时会停车。\n不会自动恢复。\n恢复：露出拿稳、松开空格，\n确认并重新准备→就绪→单击空格。',
            font=('Noto Sans CJK SC', -14), wraplength=340).pack(anchor='w', pady=8)
        ttk.Label(right, text='XY：0.70 m/s / 0.80 m/s²\n旋转：2.00 rad/s / 6.00 rad/s²\n120秒；并非实车动力学标定\n活动区：前0.5 / 后1.5 m\n左0.5 / 右0.4 m（沿用当前边界）',
            font=('Noto Sans CJK SC', -14), wraplength=340).pack(anchor='w', pady=4)
        ttk.Button(right, text='关闭仿真', command=self.close).pack(fill='x', pady=8)
        root.bind_class('RotationSimKeys', '<KeyPress-space>', self.press)
        root.bind_class('RotationSimKeys', '<KeyRelease-space>', self.release)
        root.bind_class('RotationSimKeys', '<Escape>', lambda _: self.stop())
        self.install_keys(root)
        root.bind('<FocusIn>', lambda _: self.schedule_focus())
        root.bind('<FocusOut>', lambda _: self.schedule_focus())
        root.protocol('WM_DELETE_WINDOW', self.close)
        if schedule: self.job = root.after(20, self.tick)

    def install_keys(self, widget):
        widget.bindtags(('RotationSimKeys', *widget.bindtags()))
        for child in widget.winfo_children(): self.install_keys(child)

    def schedule_focus(self):
        if self.closed: return
        if self.focus_job is not None: self.root.after_cancel(self.focus_job)
        self.focus_job = self.root.after(15, self.check_focus)

    def check_focus(self):
        self.focus_job = None
        widget = self.root.focus_displayof()
        focused = widget is not None and widget.winfo_toplevel() == self.root
        self.keys.focus(focused, self.key_state.down())

    def advance(self):
        self.reader.pump()
        now, ros = time.monotonic(), self.reader.ros_now()
        source = select_source(self.model, self.e.definition, now, ros)
        self.e.step(now, ros, source)
        return now

    def prepare(self):
        try:
            self.advance()
            if not self.keys.focused or self.keys.down or self.key_state.down():
                self.e.message = '请先点击本窗口并松开空格，再准备'; return
            self.e.prepare()
        except Exception as exc: self.fail(exc)

    def press(self, _=None):
        try:
            now = self.advance()  # An old queued click after a loop stall cannot enable.
            self.keys.press(now)
        except Exception as exc: self.fail(exc)
        return 'break'

    def release(self, _=None):
        try:
            # Reject X11 autorepeat Release/Press pairs while physically down.
            if not self.key_state.down():
                self.keys.release(self.advance(), commit=True)
        except Exception as exc: self.fail(exc)
        return 'break'

    def stop(self):
        self.keys.escape(); return 'break'

    def fail(self, exc):
        self.keys.pending = None
        self.e.stop('处理错误：'+str(exc), fault=True)
        self.e.gate.trip('LIVE_ADAPTER_EXCEPTION', hard=True)

    def tick(self):
        self.job = None
        if self.closed: return
        try:
            # Polling cancels a lost release; NEVER synthesizes startup authority.
            if self.keys.down and not self.key_state.down(): self.keys.release(time.monotonic(), commit=False)
            now = self.advance()
            if now-self.last_draw >= .033: self.draw(); self.last_draw = now
            if now-self.last_log >= .05:
                packet = [vars(s) for s in self.e.selected.samples] if self.e.selected else None
                self.log.write(json.dumps(dict(steady=now, ros=self.e.ros, report=self.e.report(),
                    coherent_source_packet=packet, geometry_valid=self.model.valid,
                    geometry_reason=self.model.reason, source_ok=self.reader.flags,
                    operator=dict(focused=self.keys.focused, key_down=self.keys.down,
                        click_pending=self.keys.pending is not None)), ensure_ascii=False)+'\n')
                self.last_log = now
            if self.logged_events < len(self.e.events):
                with (self.folder/'events.jsonl').open('a') as f:
                    for event in self.e.events[self.logged_events:]: f.write(json.dumps(event, ensure_ascii=False)+'\n')
                self.logged_events = len(self.e.events)
            if now-self.last_flush >= 1.: self.log.flush(); self.last_flush = now
        except Exception as exc: self.fail(exc); self.draw()
        if not self.closed: self.job = self.root.after(20, self.tick)

    def draw(self):
        e = self.e; r = e.report()
        self.prepare_button.configure(state='normal' if e.can_prepare() and self.keys.focused and not self.keys.down else 'disabled')
        reason = self.model.reason if not self.model.valid else (e.gate.fault or e.sup.reason)
        self.status.set(f'{e.sup.state}  |  {e.message}\n{reason}')
        hand = r['relative_hand_yaw']; car = r['relative_car_yaw']
        self.numbers.set(f'状态：{e.sup.state}\n剩余：{e.remaining:.1f} s\n杆相对角：'+
            ('未取零' if hand is None else f'{math.degrees(hand):+.1f}°')+'\n车相对角：'+
            ('未取零' if car is None else f'{math.degrees(car):+.1f}°')+
            f'\n软件车速度：{math.hypot(e.plant.velocity.vx,e.plant.velocity.vy):.2f} m/s')
        c = self.canvas; c.delete('all'); w, h = max(100, c.winfo_width()), max(100, c.winfo_height())
        scale = min((w-90)/2.6, (h-65)/3.7)
        def xy(f, l): return w/2-(l-.05)*scale, h/2-(f+.5)*scale
        def rect(bounds, fill, outline):
            xmin,xmax,ymin,ymax = bounds
            c.create_rectangle(*xy(xmax,ymax), *xy(xmin,ymin), fill=fill, outline=outline, width=2)
        rect((-2.2,1.2,-1.1,1.2), '#fbeada', '#c3a98e')
        rect((-1.5,.5,-.4,.5), '#e2f3e8', '#65a67a')
        x0,y0 = xy(0,0); c.create_oval(x0-3,y0-3,x0+3,y0+3,fill='#666')
        c.create_text(x0+6,y0+10,anchor='nw',text='原起点',font=('Noto Sans CJK SC',-13))
        def car_shape(p, color, fill):
            f,l = e.c.geometry().local(p.x,p.y); a = p.yaw-e.c.arena_heading_rad
            points=[]
            for df,dl in ((.095,.105),(.095,-.105),(-.095,-.105),(-.095,.105)):
                points.extend(xy(f+math.cos(a)*df-math.sin(a)*dl,l+math.sin(a)*df+math.cos(a)*dl))
            c.create_polygon(*points,outline=color,fill=fill,width=3)
            c.create_line(*xy(f,l),*xy(f+.19*math.cos(a),l+.19*math.sin(a)),fill=color,width=4,arrow='last')
        if e.sup.follower.mapped_goal is not None: car_shape(e.sup.follower.mapped_goal, '#db8c19', '')
        car_shape(e.plant.pose, '#2478bd', '#9bcbef')
        c.create_text(12,12,anchor='nw',text='软件车置于活动区中央；不读取实车位置作为反馈',
            font=('Noto Sans CJK SC',-14),fill='#476b7e')
        c.create_text(12,h-18,anchor='w',text='绿：沿用当前活动边界；橙：外围缓冲。旋转按整车轮廓检查。',
            font=('Noto Sans CJK SC',-13),fill='#476b7e')

    def close(self):
        if self.closed: return
        self.closed = True
        self.e.stop('关闭软件仿真窗口')
        for job in (self.job, self.focus_job):
            if job is not None: self.root.after_cancel(job)
        self.log.flush(); self.log.close()
        (self.folder/'SESSION_RESULT.json').write_text(json.dumps(dict(report=self.e.report(),
            events=self.e.events, real_sign_simulation_accepted=False,
            requires_user_and_log_review=True), ensure_ascii=False, indent=2)+'\n')
        self.key_state.close(); self.reader.close(); self.root.destroy()
