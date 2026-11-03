"""Presentation only: larger physical-pixel text, no controller/key changes."""
import tkinter as tk
from tkinter import ttk

FAMILY='Noto Sans CJK SC'
BODY=22
SECONDARY=20
STATUS=24
TITLE=30


def widgets(parent):
    yield parent
    for child in parent.winfo_children():yield from widgets(child)


def text_of(widget):
    return str(widget.cget('text')) if 'text' in widget.keys() else ''


def variable_label(root,variable):
    found=[w for w in widgets(root) if 'textvariable' in w.keys()
        and str(w.cget('textvariable'))==str(variable)]
    if len(found)!=1:raise ValueError('Expected exactly one presentation label for '+str(variable))
    return found[0]


def style(root):
    s=ttk.Style(root)
    s.configure('TLabel',font=(FAMILY,-BODY))
    s.configure('TButton',font=(FAMILY,-BODY),padding=(10,7))
    s.configure('TCheckbutton',font=(FAMILY,-BODY))
    s.configure('Demo.Stop.TButton',font=(FAMILY,-STATUS,'bold'),padding=(10,9))
    s.configure('Demo.Estop.TButton',font=(FAMILY,-STATUS,'bold'),padding=(10,9),foreground='#a21d22')


def apply_launcher_theme(root,app):
    style(root)
    root.title('小车演示 · 清晰大字版 · 1280×720')
    frame=app.detail.master
    frame.configure(padding=14)
    labels=[w for w in frame.winfo_children() if isinstance(w,ttk.Label)]
    labels[0].configure(text='小车跟随演示',font=(FAMILY,-TITLE,'bold'))
    labels[1].configure(text='1:1 · 0.70 m/s / 0.80 m/s² · 最长120秒\n'
        '打开不运动；检查通过后单击空格开始，再次空格或Esc停止',
        font=(FAMILY,-BODY),wraplength=1190)
    labels[1].pack_configure(pady=5)
    variable_label(root,app.message).configure(font=(FAMILY,-STATUS,'bold'),wraplength=1190)
    variable_label(root,app.message).pack_configure(pady=6)
    app.detail.configure(font=(FAMILY,-SECONDARY),height=3,width=72,padx=10,pady=5,
        bg='#f3f6fa',fg='#1b344b',insertwidth=2)
    checks=[w for w in frame.winfo_children() if isinstance(w,ttk.Checkbutton)]
    if len(checks)!=3:raise ValueError('Site confirmations changed; cannot apply presentation')
    captions=(
        '小车已落地、位于绿色区域内且朝向正确；Sign已拿稳可见',
        '前0.5 / 后1.5 / 左0.5 / 右0.4米及四边50厘米缓冲空旷；接线固定',
        '有人能立即断电；松键不停车，再次空格 / Esc停止；急停仅用于异常')
    for w,caption in zip(checks,captions):
        w.configure(text=caption);w.pack_configure(pady=4)
    app.check_button.master.pack_configure(pady=7)
    # The existing close/stop callback is retained exactly.
    for w in app.check_button.master.winfo_children():
        if text_of(w)=='关闭 / 停止本次程序':w.configure(style='Demo.Stop.TButton')
    labels[-1].configure(text='确认上述三项 → 自动检查和准备 → READY后单击空格开始\n'
        '再次空格或Esc停车；松键不停车；切出窗口、遮挡或超时也会停车。',
        font=(FAMILY,-SECONDARY),wraplength=1190)
    labels[-1].pack_configure(pady=5)


def apply_console_theme(root,console):
    style(root)
    # Canvas and all original model, key bindings, service callbacks and timers
    # remain owned by the accepted Console. Only widget presentation changes.
    for w in widgets(root):
        if isinstance(w,tk.Label):w.configure(font=(FAMILY,-SECONDARY))
    top=root.winfo_children()[0]
    top_labels=top.winfo_children()
    top_labels[0].configure(text='真实 Sign → 小车跟随演示',font=(FAMILY,-TITLE,'bold'))
    top_labels[0].pack_configure(pady=(8,2))
    top_labels[1].configure(text='1:1 · 0.70 m/s / 0.80 m/s² · 最长120秒 · 打开与准备均不运动',
        font=(FAMILY,-SECONDARY),wraplength=1220)
    top_labels[1].pack_configure(pady=(0,7))
    variable_label(root,console.step_text).configure(font=(FAMILY,-STATUS,'bold'),wraplength=750,pady=7)
    variable_label(root,console.status_text).configure(font=(FAMILY,-SECONDARY),wraplength=750,pady=6)
    variable_label(root,console.notice).configure(font=(FAMILY,-SECONDARY),wraplength=1220,pady=6)
    variable_label(root,console.hand_text).configure(font=(FAMILY,-BODY),wraplength=370)
    console.map.configure(width=690,height=190)
    for w in console.map.master.winfo_children():
        if isinstance(w,tk.Label) and w is not variable_label(root,console.step_text) and w is not variable_label(root,console.status_text):
            w.configure(text='蓝：实测车体   橙：目标   红：固定围栏',font=(FAMILY,-BODY))
    right=console.pad.master
    for w in right.winfo_children():
        if isinstance(w,tk.Label):
            w.configure(text='真实 Sign 输入（只读）',font=(FAMILY,-BODY),wraplength=370)
        # Hardware pad and its disabled mouse button are redundant with the
        # large position readout and map. Virtual mode retains its drag input.
        if not console.virtual and text_of(w)=='输入来自真实Sign；禁止鼠标替代':w.pack_forget()
    if not console.virtual:console.pad.pack_forget()
    console.buttons['stop'].configure(text='停止 / 结束本轮',style='Demo.Stop.TButton')
    console.buttons['estop'].configure(text='紧急停止（锁存）',style='Demo.Estop.TButton')
    # Footer is display text only; it never changes the single-click semantics.
    for w in root.winfo_children():
        if isinstance(w,tk.Label) and text_of(w).startswith('自动检查与准备'):
            w.configure(text='READY后单击空格开始  ·  再次空格 / Esc停止  ·  松键不停车',
                font=(FAMILY,-BODY),pady=7,wraplength=1220)
    root.title('小车演示 · 清晰大字版 · 单击启停 · 最长120秒')


def canvas_fonts(canvas):
    for item in canvas.find_all():
        if canvas.type(item)!='text':continue
        text=canvas.itemcget(item,'text')
        canvas.itemconfigure(item,font=(FAMILY,-SECONDARY))
        if text.startswith('绿：'):
            canvas.itemconfigure(item,text='绿：车中心目标区 · 红：车体与50cm缓冲外框',anchor='nw',
                width=max(100,canvas.winfo_width()-24))
            canvas.coords(item,12,6)
        elif text.startswith('橙虚线：'):
            canvas.itemconfigure(item,text='橙虚线：20cm硬停余量 · 紫：杆参考区（覆盖未验收）',anchor='nw',
                width=max(100,canvas.winfo_width()-24))
            canvas.coords(item,12,35)


class LargeTypeLauncherMixin:
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        apply_launcher_theme(self.root,self)


class LargeTypeConsoleMixin:
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        apply_console_theme(self.root,self)
    def draw(self,now):
        super().draw(now)
        canvas_fonts(self.map)
        policy=getattr(self,'auto_prepare',None)
        if (policy is not None and policy.phase=='READY'
                and not self.model.session_requested and not self.model.ended):
            self.notice.set('已准备，尚未使能：拿稳Sign，单击空格才开始；再次空格或Esc停止。')
