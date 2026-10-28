"""Large-type launcher without site checkboxes. No motion/key commands here."""
import queue
from pathlib import Path
import tempfile
import tkinter as tk
from tkinter import ttk


def build_launcher(app,root,instance=None,preview=False):
    app.root,app.instance,app.preview=root,instance,preview
    app.events=queue.Queue();app.busy=False;app.good_at=-1e9
    app.receiver=app.backend=None;app.trial=None;app.trace_at=0;app.partial='';app.status={}
    app.folder=Path(tempfile.mkdtemp(prefix='tracking_auto_ui_')) if not preview else None
    root.title('小车全场演示 · MCU反馈300ms · 启动修正版 · 未点击空格不运动');root.geometry('1280x720');root.minsize(1000,640)
    style=ttk.Style(root)
    for name in ('TLabel','TButton'):style.configure(name,font=('Noto Sans CJK SC',-22))
    style.configure('TButton',padding=(10,9))
    frame=ttk.Frame(root,padding=16);frame.pack(fill='both',expand=True)
    ttk.Label(frame,text='小车全场演示 · 自动检查与准备',font=('Noto Sans CJK SC',-30,'bold')).pack(anchor='w')
    ttk.Label(frame,text='无固定围栏 · 1:1 · 0.70m/s / 0.80m/s² · 最长120秒\n'
        '自检通过自动打开操作台；看到READY后，单击空格才开始运动。',wraplength=1220).pack(anchor='w',pady=8)
    app.preconditions_text=tk.StringVar(value='使用前提（按你的默认设置，并非机器检测）：\n'
        '车已放稳、活动路径和停车余量空旷、有人能立即断电。')
    ttk.Label(frame,textvariable=app.preconditions_text,foreground='#925323',wraplength=1220).pack(anchor='w',pady=8)
    app.message=tk.StringVar(value='正在自检：不会使能或转轮' if not preview else '离线界面预览：没有连接设备，不能启动实车。')
    ttk.Label(frame,textvariable=app.message,font=('Noto Sans CJK SC',-24,'bold'),
        foreground='#17466b',wraplength=1220).pack(fill='x',pady=8)
    app.detail=tk.Text(frame,height=6,wrap='word',font=('Noto Sans CJK SC',-20),
        bg='#f3f6fa',fg='#1b344b',padx=10,pady=6)
    app.detail.pack(fill='both',expand=True,pady=6)
    app.append('自动核对文件、Tenda和Vicon；真实MCU时钟、零输出和停稳握手仍须通过。\n'
        '定位/通信异常不会忽略；不会自动清故障、替你按空格或结束后自动续跑。')
    buttons=ttk.Frame(frame);buttons.pack(fill='x',pady=8)
    app.check_button=ttk.Button(buttons,text='重新检查连接',command=app.preflight);app.check_button.pack(side='left')
    # Retain the original update state/callback API, but no manual Open step.
    app.open_button=ttk.Button(buttons,text='自动打开（无需点击）',command=app.open_console)
    ttk.Button(buttons,text='关闭 / 停止本次程序',command=app.close).pack(side='right')
    ttk.Label(frame,text='READY后轻按并松开空格开始；再次空格 / Esc停止；松键不停车。\n'
        '失焦、定位丢失或故障会停车；没有障碍检测，Vicon可见不代表周围无障碍。',
        wraplength=1220).pack(anchor='w',pady=8)
    app.closed=False;root.protocol('WM_DELETE_WINDOW',app.close);app.update_buttons()
    app.poll_job=root.after(100,app.poll)
    if not preview:root.after(200,app.preflight)
