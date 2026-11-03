#!/usr/bin/env python3
"""Same real Tk renderer/input worker, SOFTWARE animation, no ROS/actuator IO."""
import argparse
from collections import deque
from dataclasses import asdict
import json
import math
from pathlib import Path
import socket
import sys
import time
from types import SimpleNamespace as NS


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=float,default=60.)
    args=ap.parse_args()
    if not 1<=args.seconds<=120:raise ValueError('read-only duration must be1..120 seconds')
    ready=Path(__file__).resolve().parent
    for pkg in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
        sys.path.insert(0,str(ready/'runtime/src'/pkg))
    import rclpy
    def forbidden(*a,**k):raise AssertionError('read-only X11 probe forbids ROS init / Python sockets')
    rclpy.init=forbidden;socket.socket=forbidden
    import tkinter as tk
    from mecanum_wand_4ws8_control.operator_console import Console
    from mecanum_wand_4ws8_control.toggle_ui import ToggleOperatorModel
    from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig
    from mecanum_wand_4ws8_control.core import Pose
    c=TrackingSafetyConfig(**json.loads((ready/'runtime/src/mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())['safety'])
    m=ToggleOperatorModel();m.instruction=lambda now:'只读远程检查：软件动画；空格仅记录按键，不准备、不使能小车'
    m.status=dict(state='STANDBY',stop_confirmed=True,hard_fault=False,graph_ok=False,
        input_health='',goal=asdict(Pose(.2,0.,c.ground_z_m,0.)))
    m.gate=dict(fault='',hard_fault=False,output_zero=True,speed_mps=0.,graph_ok=False)
    errors=[];gaps=[];key_events={'press':0,'release':0};window=None;last=None;started=time.monotonic()
    node=NS(runtime=NS(config=c,wand_reference_margin_m=0.),input_kind='shadow',
        car=Pose(0.,0.,c.ground_z_m,0.),raw_wand=Pose(0.,0.,1.,0.),trace=deque(maxlen=500))
    def pump():
        nonlocal last
        now=time.monotonic()
        if last is not None:gaps.append(now-last)
        last=now;t=now-started
        node.car=Pose(.3*math.cos(t),.3*math.sin(t),c.ground_z_m,.15*math.sin(t))
        node.trace.append((node.car.x,node.car.y));node.raw_wand=node.car
        m.status['goal']=asdict(Pose(.3*math.cos(t+.2),.3*math.sin(t+.2),c.ground_z_m,0.))
        m.status_at=m.gate_at=now;m.tick(now)
        if m.session_requested or m.activation_used:raise AssertionError('Read-only probe acquired authority')
        m.notice=f'只读软件动画 · 已运行{t:.1f}秒 · 主循环最大间隔{max(gaps,default=0.)*1000:.1f}ms · 空格按下/松开{key_events}'
    node.pump=pump;node.heartbeat=lambda:None;node.input_hint=lambda:''
    node.get_logger=lambda:NS(info=lambda *a:None,warn=lambda value:print(value,flush=True),error=errors.append)
    node.request=lambda name:False  # No service clients, publishers or transport exist.
    root=tk.Tk();window=Console(root,node,m)
    window.tracking_hand_text=lambda:'纯软件显示样本\n不接Vicon / ROS / 小车\n空格不会启动运动\nEsc或关闭可结束检查'
    top=root.winfo_children()[0].winfo_children()
    top[0].configure(text='只读远程X11检查 · 软件动画 · 不控制小车',fg='#ffda86')
    top[1].configure(text=f'运行{args.seconds:g}秒 · 检查键盘查询和绘图调度 · 空格只记录，不使能')
    for child in window.map.master.winfo_children():
        if 'text' in child.keys() and str(child.cget('text')).startswith('蓝：'):
            child.configure(text='蓝：软件动画  橙：软件目标（不是实测车体/定位）')
    root.title('只读远程X11检查 · 无ROS / 无实车输出')
    for name in ('start','recover','acknowledge','pause'):window.buttons[name].pack_forget()
    def close(reason='operator'):
        if window.closing:return
        window.closing=m.closing=True;window.cancel_release()
        elapsed=time.monotonic()-started
        diag=window.key_state.diagnostics() if hasattr(window.key_state,'diagnostics') else {}
        window.finish_close()
        completed=reason=='timer' and elapsed>=args.seconds
        result=dict(passed=completed and not errors and max(gaps,default=0.)<=.100,
            completed=completed,reason=reason,elapsed_s=elapsed,max_ui_gap_s=max(gaps,default=0.),
            keyboard=diag,key_events=key_events,errors=errors,
            ros_initialized=False,hardware_socket_opened=False,motion_started=False,
            authority_ever_acquired=bool(m.activation_used or m.session_requested))
        print('REMOTE_READONLY_RESULT '+json.dumps(result,ensure_ascii=False),flush=True)
    window.close=lambda:close('input_error' if errors else 'operator')
    window.buttons['stop'].configure(text='结束只读检查',command=lambda:close('operator'))
    window.buttons['estop'].pack_forget()
    def event(kind):
        key_events[kind]+=1;return 'break'
    root.bind_class('SafeWandKeys','<KeyPress-space>',lambda _:event('press'))
    root.bind_class('SafeWandKeys','<KeyRelease-space>',lambda _:event('release'))
    root.bind_class('SafeWandKeys','<Escape>',lambda _:close('operator'))
    root.protocol('WM_DELETE_WINDOW',lambda:close('operator'))
    root.after(round(args.seconds*1000),lambda:close('timer'))
    root.mainloop()


if __name__=='__main__':main()
