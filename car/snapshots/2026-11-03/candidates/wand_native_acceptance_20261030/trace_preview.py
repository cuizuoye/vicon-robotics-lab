"""Actual C response / synthetic hand target preview; no ROS/control authority."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import time
from types import SimpleNamespace as NS
from common import config,diagnostic as d


def run(path,stage):
    import tkinter as tk
    from mecanum_wand_4wsb_control.operator_console import Console,OperatorModel
    from mecanum_wand_4wsb_control.core import Pose
    # This preview cannot become an input device, ROS node, or car transport.
    attempts=d.deny_live_io();os.environ.pop('WAND_AUTOPREP_FLOW',None)
    model=OperatorModel();model.focused=True
    node=NS(runtime=NS(config=config(stage),software_only=True,wand_reference_margin_m=0.),
        input_kind='hardware',trace=deque(maxlen=500),car=None,raw_wand=None)
    class Preview(Console):
        def press(self,*_):return 'break'
        def release(self,*_):return 'break'
        def finish_release(self,*_):return 'break'
        def schedule_focus(self):pass
        def check_focus(self):pass
        def stop(self):self.close()
        def escape(self,*_):self.close();return 'break'
        def emergency(self,*_):self.close();return 'break'
        def loop(self):
            self.loop_job=None
            if self.closing:return
            try:
                value=json.loads(path.read_text());reference=value.get('reference')
                car=value['car'];node.car=Pose(car[0],car[1],.1,car[2]);node.trace.append(tuple(car[:2]))
                if reference:
                    x,y=reference['x'],reference['y'];yaw=reference['yaw']
                    node.raw_wand=Pose(x,y,1.3,yaw)
                    model.status=dict(mapped_goal=dict(x=x,y=y,yaw=yaw),goal=dict(x=x,y=y,yaw=yaw))
                    error=math.hypot(x-car[0],y-car[1])
                    self.hand_text.set(f'橙：输入杆映射目标（1:1）\nX={x:+.2f}m Y={y:+.2f}m\n'
                        f'目标转角 {math.degrees(yaw):+.1f}°\n蓝：真实C控制器软件车\n位置误差 {error:.2f}m')
                self.status_text.set(f'实际ROS状态：{value["state"]}\n原因：{value.get("reason") or "无"}\n'
                    f'软件车：X={car[0]:+.2f}m Y={car[1]:+.2f}m 转角={math.degrees(car[2]):+.1f}°')
                self.step_text.set('自动软件测试进行中；无需操作Sign；本窗口没有使能权')
                self.draw_tracking_map()
            except (FileNotFoundError,json.JSONDecodeError):self.step_text.set('等待独立ROS+C测试数据；无实车输出')
            self.loop_job=root.after(40,self.loop)
        def close(self):
            if self.closing:return
            self.closing=True
            if self.loop_job:root.after_cancel(self.loop_job)
            root.destroy();self.key_state.close()
    root=tk.Tk();app=Preview(root,node,model)
    root.title('6×5m＋随机旋转 · 真实ROS+C软件测试 · 实车无输出')
    top=root.winfo_children()[0].winfo_children()
    top[0].configure(text='6×5m 大范围＋随机旋转 · 自动软件测试')
    top[1].configure(text=f'档{stage} · 橙：杆目标 / 蓝：C软件车 · 空格无效 · Esc仅关预览 · 无实车输出')
    for button in app.buttons.values():button.configure(state='disabled',command=lambda:None)
    app.notice.set('只读预览：模拟杆自动运动；蓝车来自C控制器输出。窗口不发布租约或运动指令。')
    for widget in app.pad.master.winfo_children():
        if isinstance(widget,tk.Label):widget.configure(text='模拟杆输入（自动测试）')
    for widget in root.winfo_children():
        if isinstance(widget,tk.Label) and str(widget.cget('text')).startswith('READY'):
            widget.configure(text='自动软件测试 · 空格无效 · Esc仅关闭预览 · 不涉及实车')
    signal.signal(signal.SIGTERM,lambda *_:app.close())
    try:root.mainloop()
    finally:
        if attempts:raise ValueError('read-only preview attempted control IO')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--path',type=Path,required=True)
    p.add_argument('--stage',type=int,required=True);a=p.parse_args();run(a.path,a.stage)
