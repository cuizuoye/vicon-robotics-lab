"""Private-display UI fixture only; never initializes ROS or a hardware bridge."""
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import tkinter as tk

OUT=Path(os.environ['SYNC_OUTPUT'])
for name in ('mecanum_wand_4ws9_control','mecanum_wand_4ws9_hardware'):sys.path.insert(0,str(OUT/'runtime/src'/name))
from mecanum_wand_4ws9_control import operator_console as ui
from mecanum_wand_4ws9_control.rotation_core import RotationSafetyConfig
from mecanum_wand_4ws9_control.rotation_servo import RotationMapping
from mecanum_wand_4ws9_control.core import Pose,RelativeMapping
from mecanum_wand_4ws9_control.toggle_ui import ToggleOperatorModel
from mecanum_wand_4ws9_control.demo_policy import POLICY

def make(root,stage=6):
    c=RotationSafetyConfig(**json.loads((OUT/f'runtime/src/mecanum_wand_4ws9_hardware/config/stage{stage}.NOT_APPROVED.json').read_text())['safety'])
    now=time.monotonic();m=ToggleOperatorModel();m.focused=True;m.release(now)
    m.receive_status(dict(state='ARMED',gate_ready=True,reason='',epoch='preview-only',ready_to_start=True,stop_confirmed=True,
        graph_ok=True,input_health='',pilot=dict(used=False,max_active_s=120.,remaining_s=120.,prepare_blocked='',
            spatial_policy=POLICY['spatial_policy'],operator_input_mode=POLICY['operator_input_mode'])),now)
    m.receive_gate(dict(fault='',hard_fault=False,output_zero=True,graph_ok=True),now)
    m.transport_status=dict(fresh=True,ready=True,local_stop=False,fault='',baseline_current=True,
        safety_enabled=False,motion_gate_requested=False,command_progress_age_s=0.,startup_clock=dict(ready=True,qualified=True,fault=''))
    m.transport_at=now;calls=[]
    node=NS(runtime=NS(config=c,wand_reference_margin_m=.5),input_kind='hardware',trace=[],
        car=Pose(*c.geometry().world(-.5,.05),c.ground_z_m,c.arena_heading_rad),raw_wand=Pose(1.,2.,1.2,.3),
        request=lambda action:calls.append(action),input_hint=lambda:'')
    w=ui.Console(root,node,m);w.auto_prepare.phase='READY';w.draw(now)
    return w,m,calls

def close(root,w):
    for job in root.tk.call('after','info'):root.after_cancel(job)
    root.destroy()

def render(folder):
    if not os.environ.get('SYNC_PRIVATE_DISPLAY') or os.environ['DISPLAY']!=os.environ['SYNC_PRIVATE_DISPLAY']:
        raise ValueError('Private X required; never render user desktop')
    import pytest,rclpy,socket,subprocess
    from PIL import ImageGrab
    patch=pytest.MonkeyPatch()
    def denied(*a,**kw):raise ValueError('Offline preview attempted physical I/O')
    patch.setattr(rclpy,'init',denied);patch.setattr(socket,'socket',denied);patch.setattr(subprocess,'Popen',denied)
    patch.setattr(ui,'X11KeyState',lambda *a:NS(down=lambda:False,close=lambda:None))
    os.environ['WAND_AUTOPREP_FLOW']='AUTO_PREPARE_SINGLE_SPACE_V1';folder.mkdir(exist_ok=False);data={}
    try:
        for name,stage in (('tier0',0),('tier6',6),('fault',6)):
            root=tk.Tk();root.geometry('1280x720+0+0');w,m,calls=make(root,stage)
            if name=='tier6':
                mapping=RotationMapping(RelativeMapping(w.node.car,w.node.raw_wand,1.));mapping.turn=.5
                goal=mapping.goal_from_local(.1,.1)
                m.status.update(state='ACTIVE',anchors=asdict(mapping),mapped_goal=asdict(goal),goal=asdict(goal))
                m.held=True;m.notice='仅离线界面预览：数据来自软件夹具，没有实车输出'
            elif name=='fault':
                m.status.update(state='FAULT',reason='ROTATION_SELECTED_SOURCE_INVALID',hard_fault=True)
                m.gate.update(fault='ROTATION_SELECTED_SOURCE_INVALID',hard_fault=True)
                w.auto_prepare.cancel('来源不一致：已撤能，不自动恢复');m.end_trial(time.monotonic(),'软件故障示例，不是真实故障')
            for job in root.tk.call('after','info'):root.after_cancel(job)
            root.update();w.draw(time.monotonic());root.update_idletasks()
            assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
            for key in ('stop','estop'):
                b=w.buttons[key];assert b.winfo_ismapped() and b.winfo_rooty()+b.winfo_height()<=720
            assert not calls
            ImageGrab.grab(xdisplay=os.environ['DISPLAY']).save(folder/(name+'.png'))
            data[name]=dict(width=root.winfo_width(),height=root.winfo_height(),requested_height=root.winfo_reqheight(),stage=stage,physical_io=False)
            close(root,w)
        (folder/'geometry.json').write_text(json.dumps(data,indent=2)+'\n');print(json.dumps(data),flush=True)
    finally:patch.undo()

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);render(p.parse_args().output)
