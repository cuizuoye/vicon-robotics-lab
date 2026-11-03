"""No-fence UI tests; Tk is confined to a newly allocated Xephyr display."""
from dataclasses import asdict
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import tkinter as tk
import pytest

from test_tracking import app,config
OUT=Path(os.environ['TRACKING_OUTPUT']);ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0,str(ROOT/'docs/wand_demo_large_type_2026-10-26/prepared/tests'))
from test_autoprep import standby
from mecanum_wand_4ws8_control import operator_console as ui
from mecanum_wand_4ws8_control.core import Pose,RelativeMapping
from mecanum_wand_4ws8_control.rotation_servo import RotationMapping


@pytest.fixture(autouse=True)
def no_real_io(monkeypatch):
    import socket,subprocess,rclpy
    def denied(*a,**kw):pytest.fail('UI test attempted hardware/network/ROS IO')
    monkeypatch.setattr(socket,'socket',denied);monkeypatch.setattr(subprocess,'Popen',denied)
    monkeypatch.setattr(rclpy,'init',denied)


def clean(root,w):
    for job in root.tk.call('after','info'):root.after_cancel(job)
    root.destroy()


def console(root,monkeypatch):
    monkeypatch.setenv('WAND_AUTOPREP_FLOW','AUTO_PREPARE_SINGLE_SPACE_V1')
    monkeypatch.setattr(ui,'X11KeyState',lambda *a:NS(down=lambda:False,close=lambda:None))
    now=time.monotonic();m=standby(now);m.status.update(state='ARMED',gate_ready=True)
    c=config();calls=[]
    node=NS(runtime=NS(config=c,wand_reference_margin_m=0.),input_kind='hardware',trace=[(12.,-8.),(17.,-7.)],
        car=Pose(17.,-7.,c.ground_z_m,0.),raw_wand=Pose(30.,-20.,1.2,.3),
        request=lambda name:calls.append(name),input_hint=lambda:'')
    w=ui.Console(root,node,m);w.auto_prepare.phase='READY';w.draw(now)
    return w,m,calls


def test_no_fence_launcher_does_not_confirm_site_or_enable():
    root=tk.Tk();root.withdraw();w=app().App(root,preview=True)
    try:
        root.update_idletasks()
        assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
        assert '无固定围栏' in root.title() and not any(v.get() for v in w.checks)
        assert w.backend is None and str(w.open_button['state'])=='disabled'
    finally:clean(root,w)


@pytest.mark.parametrize('active',[False,True])
def test_map_is_only_autofit_view_and_stop_controls_remain_visible(monkeypatch,active):
    root=tk.Tk();root.withdraw();w,m,calls=console(root,monkeypatch)
    try:
        if active:
            mapping=RotationMapping(RelativeMapping(w.node.car,w.node.raw_wand,1.));mapping.turn=.2
            goal=mapping.goal_from_local(6.,3.)
            m.status.update(state='ACTIVE',anchors=asdict(mapping),mapped_goal=asdict(goal),goal=asdict(goal))
            m.held=True;w.draw(time.monotonic())
            assert w.map.find_withtag('rotation_target')
        root.update_idletasks()
        assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
        texts=[w.map.itemcget(i,'text') for i in w.map.find_all() if w.map.type(i)=='text']
        assert any('无固定围栏' in t for t in texts)
        assert not any('固定2.0' in t or '50cm' in t or '杆参考区' in t for t in texts)
        assert '无位移半径限制' in w.hand_text.get() and not calls
        for key in ('stop','estop'):assert w.buttons[key].winfo_manager()=='pack'
        w.buttons['stop'].invoke();w.buttons['estop'].invoke();assert calls==['stop','estop']
    finally:clean(root,w)
