"""Private Tk-only preview of the migrated large-type console."""
import importlib.util
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import tkinter as tk
from tkinter import ttk
import pytest

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');OUT=Path(os.environ['ROTATION_HW_OUTPUT'])
sys.path.insert(0,str(OUT));sys.path.insert(0,str(ROOT/'docs/wand_demo_large_type_2026-10-26/prepared/tests'))
from test_autoprep import standby
from test_migration import config
from mecanum_wand_4ws8_control import operator_console as ui
from mecanum_wand_4ws8_control.core import Pose
from mecanum_wand_4ws8_control.rotation_servo import RotationMapping
from mecanum_wand_4ws8_control.core import RelativeMapping
from dataclasses import asdict
spec=importlib.util.spec_from_file_location('rotation_app_ui',OUT/'oneclick_app.py')
app=importlib.util.module_from_spec(spec);spec.loader.exec_module(app)


@pytest.fixture(autouse=True)
def no_real_io(monkeypatch):
    import socket,rclpy,subprocess
    def denied(*a,**kw):pytest.fail('UI test attempted external I/O')
    monkeypatch.setattr(socket,'socket',denied);monkeypatch.setattr(rclpy,'init',denied)
    monkeypatch.setattr(subprocess,'Popen',denied)


def clean(root,w):
    for name in ('poll_job','loop_job','focus_job','release_job','shutdown_job','destroy_job'):
        if getattr(w,name,None):
            try:root.after_cancel(getattr(w,name))
            except tk.TclError:pass
    root.destroy()


def console(root,monkeypatch):
    monkeypatch.setenv('WAND_AUTOPREP_FLOW','AUTO_PREPARE_SINGLE_SPACE_V1')
    monkeypatch.setattr(ui,'X11KeyState',lambda *a:NS(down=lambda:False,close=lambda:None))
    now=time.monotonic();m=standby(now);m.status.update(state='ARMED',gate_ready=True)
    c=config();calls=[]
    node=NS(runtime=NS(config=c,wand_reference_margin_m=.5),input_kind='hardware',trace=[],
        car=Pose(*c.geometry().world(-.5,.05),c.ground_z_m,c.arena_heading_rad),
        raw_wand=Pose(1.,2.,1.2,.3),request=lambda name:calls.append(name),input_hint=lambda:'')
    w=ui.Console(root,node,m);w.auto_prepare.phase='READY';w.draw(now)
    return w,m,calls


def test_candidate_launcher_keeps_explicit_site_checks_and_fits(monkeypatch):
    root=tk.Tk();root.withdraw();w=app.App(root,preview=True)
    try:
        root.update_idletasks()
        assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
        assert '旋转' in root.title() and not any(v.get() for v in w.checks)
        assert w.backend is None and str(w.open_button['state'])=='disabled'
    finally:clean(root,w)


@pytest.mark.parametrize('active',[False,True])
def test_readable_console_keeps_stop_and_estop_and_yaw_arrow(monkeypatch,active):
    root=tk.Tk();root.withdraw();w,m,calls=console(root,monkeypatch)
    try:
        if active:
            mapping=RotationMapping(RelativeMapping(w.node.car,w.node.raw_wand,1.));mapping.turn=.2
            goal=mapping.goal_from_local(.1,.1)
            m.status.update(state='ACTIVE',anchors=asdict(mapping),mapped_goal=asdict(goal),goal=asdict(goal))
            m.held=True;w.draw(time.monotonic())
            assert w.map.find_withtag('rotation_target') and '杆相对角' in w.hand_text.get()
        root.update_idletasks()
        assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
        assert w.buttons['stop'].winfo_manager()=='pack' and w.buttons['estop'].winfo_manager()=='pack'
        assert not calls
        w.buttons['stop'].invoke();w.buttons['estop'].invoke();assert calls==['stop','estop']
    finally:clean(root,w)
