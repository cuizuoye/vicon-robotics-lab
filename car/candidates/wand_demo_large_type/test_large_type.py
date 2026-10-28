"""Real Tk geometry/readability and unchanged callbacks, without physical IO."""
import ast
import importlib.util
import os
from pathlib import Path
import time
from types import SimpleNamespace as NS

import pytest
import tkinter as tk
from tkinter import font as tkfont,ttk
from support import ROOT,GENERATED,config
from test_autoprep import standby
from mecanum_wand_4ws8_control import operator_console as ui
from mecanum_wand_4ws8_control.core import Pose
from mecanum_wand_4ws8_control import large_type as theme

OUT=Path(os.environ['AUTOPREP_OUTPUT'])
OLD=ROOT/'docs/wand_demo_startup_fixed_2026-10-26/prepared'
spec=importlib.util.spec_from_file_location('large_type_app_under_test',OUT/'oneclick_app.py')
app=importlib.util.module_from_spec(spec);spec.loader.exec_module(app)


@pytest.fixture(autouse=True)
def no_real_io(monkeypatch):
    import socket,rclpy,subprocess
    def forbidden(*a,**kw):pytest.fail('UI test attempted real connection or process startup')
    monkeypatch.setattr(socket,'socket',forbidden);monkeypatch.setattr(rclpy,'init',forbidden)
    monkeypatch.setattr(subprocess,'Popen',forbidden)


def clean(root,owner):
    for attr in ('poll_job','loop_job','focus_job','release_job','shutdown_job','destroy_job'):
        job=getattr(owner,attr,None)
        if job is not None:
            try:root.after_cancel(job)
            except tk.TclError:pass
    root.destroy()


def pixel_height(root,widget):
    value=widget.cget('font') if 'font' in widget.keys() else ttk.Style(root).lookup(widget.cget('style') or widget.winfo_class(),'font')
    return tkfont.Font(root=root,font=value).metrics('linespace')


def console_fixture(root,monkeypatch):
    monkeypatch.setenv('WAND_AUTOPREP_FLOW','AUTO_PREPARE_SINGLE_SPACE_V1')
    monkeypatch.setattr(ui,'X11KeyState',lambda *a:NS(down=lambda:False,close=lambda:None))
    now=time.monotonic();m=standby(now);m.status.update(state='ARMED',gate_ready=True)
    c=config();calls=[]
    node=NS(runtime=NS(config=c,wand_reference_margin_m=.5),input_kind='hardware',
        trace=[],car=Pose(*c.geometry().world(-.5,.05),c.ground_z_m,c.arena_heading_rad),
        raw_wand=Pose(1.,2.,1.2),request=lambda name:calls.append(name),input_hint=lambda:'')
    window=ui.Console(root,node,m);window.auto_prepare.phase='READY'
    window.draw(now)
    return window,m,calls


def test_launcher_large_text_all_confirmations_and_720p_fit(monkeypatch):
    root=tk.Tk();root.withdraw();geometry=[];real_geometry=root.geometry
    monkeypatch.setattr(root,'geometry',lambda value:(geometry.append(value),real_geometry(value))[1])
    w=app.App(root,preview=True)
    try:
        root.update_idletasks()
        assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
        assert geometry==['1280x720'] # A withdrawn X window itself still reports 1x1.
        assert not any(v.get() for v in w.checks) and not w.auto_open_attempted
        assert w.backend is None and w.preview
        assert str(w.check_button['state'])=='disabled'
        assert pixel_height(root,w.detail)>=24
        assert pixel_height(root,theme.variable_label(root,w.message))>=28
        checks=[v for v in theme.widgets(root) if isinstance(v,ttk.Checkbutton)]
        assert len(checks)==3 and all(pixel_height(root,v)>=26 for v in checks)
    finally:clean(root,w)


def test_console_large_text_visible_stop_and_unchanged_request_callbacks(monkeypatch):
    root=tk.Tk();root.withdraw();w,m,calls=console_fixture(root,monkeypatch)
    try:
        root.update_idletasks()
        assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
        assert not w.buttons['start'].winfo_manager() and not calls and not m.held
        assert w.buttons['stop'].winfo_manager()=='pack' and w.buttons['estop'].winfo_manager()=='pack'
        assert pixel_height(root,theme.variable_label(root,w.step_text))>=28
        assert pixel_height(root,w.buttons['stop'])>=28
        assert w.buttons['stop'].bindtags()[0]=='SafeWandKeys'
        assert not w.pad.winfo_manager() # Only redundant read-only disk removed.
        w.buttons['stop'].invoke();w.buttons['estop'].invoke()
        assert calls==['stop','estop'] and not m.held
    finally:clean(root,w)


@pytest.mark.parametrize('state',['ready','waiting','fault','long_fault'])
def test_status_variants_keep_stops_inside_window(monkeypatch,state):
    root=tk.Tk();root.withdraw();w,m,calls=console_fixture(root,monkeypatch)
    try:
        if state=='waiting':m.status.update(state='STANDBY',gate_ready=False,input_health='WAND_POSE_TIMEOUT')
        elif state in ('fault','long_fault'):
            m.status.update(state='FAULT',hard_fault=True,reason='ACTUATOR_FAULT')
            m.gate.update(fault='ACTUATOR_FAULT',hard_fault=True)
            if state=='long_fault':m.status['reason']='CAR_LIFT_OR_TILT: marker orientation inconsistent; release enable and inspect'
            w.auto_prepare.cancel('状态已撤销或存在故障；不自动确认、恢复或重试')
            m.end_trial(time.monotonic(),'故障停车；请检查提示，不会自动恢复')
        w.draw(time.monotonic());root.update_idletasks()
        assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720
        # Geometry is checked while mapped below by the renderer as well.
        assert not calls and not m.held
    finally:clean(root,w)


def test_runtime_and_launcher_behavior_are_identical_except_exact_display_overlay():
    control='mecanum_wand_4ws8_control/mecanum_wand_4ws8_control'
    relative=control+'/operator_console.py'
    old=(OLD/'runtime/src'/relative).read_text();new=(GENERATED/relative).read_text()
    block="""from .large_type import LargeTypeConsoleMixin

class LargeTypeConsole(LargeTypeConsoleMixin, AutomaticPrepareConsole):
    pass

Console = LargeTypeConsole"""
    assert new==old.replace('Console = AutomaticPrepareConsole',block)
    oldapp=ast.parse((OLD/'oneclick_app.py').read_text());newapp=ast.parse((OUT/'oneclick_app.py').read_text())
    def behavior(tree):
        return {n.name:ast.dump(n,include_attributes=False) for n in tree.body
            if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name!='App'}
    assert behavior(oldapp)==behavior(newapp)
    for p in (OLD/'runtime/src').rglob('*'):
        if not p.is_file() or any(k=='__pycache__' or k.endswith('.egg-info') for k in p.parts):continue
        r=p.relative_to(OLD/'runtime/src')
        if str(r)!=relative:assert (GENERATED/r).read_bytes()==p.read_bytes(),str(r)
