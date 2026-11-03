#!/usr/bin/env python3
"""Render ONLY a newly allocated private X display; no real desktop input."""
import argparse
from dataclasses import asdict
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import tkinter as tk
from types import SimpleNamespace as NS


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if not os.environ.get('ROTATION_HW_PRIVATE_DISPLAY') or os.environ['DISPLAY']!=os.environ['ROTATION_HW_PRIVATE_DISPLAY']:
        raise ValueError('Private display required')
    import pytest
    import test_ui as t
    from PIL import ImageGrab
    from mecanum_wand_4ws8_control.core import RelativeMapping
    from mecanum_wand_4ws8_control.rotation_servo import RotationMapping
    a.output.mkdir(exist_ok=False);report={};patch=pytest.MonkeyPatch()
    import socket,subprocess,rclpy
    def denied(*a,**kw):raise ValueError('Private render attempted non-display IO')
    patch.setattr(socket,'socket',denied);patch.setattr(subprocess,'Popen',denied);patch.setattr(rclpy,'init',denied)
    try:
        for name in ('launcher','ready','active','fault'):
            root=tk.Tk();root.geometry('1280x720+0+0')
            if name=='launcher':owner=t.app.App(root,preview=True)
            else:
                owner,m,calls=t.console(root,patch)
                if name=='active':
                    mapping=RotationMapping(RelativeMapping(owner.node.car,owner.node.raw_wand,1.));mapping.turn=.4
                    goal=mapping.goal_from_local(.12,.05)
                    m.status.update(state='ACTIVE',anchors=asdict(mapping),goal=asdict(goal),mapped_goal=asdict(goal))
                    m.held=True
                elif name=='fault':
                    m.status.update(state='FAULT',hard_fault=True,reason='ROTATION_SELECTED_SOURCE_INVALID: same-frame source validation failed')
                    m.gate.update(fault='ROTATION_SELECTED_SOURCE_INVALID',hard_fault=True)
                    owner.auto_prepare.cancel('来源不一致：已撤能，不自动恢复')
                    m.end_trial(time.monotonic(),'故障停车：检查Sign及五点，不自动恢复')
                owner.draw(time.monotonic())
            for job in root.tk.call('after','info'):root.after_cancel(job)
            root.update_idletasks();root.update()
            if name!='launcher':owner.draw(time.monotonic())
            root.update_idletasks()
            assert root.winfo_reqwidth()<=1280 and root.winfo_reqheight()<=720,(name,root.winfo_reqwidth(),root.winfo_reqheight())
            if name!='launcher':
                for key in ('stop','estop'):
                    w=owner.buttons[key]
                    assert w.winfo_ismapped() and w.winfo_rooty()+w.winfo_height()<=720
            ImageGrab.grab(xdisplay=os.environ['DISPLAY']).save(a.output/(name+'.png'))
            report[name]=dict(width=root.winfo_width(),height=root.winfo_height(),
                requested_width=root.winfo_reqwidth(),requested_height=root.winfo_reqheight(),physical_io=False)
            t.clean(root,owner)
        (a.output/'geometry.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)
    finally:patch.undo()


if __name__=='__main__':main()
