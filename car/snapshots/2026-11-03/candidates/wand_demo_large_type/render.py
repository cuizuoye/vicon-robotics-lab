#!/usr/bin/env python3
"""Render only our own isolated Tk display; never capture the user's desktop."""
import argparse
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace as NS


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if os.environ.get('UI_PRIVATE_TEST_DISPLAY')!=os.environ.get('DISPLAY'):
        raise ValueError('Screenshots require our explicitly private nested X display')
    from PIL import ImageGrab
    import tkinter as tk
    from test_large_type import app,console_fixture,clean,theme
    from pytest import MonkeyPatch
    args.output.mkdir(exist_ok=False);results=[]
    def capture(root,owner,name,stops=()):
        # No active timers or bindings call into real ROS/network: fixtures
        # are in-memory only, and the preview constructor skips preflight.
        for attr in ('poll_job','loop_job','focus_job','release_job','shutdown_job','destroy_job'):
            job=getattr(owner,attr,None)
            if job is not None:root.after_cancel(job);setattr(owner,attr,None)
        root.unbind('<FocusIn>');root.unbind('<FocusOut>')
        root.geometry('1280x720+0+0');root.update_idletasks();root.update()
        if hasattr(owner,'draw'):
            # Freshness here belongs to the explicit in-memory fixture only.
            owner.model.status_at=owner.model.gate_at=time.monotonic()
            owner.draw(time.monotonic());root.update_idletasks()
        metrics=dict(case=name,requested=[root.winfo_reqwidth(),root.winfo_reqheight()],
            actual=[root.winfo_width(),root.winfo_height()],stop_buttons=[])
        for button in stops:
            bounds=[button.winfo_rootx(),button.winfo_rooty(),button.winfo_width(),button.winfo_height()]
            assert bounds[1]>=0 and bounds[1]+bounds[3]<=720,(name,bounds)
            metrics['stop_buttons'].append(bounds)
        ImageGrab.grab(xdisplay=os.environ['DISPLAY']).crop((0,0,1280,720)).save(args.output/(name+'.png'))
        results.append(metrics)
    root=tk.Tk();window=app.App(root,preview=True)
    try:capture(root,window,'launcher')
    finally:clean(root,window)
    with MonkeyPatch.context() as patch:
        root=tk.Tk();w,m,calls=console_fixture(root,patch)
        try:
            capture(root,w,'console_ready',(w.buttons['stop'],w.buttons['estop']))
            m.status.update(state='FAULT',hard_fault=True,reason='CAR_LIFT_OR_TILT')
            m.gate.update(fault='CAR_LIFT_OR_TILT',hard_fault=True)
            w.auto_prepare.cancel('故障停车；不自动确认、恢复或重试')
            m.end_trial(time.monotonic(),'故障停车；请检查提示，不会自动恢复')
            w.draw(time.monotonic())
            capture(root,w,'console_fault',(w.buttons['stop'],w.buttons['estop']))
            assert not calls and not m.held
        finally:clean(root,w)
    (args.output/'geometry.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=True,private_display_only=True,cases=results),ensure_ascii=False))


if __name__=='__main__':main()
