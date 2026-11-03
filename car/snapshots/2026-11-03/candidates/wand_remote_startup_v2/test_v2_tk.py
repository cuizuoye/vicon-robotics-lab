"""Actual Tk / X11, private display only. No ROS init or physical socket."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace as NS
import tkinter as tk
import pytest
from mecanum_wand_4ws8_control.async_x11 import AsyncX11KeyState
from test_ui import console as base_console,clean


@pytest.fixture(autouse=True)
def no_external_io(monkeypatch):
    import socket,rclpy
    monkeypatch.setenv('WAND_REMOTE_X11','1')
    def forbidden(*a,**k):pytest.fail('private Tk test attempted ROS/hardware IO')
    monkeypatch.setattr(socket,'socket',forbidden);monkeypatch.setattr(rclpy,'init',forbidden)


def test_40ms_queries_do_not_end_a_live_software_ui_session(monkeypatch):
    class Reader:
        def __init__(self,*a):pass
        def down(self):time.sleep(.040);return False
        def close(self):pass
    root=tk.Tk();root.withdraw();w,m,calls=base_console(root,monkeypatch)
    keys=AsyncX11KeyState(':software-40ms',reader_factory=Reader)
    try:
        w.key_state=keys;w.auto_prepare=None;w.check_focus=lambda:None
        m.focused=True;m.held=m.activation_used=True;m.status['state']='ACTIVE'
        times=[];errors=[];pending=[0];original=keys.sample_or_pending
        def sample():
            value=original()
            if value is None:pending[0]+=1
            return value
        keys.sample_or_pending=sample
        w.node.get_logger=lambda:NS(info=lambda *a:None,warn=lambda *a:None,error=errors.append)
        def pump():
            now=time.monotonic();m.status_at=m.gate_at=now;m.tick(now);times.append(now)
        w.node.pump=pump;w.node.heartbeat=lambda:None
        deadline=time.monotonic()+.9
        while time.monotonic()<deadline:root.update();time.sleep(.002)
        assert len(times)>25 and pending[0]>0 and not errors
        assert max(b-a for a,b in zip(times,times[1:]))<.100
        assert not w.closing and m.session_requested and not m.ended
        assert not any(v in ('start','recover') for v in calls)
    finally:keys.close();clean(root,w)


def test_full_console_readonly_probe_runs_without_ros_or_actuation():
    out=Path(os.environ['FEEDBACK_OUTPUT'])
    p=subprocess.run([sys.executable,str(out/'remote_probe.py'),'--seconds','1'],
        capture_output=True,text=True,timeout=15,env=dict(os.environ,WAND_REMOTE_X11='1'))
    assert p.returncode==0,p.stdout+p.stderr
    line=next(v for v in p.stdout.splitlines() if v.startswith('REMOTE_READONLY_RESULT '))
    result=json.loads(line.split(' ',1)[1])
    assert result['passed'] and result['completed'],result
    assert result['keyboard']['queries']>10
    assert not result['ros_initialized'] and not result['hardware_socket_opened']
    assert not result['motion_started'] and not result['authority_ever_acquired']
