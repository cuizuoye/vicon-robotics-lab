"""Private-X Tk rendering/input-loop tests; no ROS/hardware socket."""
from collections import deque
import os
import time
from types import SimpleNamespace as NS
import tkinter as tk
import pytest

from mecanum_wand_4ws8_control import operator_console as ui
from mecanum_wand_4ws8_control.async_x11 import AsyncX11KeyState
from mecanum_wand_4ws8_control.core import Pose
from test_ui import console as base_console,clean


@pytest.fixture(autouse=True)
def no_external_io(monkeypatch):
    import socket,rclpy
    monkeypatch.setenv('WAND_REMOTE_X11','1')
    def forbidden(*a,**k):pytest.fail('private Tk test attempted physical/ROS IO')
    monkeypatch.setattr(socket,'socket',forbidden);monkeypatch.setattr(rclpy,'init',forbidden)


def test_retained_items_and_bounded_trace(monkeypatch):
    root=tk.Tk();root.withdraw();w,m,calls=base_console(root,monkeypatch)
    try:
        w.node.trace=deque([(12.+i*.001,-8.+i*.001) for i in range(500)],maxlen=500)
        w.draw(time.monotonic());items=w.map.find_all()
        for _ in range(30):w.draw(time.monotonic())
        assert w.map.find_all()==items and len(items)<=8
        trace=w.map.find_withtag('car_trace')[0]
        assert len(w.map.coords(trace))<=244 and not calls
        assert w.buttons['stop'].winfo_manager()=='pack' and w.buttons['estop'].winfo_manager()=='pack'
    finally:clean(root,w)


def test_slow_remote_key_reads_do_not_block_gui_heartbeats(monkeypatch):
    class Reader:
        def __init__(self,name):pass
        def down(self):time.sleep(.025);return False
        def close(self):pass
    root=tk.Tk();root.withdraw();w,m,calls=base_console(root,monkeypatch)
    keys=AsyncX11KeyState(':synthetic-remote',reader_factory=Reader)
    try:
        w.key_state=keys;w.auto_prepare=None
        w.model.focused=True;times=[];warm=time.monotonic()
        w.node.pending=set();w.node.service_clients={'start':NS(service_is_ready=lambda:False)}
        w.node.get_logger=lambda:NS(info=lambda *a:None,warn=lambda *a:None,error=lambda *a:None)
        def pump():
            now=time.monotonic();m.status_at=m.gate_at=now;m.tick(now);times.append(now)
        w.node.pump=pump
        w.node.heartbeat=lambda:None
        deadline=time.monotonic()+.65
        while time.monotonic()<deadline:root.update();time.sleep(.002)
        assert len(times)>15
        gaps=[b-a for a,b in zip(times,times[1:]) if a>warm+.10]
        assert max(gaps)<.100,gaps
        assert not m.session_requested and not any(v in ('start','recover') for v in calls)
    finally:keys.close();clean(root,w)
