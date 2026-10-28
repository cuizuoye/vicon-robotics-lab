#!/usr/bin/env python3
"""Private Xephyr + Tk + native C oracle; NO real keyboard, UDP or ROS nodes."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

def child(output):
    # A mistaken attempt to open a Python network socket fails this test.
    import socket
    def forbidden(*a,**k):raise AssertionError('UI smoke must never open a network socket')
    socket.socket=forbidden
    from test_stage0 import Oracle
    from mecanum_wand_4ws7_hardware.suspended_core import SuspendedController
    from mecanum_wand_4ws7_hardware.suspended_console import Worker,SuspendedConsole
    import tkinter as tk
    from PIL import ImageGrab
    from mecanum_vicon_control.transport_protocol import decode_frame,MessageType,unpack_cmd_vel,FrameFlags
    class Key:
        pressed=False
        def down(self):return self.pressed
        def close(self):pass
    class IO:
        def __init__(self):self.mcu=Oracle();self.anchor=time.monotonic();self.queue=[];self.tx=[]
        def send(self,raw,now):
            self.tx.append(raw)
            ack=self.mcu.request(raw,now-self.anchor)
            if ack:self.queue.append(ack)
        def poll(self,now):
            output,self.queue=self.queue,[]
            return output+self.mcu.packets(now-self.anchor)
        def close(self):pass
    result=dict(passed=False,real_udp=False,ros_nodes=False,real_keyboard=False)
    key=Key();io=IO();worker=Worker(SuspendedController(),io);root=tk.Tk()
    callback_errors=[]
    root.report_callback_exception=lambda *args:callback_errors.append(str(args))
    app=SuspendedConsole(root,worker,key)
    focus_trace=[];original_focus=app.focused
    def traced_focus():
        current=str(root.tk.call('focus','-displayof',root._w))
        if not focus_trace or focus_trace[-1]['path']!=current:
            focus_trace.append(dict(at=time.monotonic(),path=current))
        return original_focus()
    app.focused=traced_focus
    def nonzero():
        return sum(f.message_type==MessageType.CMD_VEL and any((v.vx_mps,v.vy_mps,v.wz_radps))
            for f in (decode_frame(raw) for raw in io.tx)
            for v in ([unpack_cmd_vel(f.payload)] if f.message_type==MessageType.CMD_VEL else []))
    def pump(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:root.update();time.sleep(.002)
    def wait(predicate,seconds=4):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            pump(.02)
            with worker.lock:
                if predicate():return
        raise AssertionError((worker.core.state,worker.core.reason,worker.core.events))
    def unlock_prepare():
        root.focus_force();pump(.10)
        assert app.focused()
        key.pressed=False;pump(.10)
        key.pressed=True;pump(.08);key.pressed=False;pump(.08)
        assert worker.core.unlocked
        app.prepare_button.invoke();wait(lambda:worker.core.state=='READY')
        assert not worker.core.status['permitted']
    try:
        root.update();root.focus_force();worker.thread.start();pump(.12)
        # Real Tcl-created ttk popdown, intentionally missing from Python's
        # widget map: reproduce the original KeyError without using real keys.
        root.tk.call('ttk::combobox::Post',str(app.combo))
        popup=str(root.tk.call('ttk::combobox::PopdownWindow',str(app.combo)))
        root.tk.call('focus','-force',popup+'.f.l');pump(.10)
        reproduced=False
        try:root.focus_displayof()
        except KeyError:reproduced=True
        assert reproduced,'regression did not reach the real unmapped Tcl popdown'
        assert app.focused() is False and not callback_errors
        assert not nonzero() and worker.core.state=='IDLE'
        key.pressed=True;pump(.10)
        assert not worker.core.unlocked and not nonzero()
        root.tk.call('ttk::combobox::Unpost',str(app.combo));root.focus_force();pump(.10)
        assert not worker.core.unlocked and not nonzero() # Held across popup closure is NOT a new press.
        key.pressed=False;pump(.10)
        for index in range(8):app.combo.current(index);pump(.03)
        app.combo.current(0)
        unlock_prepare();key.pressed=True;wait(lambda:worker.core.state=='ACTIVE')
        # Focused ttk button + repeat pair must not re-prepare or release.
        app.prepare_button.focus_force()
        root.event_generate('<KeyRelease-space>');pump(.08)
        root.event_generate('<KeyPress-space>');pump(.08)
        assert worker.core.state=='ACTIVE' and worker.core.down
        key.pressed=False;wait(lambda:worker.core.state=='COMPLETE')
        unlock_prepare();assert not worker.core.status['permitted']
        before=nonzero();pump(.15);assert nonzero()==before
        key.pressed=True;wait(lambda:worker.core.state=='ACTIVE')
        wait(lambda:worker.core.state=='COMPLETE',7.)
        before=nonzero();pump(.35)
        assert nonzero()==before and not worker.core.unlocked
        assert str(app.prepare_button['state'])=='disabled'
        app.poll();root.update_idletasks();ImageGrab.grab(xdisplay=os.environ['DISPLAY']).save(output/'suspended_console.png')
        unlock_prepare();key.pressed=True;wait(lambda:worker.core.state=='ACTIVE')
        other=tk.Toplevel(root);other.title('Private focus-loss test');other.update();other.focus_force()
        wait(lambda:worker.core.state in ('STOPPING','COMPLETE'))
        key.pressed=False;wait(lambda:worker.core.state=='COMPLETE');other.destroy();root.focus_force();pump(.10)
        unlock_prepare();key.pressed=True;wait(lambda:worker.core.state=='ACTIVE')
        time.sleep(.25) # freeze ONLY our private GUI, independent safety worker continues
        with worker.lock:assert worker.core.state in ('STOPPING','FAULT')
        key.pressed=False;wait(lambda:worker.core.state=='FAULT')
        # Unexpected input/display exceptions revoke and reschedule, rather
        # than silently terminating the GUI safety heartbeat.
        unlock_prepare();key.pressed=True;wait(lambda:worker.core.state=='ACTIVE')
        original=app.focused
        def input_error():raise RuntimeError('injected focus-query failure')
        app.focused=input_error;app.poll();app.focused=original
        key.pressed=False;wait(lambda:worker.core.state=='FAULT')
        assert '操作台输入/显示异常' in worker.core.first_fault_reason
        assert not callback_errors
        unlock_prepare();key.pressed=True;wait(lambda:worker.core.state=='ACTIVE')
        app.close()
        assert not worker.thread.is_alive()
        final=[decode_frame(raw) for raw in io.tx[-4:]]
        stop_index=next(i for i,f in enumerate(final) if f.message_type==MessageType.CONTROL_ENABLE and f.payload[0]==0)
        assert any(f.message_type==MessageType.CMD_VEL for f in final[stop_index:])
        assert all(not f.flags&FrameFlags.ENABLED for f in final[stop_index:] if f.message_type==MessageType.CMD_VEL)
        result.update(passed=True,checks=['real Tcl popdown reproduces old KeyError',
            'popdown is non-authoritative and never crashes polling','held Space across popup closure cannot enable',
            'all eight selections remain zero',
            'production flags and 10Hz cadence','prepare zero','held physical-state surrogate',
            'repeat pair ignored before ttk bindings','release stops','manual second prepare',
            'five-second one-shot while held; no automatic repeat',
            'focus loss stops','GUI freeze independent lease','UI exception revokes and reschedules',
            'window close while held revokes and joins worker','no automatic resume'],events=worker.core.events,
            callback_errors=callback_errors)
    except Exception as error:
        result.update(error=repr(error),traceback=traceback.format_exc())
    finally:
        app.close()
        result['focus_trace']=focus_trace
        result['events']=worker.core.events
        (output/'ui_summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    return 0 if result['passed'] else 1

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--child',action='store_true')
    a=p.parse_args();out=a.output.resolve()
    if a.child:return child(out)
    out.mkdir(parents=True,exist_ok=False)
    display=next(':'+str(i) for i in range(84,100) if not Path('/tmp/.X11-unix/X'+str(i)).exists())
    process=None
    try:
        with (out/'xserver.log').open('x') as log:
            process=subprocess.Popen(['Xephyr',display,'-screen','940x560','-nolisten','tcp','-noreset','-ac',
                '-title','4WS7 OFFLINE GUI TEST · native C oracle only'],stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+5
            while not Path('/tmp/.X11-unix/X'+display[1:]).exists():
                if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('private X server failed')
                time.sleep(.02)
            env=os.environ.copy();env['DISPLAY']=display
            result=subprocess.run([sys.executable,__file__,'--child','--output',str(out)],env=env,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=35)
            (out/'ui.log').write_text(result.stdout)
            if result.returncode:print(result.stdout)
            return result.returncode
    finally:
        if process and process.poll() is None:process.terminate();process.wait(timeout=3)

if __name__=='__main__':raise SystemExit(main())
