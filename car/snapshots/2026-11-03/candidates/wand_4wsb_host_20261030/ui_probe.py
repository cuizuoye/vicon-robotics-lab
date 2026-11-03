"""Exact private 4WSB Console+ROS operator in a nested test display only.

The scripted click/Esc is ONLY enabled with private domain191 and the software
namespace. No hardware Runtime/bridge is constructed. This is not a launcher.
"""
import argparse
import faulthandler
import json
import math
import os
from pathlib import Path
import signal
import time
from types import SimpleNamespace as NS
from bootstrap import config
from prepare import save


def run(a):
    if (os.environ.get('ROS_DOMAIN_ID')!='191' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST'
            or not a.namespace.startswith('/host4wsb_') or os.environ.get('WAND_PRIVATE_TEST_DISPLAY')!=os.environ.get('DISPLAY')):
        raise ValueError('scripted UI restricted to nested software test display/domain')
    import tkinter as tk
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from mecanum_wand_4wsb_control.operator_console import OperatorNode,Console
    from mecanum_wand_4wsb_control.toggle_ui import ToggleOperatorModel
    a.output.mkdir(parents=True,exist_ok=False)
    save(a.output/'PID.json',dict(pid=os.getpid(),software_only=True))
    stack=(a.output/'python_stacks.log').open('w')
    faulthandler.register(signal.SIGUSR1,file=stack,all_threads=True)
    rclpy.init(args=[],domain_id=191,signal_handler_options=SignalHandlerOptions.NO)
    def guard(node):
        if node.context.get_domain_id()!=191 or node.get_namespace()!=a.namespace:raise ValueError('UI test escaped')
    runtime=NS(namespace=a.namespace,world='map',config=config(a.stage),mode='hardware',
               software_only=True,guard=guard,wand_reference_margin_m=0.)
    model=ToggleOperatorModel();node=OperatorNode(model,runtime);root=tk.Tk()
    if a.mode=='full_geometry':
        root.overrideredirect(True)
        root.geometry('%dx%d+0+0'%(root.winfo_screenwidth(),root.winfo_screenheight()))
    probes=[];started=time.monotonic();active=None;stop_at=None;clicked=False
    previous=started;peak_items=peak_callbacks=0;gaps=[];draw_cost=[]
    class ProbeConsole(Console):
        def loop(self):
            nonlocal previous,peak_items,peak_callbacks
            begin=time.monotonic();gaps.append(begin-previous);previous=begin
            super().loop()
            if self.closing:return
            items=len(self.map.find_all());peak_items=max(peak_items,items)
            callbacks=len(root.tk.call('after','info'));peak_callbacks=max(peak_callbacks,callbacks)
            if not probes or begin-probes[-1]['t']>=.2:
                rss=int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
                row=dict(t=begin,wall_s=begin-started,rss_bytes=rss,items=items,callbacks=callbacks,
                    width=root.winfo_width(),height=root.winfo_height(),state=model.status.get('state'),
                    last_gap_ms=gaps[-1]*1000)
                probes.append(row)
                save(a.output/'heartbeat.json',row)
        def draw(self,now):
            t=time.monotonic();super().draw(now);draw_cost.append(time.monotonic()-t)
    app=ProbeConsole(root,node,model)
    root.after(100,root.focus_force)
    if a.mode=='full_geometry':
        root.geometry('%dx%d+0+0'%(root.winfo_screenwidth(),root.winfo_screenheight()))
    def operate():
        nonlocal active,stop_at,clicked
        now=time.monotonic();state=model.status.get('state')
        if not clicked and model.start_ready(now):
            # Same event-adapter methods as a real click. No auto-enable code
            # is installed in the production Console or launcher.
            app.press();app.finish_release();clicked=True
        if state=='ACTIVE' and active is None:active=now
        if active is not None and stop_at is None and now-active>=a.duration:
            app.escape();stop_at=now
        if active is not None and state not in ('ACTIVE','ARMED') and stop_at is None:
            stop_at=now
        if (stop_at is not None and now-stop_at>2.) or now-started>a.duration+12.:
            app.close();return
        if not app.closing:root.after(20,operate)
    root.after(100,operate)
    try:root.mainloop()
    finally:
        node.close_node()
        if rclpy.ok():rclpy.shutdown()
        faulthandler.unregister(signal.SIGUSR1);stack.close()
    stable=[v['rss_bytes'] for v in probes if v['wall_s']>=10.]
    ended=stop_at-active if stop_at and active else 0.
    passed=bool(clicked and active and ended>=a.duration-.15 and peak_items<=7
        and (max(gaps[10:]) if len(gaps)>10 else 1.)<=.10
        and (not stable or max(stable)-min(stable)<32*1024**2))
    def stats(v):
        s=sorted(v)
        return dict(samples=len(s),p95_ms=s[int((len(s)-1)*.95)]*1000,max_ms=max(s)*1000) if s else None
    result=dict(passed=passed,mode=a.mode,active_s=ended,requested_s=a.duration,
        actual_console_class=True,actual_ros_operator=True,software_actuator_only=True,
        nested_display=os.environ['DISPLAY'],host_desktop_fullscreen_tested=False,
        gap=stats(gaps[10:]),draw=stats(draw_cost),peak_items=peak_items,
        peak_pending_callbacks=peak_callbacks,rss_peak_bytes=max((v['rss_bytes'] for v in probes),default=0),
        rss_range_after10s_bytes=max(stable)-min(stable) if stable else None,
        final_state=model.status.get('state'),hardware_opened=False,physical_qualified=False)
    save(a.output/'PROBES.json',probes);save(a.output/'RESULT.json',result)
    print(json.dumps(result),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--namespace',required=True);p.add_argument('--stage',type=int,required=True)
    p.add_argument('--duration',type=float,required=True)
    p.add_argument('--mode',choices=('normal','full_geometry'),required=True)
    p.add_argument('--output',type=Path,required=True)
    raise SystemExit(0 if run(p.parse_args())['passed'] else 1)
