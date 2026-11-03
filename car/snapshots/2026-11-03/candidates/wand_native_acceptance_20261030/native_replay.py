"""Native GNOME window/maximize/fullscreen, recorded display ONLY, no ROS.

Never initialize a ROS context, construct a device transport, send a key or
alter a global display/driver setting. Esc cancels this read-only replay.
"""
import argparse
from collections import deque
from datetime import datetime
import faulthandler
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace as NS
from common import ROOT,HOST,HERE,OUT,config,save,diagnostic as d,protected

MODES=('window','maximized','fullscreen')
SOURCE=HOST/'ui_600s_actual_pixels/normal/observer/production_observer.jsonl'


def extract(out,duration=600.):
    start=None;count=0;last=0.;initial={}
    with SOURCE.open() as src,(out/'display_events.jsonl').open('x') as dst:
        for line in src:
            row=json.loads(line);topic=row['topic']
            if topic not in d.TOPICS:continue
            value=row['data'];value=json.loads(value) if isinstance(value,str) else value
            at=row['received_steady']
            selected=d.selected_data(topic,value)
            if start is None:
                if topic!='supervisor_status' or value.get('state')!='ACTIVE':
                    initial[topic]=selected;continue
                start=at
                for k,v in initial.items():
                    dst.write(json.dumps([0.,k,v],ensure_ascii=False)+'\n');count+=1
            elapsed=at-start
            if elapsed>duration:break
            dst.write(json.dumps([elapsed,topic,selected],ensure_ascii=False)+'\n')
            count+=1;last=elapsed
    if start is None or last<duration-.1:raise ValueError('recorded ACTIVE display sequence incomplete')
    value=dict(source=str(SOURCE),source_sha256=d.sha(SOURCE),events=count,duration_s=duration,
        last_event_s=last,fields='display only; authority/lease/commands/addresses excluded',
        renderer='sealed prepared_v9 Console; no ROS or hardware construction')
    save(out/'DATASET.json',value);return value


def run_gui(a):
    from mecanum_wand_4wsb_control import operator_console as module
    from mecanum_wand_4wsb_control.core import Pose
    attempts=d.deny_live_io()
    import tkinter as tk
    out=a.output;meta=json.loads((out/'DATASET.json').read_text())
    events=(out/'display_events.jsonl').open()
    log=(out/(a.mode+'_ui.jsonl')).open('x',buffering=1)
    stack=(out/(a.mode+'_stacks.log')).open('x')
    faulthandler.register(signal.SIGUSR1,file=stack,all_threads=True)
    model=module.OperatorModel()
    hint='只读历史回放 · 无ROS/无硬件输出 · 空格无效 · Esc结束回放'
    model.can_start=model.can_recover=model.start_ready=lambda *_:False
    model.instruction=lambda *_:hint
    model.notice=hint
    class RecordedNode:
        input_kind='hardware'
        runtime=NS(config=config(7),wand_reference_margin_m=0.,software_only=True)
        car=raw_wand=None
        trace=deque(maxlen=500)
        consumed=0;last=None;next_event=None;started=None
        def __init__(self):self.advance()
        def advance(self):
            line=events.readline();self.next_event=json.loads(line) if line else None
        def pump(self):
            now=time.monotonic();elapsed=now-self.started
            while self.next_event is not None and self.next_event[0]<=elapsed:
                _,topic,value=self.next_event;self.consumed+=1
                if topic.endswith('_pose'):
                    x,y,z,w=(value[k] for k in ('qx','qy','qz','qw'))
                    p=Pose(value['x'],value['y'],value['z'],math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
                    if topic=='car_pose':self.car=p;self.trace.append((p.x,p.y))
                    else:self.raw_wand=p
                elif topic=='supervisor_status':model.status,model.status_at=value,now
                else:model.gate,model.gate_at=value,now
                self.advance()
            return elapsed
        def input_hint(self):return hint
        def request(self,*_):raise RuntimeError('READONLY: control request blocked')
        def heartbeat(self):raise RuntimeError('READONLY: no operator lease')
    node=RecordedNode()
    class ReadonlyConsole(module.Console):
        def press(self,*_):return 'break'
        def release(self,*_):return 'break'
        def finish_release(self,*_):return 'break'
        def schedule_focus(self):pass
        def check_focus(self):pass
        def escape(self,*_):self.close();return 'break'
        def draw(self,now):
            super().draw(now);self.step_text.set(hint)
            for b in self.buttons.values():b.configure(state='disabled')
        def loop(self):
            self.loop_job=None
            if self.closing:return
            entered=time.monotonic();key_at=entered
            self.key_state.down()  # identical read-only X11 query, no synthesis
            key_end=time.monotonic();elapsed=node.pump();pump_end=time.monotonic()
            draw_ms=None
            if pump_end-self.last_draw>=.033:
                self.draw(pump_end);draw_ms=(time.monotonic()-pump_end)*1000;self.last_draw=pump_end
            row=dict(steady=time.monotonic(),elapsed_s=elapsed,
                gap_ms=None if self.previous is None else (entered-self.previous)*1000,
                key_ms=(key_end-key_at)*1000,pump_ms=(pump_end-key_end)*1000,draw_ms=draw_ms,
                items=len(self.map.find_all()),trace_points=len(node.trace),events_consumed=node.consumed,
                root_size=[root.winfo_width(),root.winfo_height()],
                canvas_size=[self.map.winfo_width(),self.map.winfo_height()],
                wm_fullscreen=bool(root.attributes('-fullscreen')),wm_zoomed=bool(root.attributes('-zoomed')),
                callbacks=len(root.tk.call('after','info')),control_io_attempts=len(attempts))
            log.write(json.dumps(row)+'\n');self.previous=entered
            if elapsed>=meta['duration_s']:self.close(completed=True)
            else:self.loop_job=root.after(20,self.loop)
        def close(self,completed=False):
            if self.closing:return
            self.closing=True
            if self.loop_job:root.after_cancel(self.loop_job)
            save(out/(a.mode+'_completion.json'),dict(completed=completed,events_consumed=node.consumed,
                expected_events=meta['events'],control_io_attempts=len(attempts),ros_initialized=False,
                hardware_opened=False,motion_started=False,authority_ever_acquired=False))
            root.destroy();self.key_state.close()
    root=tk.Tk();app=ReadonlyConsole(root,node,model);app.previous=None
    root.title('4WSB只读验收 · '+a.mode+' · 无ROS/无实车输出 · Esc退出')
    top=root.winfo_children()[0].winfo_children()
    top[0].configure(text='4WSB 原生桌面只读对照 · '+a.mode)
    top[1].configure(text=hint)
    for b in app.buttons.values():b.configure(state='disabled',command=lambda:None)
    # GNOME may discard a pre-map -zoomed request. Request the OWN window
    # state after it is mapped, then verify EWMH and real size independently.
    if a.mode=='maximized':root.after(300,lambda:root.attributes('-zoomed',True))
    elif a.mode=='fullscreen':root.after(300,lambda:root.attributes('-fullscreen',True))
    node.started=time.monotonic()+1.5
    def window_info():
        save(out/(a.mode+'_window.json'),dict(client=root.winfo_id(),frame=root.frame(),
            width=root.winfo_width(),height=root.winfo_height(),screen=[root.winfo_screenwidth(),root.winfo_screenheight()],
            fullscreen=bool(root.attributes('-fullscreen')),zoomed=bool(root.attributes('-zoomed'))))
    root.after(1200,window_info)
    signal.signal(signal.SIGTERM,lambda *_:app.close())
    try:root.mainloop()
    finally:events.close();log.close();faulthandler.unregister(signal.SIGUSR1);stack.close()


def probe(a):
    from mecanum_wand_4wsb_control.x11_key_state import X11KeyState
    key=X11KeyState(os.environ['DISPLAY'])
    with (a.output/(a.mode+'_x11.jsonl')).open('x',buffering=1) as f:
        while True:
            start=time.monotonic();key.down();end=time.monotonic()
            f.write(json.dumps(dict(steady=end,rtt_ms=(end-start)*1000))+'\n');time.sleep(.1)


def summarize(out,mode,returncode,watchdog):
    rows=[json.loads(s) for s in (out/(mode+'_ui.jsonl')).read_text().splitlines()]
    x11=[json.loads(s)['rtt_ms'] for s in (out/(mode+'_x11.jsonl')).read_text().splitlines()]
    stable=[r for r in rows if r['elapsed_s']>=2.]
    p=out/(mode+'_completion.json');c=json.loads(p.read_text()) if p.exists() else {}
    sizes=sorted(set(tuple(r['root_size']) for r in stable))
    wm=all(r['wm_fullscreen'] for r in stable) if mode=='fullscreen' else all(r['wm_zoomed'] for r in stable) if mode=='maximized' else True
    ewmh=json.loads((out/(mode+'_ewmh.json')).read_text()).get('stdout','')
    if mode=='fullscreen':wm=wm and '_NET_WM_STATE_FULLSCREEN' in ewmh
    elif mode=='maximized':wm=wm and all(x in ewmh for x in ('_NET_WM_STATE_MAXIMIZED_HORZ','_NET_WM_STATE_MAXIMIZED_VERT'))
    gaps=[r['gap_ms'] for r in stable if r['gap_ms'] is not None]
    return dict(mode=mode,passed=bool(returncode==0 and not watchdog and c.get('completed')
        and c.get('events_consumed')==c.get('expected_events') and c.get('control_io_attempts')==0
        and wm and gaps and max(gaps)<=100. and max(r['items'] for r in stable)==7),
        returncode=returncode,watchdog=watchdog,completion=c,sizes=sizes,wm_state_observed=wm,
        gap_ms=d.quantiles(gaps),key_ms=d.quantiles([r['key_ms'] for r in stable]),
        draw_ms=d.quantiles([r['draw_ms'] for r in stable]),x11_ms=d.quantiles(x11),
        max_items=max((r['items'] for r in rows),default=0),
        max_callbacks=max((r['callbacks'] for r in rows),default=0),
        ros_initialized=False,hardware_output=False)


def compare(a):
    if os.environ.get('DISPLAY')!=':1' or os.environ.get('XDG_SESSION_TYPE')!='x11':
        raise ValueError('confirmed local native X11 desktop :1 required; not SSH/nested display')
    out=a.output;out.mkdir(parents=True,exist_ok=False)
    before=protected();save(out/'PROTECTED_BEFORE.json',before)
    dataset=extract(out,a.duration)
    start_wall=datetime.now().astimezone().isoformat()
    save(out/'desktop_before.json',d.capture(['xrandr','--current']))
    save(out/'gpu_before.json',d.capture(['nvidia-smi','--query-gpu=name,driver_version,temperature.gpu,utilization.gpu,memory.used','--format=csv']))
    xlog=Path('/home/PRIMARY_USER/.local/share/xorg/Xorg.1.log');offset=xlog.stat().st_size if xlog.exists() else 0
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1');env.pop('WAND_AUTOPREP_FLOW',None)
    summary=dict(passed=False,scope='native recorded renderer/X11 only; no live ROS/hardware',
        dataset=dataset,phases=[],source_sha256=d.sha(Path(__file__)),driver_changed=False,kernel_changed=False)
    for mode in a.modes:
        save(out/'PROGRESS.json',dict(mode=mode,duration_s=a.duration,hardware_output=False))
        child=observer=None;watchdog=False;stack_sent=False;state_captured=False
        with (out/(mode+'_process.log')).open('x') as log,(out/(mode+'_system.jsonl')).open('x',buffering=1) as system:
            try:
                child=subprocess.Popen([sys.executable,str(Path(__file__)),'gui','--output',str(out),'--mode',mode],env=env,stdout=log,stderr=log)
                observer=subprocess.Popen([sys.executable,str(Path(__file__)),'probe','--output',str(out),'--mode',mode],env=env,stdout=log,stderr=log)
                began=last_sample=time.monotonic();pids=d.named_pids()
                while child.poll() is None:
                    now=time.monotonic();path=out/(mode+'_ui.jsonl')
                    if now-last_sample>=.5:
                        system.write(json.dumps(d.system_sample(dict(pids,ui=child.pid,probe=observer.pid)))+'\n');last_sample=now
                    info=out/(mode+'_window.json')
                    if info.exists() and not state_captured:
                        value=json.loads(info.read_text());save(out/(mode+'_ewmh.json'),d.capture(['xprop','-id',value['frame'],'_NET_WM_STATE']))
                        state_captured=True
                    age=time.time()-path.stat().st_mtime if path.exists() else 0.
                    if age>.5 and not stack_sent:child.send_signal(signal.SIGUSR1);stack_sent=True
                    if age>2. or now-began>a.duration+18.:watchdog=True;break
                    time.sleep(.1)
            finally:
                if child is not None:d.terminate_owned(child)
                if observer is not None:d.terminate_owned(observer)
        r=summarize(out,mode,child.returncode,watchdog)
        summary['phases'].append(r);save(out/'RESULT.json',summary)
        print(json.dumps(r),flush=True)
        if not r['passed']:break
    after=protected();save(out/'PROTECTED_AFTER.json',after)
    summary['protected_unchanged']=before==after
    summary['passed']=len(summary['phases'])==len(a.modes) and all(r['passed'] for r in summary['phases']) and before==after
    if xlog.exists():
        with xlog.open('rb') as f:f.seek(offset);(out/'xorg_increment.log').write_bytes(f.read(100000))
    for name,selector in (('kernel',['-k']),('gnome',['_COMM=gnome-shell'])):
        save(out/(name+'_journal.json'),d.capture(['journalctl','-b','--since',start_wall,'--no-pager','-o','short-iso',*selector]))
    save(out/'gpu_after.json',d.capture(['nvidia-smi','--query-gpu=name,driver_version,temperature.gpu,utilization.gpu,memory.used','--format=csv']))
    save(out/'RESULT.json',summary);return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('compare','gui','probe'))
    p.add_argument('--output',type=Path,required=True);p.add_argument('--mode',choices=MODES)
    p.add_argument('--modes',choices=MODES,nargs='+',default=list(MODES));p.add_argument('--duration',type=float,default=600.)
    a=p.parse_args()
    if a.action=='compare':raise SystemExit(0 if compare(a)['passed'] else 1)
    elif a.action=='gui':run_gui(a)
    else:probe(a)
