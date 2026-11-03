#!/usr/bin/env python3
"""Readonly transport recorder. Never imports ROS or an actuator package."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import select
import threading
import time
from probe import Probe

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=['readonly'],required=True)
    ap.add_argument('--seconds',type=float,default=60.);ap.add_argument('--session',type=Path,required=True)
    a=ap.parse_args()
    if not 1<=a.seconds<=120:raise ValueError('Duration1..120 only')
    probe=Probe();incoming=queue.Queue(64);outgoing=queue.Queue(128)
    closing=threading.Event();broken=threading.Event()
    def read():
        buf=b''
        try:
            while not closing.is_set():
                if not select.select([0],[],[],.05)[0]:continue
                chunk=os.read(0,16384)
                if not chunk:broken.set();break
                buf+=chunk
                while b'\n' in buf:
                    line,buf=buf.split(b'\n',1)
                    if len(line)>16384:raise ValueError('FRAME_LIMIT')
                    p=json.loads(line);p['_reader_received']=time.monotonic();incoming.put_nowait(p)
                if len(buf)>16384:raise ValueError('FRAME_LIMIT')
        except Exception:broken.set()
    def write():
        try:
            with (a.session/'events.jsonl').open('a') as f:
                while True:
                    v=outgoing.get();line=json.dumps(v,ensure_ascii=False,allow_nan=False)+'\n'
                    if v['kind'] in ('audit','sample','echo_sample','result'):f.write(line);f.flush()
                    if v['kind'] in ('ready','challenge','close','state','result'):
                        b=line.encode()
                        while b:b=b[os.write(1,b):]
                    outgoing.task_done()
        except Exception:broken.set()
    r=threading.Thread(target=read,daemon=True);r.start()
    threading.Thread(target=write,daemon=True).start()
    def emit(v):
        try:outgoing.put_nowait(v)
        except queue.Full:broken.set()
    started=time.monotonic();challenge_at=state_at=audit_at=-math.inf;complete=False;trace=[]
    emit(dict(kind='ready',mode='readonly',hardware_entry=False))
    try:
        while True:
            now=time.monotonic();probe.tick(now)
            if broken.is_set():probe.fail('TRANSPORT_CLOSED_OR_BACKPRESSURE',now)
            for _ in range(16):
                try:p=incoming.get_nowait()
                except queue.Empty:break
                cid=p.get('connection')
                if type(cid) is not str or not 1<=len(cid)<=80:probe.fail('CONNECTION_SCHEMA',now);break
                if p.get('kind')=='open':
                    if not probe.open(cid,now):emit(dict(kind='close',connection=cid,reason='OWNER_EXISTS'))
                elif p.get('kind')=='disconnect':probe.disconnect(cid,now)
                elif p.get('kind')=='wire':
                    before=probe.raw_count
                    echoes=len(probe.echo_samples)
                    probe.receive(cid,p.get('data'),time.monotonic(),node_received=p.get('node_received_steady'),
                        reader_received=p.get('_reader_received'),node_gap=p.get('node_event_gap_max_s'))
                    if probe.raw_count!=before:emit(dict(kind='sample',**probe.samples[-1]))
                    if len(probe.echo_samples)!=echoes:emit(dict(kind='echo_sample',**probe.echo_samples[-1]))
                else:probe.fail('TRANSPORT_SCHEMA',now)
            now=time.monotonic();t=0. if probe.first is None else now-probe.first
            if now-challenge_at>=.018:
                challenge=probe.challenge(now)
                if challenge:emit(challenge)
                challenge_at=now
            if now-state_at>=.045 or probe.closed:
                car=dict(x=.8*math.cos(t*1.6),y=.6*math.sin(t*1.6),yaw=t*.3,z=.1)
                goal=dict(x=.8*math.cos(t*1.6+.4),y=.6*math.sin(t*1.6+.4),yaw=t*.3+.2,z=.1)
                trace.append([car['x'],car['y']]);trace=trace[-300:]
                summary=probe.summary(now)
                emit(dict(kind='state',mode='readonly',software_only=True,physical_output=False,
                    state='STRICT_FAILED_OBSERVING' if probe.strict.fault else 'READONLY_PROBE',
                    car=car,goal=goal,reference=None,trace=trace[::3],reason=probe.strict.fault,
                    channel=dict(fault=''),diagnostics=summary,remaining_s=max(0.,a.seconds-t),
                    instruction=('严格校验已失败：'+probe.strict.fault+'。仅继续只读动画/记录；绝无运动权限'
                        if probe.strict.fault else '只读链路诊断：动画仅为软件样例；100ms严格校验仍生效'),
                    session_requested=False,trial_ended=False))
                state_at=now
            if now-audit_at>=.2:emit(dict(kind='audit',**probe.summary(now)));audit_at=now
            if probe.first is not None and t>=a.seconds:complete=True;break
            if probe.closed:break
            if now-started>180:probe.fail('WAITING_BROWSER_TIMEOUT',now);break
            time.sleep(.005)
    finally:
        now=time.monotonic();result=dict(probe.summary(now),passed=complete and not probe.strict.fault,
            completed=complete,reason=probe.end_reason or probe.strict.fault or 'TIMER_COMPLETE',
            diagnostic_completed=complete,qualification='READONLY OBSERVER; NO ROS/Vicon/car')
        (a.session/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        emit(dict(kind='result',result=result))
        until=time.monotonic()+.5
        while outgoing.unfinished_tasks and time.monotonic()<until:time.sleep(.005)
        closing.set();r.join(.1)

if __name__=='__main__':main()
