#!/usr/bin/env python3
"""Bounded browser evidence loop. Readonly never imports/initializes ROS."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import queue
import select
import sys
import threading
import time
from channel import Channel


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=('readonly','software'),required=True)
    ap.add_argument('--seconds',type=float,default=60.);ap.add_argument('--session',type=Path,required=True)
    args=ap.parse_args()
    if not 1<=args.seconds<=120:raise ValueError('duration1..120 required')
    ready=Path(__file__).resolve().parent
    channel=Channel(readonly=args.mode=='readonly');incoming=queue.Queue(maxsize=64);outgoing=queue.Queue(maxsize=128)
    io_failed=threading.Event();eof=threading.Event();closing=threading.Event()
    def reader():
        try:
            buffered=b''
            while not closing.is_set():
                if not select.select([0],[],[],.05)[0]:continue
                chunk=os.read(0,16384)
                if not chunk:break
                buffered+=chunk
                while b'\n' in buffered:
                    line,buffered=buffered.split(b'\n',1)
                    if len(line)>16384:raise ValueError('INPUT_FRAME_LIMIT')
                    incoming.put_nowait(json.loads(line))
                if len(buffered)>16384:raise ValueError('INPUT_FRAME_LIMIT')
        except Exception:io_failed.set()
        finally:eof.set()
    def writer():
        try:
            with (args.session/'events.jsonl').open('a',encoding='utf-8') as audit:
                while True:
                    value=outgoing.get()
                    line=json.dumps(value,allow_nan=False,ensure_ascii=False,separators=(',',':'))
                    if value['kind'] in ('audit','result'):
                        audit.write(line+'\n');audit.flush()
                    if value['kind']!='audit':
                        remaining=(line+'\n').encode()
                        while remaining:remaining=remaining[os.write(1,remaining):]
                    outgoing.task_done()
        except Exception:io_failed.set()
    reader_thread=threading.Thread(target=reader,daemon=True,name='browser-input-transport-only')
    reader_thread.start()
    threading.Thread(target=writer,daemon=True,name='browser-status-transport-only').start()
    def emit(value):
        try:outgoing.put_nowait(value)
        except queue.Full:io_failed.set()
    rig=None;trace=deque(maxlen=500);challenge_at=state_at=audit_at=-math.inf
    complete=False;report=None;start=time.monotonic();last_state=None
    try:
        if args.mode=='software':
            for pkg in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
                sys.path.insert(0,str(ready/'runtime/src'/pkg))
            from software_runtime import SoftwareRig
            rig=SoftwareRig(ready,channel)
        emit(dict(kind='ready',mode=args.mode,hardware_entry=False))
        while True:
            tick_start=time.monotonic();channel.tick(tick_start)
            if eof.is_set() or io_failed.is_set():channel.stop('TRANSPORT_EOF_OR_BACKPRESSURE')
            for _ in range(16):
                try:packet=incoming.get_nowait()
                except queue.Empty:break
                kind=packet.get('kind');connection=packet.get('connection')
                if type(connection) is not str or not 1<=len(connection)<=80:channel.stop('CONNECTION_SCHEMA');break
                if kind=='open':
                    if not channel.open(connection,tick_start):emit(dict(kind='close',connection=connection,reason='OTHER_WINDOW_OR_ENDED_SESSION'))
                elif kind=='disconnect':channel.disconnect(connection)
                elif kind=='wire':channel.receive(connection,packet.get('data'),tick_start)
                else:channel.stop('TRANSPORT_SCHEMA')
            if rig:rig.step(time.monotonic())
            now=time.monotonic()
            if now-challenge_at>=.018:
                challenge=channel.challenge(now)
                if challenge:emit(challenge)
                challenge_at=now
            if args.mode=='readonly':
                t=0. if channel.started is None else now-channel.started
                car=dict(x=.4*math.cos(t),y=.4*math.sin(t),yaw=.2*math.sin(t),z=.1)
                goal=dict(x=.4*math.cos(t+.2),y=.4*math.sin(t+.2),yaw=.2*math.sin(t+.2),z=.1)
                trace.append((car['x'],car['y']))
                state=dict(state='READONLY',car=car,goal=goal,reference=None,trace=list(trace)[::5],
                    instruction='只读软件动画：空格仅记录，不准备、不使能、不连接小车',
                    reason=channel.fault,session_requested=False,trial_ended=channel.stopped,
                    remaining_s=max(0.,args.seconds-t))
            else:state=rig.snapshot(now)
            if now-state_at>=.045 or channel.stopped:
                last_state=dict(kind='state',mode=args.mode,software_only=True,physical_output=False,
                    channel=channel.summary(now),**state)
                emit(last_state);state_at=now
            if now-audit_at>=.2:
                emit(dict(kind='audit',at=now,mode=args.mode,channel=channel.summary(now),state=state['state']))
                audit_at=now
            if channel.started is not None and now-channel.started>=args.seconds:
                complete=True;break
            if channel.stopped:break
            if now-start>180.:
                channel.stop('WAITING_FOR_BROWSER_TIMEOUT');break
            time.sleep(max(0.,.010-(time.monotonic()-tick_start)))
    except Exception as error:
        channel.stop(type(error).__name__+': '+str(error)[:160]);raise
    finally:
        stop_evidence=rig.close() if rig else None
        now=time.monotonic()
        report=dict(passed=complete and not channel.fault,completed=complete,
            mode=args.mode,reason=channel.fault or 'TIMER_COMPLETE',**channel.summary(now),
            ros_initialized=rig is not None,ros_domain=187 if rig else None,
            hardware_bridge_constructed=False,hardware_socket_opened=False,physical_motion_started=False,
            authority_ever_acquired=False if not rig else rig.operator.model.activation_used,
            software_stop_evidence=stop_evidence,
            qualification='SOFTWARE ONLY; not physical Sign/car acceptance',last_state=last_state)
        (args.session/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        emit(dict(kind='result',result=report))
        deadline=time.monotonic()+.5
        while outgoing.unfinished_tasks and time.monotonic()<deadline:time.sleep(.005)
        closing.set();reader_thread.join(timeout=.10)


if __name__=='__main__':main()
