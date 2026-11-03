#!/usr/bin/env python3
"""One authorised <=60s disabled-zero ROS window; never run the operator UI.

Reuses the existing final wire allowlist and timing-analysis helpers. No select,
CONTROL_ENABLE, clear, ESTOP, retry or motion can pass the last socket wrapper.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import socket
import statistics
import sys
import tempfile
import time

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
READY=ROOT/'docs/wand_feedback_guard_2026-10-27/prepared'
sys.path.insert(0,str(ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'))
from launch_cli import check
for p in (ROOT/'src/mecanum_vicon_control',*(READY/'runtime/src'/n for n in
        ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'))):
    sys.path.insert(0,str(p))
# Import candidate modules BEFORE historical diagnostics adjust sys.path.
from mecanum_wand_4ws8_hardware.startup_clock import StartupProgressSocket
from mecanum_wand_4ws8_hardware.kernel_ingress import KernelIngressSocket
from mecanum_wand_4ws8_hardware.receive_event import ReceiveEventPump
from mecanum_wand_4ws8_hardware.stage0_session import Stage0Session
from mecanum_wand_4ws8_hardware.stage0_contract import decode_motion_status
sys.path.insert(0,str(ROOT/'tools'))
from verify_eventio_unarmed_20261026 import (
    ZeroOnlySocket,offline_check,clock_failure,ESP,INTERFACE,M,unpack_wheel_telemetry,decode_frame)
from trace_eventio_unarmed_20261026 import frame_fields,os_snapshot,summarise

MAX_WINDOW_S=60.


class PermanentZeroSocket(ZeroOnlySocket):
    def __init__(self,base):
        super().__init__(base);self.stopped_at=None
    def sendto(self,data,destination):
        if self.stopped_at is not None:raise OSError('diagnostic TX permanently stopped')
        return super().sendto(data,destination)
    def stop(self):
        if self.stopped_at is None:self.stopped_at=time.monotonic()


def live(folder):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    # Same ownership lock as every actual transport. No stealing/killing.
    lock=open('/tmp/wand_mc_link_46001.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    ctx=Context();rclpy.init(context=ctx,domain_id=169)
    node=Node('feedback_guard_zero_diagnostic',context=ctx,enable_rosout=False,start_parameter_services=False)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(node)
    raw=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    raw.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,INTERFACE.encode()+b'\0')
    raw.bind(('192.168.50.2',46001));raw.setblocking(False)
    audit=PermanentZeroSocket(raw);ingress=KernelIngressSocket(audit)
    session=Stage0Session(stage=4)
    wrapped=StartupProgressSocket(ingress,ESP[0],lambda:session.session_id,str(folder/'native_clock.jsonl'))
    state=dict(reason='',error='',ready_at=None,acks=0,wheels=0,status=0)
    samples=[];events=[];wheels=[];diagnostics=[];trip={};pump=None
    started=time.monotonic();initial=os_snapshot()
    def event(name,**fields):events.append(dict(event=name,at=time.monotonic(),**fields))
    def stop(reason,error=True):
        if audit.stopped_at is not None:return
        audit.stop()  # Before peeking, reporting, or any further callback.
        now=time.monotonic();state['reason']=reason
        if error:state['error']=reason
        trip.update(at=now,elapsed_s=now-started,reason=reason,
            clock=wrapped.clock.snapshot(now),
            source_ages_ms={M(k).name:(now-v)*1000 for k,v in wrapped.clock.accepted_at.items()},
            last_packets=samples[-4:],pump_error=pump.error,
            pump_alive=pump.thread.is_alive(),budget_exhaustions=ingress.budget_exhaustions)
        try:trip['queue_at_stop']=ingress.peek()
        except Exception as problem:trip['queue_at_stop']=dict(error=str(problem))
        event('TX_STOP',reason=reason)
        print('TX_STOP '+reason+' elapsed='+str(round(now-started,3)),flush=True)
    # Record ALL dequeued native datagrams, including ones rejected by the
    # sample clock, without renewing their source stamps.
    receive_one=ingress.recvfrom
    def recorded(size):
        data,peer=receive_one(size)
        row=dict(ingress.last_receive,after_tx_stop=audit.stopped_at is not None)
        try:row.update(frame_fields(data))
        except Exception as error:row['decode_error']=str(error)
        samples.append(row)
        return data,peer
    ingress.recvfrom=recorded
    def receive():
        event('ROS_DRAIN_ENTER');ingress.begin_drain()
        try:
            if pump.error:raise ValueError(pump.error)
            while True:
                try:data,peer=wrapped.recvfrom(512)
                except BlockingIOError:break
                if peer!=ESP:raise ValueError('unexpected telemetry peer')
                f=session.accept(data);now=time.monotonic()
                if f.message_type==M.HELLO_ACK:state['acks']+=1
                elif f.message_type==M.WHEEL_TELEMETRY:
                    t=unpack_wheel_telemetry(f.payload)
                    wheels.append(dict(at=now,fault=t.fault_bits,state=t.state_flags,rpm=list(t.wheel_rpm)))
                    if t.fault_bits or f.flags or any(t.wheel_rpm) or not t.state_flags&8 or t.state_flags&(2|256):
                        raise ValueError('native fault or nonzero/enabled wheel state '+hex(t.fault_bits))
                    state['wheels']+=1
                elif f.message_type==M.LINK_STATUS:
                    d=decode_motion_status(f.payload);diagnostics.append(dict(at=now,status=d))
                    if (f.flags or d['permitted'] or any(d['compares']+d['raw_duty']+d['target_cps']+
                            d['body_milli']+d['effective_cps']+d['granted_direction']) or d['fault_wheel']
                            or d['watchdog_reset'] or d['first_feedback_fault']['reason']):
                        raise ValueError('native diagnostic fault or nonzero/enabled output')
                    state['status']+=1
            if ingress.error:stop('KERNEL_METADATA_INVALID: '+ingress.error)
            if wrapped.clock.fault:stop(wrapped.clock.fault)
            if wrapped.clock.notice=='HELLO_RTT_EXCEEDED':stop('HELLO_RTT_EXCEEDED')
        except Exception as problem:stop(str(problem))
        finally:
            ingress.end_drain();pump.acknowledge();event('ROS_DRAIN_EXIT')
    guard=node.create_guard_condition(receive)
    class TracedGuard:
        def trigger(self):event('WORKER_GUARD_TRIGGER');guard.trigger()
    pump=ReceiveEventPump(wrapped,TracedGuard())
    def tick():
        if audit.stopped_at is not None:return
        receive()  # Candidate order: bounded dequeue before freshness decision.
        if audit.stopped_at is not None:return
        now=time.monotonic();event('SAFETY_TICK',source_ages_ms={
            M(k).name:1000*(now-v) for k,v in wrapped.clock.accepted_at.items()})
        try:
            if now-started>=MAX_WINDOW_S:stop('FINITE_WINDOW_COMPLETE',False);return
            reason=clock_failure(now,started,state['ready_at'],wrapped.clock.ready(now))
            if reason:stop(reason);return
            if wrapped.clock.ready(now) and state['ready_at'] is None:
                state['ready_at']=now;event('FIRST_READY')
                print('UNARMED_READY rtt_ms='+str(wrapped.clock.rtt_s*1000),flush=True)
            if session.connected and state['wheels'] and state['status']:
                wrapped.sendto(session.cmd_vel(0,0,0,600,False,int(now*1000)&0xffffffff),ESP)
        except Exception as problem:stop(str(problem))
    timer=node.create_timer(.020,tick);report={}
    try:
        event('ONE_HELLO_SEND')
        wrapped.sendto(session.new_hello(int(time.monotonic()*1000)&0xffffffff),ESP)
        while audit.stopped_at is None or time.monotonic()-audit.stopped_at<.5:
            executor.spin_once(timeout_sec=.02)
            if audit.stopped_at is None and time.monotonic()-started>MAX_WINDOW_S+.1:
                stop('DIAGNOSTIC_WALL_LIMIT')
        before=[r for r in samples if not r['after_tx_stop']]
        delays=[r['kernel_to_ros_ms'] for r in samples]
        commands=[r for r in audit.sent if r['kind']=='CMD_VEL']
        report=dict(completed=state['reason']=='FINITE_WINDOW_COMPLETE',error=state['error'],
            trip=trip,hello_attempts=wrapped.hello_attempts,hello_acks=state['acks'],
            elapsed_s=time.monotonic()-started,all_tx_disabled_zero_or_one_hello=True,
            command_count=len(commands),tx_after_stop=sum(r['issued_steady']>audit.stopped_at for r in audit.sent),
            command_max_gap_ms=max(((b['issued_steady']-a['issued_steady'])*1000 for a,b in zip(commands,commands[1:])),default=None),
            streams_before_stop=summarise(before),kernel_to_ros_median_ms=statistics.median(delays) if delays else None,
            kernel_to_ros_max_ms=max(delays,default=None),rx_overflow_max=max((r['rxq_overflow_total'] or 0 for r in samples),default=0),
            wheel_faults=sorted({r['fault'] for r in wheels}),feedback_protection_ms=200,
            motion_started=False,enabled_or_clear_or_select_sent=False,firmware_or_network_changed=False,
            initial_os=initial,final_os=os_snapshot(),
            scope='Candidate kernel ingress, startup clock, bounded receive and ROS timer; NOT full operator UI or motion qualification')
    finally:
        audit.stop();timer.cancel();pump.stop();wrapped.close()
        executor.remove_node(node);executor.shutdown();node.destroy_node();ctx.shutdown();lock.close()
        for name,value in (('RESULT.json',report),('kernel_receive.json',samples),('outgoing.json',audit.sent),
                ('events.json',events),('wheels.json',wheels),('diagnostics.json',diagnostics)):
            (folder/name).write_text(json.dumps(value,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('initial_os','final_os','trip')},indent=2),flush=True)
    return 0 if report.get('completed') else 1


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--offline-check',action='store_true')
    ap.add_argument('--confirm-unarmed-zero-only',action='store_true');a=ap.parse_args()
    if a.offline_check:offline_check();return 0
    if not a.confirm_unarmed_zero_only:ap.error('explicit zero-only confirmation required')
    m=check(READY/'build_manifest.json')
    if not m.get('feedback_guard_passed'):raise ValueError('offline verification required before I/O')
    # Verify imports resolve to this exact candidate, never the historical tool.
    import inspect
    if not Path(inspect.getfile(StartupProgressSocket)).is_relative_to(READY):raise ValueError('wrong transport import')
    os.umask(0o077);folder=Path(tempfile.mkdtemp(prefix='unarmed_zero_',dir=READY.parent))
    os.environ.update(ROS_AUTOMATIC_DISCOVERY_RANGE='OFF',ROS_LOG_DIR=str(folder/'ros_logs'))
    print('EVIDENCE='+str(folder),flush=True)
    return live(folder)


if __name__=='__main__':raise SystemExit(main())
