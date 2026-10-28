#!/usr/bin/env python3
"""ONE finite disabled-zero window with kernel/ROS/source timing, never motion.

Same deployed clock/event pump, unchanged 40ms HELLO and 200ms freshness.
Stop TX on first failure; inspect queue without consumption; 0.5s receive-only
tail. No retry, clear, mode select, enable, serial/flash, or network changes.
"""
import argparse
import array
import fcntl
import json
import os
from pathlib import Path
import select
import socket
import statistics
import struct
import tempfile
import time

from verify_eventio_unarmed_20261026 import (
    ROOT,READY,ESP,INTERFACE,ZeroOnlySocket,clock_failure,offline_check,
    M,decode_frame,unpack_wheel_telemetry,decode_motion_status,
    Stage0Session,StartupProgressSocket,ReceiveEventPump)

SO_TIMESTAMPNS=35
SO_RXQ_OVFL=40
FIONREAD=0x541B
MAX_WINDOW_S=60.
RX_TAIL_S=.5

def decode_ancillary(ancillary):
    stamps=[];drops=[]
    for level,kind,value in ancillary:
        if level==socket.SOL_SOCKET and kind==SO_TIMESTAMPNS and len(value)>=16:
            sec,nsec=struct.unpack('=qq',value[:16]);stamps.append(sec*10**9+nsec)
        elif level==socket.SOL_SOCKET and kind==SO_RXQ_OVFL and len(value)>=4:
            drops.append(struct.unpack('=I',value[:4])[0])
    if len(stamps)!=1:raise ValueError('exactly one kernel timestamp required')
    return stamps[0],drops[-1] if drops else None

def frame_fields(data):
    f=decode_frame(data)
    return dict(kind=M(f.message_type).name,source_ms=f.sender_time_ms,
        sequence=f.sequence,session=f.session_id,flags=f.flags,payload_hex=f.payload.hex())

class TimestampedZeroSocket(ZeroOnlySocket):
    def __init__(self,base):
        super().__init__(base);self.received=[];self.tx_stopped=False;self.stop_mono_ns=None
    def sendto(self,data,destination):
        if self.tx_stopped:raise OSError('diagnostic TX permanently stopped; no recovery')
        return super().sendto(data,destination)
    def recvfrom(self,size):
        data,ancillary,msg_flags,peer=self.base.recvmsg(size,256)
        mono=time.monotonic_ns();real=time.time_ns()
        kernel,drops=decode_ancillary(ancillary)
        row=dict(received_mono_ns=mono,received_real_ns=real,kernel_real_ns=kernel,
            kernel_to_ros_ms=(real-kernel)/1e6,rxq_overflow_total=drops,peer=peer,
            msg_flags=msg_flags,after_tx_stop=self.tx_stopped)
        try:row.update(frame_fields(data))
        except Exception as error:row['decode_error']=str(error)
        self.received.append(row)
        if msg_flags&(socket.MSG_TRUNC|socket.MSG_CTRUNC):raise ValueError('data or ancillary truncated')
        return data,peer
    def stop_tx(self):
        self.tx_stopped=True
        if self.stop_mono_ns is None:self.stop_mono_ns=time.monotonic_ns()

def peek_queue(raw):
    result=dict(before_mono_ns=time.monotonic_ns(),before_real_ns=time.time_ns())
    result['select_readable']=bool(select.select([raw],[],[],0)[0])
    size=array.array('i',[0]);fcntl.ioctl(raw.fileno(),FIONREAD,size,True)
    result['first_pending_datagram_bytes']=size[0] # Not total queue bytes.
    try:
        data,ancillary,flags,peer=raw.recvmsg(1024,256,socket.MSG_PEEK|socket.MSG_DONTWAIT)
        kernel,drops=decode_ancillary(ancillary)
        result.update(datagram_available=True,kernel_real_ns=kernel,
            rxq_overflow_total=drops,peer=peer,msg_flags=flags,frame=frame_fields(data))
    except BlockingIOError:result['datagram_available']=False
    result.update(after_mono_ns=time.monotonic_ns(),after_real_ns=time.time_ns())
    return result

def os_snapshot():
    def read(path):
        try:return Path(path).read_text().strip()
        except OSError:return None
    return dict(wall_ns=time.time_ns(),realtime_monotonic_offset_ns=time.time_ns()-time.monotonic_ns(),
        interface={k:read('/sys/class/net/'+INTERFACE+'/statistics/'+k)
                   for k in ('rx_packets','rx_errors','rx_dropped','tx_packets','tx_errors','tx_dropped')},
        udp_global_counters=read('/proc/net/snmp'))

def summarise(rows):
    out={}
    for name in ('WHEEL_TELEMETRY','LINK_STATUS'):
        stream=[r for r in rows if r.get('kind')==name]
        out[name]=dict(frames=len(stream),
            max_kernel_gap_ms=max(((b['kernel_real_ns']-a['kernel_real_ns'])/1e6 for a,b in zip(stream,stream[1:])),default=None),
            max_ros_gap_ms=max(((b['received_mono_ns']-a['received_mono_ns'])/1e6 for a,b in zip(stream,stream[1:])),default=None),
            max_source_step_ms=max((((b['source_ms']-a['source_ms'])&0xffffffff) for a,b in zip(stream,stream[1:])),default=None))
    return out

def live(folder):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    ctx=Context();rclpy.init(context=ctx,domain_id=169)
    node=Node('eventio_unarmed_kernel_trace',context=ctx,enable_rosout=False,start_parameter_services=False)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(node)
    raw=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    raw.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,INTERFACE.encode()+b'\0')
    raw.setsockopt(socket.SOL_SOCKET,SO_TIMESTAMPNS,1);raw.setsockopt(socket.SOL_SOCKET,SO_RXQ_OVFL,1)
    raw.bind(('192.168.50.2',46001));raw.setblocking(False)
    audit=TimestampedZeroSocket(raw);session=Stage0Session(stage=4)
    wrapped=StartupProgressSocket(audit,ESP[0],lambda:session.session_id,str(folder/'native_clock.jsonl'))
    state=dict(error='',termination='',acks=0,ready_at=None,next_send=0.,wheel_count=0,status_count=0)
    events=[];wheels=[];diagnostics=[];trip={};pump=None
    started=time.monotonic();initial_os=os_snapshot()
    def checkpoint(event,**fields):
        events.append(dict(event=event,mono_ns=time.monotonic_ns(),real_ns=time.time_ns(),**fields))
    def stop(reason,error=True):
        if audit.tx_stopped:return
        audit.stop_tx() # Must precede any peek, further callback, or report work.
        state['termination']=reason
        if error:state['error']=reason
        now=time.monotonic()
        trip.update(reason=reason,mono_ns=time.monotonic_ns(),real_ns=time.time_ns(),
            elapsed_s=now-started,clock=wrapped.clock.snapshot(now),
            accepted_source_times={M(k).name:v for k,v in wrapped.clock.accepted_at.items()},
            source_ages_ms={M(k).name:1000*(now-v) for k,v in wrapped.clock.accepted_at.items()},
            last_dequeued_packets=audit.received[-4:],
            pump_alive=pump.thread.is_alive(),pump_acknowledged=pump.dispatched.is_set(),
            pump_error=pump.error)
        try:trip['queue_at_stop']=peek_queue(raw)
        except Exception as problem:trip['queue_snapshot_error']=str(problem)
        checkpoint('TX_STOP',reason=reason)
        print('TX_STOP '+reason+' after_s='+str(round(now-started,3)),flush=True)
    def receive():
        checkpoint('ROS_GUARD_ENTER')
        try:
            if pump.error:raise ValueError('receive pump '+pump.error)
            while True:
                try:data,peer=wrapped.recvfrom(1024)
                except BlockingIOError:break
                if peer!=ESP:raise ValueError('unexpected telemetry peer')
                f=session.accept(data);now=time.monotonic()
                if f.message_type==M.HELLO_ACK:state['acks']+=1
                elif f.message_type==M.WHEEL_TELEMETRY:
                    t=unpack_wheel_telemetry(f.payload)
                    wheels.append(dict(at=now,fault=t.fault_bits,state=t.state_flags,
                        rpm=list(t.wheel_rpm),sequence=t.last_command_sequence,sample_ms=t.sample_time_ms))
                    if (t.fault_bits or f.flags or any(t.wheel_rpm) or not t.state_flags&8 or t.state_flags&(2|256)):
                        raise ValueError('native fault or nonzero/enabled wheel state '+hex(t.fault_bits))
                    state['wheel_count']+=1
                elif f.message_type==M.LINK_STATUS:
                    d=decode_motion_status(f.payload);diagnostics.append(dict(at=now,status=d))
                    if (f.flags or d['permitted'] or any(d['compares']+d['raw_duty']+d['target_cps']+
                        d['body_milli']+d['effective_cps']+d['granted_direction']) or d['fault_wheel'] or
                        d['watchdog_reset'] or d['first_feedback_fault']['reason']):
                        raise ValueError('native diagnostic fault or nonzero/enabled output')
                    state['status_count']+=1
            if wrapped.clock.fault:stop(wrapped.clock.fault)
            if wrapped.clock.notice=='HELLO_RTT_EXCEEDED':stop('HELLO_RTT_EXCEEDED')
        except Exception as problem:stop(str(problem))
        finally:
            pump.acknowledge();checkpoint('ROS_GUARD_EXIT')
    guard=node.create_guard_condition(receive)
    class TracedGuard:
        def trigger(self):
            checkpoint('WORKER_GUARD_TRIGGER');guard.trigger()
    pump=ReceiveEventPump(wrapped,TracedGuard())
    def tick():
        if audit.tx_stopped:return
        now=time.monotonic();checkpoint('SAFETY_TICK')
        try:
            clock=wrapped.clock
            if pump.error or clock.fault:raise ValueError(pump.error or clock.fault)
            reason=clock_failure(now,started,state['ready_at'],clock.ready(now))
            if reason:stop(reason);return
            if clock.ready(now) and state['ready_at'] is None:
                state['ready_at']=now;checkpoint('FIRST_READY')
                print('UNARMED_CLOCK_READY rtt_ms='+str(round(clock.rtt_s*1000,3)),flush=True)
            if now-started>=MAX_WINDOW_S:stop('FINITE_WINDOW_COMPLETE',error=False);return
            if session.connected and state['wheel_count'] and state['status_count'] and now>=state['next_send']:
                wrapped.sendto(session.cmd_vel(0,0,0,600,False,int(now*1000)&0xffffffff),ESP)
                state['next_send']=now+.020
        except Exception as problem:stop(str(problem))
    timer=node.create_timer(.01,tick);report={}
    try:
        checkpoint('ONE_HELLO_SEND')
        wrapped.sendto(session.new_hello(int(time.monotonic()*1000)&0xffffffff),ESP)
        while True:
            executor.spin_once(timeout_sec=.025)
            if audit.tx_stopped:
                if (time.monotonic_ns()-audit.stop_mono_ns)/1e9>=RX_TAIL_S:break
            elif time.monotonic()-started>MAX_WINDOW_S+1:
                stop('DIAGNOSTIC_WALL_LIMIT')
        end=time.monotonic();final_os=os_snapshot()
        try:final_queue=peek_queue(raw)
        except Exception as problem:final_queue=dict(error=str(problem))
        samples=audit.received;before_stop=[r for r in samples if not r['after_tx_stop']]
        delays=[r['kernel_to_ros_ms'] for r in samples]
        tx_cmd=[r for r in audit.sent if r['kind']=='CMD_VEL']
        report=dict(window_completed=state['termination']=='FINITE_WINDOW_COMPLETE',
            error=state['error'],elapsed_s=end-started,hello_attempts=wrapped.hello_attempts,
            hello_acks=state['acks'],startup_clock_at_end=wrapped.clock.snapshot(end),
            ready_after_s=None if state['ready_at'] is None else state['ready_at']-started,
            trip=trip,final_queue=final_queue,command_count=wrapped.command_count,
            tx_after_stop=sum(r['issued_steady']*1e9>audit.stop_mono_ns for r in audit.sent),
            all_outgoing_commands_disabled_zero=True,control_enable_clear_or_estop_sent=False,
            rehandshake_or_retry=False,clock_frozen=wrapped.clock.frozen,motion_started=False,
            firmware_or_network_settings_changed=False,protection_ms=200,
            kernel_to_ros_dispatch_median_ms=statistics.median(delays) if delays else None,
            kernel_to_ros_dispatch_max_ms=max(delays,default=None),
            rx_queue_overflow_max=max((r['rxq_overflow_total'] or 0 for r in samples),default=0),
            streams_before_stop=summarise(before_stop),streams_including_receive_only_tail=summarise(samples),
            command_max_send_gap_ms=max(((b['issued_steady']-a['issued_steady'])*1000 for a,b in zip(tx_cmd,tx_cmd[1:])),default=None),
            wheel_fault_values=sorted({r['fault'] for r in wheels}),initial_os=initial_os,final_os=final_os,
            trace_scope='Actual deployed receive pump + clock in a ROS node; not full operator console')
    finally:
        audit.stop_tx();timer.cancel();pump.stop();wrapped.close()
        executor.remove_node(node);executor.shutdown();node.destroy_node();ctx.shutdown()
        for name,value in (('RESULT.json',report),('kernel_receive.json',audit.received),
            ('outgoing.json',audit.sent),('ros_events.json',events),('wheels.json',wheels),('diagnostics.json',diagnostics)):
            with (folder/name).open('x') as stream:json.dump(value,stream,indent=2)
    small={k:v for k,v in report.items() if k not in ('initial_os','final_os','trip','final_queue')}
    small['trip_summary']={k:v for k,v in trip.items() if k not in ('last_dequeued_packets','accepted_source_times')}
    print(json.dumps(small,indent=2),flush=True)
    # A reproduced protection trip is a diagnostic result, never a stability pass.
    return 0 if report.get('window_completed') else 1

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--offline-check',action='store_true')
    ap.add_argument('--confirm-unarmed-zero-only',action='store_true')
    args=ap.parse_args()
    if args.offline_check:offline_check();return 0
    if not args.confirm_unarmed_zero_only:ap.error('explicit disabled-zero-only confirmation required')
    os.umask(0o077)
    folder=Path(tempfile.mkdtemp(prefix='kernel_zero_trace_',dir=READY.parent))
    os.environ.update(ROS_AUTOMATIC_DISCOVERY_RANGE='OFF',ROS_LOG_DIR=str(folder/'ros_logs'))
    print('EVIDENCE='+str(folder),flush=True)
    return live(folder)

if __name__=='__main__':raise SystemExit(main())
