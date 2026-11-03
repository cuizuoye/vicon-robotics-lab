#!/usr/bin/env python3
"""Ten seconds RX ONLY from existing expired unarmed session, no datagram TX.

No HELLO/retry/CMD/clear/enable. No changes to firmware or network settings.
Kernel timestamp versus actual ROS guard dispatch, using deployed event pump.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import statistics
import struct
import sys
import tempfile
import time

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
BASE=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio'
READY=BASE/'ready_v2'
sys.path[:0]=[str(ROOT/'src/mecanum_vicon_control'),str(READY/'runtime/src/mecanum_wand_4ws8_hardware')]
from mecanum_vicon_control.transport_protocol import MessageType as M,decode_frame,unpack_wheel_telemetry
from mecanum_wand_4ws8_hardware.receive_event import ReceiveEventPump

class ReceiveOnly:
    def __init__(self,base):self.base=base
    def __getattr__(self,key):
        if key in ('send','sendto','sendmsg','sendall','connect'):raise RuntimeError('passive observer forbids TX/connect')
        return getattr(self.base,key)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--confirm-receive-only',action='store_true',required=True);ap.parse_args()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    os.umask(0o077)
    folder=Path(tempfile.mkdtemp(prefix='passive_rx_',dir=BASE))
    print('EVIDENCE='+str(folder),flush=True)
    os.environ.update(ROS_AUTOMATIC_DISCOVERY_RANGE='OFF',ROS_LOG_DIR=str(folder/'ros_logs'))
    ctx=Context();rclpy.init(context=ctx,domain_id=169)
    node=Node('eventio_receive_only',context=ctx,enable_rosout=False,start_parameter_services=False)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(node)
    raw=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    raw.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,b'wlx000000000000\0')
    raw.setsockopt(socket.SOL_SOCKET,35,1)  # Linux x86_64 SO_TIMESTAMPNS.
    raw.bind(('192.168.50.2',46001));raw.setblocking(False)
    sock=ReceiveOnly(raw);rows=[];errors=[];pump=None
    def receive():
        try:
            if pump.error:raise RuntimeError(pump.error)
            while True:
                try:data,ancillary,flags,peer=sock.recvmsg(1024,128)
                except BlockingIOError:break
                now=time.monotonic_ns();real=time.time_ns()
                if peer!=('192.168.50.1',46000):continue
                f=decode_frame(data)
                stamps=[struct.unpack('=qq',value[:16]) for level,kind,value in ancillary
                        if level==socket.SOL_SOCKET and kind==35 and len(value)>=16]
                if len(stamps)!=1:raise ValueError('kernel timestamp absent')
                sec,nsec=stamps[0];kernel=sec*1000000000+nsec
                row=dict(kind=M(f.message_type).name,session=f.session_id,source_ms=f.sender_time_ms,
                    sequence=f.sequence,received_mono_ns=now,kernel_real_ns=kernel,
                    dispatch_ms=(real-kernel)/1e6,flags=f.flags)
                if f.message_type==M.WHEEL_TELEMETRY:
                    t=unpack_wheel_telemetry(f.payload);row.update(fault=t.fault_bits,rpm=list(t.wheel_rpm),state=t.state_flags)
                rows.append(row)
        except Exception as error:errors.append(str(error))
        finally:pump.acknowledge()
    guard=node.create_guard_condition(receive);pump=ReceiveEventPump(sock,guard)
    started=time.monotonic();offset_start=time.time_ns()-time.monotonic_ns()
    try:
        while time.monotonic()-started<10 and not errors:executor.spin_once(timeout_sec=.03)
        end=time.monotonic();offset_end=time.time_ns()-time.monotonic_ns()
        # Include a terminal blind interval, not just gaps between received pairs.
        tail_ms=(time.monotonic_ns()-rows[-1]['received_mono_ns'])/1e6 if rows else None
    finally:
        pump.stop();sock.close();executor.remove_node(node);executor.shutdown();node.destroy_node();ctx.shutdown()
    results={}
    for name in ('WHEEL_TELEMETRY','LINK_STATUS'):
        stream=[r for r in rows if r['kind']==name]
        results[name]=dict(frames=len(stream),
            max_kernel_gap_ms=max(((b['kernel_real_ns']-a['kernel_real_ns'])/1e6 for a,b in zip(stream,stream[1:])),default=None),
            max_ros_receive_gap_ms=max(((b['received_mono_ns']-a['received_mono_ns'])/1e6 for a,b in zip(stream,stream[1:])),default=None),
            max_native_sample_step_ms=max((((b['source_ms']-a['source_ms'])&0xffffffff) for a,b in zip(stream,stream[1:])),default=None))
    delays=[r['dispatch_ms'] for r in rows]
    report=dict(receive_only=True,datagrams_sent=0,rehandshake_or_retry=False,motion_started=False,
        seconds=end-started,received_frames=len(rows),terminal_receive_gap_ms=tail_ms,
        errors=errors,pump_error=pump.error,streams=results,
        kernel_to_ros_dispatch_median_ms=statistics.median(delays) if delays else None,
        kernel_to_ros_dispatch_max_ms=max(delays,default=None),
        wheel_fault_values=sorted({r['fault'] for r in rows if 'fault' in r}),
        realtime_monotonic_offset_delta_ns=offset_end-offset_start)
    for name,value in (('RESULT.json',report),('packets.json',rows)):
        with (folder/name).open('x') as stream:json.dump(value,stream,indent=2)
    print(json.dumps(report,indent=2),flush=True)
    return 0 if not errors and rows else 1

if __name__=='__main__':raise SystemExit(main())
