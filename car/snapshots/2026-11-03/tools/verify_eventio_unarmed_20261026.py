#!/usr/bin/env python3
"""Finite real ROS event-RX check, HELLO + disabled zero ONLY; no GUI/control.

No clear, mode selection, enable, ESTOP, motion, serial, firmware or network
configuration API is present. All outgoing frames pass a strict final guard.
Uses the deployed candidate's unchanged startup clock and event receive code.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import statistics
import sys
import tempfile
import time

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
READY=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'
ESP=('192.168.50.1',46000)
INTERFACE='wlx000000000000'
for path in (ROOT/'src/mecanum_vicon_control',
    READY/'runtime/src/mecanum_wand_4ws8_hardware',
    READY/'runtime/src/mecanum_wand_4ws8_control'):
    sys.path.insert(0,str(path))
from mecanum_vicon_control.transport_protocol import (
    MessageType as M, FrameFlags as F, decode_frame, encode_frame,
    unpack_cmd_vel, pack_cmd_vel, unpack_wheel_telemetry)
from mecanum_wand_4ws8_hardware.startup_clock import StartupProgressSocket
from mecanum_wand_4ws8_hardware.receive_event import ReceiveEventPump
from mecanum_wand_4ws8_hardware.stage0_session import Stage0Session
from mecanum_wand_4ws8_hardware.stage0_contract import decode_motion_status

def validate_outgoing(data,destination):
    f=decode_frame(data)
    if destination!=ESP or not f.session_id:raise ValueError('diagnostic destination/session rejected')
    if f.message_type==M.HELLO:
        if f.flags or f.payload:raise ValueError('only plain empty HELLO allowed')
    elif f.message_type==M.CMD_VEL:
        v=unpack_cmd_vel(f.payload)
        if f.flags or f.payload!=pack_cmd_vel(0.,0.,0.,600):
            raise ValueError('only exact disabled zero 600ms CMD allowed')
    else:raise ValueError('no control/enable/clear/ESTOP/motion frame allowed')
    return f

def clock_failure(now,started,ready_at,ready):
    if ready_at is not None and not ready:return 'qualified source freshness lost'
    if ready_at is None and now-started>2 and not ready:return 'startup clock not ready within two seconds'
    return ''

class ZeroOnlySocket:
    def __init__(self,base):self.base,self.sent=base,[]
    def __getattr__(self,key):return getattr(self.base,key)
    def sendto(self,data,destination):
        f=validate_outgoing(data,destination)
        if f.message_type==M.HELLO and self.sent:raise ValueError('exactly one HELLO; no retry/reconnect')
        now=time.monotonic();result=self.base.sendto(data,destination)
        self.sent.append(dict(kind=M(f.message_type).name,sequence=f.sequence,flags=f.flags,
            source_ms=f.sender_time_ms,issued_steady=now,disabled_zero=f.message_type==M.CMD_VEL))
        return result

def offline_check():
    valid=[encode_frame(M.HELLO,1,123,10),encode_frame(M.CMD_VEL,2,123,20,pack_cmd_vel(0,0,0,600))]
    for raw in valid:validate_outgoing(raw,ESP)
    invalid=[encode_frame(kind,3,123,30,b'') for kind in (M.CONTROL_ENABLE,M.ESTOP,M.HEARTBEAT)]
    invalid += [encode_frame(M.CMD_VEL,3,123,30,pack_cmd_vel(*v,600))
                for v in ((.001,0,0),(0,-.001,0),(0,0,.001))]
    invalid += [encode_frame(M.CMD_VEL,3,123,30,pack_cmd_vel(0,0,0,600),F.ENABLED),
                encode_frame(M.CMD_VEL,3,123,30,pack_cmd_vel(0,0,0,150))]
    for raw in invalid:
        try:validate_outgoing(raw,ESP)
        except ValueError:pass
        else:raise AssertionError('unsafe outgoing frame was accepted')
    try:validate_outgoing(valid[0],('127.0.0.1',46000))
    except ValueError:pass
    else:raise AssertionError('wrong destination accepted')
    assert clock_failure(6,0,.5,False)=='qualified source freshness lost'
    assert clock_failure(3,0,None,False)=='startup clock not ready within two seconds'
    assert not clock_failure(1,0,None,False)
    assert not clock_failure(6,0,.5,True)
    print('PASS: two safe frame forms accepted; nine unsafe forms/destinations rejected; no I/O.')

def live(folder):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    ctx=Context();rclpy.init(context=ctx,domain_id=169)
    node=Node('eventio_unarmed_verification',context=ctx,enable_rosout=False,start_parameter_services=False)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(node)
    raw=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    raw.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,INTERFACE.encode()+b'\0')
    raw.bind(('192.168.50.2',46001));raw.setblocking(False)
    audit=ZeroOnlySocket(raw);session=Stage0Session(stage=4)
    wrapped=StartupProgressSocket(audit,ESP[0],lambda:session.session_id,str(folder/'native_receive.jsonl'))
    state=dict(error='',acks=0,ready_at=None,done=False,next_send=0.,wheel_count=0,status_count=0)
    wheels=[];diagnostics=[];pump=None
    started=time.monotonic()
    def fail(reason):state['error']=state['error'] or str(reason)
    def receive():
        try:
            if pump.error:raise ValueError('receive pump '+pump.error)
            while True:
                try:data,peer=wrapped.recvfrom(1024)
                except BlockingIOError:break
                if peer!=ESP:raise ValueError('unexpected telemetry source port')
                f=session.accept(data);now=time.monotonic()
                if f.message_type==M.HELLO_ACK:
                    state['acks']+=1
                elif f.message_type==M.WHEEL_TELEMETRY:
                    t=unpack_wheel_telemetry(f.payload)
                    if (t.fault_bits or f.flags or any(t.wheel_rpm)
                            or not t.state_flags&8 or t.state_flags&(2|256)):
                        raise ValueError('native wheel not fault-free/disabled/stationary: '+hex(t.fault_bits))
                    state['wheel_count']+=1
                    wheels.append(dict(at=now,fault=t.fault_bits,state=t.state_flags,
                        rpm=list(t.wheel_rpm),sequence=t.last_command_sequence,sample_ms=t.sample_time_ms))
                elif f.message_type==M.LINK_STATUS:
                    d=decode_motion_status(f.payload)
                    if (f.flags or d['permitted'] or any(d['compares']+d['raw_duty']+d['target_cps']+
                        d['body_milli']+d['effective_cps']+d['granted_direction']) or d['fault_wheel'] or
                        d['watchdog_reset'] or d['first_feedback_fault']['reason']):
                        raise ValueError('native diagnostic not fault-free disabled zero')
                    state['status_count']+=1
                    diagnostics.append(dict(at=now,status=d))
            if wrapped.clock.fault:fail(wrapped.clock.fault)
            if wrapped.clock.notice=='HELLO_RTT_EXCEEDED':fail('HELLO_RTT_EXCEEDED')
        except Exception as error:fail(error)
        finally:pump.acknowledge()
    guard=node.create_guard_condition(receive);pump=ReceiveEventPump(wrapped,guard)
    def tick():
        if state['error'] or state['done']:return
        try:
            now=time.monotonic();clock=wrapped.clock
            if pump.error or clock.fault:raise ValueError(pump.error or clock.fault)
            reason=clock_failure(now,started,state['ready_at'],clock.ready(now))
            if reason:raise ValueError(reason)
            if clock.ready(now) and state['ready_at'] is None:
                state['ready_at']=now
                print('UNARMED_CLOCK_READY rtt_ms='+str(round(clock.rtt_s*1000,3)),flush=True)
            if state['ready_at'] is not None and now-state['ready_at']>=10:
                state['done']=True;return
            if (session.connected and state['wheel_count'] and state['status_count'] and now>=state['next_send']):
                wrapped.sendto(session.cmd_vel(0.,0.,0.,600,False,int(now*1000)&0xffffffff),ESP)
                state['next_send']=now+.020
        except Exception as error:fail(error)
    timer=node.create_timer(.01,tick)
    report={}
    try:
        wrapped.sendto(session.new_hello(int(time.monotonic()*1000)&0xffffffff),ESP)
        while not state['done'] and not state['error'] and time.monotonic()-started<13:
            executor.spin_once(timeout_sec=.03)
        now=time.monotonic()
        samples=[w['sequence'] for w in wheels]
        gaps={name:max((b['at']-a['at'] for a,b in zip(rows,rows[1:])),default=0)*1000
              for name,rows in (('wheel',wheels),('diagnostic',diagnostics))}
        report=dict(passed=bool(state['done'] and not state['error'] and wrapped.clock.ready(now)
                    and len(set(samples))>=10),error=state['error'],elapsed_s=now-started,
            hello_attempts=wrapped.hello_attempts,hello_acks=state['acks'],
            startup_clock=wrapped.clock.snapshot(now),ready_after_s=None if state['ready_at'] is None else state['ready_at']-started,
            command_count=wrapped.command_count,all_outgoing_commands_disabled_zero=True,
            control_enable_clear_or_estop_sent=False,clock_frozen=False,operator_or_gui_started=False,
            motion_started=False,network_settings_changed=False,firmware_written=False,
            wheel_frames=len(wheels),diagnostic_frames=len(diagnostics),fault_values=sorted({w['fault'] for w in wheels}),
            unique_command_feedback_sequences=len(set(samples)),max_receive_gaps_ms=gaps,
            transport_components='Actual ROS guard + deployed ReceiveEventPump + StartupProgressSocket; not the full operator console',
            after_exit='Sender closes; existing ESP/MCU idle lease timeout may subsequently be reported; no clear/reconnect.')
    finally:
        timer.cancel();pump.stop();wrapped.close()
        executor.remove_node(node);executor.shutdown();node.destroy_node();ctx.shutdown()
        for name,value in (('RESULT.json',report),('outgoing.json',audit.sent),('wheels.json',wheels),('diagnostics.json',diagnostics)):
            with (folder/name).open('x') as stream:json.dump(value,stream,indent=2)
    print(json.dumps(report,indent=2),flush=True)
    return 0 if report.get('passed') else 1

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--offline-check',action='store_true')
    ap.add_argument('--confirm-unarmed-zero-only',action='store_true')
    args=ap.parse_args()
    if args.offline_check:offline_check();return 0
    if not args.confirm_unarmed_zero_only:ap.error('explicit unarmed-only confirmation required')
    os.umask(0o077)
    folder=Path(tempfile.mkdtemp(prefix='unarmed_eventio_',dir=READY.parent));folder.chmod(0o700)
    os.environ.update(ROS_AUTOMATIC_DISCOVERY_RANGE='OFF',ROS_LOG_DIR=str(folder/'ros_logs'))
    print('EVIDENCE='+str(folder),flush=True)
    return live(folder)

if __name__=='__main__':raise SystemExit(main())
