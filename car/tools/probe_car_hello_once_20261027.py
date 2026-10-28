#!/usr/bin/env python3
"""One HELLO, no CMD/control/clear/enable. Then <=3s receive-only."""
import argparse
import fcntl
import json
from pathlib import Path
import select
import socket
import sys
import tempfile
import time

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0,str(ROOT/'src/mecanum_vicon_control'))
from mecanum_vicon_control.mc_link_session import McLinkSession
from mecanum_vicon_control.transport_protocol import MessageType as M,decode_frame,unpack_wheel_telemetry


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--confirm-one-hello-no-motion',action='store_true',required=True);ap.parse_args()
    folder=Path(tempfile.mkdtemp(prefix='hello_only_',dir=ROOT/'docs/wand_feedback_300ms_2026-10-27'))
    lock=open('/tmp/wand_mc_link_46001.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rows=[];sent=[];session=McLinkSession()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,b'wlx000000000000\0')
        sock.bind(('192.168.50.2',46001));sock.setblocking(False)
        at=time.monotonic();raw=session.new_hello(int(at*1000)&0xffffffff);f=decode_frame(raw)
        if f.message_type!=M.HELLO or f.payload or f.flags:raise ValueError('not the fixed empty HELLO')
        sent.append(dict(at=at,kind=M(f.message_type).name,session=f.session_id,raw_hex=raw.hex()))
        sock.sendto(raw,('192.168.50.1',46000))
        # There is deliberately NO sending code in this loop, no retry and no
        # session.accept (which could be mistaken for granted motion authority).
        while time.monotonic()-at<3.:
            if not select.select([sock],[],[],min(.05,max(0.,3.-(time.monotonic()-at))))[0]:continue
            data,peer=sock.recvfrom(1024);received=time.monotonic()
            row=dict(received=received,after_hello_ms=(received-at)*1000,peer=peer,raw_hex=data.hex())
            try:
                f=decode_frame(data);row.update(kind=M(f.message_type).name,session=f.session_id,
                    flags=f.flags,source_ms=f.sender_time_ms,same_session=f.session_id==session.session_id)
                if f.message_type==M.WHEEL_TELEMETRY:
                    t=unpack_wheel_telemetry(f.payload)
                    row.update(fault_bits=t.fault_bits,state_flags=t.state_flags,rpm=list(t.wheel_rpm))
            except ValueError as error:row['decode_error']=str(error)
            rows.append(row)
    lock.close()
    result=dict(hello_sent=1,commands_or_controls_sent=0,motion_started=False,seconds=time.monotonic()-at,
        packets=len(rows),same_session_packets=sum(r.get('same_session',False) for r in rows),
        hello_ack_after_ms=[r['after_hello_ms'] for r in rows if r.get('kind')=='HELLO_ACK' and r.get('same_session')],
        fault_bits=sorted({r['fault_bits'] for r in rows if 'fault_bits' in r}),
        all_received_wheel_rpm_zero=all(not any(r['rpm']) for r in rows if 'rpm' in r),
        firmware_or_network_changed=False,evidence=str(folder))
    for name,value in (('RESULT.json',result),('received.json',rows),('sent.json',sent)):
        (folder/name).write_text(json.dumps(value,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
