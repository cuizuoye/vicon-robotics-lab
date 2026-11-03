"""Instrument the frozen STOPFIX zero-only probe. Never enable/clear/change network."""
import argparse
import importlib.util
import json
from pathlib import Path
import socket
import statistics
import sys
import tempfile
import time
from unittest.mock import patch

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
DEPLOY=ROOT/'candidates/wand_stopfix_deploy_20261102'
sys.path.insert(0,str(DEPLOY))
from ground_entry import check,current
from build import RECORD,sha
READY=current();sys.path.insert(0,str(READY));import bootstrap
import idle_probe
from mecanum_vicon_control.transport_protocol import decode_frame,MessageType as M,unpack_wheel_telemetry
from mecanum_wand_4wsc_hardware.stage0_contract import decode_motion_status

KERNEL=ROOT/'candidates/wand_feedback_guard/kernel_ingress.py'
spec=importlib.util.spec_from_file_location('reused_kernel_metadata',KERNEL)
kernel=importlib.util.module_from_spec(spec);spec.loader.exec_module(kernel)
REAL_SOCKET=socket.socket
OUT=ROOT/'docs/wand_stopfix_feedback_2026-11-02'

def save(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

class AuditSocket:
    def __init__(self,raw,destination,tail=.5):
        self.raw,self.destination,self.tail=raw,destination,tail
        self.sent=[];self.received=[];self.closed=False;self.readonly_tail=False
        raw.setsockopt(socket.SOL_SOCKET,kernel.SO_TIMESTAMPNS,1)
        raw.setsockopt(socket.SOL_SOCKET,kernel.SO_RXQ_OVFL,1)
    def __getattr__(self,name):return getattr(self.raw,name)
    def sendto(self,data,destination):
        if self.closed or self.readonly_tail:raise OSError('diagnostic TX revoked')
        f=idle_probe.zero_only(data)
        if destination!=self.destination:raise ValueError('diagnostic destination mismatch')
        value=self.raw.sendto(data,destination)
        self.sent.append(dict(at=time.monotonic(),kind=int(f.message_type),source_ms=f.sender_time_ms,
            sequence=f.sequence,session=f.session_id,flags=f.flags,payload_hex=f.payload.hex()))
        return value
    def recvfrom(self,size):
        data,items,flags,peer=self.raw.recvmsg(size,256)
        mono,real=time.monotonic_ns(),time.time_ns();stamp,drops=kernel.ancillary_fields(items)
        if flags&(socket.MSG_TRUNC|socket.MSG_CTRUNC):raise OSError('truncated diagnostic packet')
        f=decode_frame(data)
        self.received.append(dict(at=mono/1e9,real_ns=real,kernel_real_ns=stamp,kernel_to_user_ms=(real-stamp)/1e6,
            rxq_overflow=drops,peer=peer,tail=self.readonly_tail,kind=int(f.message_type),source_ms=f.sender_time_ms,
            sequence=f.sequence,session=f.session_id,flags=f.flags,payload_hex=f.payload.hex()))
        if len(self.received)>5000:raise OSError('finite capture bound exceeded')
        return data,peer
    def close(self):
        if self.closed:return
        self.readonly_tail=True;end=time.monotonic()+self.tail
        try:
            while time.monotonic()<end:
                try:self.recvfrom(512)
                except BlockingIOError:time.sleep(.003)
        finally:self.raw.close();self.closed=True

def analyse(folder,result,audit,clock):
    rows=audit.received;streams={};faults=[];acks=[x for x in rows if x['kind']==2 and x['session']==clock.session_id and x['sequence']==clock.hello_sequence]
    ack=acks[0] if acks else None
    for kind in (48,49):
        selected=[x for x in rows if x['kind']==kind and x['session']==clock.session_id]
        gaps=[(b['kernel_real_ns']-a['kernel_real_ns'])/1e6 for a,b in zip(selected,selected[1:])]
        ages=[]
        for row in selected:
            if ack and clock.source_at is not None:
                delta=(row['source_ms']-ack['source_ms'])&0xffffffff
                if delta<0x80000000:
                    mapped=clock.source_at+delta/1000
                    ages.append((row['at']-mapped)*1000-row['kernel_to_user_ms'])
            if kind==48:faults.append(unpack_wheel_telemetry(bytes.fromhex(row['payload_hex'])).fault_bits)
        streams[M(kind).name]=dict(raw_frames=len(selected),max_kernel_receive_gap_ms=max(gaps,default=None),
            max_ack_anchor_age_at_kernel_ms=max(ages,default=None),kernel_age_not_one_way_delay=True)
    delay=[x['kernel_to_user_ms'] for x in rows]
    native_path=folder/'native_clock.jsonl';native=[json.loads(x) for x in native_path.read_text().splitlines()] if native_path.exists() else []
    rejected=[x for x in native if x['accepted_sample_steady'] is None]
    report=dict(passed=result.get('passed',False),probe_result=result,clock=clock.snapshot(time.monotonic()),
        fault_values=sorted(set(faults)),streams=streams,raw_frames=len(rows),clock_rejected_frames=len(rejected),
        max_kernel_to_user_ms=max(delay,default=None),median_kernel_to_user_ms=statistics.median(delay) if delay else None,
        max_socket_overflow=max((x['rxq_overflow'] or 0 for x in rows),default=0),
        max_tx_gap_ms=max(((b['at']-a['at'])*1000 for a,b in zip(audit.sent,audit.sent[1:])),default=None),
        protection_s=.3,guard_relaxed=False,real_hardware_zero_only=True,enable_sent=False,select_sent=False,clear_fault_sent=False,
        nonzero_sent=False,network_changed=False,firmware_written=False,receive_only_tail_s=audit.tail,
        reused_kernel_metadata_source=str(KERNEL),reused_kernel_source_sha256=sha(KERNEL),frozen_probe_source_sha256=sha(READY/'idle_probe.py'),
        scope='Same frozen probe/clock and original300ms guards; instrumentation only. MCU/HELLO anchored sample age is not measured one-way network delay.')
    save(folder/'kernel_receive.json',rows);save(folder/'outgoing.json',audit.sent);save(folder/'RESULT.json',report)
    print(json.dumps(dict(report,folder=str(folder)),ensure_ascii=False),flush=True)

def live(seconds):
    check(READY);OUT.mkdir(parents=True,exist_ok=True)
    folder=Path(tempfile.mkdtemp(prefix='capture_',dir=OUT));p=json.loads((READY/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
    destination=(p['hardware']['esp_ip'],46000);audits=[];wrapped=[];original=idle_probe.StartupProgressSocket
    def factory(*args,**kw):
        a=AuditSocket(REAL_SOCKET(*args,**kw),destination);audits.append(a);return a
    def traced(*args,**kw):
        kw['trace_path']=str(folder/'native_clock.jsonl');value=original(*args,**kw);wrapped.append(value);return value
    result={}
    with patch.object(idle_probe.socket,'socket',factory),patch.object(idle_probe,'StartupProgressSocket',traced):
        try:result=idle_probe.run(p,RECORD,folder/'PROBE',idle_probe.CONSENT,seconds)
        except (ValueError,OSError) as error:result=dict(passed=False,reason=str(error))
    if not audits or not wrapped:raise ValueError('capture failed before proof construction')
    analyse(folder,result,audits[0],wrapped[0].clock)

def offline():
    from mecanum_vicon_control.transport_protocol import encode_frame,pack_cmd_vel,FrameFlags
    valid=[encode_frame(M.HELLO,1,123,0),encode_frame(M.CMD_VEL,2,123,10,pack_cmd_vel(0,0,0,600))]
    class Fake:
        def setsockopt(self,*args):pass
        def sendto(self,*args):return 1
        def close(self):pass
    a=AuditSocket(Fake(),('192.168.50.1',46000),0)
    for raw in valid:a.sendto(raw,a.destination)
    for raw in ([encode_frame(k,3,123,20) for k in (M.CONTROL_ENABLE,M.ESTOP,M.HEARTBEAT)]+
                [encode_frame(M.CMD_VEL,3,123,20,pack_cmd_vel(.001,0,0,600)),encode_frame(M.CMD_VEL,3,123,20,pack_cmd_vel(0,0,0,600),FrameFlags.ENABLED)]):
        try:a.sendto(raw,a.destination)
        except ValueError:pass
        else:raise AssertionError('unsafe TX accepted')
    a.close()
    try:a.sendto(valid[0],a.destination)
    except OSError:pass
    else:raise AssertionError('TX after close accepted')
    print('PASS: original zero-only whitelist, no hardware, closed-TX rejection')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--live-zero-only',action='store_true');ap.add_argument('--seconds',type=float,default=12.);args=ap.parse_args()
    if not 1<=args.seconds<=30:raise ValueError('finite1..30s window')
    if args.live_zero_only:live(args.seconds)
    else:offline()
