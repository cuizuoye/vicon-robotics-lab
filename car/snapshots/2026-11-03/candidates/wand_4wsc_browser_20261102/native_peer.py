"""Loopback-only exact C firmware peer; uncalibrated software encoders, not hardware."""
import ctypes
from collections import deque
import math
import multiprocessing as mp
from pathlib import Path
import queue
import socket
import threading
import time
import bootstrap
from mecanum_vicon_control.transport_protocol import MessageType,decode_frame,unpack_wheel_telemetry
from mecanum_vicon_control.mecanum_kinematics import WheelSpeeds,wheel_angular_to_body
from mecanum_wand_4wsc_hardware.stage0_contract import decode_status_frame

LIB=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_firmware_4wsc_2026-10-30/prepared_v4/native/libstage0_oracle.so')

class Oracle:
    def __init__(self):
        self.lib=ctypes.CDLL(str(LIB));self.lib.oracle_init()
        self.lib.oracle_request.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint32,ctypes.c_void_p]
        self.lib.oracle_request.restype=ctypes.c_size_t
        self.lib.oracle_tick.argtypes=[ctypes.c_uint32]
        self.lib.oracle_packet.argtypes=[ctypes.c_uint,ctypes.c_void_p];self.lib.oracle_packet.restype=ctypes.c_size_t
        self.now=0;self.pose=[0.,0.,0.];self.status=self.wheels=None;self.frames=[]
    def request(self,raw,now=None):
        src=ctypes.create_string_buffer(raw);dst=ctypes.create_string_buffer(512)
        n=self.lib.oracle_request(src,len(raw),self.now if now is None else now,dst)
        return dst.raw[:n]
    def packet(self,kind):
        out=ctypes.create_string_buffer(512);n=self.lib.oracle_packet(kind,out)
        return out.raw[:n]
    def step(self,now):
        while self.now+10<=now:
            self.now+=10;self.lib.oracle_tick(self.now)
            raw=self.packet(0x30)
            if not raw:continue
            self.wheels=unpack_wheel_telemetry(decode_frame(raw).payload)
            d=self.packet(0x31);self.status=decode_status_frame(d)
            w=WheelSpeeds(*(v*math.pi/30 for v in self.wheels.wheel_rpm))
            x,y,a=wheel_angular_to_body(w,.03,.061,.085);h=self.pose[2]
            self.pose[0]+=(math.cos(h)*x-math.sin(h)*y)*.01
            self.pose[1]+=(math.sin(h)*x+math.cos(h)*y)*.01;self.pose[2]+=a*.01
            self.frames=[raw,d]

class _LocalPeer:
    def __init__(self,port,oracle=None):
        if type(port) is not int or not 1024<=port<=65535 or port in (46000,46001):raise ValueError('dedicated loopback port required')
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('127.0.0.1',port));self.sock.settimeout(.003)
        self.oracle=Oracle() if oracle is None else oracle;self.start=time.monotonic()-self.oracle.now/1000;self.done=threading.Event();self.lock=threading.Lock()
        self.last=None;self.address=None;self.feedback=True;self.receive=True;self.error='';self.sent=self.received=0
        self.peak_duty=0;self.peak_rpm=0.;self.permit_ever=False
        self.wire_history=deque(maxlen=500);self.anchor=None
        self.thread=threading.Thread(target=self.run,daemon=True,name='loopback-4WSC-C-only');self.thread.start()
    def run(self):
        pending=deque(maxlen=64)
        try:
            while not self.done.is_set():
                try:
                    raw,addr=self.sock.recvfrom(512)
                    if addr[0]!='127.0.0.1':continue
                except socket.timeout:raw=None
                with self.lock:
                    now=time.monotonic()
                    if raw and self.receive:
                        if len(pending)>=64:raise ValueError('loopback ingress overflow')
                        pending.append((raw,addr));self.received+=1
                    # Production ISR order: queued requests -> control tick ->
                    # cached native snapshot. Never process at a PREVIOUS tick
                    # and give the ACK an artificially old MCU timestamp.
                    target=int((now-self.start)*1000)
                    while self.oracle.now+10<=target:
                        tick=self.oracle.now+10;replies=[]
                        while pending:
                            request,address=pending.popleft();f=decode_frame(request)
                            if f.message_type==MessageType.HELLO:self.anchor=(tick,f.sender_time_ms)
                            local,source=self.anchor or (tick,f.sender_time_ms)
                            self.wire_history.append(dict(local_ms=tick,sender_ms=f.sender_time_ms,
                                kind=int(f.message_type),sequence=f.sequence,session=f.session_id,
                                local_elapsed=tick-local,source_elapsed=(f.sender_time_ms-source)&0xffffffff,
                                skew_ms=(tick-local)-((f.sender_time_ms-source)&0xffffffff),
                                fault_before=None if self.oracle.wheels is None else self.oracle.wheels.fault_bits))
                            self.address=address;reply=self.oracle.request(request,now=tick)
                            if reply:replies.append((reply,address))
                        self.oracle.step(tick)
                        if self.feedback:
                            for reply,address in replies:self.sock.sendto(reply,address)
                    if self.oracle.status:
                        self.peak_duty=max(self.peak_duty,max(map(abs,self.oracle.status['raw_duty'])))
                        self.peak_rpm=max(self.peak_rpm,max(map(abs,self.oracle.wheels.wheel_rpm)))
                        self.permit_ever|=self.oracle.status['permitted']
                    if self.address and self.feedback and self.oracle.frames and (self.last is None or now-self.last>=.019):
                        for f in self.oracle.frames:self.sock.sendto(f,self.address);self.sent+=1
                        self.last=now
        except Exception as e:self.error=repr(e)
    def sample(self):
        with self.lock:
            return dict(at=self.start+self.oracle.now/1000,pose=self.oracle.pose.copy(),
                status=self.oracle.status.copy() if self.oracle.status else None,peer_error=self.error)
    def close(self):
        self.done.set();self.thread.join(timeout=2);self.sock.close()

def _latest(channel,value):
    try:channel.put_nowait(value)
    except queue.Full:
        try:channel.get_nowait()
        except queue.Empty:return
        try:channel.put_nowait(value)
        except queue.Full:pass

def _peer_child(port,commands,samples,ready,stop):
    peer=None
    try:
        peer=_LocalPeer(port);ready.put(dict(ready=True))
        while not stop.is_set():
            for _ in range(8):
                try:key,value=commands.get_nowait()
                except queue.Empty:break
                if key not in ('feedback','receive') or type(value) is not bool:raise ValueError('test control schema')
                setattr(peer,key,value)
            sample=peer.sample()
            sample.update(peak_duty=peer.peak_duty,peak_rpm=peer.peak_rpm,permit_ever=peer.permit_ever,
                history=list(peer.wire_history)[-12:])
            _latest(samples,sample);time.sleep(.005)
    except BaseException as e:
        _latest(samples,dict(peer_error=repr(e)))
        try:ready.put_nowait(dict(ready=False,error=repr(e)))
        except queue.Full:pass
    finally:
        if peer is not None:peer.close()
        for channel in (commands,samples,ready):channel.cancel_join_thread()

class Peer:
    """Independent-process MCU, not a GIL-sharing ROS/NumPy worker thread."""
    def __init__(self,port):
        self.port=port;ctx=mp.get_context('spawn')
        self.commands=ctx.Queue(maxsize=8);self.samples=ctx.Queue(maxsize=1);self.ready=ctx.Queue(maxsize=1);self.stop=ctx.Event()
        self.closed=False;self._feedback=self._receive=True
        self.value=dict(at=time.monotonic(),pose=[0.,0.,0.],status=None,peer_error='',peak_duty=0,peak_rpm=0.,permit_ever=False,history=[])
        self.process=ctx.Process(target=_peer_child,args=(port,self.commands,self.samples,self.ready,self.stop),daemon=True,name='4wsc-independent-C-MCU')
        self.process.start()
        try:
            first=self.ready.get(timeout=5.)
            if not first.get('ready'):raise ValueError(first)
        except BaseException:self.close();raise
    def sample(self):
        if self.closed:return self.value.copy()
        for _ in range(2):
            try:value=self.samples.get_nowait()
            except queue.Empty:break
            self.value.update(value)
        if not self.closed and not self.process.is_alive():self.value['peer_error']='C_MCU_PROCESS_EXITED'
        return self.value.copy()
    @property
    def peak_duty(self):return self.sample()['peak_duty']
    @property
    def peak_rpm(self):return self.sample()['peak_rpm']
    @property
    def permit_ever(self):return self.sample()['permit_ever']
    @property
    def wire_history(self):return self.sample()['history']
    @property
    def feedback(self):return self._feedback
    @feedback.setter
    def feedback(self,value):
        if bool(value)!=self._feedback:self.commands.put_nowait(('feedback',bool(value)));self._feedback=bool(value)
    @property
    def receive(self):return self._receive
    @receive.setter
    def receive(self,value):
        if bool(value)!=self._receive:self.commands.put_nowait(('receive',bool(value)));self._receive=bool(value)
    def close(self):
        if self.closed:return
        self.sample();self.closed=True;self.stop.set();self.process.join(timeout=2.)
        if self.process.is_alive():self.process.terminate();self.process.join(timeout=2.)
        for channel in (self.commands,self.samples,self.ready):channel.close();channel.cancel_join_thread()
