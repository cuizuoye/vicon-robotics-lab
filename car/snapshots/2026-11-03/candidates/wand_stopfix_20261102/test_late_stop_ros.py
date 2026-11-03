"""Actual ROS/UDP/C stop-path queue injection. Dedicated loopback ports only."""
import heapq
import json
import os
from pathlib import Path
import select
import socket
import struct
import sys
import tempfile
import threading
import time
from build import OUT,sha,save
from ground_entry import free_port,profile

READY=OUT/'host'
sys.path.insert(0,str(READY));import bootstrap
from native_peer import Peer
from mecanum_vicon_control.transport_protocol import decode_frame,MessageType,FrameFlags,unpack_cmd_vel,unpack_wheel_telemetry

class DelayedStopPeer:
    def __init__(self,port):
        if not 12000<=port<20000:raise ValueError('dedicated loopback proxy only')
        self.port=port;self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1',port));other=free_port()
        while other==port:other=free_port()
        self.inner=Peer(other);self.dest=('127.0.0.1',other);self.bridge=None
        self.delay_ms=0;self.delay_until=0.;self.delayed=0;self.max_fault=0;self.max_delay_s=0.;self.error=''
        self.done=threading.Event();self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
    def loop(self):
        pending=[];serial=0
        try:
            while not self.done.is_set():
                if select.select([self.sock],[],[],.003)[0]:
                    raw,addr=self.sock.recvfrom(512);now=time.monotonic();f=decode_frame(raw)
                    if addr==self.dest:
                        if f.message_type==MessageType.WHEEL_TELEMETRY:self.max_fault|=unpack_wheel_telemetry(f.payload).fault_bits
                        if self.bridge:self.sock.sendto(raw,self.bridge)
                    elif addr[0]=='127.0.0.1':
                        self.bridge=addr
                        stop=(f.message_type==MessageType.CONTROL_ENABLE and f.payload==struct.pack('<BBH',0,0,0))
                        if f.message_type==MessageType.CMD_VEL:
                            v=unpack_cmd_vel(f.payload)
                            stop=not f.flags&int(FrameFlags.ENABLED) and not any((v.vx_mps,v.vy_mps,v.wz_radps))
                        delay=self.delay_ms*.001 if stop and now<self.delay_until else 0.
                        if delay:self.delayed+=1
                        serial+=1;heapq.heappush(pending,(now+delay,serial,now,raw,delay))
                now=time.monotonic()
                while pending and pending[0][0]<=now:
                    _,_,received,raw,delay=heapq.heappop(pending)
                    if delay:self.max_delay_s=max(self.max_delay_s,now-received)
                    self.sock.sendto(raw,self.dest)
                if len(pending)>128:raise ValueError('test proxy overflow')
        except Exception as e:self.error=repr(e)
    def sample(self):
        value=self.inner.sample()
        if self.error:value['peer_error']=self.error
        return value
    def __getattr__(self,name):
        if name in ('peak_duty','peak_rpm','permit_ever','wire_history'):
            return getattr(self.inner,name)
        raise AttributeError(name)
    def close(self):
        self.done.set();self.thread.join(1);self.sock.close();self.inner.close()

def one(folder,peer=None,late=False,persistent=False):
    path,consent=profile(READY,folder,True)
    if peer:
        p=json.loads(path.read_text());p['hardware']['command_port']=peer.port;save(path,p)
    os.environ.update(WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(path),WAND_PROFILE_APPROVAL=sha(path),
        WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign',ROS_DOMAIN_ID='196',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    from control_channel import Channel
    from ground_runtime import GroundRig
    rt=bootstrap.runtime();peer=peer or DelayedStopPeer(rt.command_port);peer.delay_ms=0
    channel=Channel(readonly=True);rig=GroundRig(rt,channel,folder,peer_oracle=peer,keep_peer=True)
    seq=0;pressed=released=down=False;press_at=None;stop=None;began=time.monotonic()
    try:
        channel.open('offline-late-stop-only',began)
        while time.monotonic()-began<25:
            now=time.monotonic();channel.begin_tick(now);events=[]
            if not pressed and rig.operator.prepare.phase=='READY':
                down=pressed=True;press_at=now;events=[dict(kind='press')]
            elif pressed and not released and now-press_at>=.04:
                down=False;released=True;events=[dict(kind='release')]
            challenge=channel.challenge(now)
            if challenge:
                seq+=1;data=dict(type='pulse',nonce=challenge['nonce'],sequence=seq,focused=True,visible=True,space=down,ui_gap_ms=10,events=events)
                if channel.receive(channel.owner,data,now):channel.events.extend(events)
            channel.finish_tick(now);rig.step(now)
            if rig.active_since is not None and now-rig.active_since>=2.:
                if late:peer.delay_ms=140;peer.delay_until=float('inf') if persistent else now+.30
                channel.stop('OFFLINE_LATE_STOP' if late else 'OFFLINE_REOPEN_STOP');break
            if channel.stopped:raise ValueError(channel.fault)
            time.sleep(.003)
        assert rig.activation_ever and pressed and released
        stop=rig.close();assert stop['gate_output_zero'],stop
        if persistent:
            # Fresh zero-RPM/pose feedback may genuinely confirm the stop before
            # the independent command-progress timeout. If it cannot, keep the
            # unconfirmed result; neither outcome may resume authority.
            assert stop['state'] in ('PAUSED','STOPPING','STOP_UNCONFIRMED'),stop
            assert not stop['stop_confirmed'] or stop['state']=='PAUSED',stop
            assert not peer.sample()['status']['permitted']
        else:assert stop['stop_confirmed'],stop
        assert not peer.max_fault and not peer.error,(peer.max_fault,peer.error)
        if late:assert peer.delayed>0 and peer.max_delay_s>=.12
        result=dict(passed=True,physical_output=False,actual_ROS_UDP_native_C=True,late_stop_injected=late,
            persistent_invalid_stop_stream=persistent,burst_s=None if persistent else .30 if late else 0.,
            observed_confirmation_unavailable=not stop['stop_confirmed'],no_automatic_motion_resume=True,
            stop_delay_ms=140 if late else 0,delayed_frames=peer.delayed,max_actual_proxy_delay_s=peer.max_delay_s,
            native_fault_bits=peer.max_fault,stop=stop,actual_hardware_trial=False)
        save(folder/'RESULT.json',result);return peer,result
    finally:
        if stop is None:rig.close()

def main():
    root=Path(tempfile.mkdtemp(prefix='late_stop_ros_',dir=OUT));peer=None
    try:
        a=root/'late';a.mkdir();peer,first=one(a,late=True)
        b=root/'reopen_without_C_reset';b.mkdir();peer,second=one(b,peer,late=False)
        c=root/'persistent_invalid_stream';c.mkdir();peer,third=one(c,peer,late=True,persistent=True)
        result=dict(passed=True,physical_output=False,new_native_C=True,late_stop_confirmed=True,reopen_same_C_without_reset=True,
            results=[str(a/'RESULT.json'),str(b/'RESULT.json'),str(c/'RESULT.json')],persistent_delayed_stop_stays_revoked=True,candidate_flashed=False)
        save(root/'RESULT.json',result);print(json.dumps(dict(result,folder=str(root))),flush=True)
    finally:
        if peer:peer.close()
if __name__=='__main__':main()
