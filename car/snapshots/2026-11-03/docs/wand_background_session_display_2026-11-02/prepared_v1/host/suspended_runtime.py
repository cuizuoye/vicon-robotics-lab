"""Browser foreground proof -> original five-second hold-to-run controller."""
import fcntl
import json
from pathlib import Path
import socket
import time
from types import SimpleNamespace
import bootstrap
from mecanum_wand_4wsc_hardware.suspended_core import SuspendedController

class SuspendedRig:
    def __init__(self,rt,channel,session):
        self.rt,self.channel,self.session=rt,channel,session;self.physical=not rt.software_only
        self.bridge_constructed=False;self.ros_initialized=False;self.ros_initialized_ever=False;self.activation_ever=False;self.active_since=None
        self.core=SuspendedController(stage=rt.config.stage);self.prepared=False;self.generation=-1
        self.operator=SimpleNamespace(model=SimpleNamespace(status={},gate={},held=False,key_down=False,click_pending=False,blocked=False,notice='',events=[]))
        self.peer=None
        if rt.software_only:
            from native_peer import Peer
            self.peer=Peer(rt.command_port)
        self.owner=open('/tmp/wand_mc_link_'+str(rt.telemetry_port)+'.lock','a');fcntl.flock(self.owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,rt.interface.encode()+b'\0')
        self.sock.bind(('0.0.0.0',rt.telemetry_port));self.sock.setblocking(False)
        self.log=(session/'SUSPENDED.jsonl').open('x')

    def send(self,raw):self.sock.sendto(raw,(self.rt.ip,self.rt.command_port))

    def step(self,now):
        proof=self.channel.evidence(now)
        if self.channel.stopped and self.core.state not in ('STOPPING','COMPLETE','FAULT'):
            manual=self.channel.fault in ('CLIENT_ESC','CLIENT_STOP_BUTTON','OPERATOR_SECOND_SPACE')
            self.core.stop(now,self.channel.fault,fault=not manual,estop=self.channel.fault=='OPERATOR_ESTOP')
        if proof and proof['generation']!=self.generation:
            self.generation=proof['generation']
            while self.channel.events:
                e=self.channel.events.popleft()
                if e['kind'] not in ('press','release'):raise ValueError('key events only')
                self.core.operator(now,True,e['kind']=='press')
            self.core.operator(now,True,proof['space'])
            if not self.prepared and self.core.unlocked and not self.core.down:
                self.prepared=self.core.prepare(now,self.rt.profile.get('suspended_direction','前进'))
        for _ in range(32):
            try:raw,addr=self.sock.recvfrom(512)
            except BlockingIOError:break
            if addr[0]==self.rt.ip:self.core.receive(raw,time.monotonic())
        for raw in self.core.step(time.monotonic()):self.send(raw)
        if self.core.state in ('WAIT_ENABLE','ACTIVE'):
            if self.active_since is None:self.active_since=now
            self.activation_ever=True
        if self.activation_ever and self.core.state in ('STOPPING','COMPLETE','FAULT') and not self.channel.stopped:
            if not self.core.reason.startswith('5秒单次测试结束'):self.channel.stop(self.core.reason)
        self.operator.model.status=dict(state=self.core.state,reason=self.core.reason)
        self.operator.model.held=self.core.down
        self.log.write(json.dumps(dict(at=now,state=self.core.state,status=self.core.status,reason=self.core.reason),ensure_ascii=False)+'\n');self.log.flush()

    def snapshot(self,now):
        return dict(state=self.core.state,reason=self.core.reason,stage=self.rt.config.stage,purpose='suspended',
            limits=dict(speed=self.rt.config.max_speed_mps,accel=self.rt.config.max_accel_mps2,yaw=self.rt.config.max_yaw_rate_radps,yaw_accel=self.rt.config.max_yaw_accel_radps2),
            instruction='单次悬空：轻按松开空格解锁→自动准备（不转轮）→READY后按住空格；松键停，最长5秒；本窗口不重跑',
            prepare_phase=self.core.state,car=None,wand=None,goal=None,target_policy='DIRECT_SIGN_1TO1',trace=[],
            gate=dict(output_zero=bool(self.core.status and not self.core.status['permitted']),fault=self.core.first_fault_reason),
            session_requested=self.core.down,trial_ended=self.core.state in ('COMPLETE','FAULT'),
            remaining_s=5. if self.core.action_at is None else max(0.,5.-(now-self.core.action_at)))

    def close(self):
        now=time.monotonic()
        if self.core.state not in ('COMPLETE','FAULT'):self.core.stop(now,self.channel.fault or '结束本轮')
        try:
            while time.monotonic()-now<1.5 and self.core.state=='STOPPING':
                for raw in self.core.step(time.monotonic()):self.send(raw)
                for _ in range(32):
                    try:raw,addr=self.sock.recvfrom(512)
                    except BlockingIOError:break
                    if addr[0]==self.rt.ip:self.core.receive(raw,time.monotonic())
                time.sleep(.003)
            confirmed=self.core.stationary(time.monotonic()) and self.core.state in ('COMPLETE','FAULT')
            result=dict(stop_confirmed=bool(confirmed),gate_output_zero=bool(confirmed),state=self.core.state,reason=self.core.reason,
                physical_stop_distance_verified=False,actual_hardware_trial=self.physical)
            (self.session/'STOP.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');return result
        finally:
            self.sock.close();self.owner.close();self.log.close()
            if self.peer is not None:self.peer.close()
