"""Offline real Python bridge + native 4WS8 C oracle, no ROS/socket startup."""
import collections
import ctypes
import importlib.util
import itertools
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from support import ROOT, GENERATED, config
from mecanum_vicon_control.transport_protocol import (
    MessageType as M, FrameFlags as F, encode_frame, decode_frame,
    unpack_wheel_telemetry, pack_cmd_vel)
from mecanum_wand_4ws8_hardware.startup_clock import HandshakeSampleClock, StartupProgressSocket
from mecanum_wand_4ws8_hardware.commissioning import McuSampleClock, AuthoritySigner


@pytest.fixture(autouse=True)
def offline_only(monkeypatch):
    import socket, rclpy
    def forbidden(*a, **k): pytest.fail('offline clock tests attempted real socket/ROS init')
    monkeypatch.setattr(socket, 'socket', forbidden)
    monkeypatch.setattr(rclpy, 'init', forbidden)


def ack(clock, *, at=10., delay=.02, tick=1000, sid=123):
    clock.begin(sid, 1, int(at*1000), at)
    frame = decode_frame(encode_frame(M.HELLO_ACK, 1, sid, tick, flags=F.RESPONSE))
    return clock.acknowledge(frame, at+delay)


def qualify(clock, at=10., tick=1000):
    assert ack(clock, at=at, tick=tick)
    clock.native_seen = {M.WHEEL_TELEMETRY, M.LINK_STATUS}
    clock.stationary_ready = True
    for offset in range(100, 701, 100):
        for kind in (M.WHEEL_TELEMETRY, M.LINK_STATUS):
            assert clock.observe(kind, (tick+offset)&0xffffffff, at+offset/1000.+.02) is not None
    assert clock.ready(at+.72)


def test_delayed_first_telemetry_does_not_define_clock_offset():
    c=HandshakeSampleClock();assert ack(c)
    # Actual failing trace pattern: MCU advances 100ms, receipt advances ~20ms.
    c.native_seen={M.WHEEL_TELEMETRY,M.LINK_STATUS};c.stationary_ready=True
    assert c.observe(M.WHEEL_TELEMETRY,1100,10.18)==pytest.approx(10.1)
    assert c.observe(M.WHEEL_TELEMETRY,1200,10.20)==pytest.approx(10.2)
    assert not c.ready(10.20)  # Two frames never make the system ready.


@pytest.mark.parametrize('delay',[.041,.06,.20,.5,-.001,float('nan')])
def test_unbounded_or_invalid_hello_round_trip_never_qualifies(delay):
    c=HandshakeSampleClock();assert not ack(c,delay=delay)
    assert not c.qualified and not c.ready(10.5)
    assert c.observe(M.WHEEL_TELEMETRY,1100,10.3) is None


@pytest.mark.parametrize('field,value',[('session_id',124),('sequence',2),('flags',0),('payload',b'x')])
def test_wrong_ack_cannot_anchor_or_connect(field,value):
    c=HandshakeSampleClock();c.begin(123,1,10000,10.)
    fields=dict(message_type=M.HELLO_ACK,sequence=1,session_id=123,sender_time_ms=1000,flags=F.RESPONSE,payload=b'')
    fields[field]=value
    assert not c.acknowledge(decode_frame(encode_frame(**fields)),10.02)
    assert c.local_baseline is None


def test_proof_requires_both_advancing_native_streams_span_and_measured_standstill():
    c=HandshakeSampleClock();assert ack(c)
    c.native_seen={M.WHEEL_TELEMETRY,M.LINK_STATUS}
    for i in range(1,8):c.observe(M.WHEEL_TELEMETRY,1000+i*100,10.+i*.1+.01)
    assert not c.ready(10.71)
    for i in range(8,16):
        for kind in (M.WHEEL_TELEMETRY,M.LINK_STATUS):c.observe(kind,1000+i*100,10.+i*.1+.01)
    assert not c.ready(11.51)
    c.stationary_ready=True
    assert c.ready(11.51) and not c.ready(12.)


def test_arming_freezes_proof_and_never_allows_a_new_clock_anchor():
    c=HandshakeSampleClock();qualify(c);c.freeze(10.72)
    c.stationary_ready=False  # A permitted moving car is not a stationary car.
    assert c.ready(10.72)
    with pytest.raises(OSError):c.begin(124,1,10720,10.72)
    baseline=c.local_baseline
    assert c.observe(M.WHEEL_TELEMETRY,1900,10.74) is None  # Genuine future jump.
    assert c.local_baseline==baseline
    assert c.observe(M.WHEEL_TELEMETRY,1700,10.74) is None  # Replay.
    assert not c.ready(11.)


def test_wraparound_and_session_restart_do_not_reuse_old_ready_evidence():
    c=HandshakeSampleClock();qualify(c,tick=0xffffff00)
    c.begin(124,1,10800,10.8)
    assert not c.ready(10.8) and c.accepted_at=={} and c.by_kind=={}
    assert c.latest_tick is None and not c.calibrated


def test_native_fault_survives_handshake_attempt_and_never_becomes_ready():
    c=HandshakeSampleClock();qualify(c);c.fault='NATIVE_FAULT_0x40000'
    with pytest.raises(OSError):c.begin(124,1,10800,10.8)
    assert c.fault=='NATIVE_FAULT_0x40000' and not c.ready(10.72)


def test_real_failed_telemetry_schedule_with_synthetic_bounded_ack_anchor():
    path=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle/ready/oneclick_session_53vp_i3d/mcu_receive.jsonl'
    records=[json.loads(line) for line in itertools.islice(path.open(),6)]
    old=McuSampleClock();new=HandshakeSampleClock()
    # The original capture did not save ACK payloads. This is explicitly a
    # synthetic qualified ACK plus REAL telemetry scheduling, not a claim that
    # the captured hardware handshake met the new RTT cap.
    first=records[0];start=first['received_steady']-.30
    source_ms=int(start*1000);start=source_ms/1000.
    assert ack(new,at=start,delay=.02,tick=first['tick']-200,sid=first['session'])
    accepted=[];old_accepted=[]
    for r in records:
        kind=M[r['kind']];frame=decode_frame(encode_frame(kind,r['sequence'],r['session'],r['tick'],bytes.fromhex(r['payload_hex'])))
        new.note_native(frame)
        accepted.append(new.observe(kind,r['tick'],r['received_steady']) is not None)
        old_accepted.append(old.observe(kind,r['tick'],r['received_steady']) is not None)
    assert old_accepted==[True,True,False,False,False,False]
    assert all(accepted) and not new.ready(records[-1]['received_steady'])


class Oracle:
    def __init__(self):
        self.lib=ctypes.CDLL(str(ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/native/libstage0_oracle.so'))
        self.lib.oracle_request.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint32,ctypes.c_void_p]
        self.lib.oracle_request.restype=ctypes.c_size_t
        self.lib.oracle_packet.argtypes=[ctypes.c_uint,ctypes.c_void_p];self.lib.oracle_packet.restype=ctypes.c_size_t
        self.lib.oracle_tick.argtypes=[ctypes.c_uint32];self.lib.oracle_init()
        self.out=ctypes.create_string_buffer(512)
    def tick(self,ms):self.lib.oracle_tick(ms)
    def request(self,raw,ms):
        buf=ctypes.create_string_buffer(raw)
        n=self.lib.oracle_request(buf,len(raw),ms,self.out)
        return self.out.raw[:n]
    def packet(self,kind):
        n=self.lib.oracle_packet(kind,self.out);return self.out.raw[:n]


class NativeFakeSocket:
    def __init__(self,clock,ack_delay=.01,first_telemetry_delay=.09):
        self.clock=clock;self.oracle=Oracle();self.queue=[];self.sent=[]
        self.ack_delay=ack_delay;self.first_telemetry_delay=first_telemetry_delay
        self.samples=0;self.next_sample=100;self.allow_telemetry=True
    def setsockopt(self,*a):pass
    def bind(self,*a):pass
    def setblocking(self,*a):pass
    def close(self):pass
    def ms(self):return round((self.clock.now-10.)*1000)
    def sendto(self,raw,destination):
        frame=decode_frame(raw);self.sent.append((self.clock.now,frame,raw))
        response=self.oracle.request(raw,self.ms())
        if response:self.queue.append((self.clock.now+self.ack_delay,response))
        return len(raw)
    def advance(self):
        ms=self.ms();self.oracle.tick(ms)
        while ms>=self.next_sample:
            self.samples+=1
            if self.allow_telemetry:
                for kind in (M.WHEEL_TELEMETRY,M.LINK_STATUS):
                    raw=self.oracle.packet(kind)
                    if raw:
                        delay=self.first_telemetry_delay if self.samples==1 else .01
                        self.queue.append((self.clock.now+delay,raw))
            self.next_sample+=100
    def recvfrom(self,size):
        self.queue.sort(key=lambda pair:pair[0])
        if self.queue and self.queue[0][0]<=self.clock.now:
            _,raw=self.queue.pop(0);return raw,('192.168.50.1',46000)
        raise BlockingIOError


def actual_bridge(monkeypatch,ack_delay=.01):
    import socket
    from rclpy.node import Node
    from mecanum_wand_4ws8_hardware import nodes,startup_clock,startup_bridge,continuous_bridge
    from mecanum_vicon_control import cmd_vel_udp_bridge as legacy
    clock=NS(now=10.);fake=NativeFakeSocket(clock,ack_delay);published=collections.defaultdict(list);logs=[]
    for module in (nodes,startup_clock,startup_bridge,continuous_bridge,legacy):
        monkeypatch.setattr(module,'time',NS(monotonic=lambda:clock.now))
    overrides=dict(esp_ip='192.168.50.1',network_interface='fake0',bounded_motion_mode=True,
        required_motion_tag='4WS8',wire_timeout_ms=600,max_vx_mps=.70,max_vy_mps=.70,
        max_wz_radps=.10,command_timeout_s=.10,link_receive_timeout_s=.20)
    monkeypatch.setattr(Node,'__init__',lambda self,*a,**k:setattr(self,'_offline_parameters',{}))
    monkeypatch.setattr(Node,'declare_parameter',lambda self,k,v:self._offline_parameters.setdefault(k,v))
    monkeypatch.setattr(Node,'get_parameter',lambda self,k:NS(value=overrides.get(k,self._offline_parameters[k])))
    monkeypatch.setattr(Node,'create_timer',lambda *a,**k:None)
    monkeypatch.setattr(Node,'create_subscription',lambda *a,**k:None)
    monkeypatch.setattr(Node,'create_publisher',lambda self,kind,topic,*a,**k:NS(publish=lambda msg:published[topic].append(msg)))
    monkeypatch.setattr(Node,'get_logger',lambda self:NS(info=logs.append,warn=logs.append,error=logs.append))
    monkeypatch.setattr(Node,'get_clock',lambda self:NS(now=lambda:NS(nanoseconds=round((100.+clock.now)*1e9))))
    runtime=NS(mode='hardware',telemetry_port=46001,command_port=46000,config=config(),
        ip='192.168.50.1',interface='fake0',motion_tag='4WS8',guard=lambda node:None)
    monkeypatch.setattr(nodes,'runtime_from_environment',lambda:runtime)
    monkeypatch.setattr(nodes,'open',lambda *a,**k:NS(close=lambda:None),raising=False)
    monkeypatch.setattr(nodes.fcntl,'flock',lambda *a:None)
    monkeypatch.setattr(socket,'socket',lambda *a,**k:fake)
    monkeypatch.setenv('WAND_4WS8_CHANNEL_KEY','12'*32)
    b=nodes.HardwareBridge()
    def step(seconds=.02):
        clock.now+=seconds;fake.advance();b.tick()
    def run(seconds):
        for _ in range(round(seconds/.02)):step()
    def status():return json.loads(published['transport_status'][-1].data)
    signer=AuthoritySigner('12'*32,4)
    def signed(state='STANDBY',epoch='',permit=False,command=None):
        from mecanum_wand_4ws8_control.safety_core import ZERO
        b.on_signed(NS(data=signer.encode(command or ZERO,clock.now,100.+clock.now,state=state,epoch=epoch,permit=permit)))
    signed()  # Establish the unchanged signed disabled owner before any arm.
    return NS(b=b,clock=clock,socket=fake,published=published,logs=logs,step=step,run=run,status=status,signed=signed)


def test_actual_constructed_bridge_c_oracle_warms_up_without_false_timeout_or_enable(monkeypatch):
    h=actual_bridge(monkeypatch);h.run(1.2)
    s=h.status()
    assert s['fresh'] and s['ready'] and not s['local_stop'] and not s['fault']
    assert h.b.last_motion_status['selected'] and h.b.last_motion_status['stage']==4
    assert s['startup_clock']['ready'] and h.b.last_mcu_fault_bits==0
    assert not h.b.user_arm_seen and not h.b.safety_enabled
    assert not any(frame.flags & F.ENABLED for _,frame,_ in h.socket.sent)
    controls=[frame.payload for _,frame,_ in h.socket.sent if frame.message_type==M.CONTROL_ENABLE]
    assert controls==[b'\x00\x00\x04\x78']
    assert h.status()['startup_clock']['qualified']


@pytest.mark.parametrize('delay',[.06,.20])
def test_actual_slow_ack_only_sends_bounded_hellos_never_zero_enable_or_clear(monkeypatch,delay):
    h=actual_bridge(monkeypatch,delay);h.run(1.5)
    assert len(h.socket.sent)==3 and all(f.message_type==M.HELLO for _,f,_ in h.socket.sent)
    assert h.b.local_stop_latched and not h.status()['ready']
    assert 'STARTUP_HELLO_RTT_OR_LINK_NOT_QUALIFIED' in h.status()['startup_clock']['fault']


def test_actual_unarmed_reconnect_waits_old_lease_and_rebuilds_selection_and_proof(monkeypatch):
    h=actual_bridge(monkeypatch);h.run(1.2);assert h.status()['ready']
    first=h.b.session.session_id
    h.socket.allow_telemetry=False;h.run(.45)
    assert not h.b.session.connected and not h.status()['ready']
    assert h.b.stage0_selection_at is None and h.b.prepared_session is None
    last_old=max(t for t,f,_ in h.socket.sent if f.session_id==first and f.message_type==M.CMD_VEL)
    h.socket.allow_telemetry=True;h.run(1.7)
    hellos=[(t,f) for t,f,_ in h.socket.sent if f.message_type==M.HELLO]
    assert len(hellos)==2 and hellos[1][0]-last_old>=.65-1e-8
    assert h.b.session.session_id!=first and h.status()['ready'] and not h.b.local_stop_latched
    assert h.b.stage0_selection_session==h.b.session.session_id
    assert not any(f.flags & F.ENABLED for _,f,_ in h.socket.sent)


def test_actual_prepare_freezes_clock_and_reconnect_never_auto_restores_arm(monkeypatch):
    h=actual_bridge(monkeypatch);h.run(1.2);assert h.status()['ready']
    h.signed('ARMED','human-session',True);h.run(.1)
    assert h.b.user_arm_seen and h.b.sock.clock.frozen
    h.socket.allow_telemetry=False;h.run(1.2)
    assert len([f for _,f,_ in h.socket.sent if f.message_type==M.HELLO])==1
    assert h.b.local_stop_latched and not h.status()['ready']
    assert not any(f.flags & F.ENABLED for _,f,_ in h.socket.sent)


def test_actual_native_fault_is_not_cleared_hidden_or_reconnected(monkeypatch):
    h=actual_bridge(monkeypatch);h.run(1.2)
    # Native C source-age violation, injected only into the local oracle.
    from mecanum_wand_4ws8_hardware.stage0_session import Stage0Session
    sid=h.b.session.session_id
    bad=encode_frame(M.CMD_VEL,h.b.session.sequence+100,sid,int(h.clock.now*1000)+80,pack_cmd_vel(0,0,0,600))
    h.socket.oracle.request(bad,h.socket.ms());h.run(.4)
    assert h.b.local_stop_latched and not h.status()['ready']
    assert h.b.sock.clock.fault.startswith('NATIVE_FAULT_0x40000')
    h.run(1.)
    assert len([f for _,f,_ in h.socket.sent if f.message_type==M.HELLO])==1
    assert all(not (f.message_type==M.CONTROL_ENABLE and f.payload[1]) for _,f,_ in h.socket.sent)


def test_new_ui_requires_explicit_clock_ready_and_explains_startup_not_source_conflict():
    from support import model
    from mecanum_wand_4ws8_control.novice_ui import prepare_connection_ok,backend_hint
    m=model();m.transport_at=10.;m.status['input_health']='FEEDBACK_TIMEOUT'
    m.transport_status=dict(fresh=True,local_stop=False,fault='',safety_enabled=False,motion_gate_requested=False)
    assert not prepare_connection_ok(m,10.)
    m.transport_status['startup_clock']=dict(ready=False,phase='WAIT_CLOCK_WINDOW')
    assert not prepare_connection_ok(m,10.) and '建基准' in backend_hint(m,10.)
    m.transport_status['startup_clock']=dict(ready=True,phase='READY')
    assert prepare_connection_ok(m,10.)


def test_cumulative_future_drift_cannot_be_hidden_by_repeated_small_rebases():
    c=HandshakeSampleClock();qualify(c);c.freeze(10.72)
    anchor=c.local_baseline
    results=[c.observe(M.WHEEL_TELEMETRY,1700+i*20,10.72+i*.01) for i in range(1,8)]
    assert any(x is None for x in results) and c.local_baseline==anchor


def test_backlog_and_replay_cannot_refresh_stale_acceptance_caches():
    c=HandshakeSampleClock();qualify(c);before=dict(c.accepted_at);anchor=c.local_baseline
    assert c.observe(M.WHEEL_TELEMETRY,1800,11.10) is None
    assert c.observe(M.WHEEL_TELEMETRY,1800,11.12) is None
    assert c.accepted_at==before and c.local_baseline==anchor and not c.ready(11.12)


def test_a_hello_delayed_inside_the_pc_is_rejected_before_socket_submission():
    c=HandshakeSampleClock()
    with pytest.raises(OSError,match='already delayed'):c.begin(123,1,10000,10.020)
    assert not c.qualified and c.local_baseline is None


def test_actual_early_prepare_cannot_bypass_clock_proof(monkeypatch):
    h=actual_bridge(monkeypatch)
    h.signed('ARMED','too-early',True);h.run(.2)
    assert h.b.local_stop_latched and not h.status()['ready']
    assert not h.socket.sent  # No HELLO, control, enable or clear after bad arm intent.


def test_actual_wrong_session_caches_cannot_report_feedback_fresh(monkeypatch):
    h=actual_bridge(monkeypatch);h.run(1.2);assert h.status()['ready']
    h.b.session.session_id=(h.b.session.session_id+1)&0xffffffff
    assert not h.b.feedback_is_fresh(h.clock.now)


def test_actual_native_crc_error_blocks_without_reconnect_or_fault_clear(monkeypatch):
    h=actual_bridge(monkeypatch);h.run(1.2)
    raw=h.socket.oracle.packet(M.LINK_STATUS)
    h.socket.queue.append((h.clock.now,raw[:-1]+bytes([raw[-1]^1])))
    h.run(.4)
    assert h.b.local_stop_latched and h.b.sock.clock.fault=='NATIVE_DECODE_INVALID'
    assert not h.status()['ready']
    assert len([f for _,f,_ in h.socket.sent if f.message_type==M.HELLO])==1


@pytest.mark.parametrize('flags',[int(F.RESPONSE),int(F.ACK_REQUIRED),16,int(F.ENABLED),int(F.FAULT)])
def test_actual_native_invalid_header_flags_never_count_as_a_healthy_sample(monkeypatch,flags):
    h=actual_bridge(monkeypatch);h.run(1.2)
    f=decode_frame(h.socket.oracle.packet(M.WHEEL_TELEMETRY))
    raw=encode_frame(f.message_type,f.sequence,f.session_id,f.sender_time_ms,f.payload,flags)
    h.socket.queue.append((h.clock.now,raw));h.run(.2)
    assert h.b.local_stop_latched and h.b.sock.clock.fault=='NATIVE_DECODE_INVALID'
    assert not h.status()['ready']


def test_actual_prepared_active_flow_keeps_clock_valid_when_native_permit_is_granted(monkeypatch):
    from mecanum_wand_4ws8_control.core import Velocity
    h=actual_bridge(monkeypatch);h.run(1.2)
    for _ in range(50):h.signed('ARMED','manual-active',True);h.step()
    assert h.status()['ready'] and h.b.prepared_epoch=='manual-active'
    for _ in range(30):h.signed('ACTIVE','manual-active',True);h.step()
    assert h.b.last_motion_status['permitted'] and h.status()['fresh']
    assert h.b.sock.clock.frozen and not h.b.local_stop_latched and not h.b.sock.clock.fault
    for _ in range(30):h.signed('ACTIVE','manual-active',True,Velocity(.08,0.,0.));h.step()
    assert h.b.last_mcu_fault_bits==0 and h.b.last_motion_status['permitted']
    assert any(f.message_type==M.CMD_VEL and f.flags & F.ENABLED and any(f.payload[:6])
               for _,f,_ in h.socket.sent)
    h.signed();h.run(.2)
    assert not h.b.safety_enabled and not h.b.motion_gate_requested


def test_only_startup_transport_and_hint_files_change_not_motion_or_safety_cores():
    import ast
    old=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle/ready/runtime/src'
    hardware='mecanum_wand_4ws8_hardware';control='mecanum_wand_4ws8_control'
    allowed={f'{hardware}/{hardware}/nodes.py',f'{control}/{control}/novice_ui.py'}
    for path in old.rglob('*'):
        if not path.is_file() or any(p=='__pycache__' or p.endswith('.egg-info') for p in path.parts):continue
        relative=str(path.relative_to(old))
        if relative not in allowed:assert (GENERATED/relative).read_bytes()==path.read_bytes(),relative
    def methods(path):
        module=ast.parse(path.read_text())
        cls=next(n for n in module.body if isinstance(n,ast.ClassDef) and n.name=='HardwareBridge')
        return {n.name:n for n in cls.body if isinstance(n,ast.FunctionDef)}
    a=methods(old/hardware/hardware/'nodes.py');b=methods(GENERATED/hardware/hardware/'nodes.py')
    for name in ('on_signed','on_cmd','on_enable','on_control_enable','on_clear_faults','on_estop',
                 'queue_control_action','stop_wire','baseline_is_current','baseline_is_prepared',
                 'reset_preparation','latch_local_stop'):
        assert ast.dump(a[name])==ast.dump(b[name]),name
    # All original safety decisions remain identical; only the last diagnostic
    # publication gained startup-clock fields.
    assert [ast.dump(n) for n in a['safety_tick'].body[:-1]]==[
        ast.dump(n) for n in b['_legacy_safety_tick'].body[:-1]]
