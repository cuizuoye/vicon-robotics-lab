"""Offline native-C/Python transport regression; no hardware or ROS init."""
import importlib.util
import json
import os
from pathlib import Path
import socket
import struct
from types import SimpleNamespace as NS

import pytest
from mecanum_vicon_control.transport_protocol import MessageType as M, FrameFlags as F, encode_frame, decode_frame
from mecanum_wand_4ws8_hardware import nodes, startup_clock, kernel_ingress
from mecanum_wand_4ws8_hardware.startup_clock import StartupProgressSocket
from mecanum_wand_4ws8_hardware.kernel_ingress import KernelIngressSocket, SO_TIMESTAMPNS, MAX_DRAIN_PACKETS

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
OUT=Path(os.environ['FEEDBACK_OUTPUT'])
spec=importlib.util.spec_from_file_location('feedback_native_harness',
    ROOT/'docs/wand_demo_large_type_2026-10-26/prepared/tests/test_startup_clock.py')
native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
ORIGINAL_ACTUAL=native.actual_bridge


@pytest.fixture(autouse=True)
def offline_only(monkeypatch):
    import rclpy
    def forbidden(*a,**k):pytest.fail('physical I/O or ROS init attempted')
    monkeypatch.setattr(socket,'socket',forbidden)
    monkeypatch.setattr(rclpy,'init',forbidden)


def test_ancillary_is_exact_and_not_a_source_time():
    assert kernel_ingress.ancillary_fields([(socket.SOL_SOCKET,SO_TIMESTAMPNS,
        struct.pack('=qq',10,123))])==(10000000123,None)
    for items in ([],[(socket.SOL_SOCKET,SO_TIMESTAMPNS,b'x')],
        [(socket.SOL_SOCKET,SO_TIMESTAMPNS,struct.pack('=qq',10,10**9))],
        [(socket.SOL_SOCKET,SO_TIMESTAMPNS,struct.pack('=qq',10,0))]*2):
        with pytest.raises(OSError):kernel_ingress.ancillary_fields(items)


def wire(monkeypatch,now=11.):
    sent=[]
    monkeypatch.setattr(startup_clock,'time',NS(monotonic=lambda:now))
    s=StartupProgressSocket(NS(sendto=lambda data,dest:sent.append(decode_frame(data))),
        '192.168.50.1',lambda:123)
    native.qualify(s.clock)
    return s,sent


@pytest.mark.parametrize('flags',[0,int(F.ACK_REQUIRED)])
@pytest.mark.parametrize('fault',['','NATIVE_FAULT_0x20','RECONNECT_AFTER_ARM_REQUIRES_NEW_MANUAL_SESSION'])
def test_exact_disabled_stop_survives_stale_or_fault_without_clearing_it(monkeypatch,fault,flags):
    s,sent=wire(monkeypatch);s.clock.fault=fault
    before=s.clock.snapshot(11.)
    assert not s.clock.ready(11.)
    data=encode_frame(M.CONTROL_ENABLE,1,123,11000,struct.pack('<BBH',0,0,0),flags)
    s.sendto(data,('192.168.50.1',46000))
    assert len(sent)==1 and sent[0].payload==b'\0'*4
    assert s.clock.snapshot(11.)==before and not s.clock.frozen


@pytest.mark.parametrize('payload,flags,sid,qualified',[
    (b'\0'*4,0,124,True),(b'\0'*4,0,123,False),
    (b'\0'*4,int(F.ENABLED),123,True),(b'\0'*4,int(F.RESPONSE),123,True),
    (struct.pack('<BBH',1,0,0),0,123,True),(struct.pack('<BBH',0,1,0),0,123,True),
    (struct.pack('<BBH',0,0,0x7804),0,123,True),(b'\0'*3,0,123,True),
    (struct.pack('<BBH',0,0,1),0,123,True)])
def test_stop_exception_never_allows_enable_clear_select_or_bad_session(monkeypatch,payload,flags,sid,qualified):
    s,sent=wire(monkeypatch);s.clock.qualified=qualified
    with pytest.raises(OSError):s.sendto(encode_frame(M.CONTROL_ENABLE,1,sid,11000,payload,flags),('192.168.50.1',46000))
    assert not sent


class InfiniteRaw:
    def __init__(self):self.calls=0
    def setsockopt(self,*a):pass
    def recvmsg(self,*a):
        self.calls+=1
        return b'x',[(socket.SOL_SOCKET,SO_TIMESTAMPNS,struct.pack('=qq',100,0))],0,('bad',1)


def test_raw_packet_flood_including_wrong_peers_is_bounded(monkeypatch):
    monkeypatch.setattr(kernel_ingress,'time',NS(monotonic=lambda:1.,monotonic_ns=lambda:10**9,time_ns=lambda:100*10**9))
    raw=InfiniteRaw();s=KernelIngressSocket(raw)
    with pytest.raises(RuntimeError):s.recvfrom(512)
    s.begin_drain()
    for _ in range(MAX_DRAIN_PACKETS):s.recvfrom(512)
    with pytest.raises(BlockingIOError):s.recvfrom(512)
    s.end_drain()
    assert raw.calls==32 and s.budget_exhaustions==1


def test_time_budget_leaves_safety_loop_runnable(monkeypatch):
    clock=NS(now=1.)
    monkeypatch.setattr(kernel_ingress,'time',NS(monotonic=lambda:clock.now))
    raw=InfiniteRaw();s=KernelIngressSocket(raw);s.begin_drain();clock.now+=.0031
    with pytest.raises(BlockingIOError):s.recvfrom(512)
    assert raw.calls==0


@pytest.mark.parametrize('flags',[socket.MSG_TRUNC,socket.MSG_CTRUNC])
def test_truncated_data_latches_metadata_failure(monkeypatch,flags):
    raw=InfiniteRaw();raw.recvmsg=lambda *a:(b'x',[],flags,('bad',1))
    s=KernelIngressSocket(raw);s.begin_drain()
    with pytest.raises(BlockingIOError):s.recvfrom(512)
    assert s.error.startswith('truncated')


def actual(monkeypatch,ack_delay=.01):
    from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig
    monkeypatch.setattr(native,'config',lambda:TrackingSafetyConfig(**json.loads((OUT/
        'runtime/src/mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json').read_text())['safety']))
    def recvmsg(fake,size,ancsize):
        data,peer=fake.recvfrom(size)
        # Synthetic kernel metadata; source/packet content is from native C.
        return data,[(socket.SOL_SOCKET,SO_TIMESTAMPNS,struct.pack('=qq',100,0))],0,peer
    monkeypatch.setattr(native.NativeFakeSocket,'recvmsg',recvmsg,raising=False)
    monkeypatch.setattr(KernelIngressSocket,'peek',lambda self:dict(offline_fake=True))
    monkeypatch.setattr(kernel_ingress,'time',NS(monotonic=lambda:0.,monotonic_ns=lambda:0,time_ns=lambda:100*10**9))
    return ORIGINAL_ACTUAL(monkeypatch,ack_delay)


def armed(monkeypatch):
    h=actual(monkeypatch);h.run(1.2)
    assert h.status()['ready'] and not h.b.user_arm_seen,(h.status(),h.logs)
    for _ in range(50):h.signed('ARMED','regression-epoch',True);h.step()
    for _ in range(30):h.signed('ACTIVE','regression-epoch',True);h.step()
    assert not h.b.local_stop_latched and h.b.last_motion_status['permitted']
    return h


def test_actual_native_warmup_prepare_motion_and_disable(monkeypatch):
    from mecanum_wand_4ws8_control.core import Velocity
    h=armed(monkeypatch)
    for _ in range(20):h.signed('ACTIVE','regression-epoch',True,Velocity(.08,0.,0.));h.step()
    assert h.b.last_mcu_fault_bits==0
    assert any(f.flags&F.ENABLED and any(f.payload[:6]) for _,f,_ in h.socket.sent if f.message_type==M.CMD_VEL)
    h.signed();h.run(.2)
    assert not h.b.safety_enabled and not h.b.motion_gate_requested


def test_timer_drains_available_fresh_native_before_timeout_decision(monkeypatch):
    h=armed(monkeypatch)
    # Hold ROS callbacks while C keeps sampling. At next timer, old host cache
    # is >200ms but genuinely advancing same-session packets are already queued.
    h.clock.now+=.19;h.socket.advance();h.clock.now+=.011
    assert max(h.clock.now-v for v in h.b.sock.clock.accepted_at.values())>.20
    before=h.b.sock.clock.local_baseline
    h.signed('ACTIVE','regression-epoch',True)
    assert not h.b.policy.fault and h.b.feedback_is_fresh(h.clock.now)
    assert h.b.sock.clock.local_baseline==before


def test_real_214ms_feedback_gap_still_stops_and_sends_exact_disable(monkeypatch):
    h=armed(monkeypatch);h.socket.allow_telemetry=False;h.socket.queue.clear()
    h.clock.now+=.214
    h.signed('ACTIVE','regression-epoch',True)
    assert h.b.policy.fault=='MCU_FEEDBACK_TIMEOUT'
    assert not h.b.safety_enabled and not h.b.motion_gate_requested
    # Flush queued disable via real transport sender while stale, no re-HELLO.
    h.b.tick()
    controls=[f for _,f,_ in h.socket.sent if f.message_type==M.CONTROL_ENABLE]
    assert controls[-1].payload==b'\0'*4
    assert not any('CONTROL_DISABLE发送失败' in x for x in h.logs)
    hellos=len([f for _,f,_ in h.socket.sent if f.message_type==M.HELLO])
    h.socket.allow_telemetry=True;h.run(.3)
    assert not h.b.safety_enabled and h.b.policy.fault
    assert len([f for _,f,_ in h.socket.sent if f.message_type==M.HELLO])==hellos


def test_queued_replay_cannot_prevent_real_timeout(monkeypatch):
    h=armed(monkeypatch)
    raw=h.socket.oracle.packet(M.WHEEL_TELEMETRY)
    h.socket.allow_telemetry=False;h.socket.queue.clear();h.clock.now+=.214
    h.socket.queue.extend([(h.clock.now,raw)]*64)
    h.signed('ACTIVE','regression-epoch',True)
    assert h.b.policy.fault=='MCU_FEEDBACK_TIMEOUT' and not h.b.safety_enabled
    assert h.b.ingress.budget_exhaustions>0


@pytest.mark.parametrize('name',[
    'test_actual_constructed_bridge_c_oracle_warms_up_without_false_timeout_or_enable',
    'test_actual_unarmed_reconnect_waits_old_lease_and_rebuilds_selection_and_proof',
    'test_actual_prepare_freezes_clock_and_reconnect_never_auto_restores_arm',
    'test_actual_native_fault_is_not_cleared_hidden_or_reconnected',
    'test_actual_early_prepare_cannot_bypass_clock_proof',
    'test_actual_wrong_session_caches_cannot_report_feedback_fresh',
    'test_actual_native_crc_error_blocks_without_reconnect_or_fault_clear',
    'test_actual_prepared_active_flow_keeps_clock_valid_when_native_permit_is_granted'])
def test_reused_native_c_regressions_against_new_bridge(monkeypatch,name):
    monkeypatch.setattr(native,'actual_bridge',actual)
    getattr(native,name)(monkeypatch)


@pytest.mark.parametrize('delay',[.06,.20])
def test_reused_slow_ack_is_rejected(monkeypatch,delay):
    monkeypatch.setattr(native,'actual_bridge',actual)
    native.test_actual_slow_ack_only_sends_bounded_hellos_never_zero_enable_or_clear(monkeypatch,delay)


def test_diagnostic_wire_allowlist_is_stricter_than_motion_runtime(monkeypatch):
    import importlib.util
    spec=importlib.util.spec_from_file_location('feedback_zero_diagnostic',Path(__file__).with_name('diagnose.py'))
    d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
    d.offline_check()
    sent=[];s=d.PermanentZeroSocket(NS(sendto=lambda data,dest:sent.append(data)))
    s.sendto(encode_frame(M.HELLO,1,123,10),d.ESP)
    with pytest.raises(ValueError):s.sendto(encode_frame(M.HELLO,2,123,20),d.ESP)
    with pytest.raises(ValueError):s.sendto(encode_frame(M.CONTROL_ENABLE,2,123,20,b'\0'*4),d.ESP)
    s.stop();first=s.stopped_at;s.stop()
    with pytest.raises(OSError):s.sendto(encode_frame(M.HELLO,3,123,30),d.ESP)
    assert len(sent)==1 and s.stopped_at==first and d.MAX_WINDOW_S==60.


def test_only_reviewed_transport_files_changed_all_motion_ui_and_parents_preserved():
    m=json.loads((OUT/'build_manifest.json').read_text())
    for p,digest in m['protected_input_hashes'].items():
        import hashlib
        assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest,p
    changes=json.loads((OUT/'PRIVATE_CHANGE_INVENTORY.json').read_text())
    assert {Path(n).name for n in changes}=={'nodes.py','startup_clock.py','kernel_ingress.py'}
    assert m['feedback_protection_ms']==200 and m['firmware_changed'] is False
