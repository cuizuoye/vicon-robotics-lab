"""Production-wire regressions: recorded device bytes + native C; NO hardware."""
import hashlib
import importlib.util
import json
from pathlib import Path
import re

import pytest

from test_stage0 import Harness, Oracle, SuspendedController, GENERATED
from mecanum_vicon_control.transport_protocol import (
    decode_frame, encode_frame, MessageType, FrameFlags, ProtocolError,
)
from mecanum_wand_4ws7_hardware.stage0_contract import decode_status_frame

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
SOURCE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws7_stage0'
TRACE=ROOT/'docs/wand_4ws7_stage0_2026-10-23/ready_v2/session_vrxy6u1e/bench/raw_link.jsonl'
BEFORE=ROOT/'docs/wand_4ws7_stage0_2026-10-23/host_fix_20261023/source_before'


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    import socket
    def forbidden(*args,**kwargs):raise AssertionError('offline wire tests must not open a socket')
    monkeypatch.setattr(socket,'socket',forbidden)


def packet(h,kind=MessageType.WHEEL_TELEMETRY,flags=None,session=None,payload=None):
    n=h.mcu.dll.oracle_packet(kind,h.mcu.out)
    f=decode_frame(h.mcu.out.raw[:n])
    return encode_frame(kind,f.sequence,f.session_id if session is None else session,
        f.sender_time_ms,f.payload if payload is None else payload,flags=f.flags if flags is None else flags)


def test_oracle_publish_flags_and_cadence_match_preserved_production_main():
    production=(ROOT/'stm32/mecanum_f407/Src/four_link_bench_main.c').read_text()
    oracle=(SOURCE/'tests/oracle.c').read_text()
    expression=lambda s:re.sub(r'\s+','',re.search(r'const uint16_t flags=(.*?);',s,re.S).group(1))
    assert expression(production)==expression(oracle)
    assert 'now - last_telemetry >= 100U' in production
    h=Harness();h.prepare()
    assert h.mcu.period_ms==100
    for kind in (MessageType.WHEEL_TELEMETRY,MessageType.LINK_STATUS):
        assert decode_frame(packet(h,kind)).flags==0
    h.step(True,seconds=3.)
    assert decode_frame(packet(h)).flags==FrameFlags.ENABLED


def test_original_response_rule_reproduces_failure_with_real_device_stream():
    row=next(json.loads(x) for x in TRACE.read_text().splitlines()
        if json.loads(x)['kind']=='rx' and decode_frame(bytes.fromhex(json.loads(x)['hex'])).message_type==MessageType.LINK_STATUS)
    f=decode_frame(bytes.fromhex(row['hex']))
    name='mecanum_wand_4ws7_hardware.previous_suspended_core'
    spec=importlib.util.spec_from_file_location(name,BEFORE/'host/suspended_core.py')
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    core=old.SuspendedController(lambda:f.session_id);core.session.new_hello(0)
    core.state='BASELINE';core.session.connected=True;core.receive(bytes.fromhex(row['hex']),1.)
    assert core.state=='STOPPING' and 'MCU response flag required' in core.reason


def test_replay_all_real_device_frames_without_rewriting_headers_or_payloads():
    # Actual failed attempt stopped before mode selection. This is wire ingestion
    # evidence, NOT a successful physical preparation/motion or synthetic READY.
    assert hashlib.sha256(TRACE.read_bytes()).hexdigest()=='7b416d6fe55d7af714065e11587d700bd7a7d8ecc988f35e94a45846d3f07210'
    core=None;count=0;hello_count=0
    for line in TRACE.read_text().splitlines():
        row=json.loads(line);raw=bytes.fromhex(row['hex']);f=decode_frame(raw);now=row['steady']
        if row['kind']=='tx':
            if f.message_type==MessageType.HELLO:
                hello_count+=1;core=SuspendedController(lambda:f.session_id)
                core.operator(now-.02,True,True);core.operator(now-.01,True,False)
                assert core.prepare(now,'前进')
                assert decode_frame(core.pending[0]).session_id==f.session_id
            else:core.session.sequence=f.sequence
        else:
            core.receive(raw,now)
            assert core.state in ('HELLO','BASELINE'),core.reason
            if f.message_type in (MessageType.WHEEL_TELEMETRY,MessageType.LINK_STATUS):
                count+=1
                assert core.rx_sequences[f.message_type]==f.sequence
    assert hello_count==2 and count==944
    assert core.status['tag']=='4WS7' and core.wheel.wheel_rpm==(0,0,0,0)
    assert not core.status['permitted'] and not core.status['selected']


def test_stream_cannot_establish_session_or_freshness_before_valid_hello_ack():
    row=next(json.loads(x) for x in TRACE.read_text().splitlines()
        if json.loads(x)['kind']=='rx' and decode_frame(bytes.fromhex(json.loads(x)['hex'])).message_type==MessageType.LINK_STATUS)
    raw=bytes.fromhex(row['hex']);f=decode_frame(raw)
    core=SuspendedController(lambda:f.session_id)
    core.operator(0,True,True);core.operator(.01,True,False);core.prepare(.02,'前进')
    core.receive(raw,.03)
    assert core.state=='HELLO' and not core.session.connected
    assert core.status is None and not core.rx_sequences and not core.fresh(.03)


@pytest.mark.parametrize('flags',[0,1,3,4,8,16])
def test_hello_ack_still_requires_exact_response_flags(flags):
    h=Harness();h.core.operator(0,True,True);h.core.operator(.01,True,False)
    assert h.core.prepare(.02,'前进')
    h.core.receive(encode_frame(MessageType.HELLO_ACK,1,h.core.session.session_id,20,flags=flags),.02)
    assert h.core.state=='STOPPING'


@pytest.mark.parametrize('defect',['payload','sequence'])
def test_hello_ack_payload_and_sequence_are_not_relaxed(defect):
    h=Harness();h.core.operator(0,True,True);h.core.operator(.01,True,False);h.core.prepare(.02,'前进')
    raw=encode_frame(MessageType.HELLO_ACK,2 if defect=='sequence' else 1,h.core.session.session_id,20,
        b'x' if defect=='payload' else b'',FrameFlags.RESPONSE)
    h.core.receive(raw,.02);assert h.core.state=='STOPPING'


@pytest.mark.parametrize('kind',[MessageType.WHEEL_TELEMETRY,MessageType.LINK_STATUS])
@pytest.mark.parametrize('flags',[1,2,3,16,32])
def test_stream_does_not_accept_reply_request_or_unknown_flags(kind,flags):
    h=Harness();h.prepare();h.core.receive(packet(h,kind,flags),h.now)
    assert h.core.state=='STOPPING' and 'stream type/flags' in h.core.reason


@pytest.mark.parametrize('flags',[FrameFlags.ENABLED,FrameFlags.FAULT])
def test_wheel_header_must_match_actual_enable_and_fault_payload(flags):
    h=Harness();h.prepare();h.core.receive(packet(h,flags=flags),h.now)
    assert h.core.state=='STOPPING' and 'flags disagree' in h.core.reason


@pytest.mark.parametrize('defect',['crc','foreign_session','duplicate','future_sample','future_ack','wrong_identity'])
def test_source_integrity_and_freshness_checks_remain_enforced(defect):
    h=Harness();h.prepare();kind=MessageType.LINK_STATUS if defect=='wrong_identity' else MessageType.WHEEL_TELEMETRY
    raw=packet(h,kind);f=decode_frame(raw)
    if defect=='crc':raw=raw[:-1]+bytes([raw[-1]^1])
    if defect=='foreign_session':raw=packet(h,session=h.core.session.session_id+1)
    if defect=='future_sample':raw=encode_frame(kind,f.sequence,f.session_id,f.sender_time_ms+10000,f.payload,flags=f.flags)
    if defect in ('future_ack','wrong_identity'):
        payload=bytearray(f.payload)
        if defect=='future_ack':payload[-4:]=(h.core.session.sequence+100).to_bytes(4,'little')
        else:payload[:4]=b'4W20'
        raw=encode_frame(kind,f.sequence,f.session_id,f.sender_time_ms,payload,flags=f.flags)
    if defect=='duplicate':h.core.receive(raw,h.now)
    previous=(h.core.wheel_at,h.core.status_at)
    h.core.receive(raw,h.now+.01)
    if defect in ('crc','future_ack','wrong_identity'):assert h.core.state=='STOPPING'
    else:
        assert (h.core.wheel_at,h.core.status_at)==previous
        assert h.core.state=='READY'


def test_frame_level_status_decoder_accepts_real_stream_and_rejects_bad_flags():
    h=Harness();h.prepare();raw=packet(h,MessageType.LINK_STATUS)
    assert decode_status_frame(raw)['tag']=='4WS7'
    with pytest.raises(ValueError,match='header'):
        decode_status_frame(packet(h,MessageType.LINK_STATUS,FrameFlags.RESPONSE))


@pytest.mark.parametrize('period_ms',[50,100])
def test_realistic_cadence_single_five_seconds_release_and_manual_reprepare(period_ms):
    h=Harness();h.mcu=Oracle(period_ms);h.prepare()
    assert not h.nonzero()
    h.step(True,seconds=5.3);h.step(True,seconds=1.5)
    assert h.core.state=='COMPLETE'
    before=len(h.nonzero());h.step(True,seconds=.3);assert len(h.nonzero())==before
    h.step(False);h.prepare('左移');assert not h.core.status['permitted']
    h.step(True,seconds=.4);assert h.core.state=='ACTIVE'
    h.step(False,seconds=1.8);assert h.core.state=='COMPLETE'
