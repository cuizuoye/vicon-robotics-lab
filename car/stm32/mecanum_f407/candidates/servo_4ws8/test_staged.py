"""Independent 4WS8: real C controller, synthetic encoders; no network IO."""
import ctypes
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
import pytest

GENERATED=Path(os.environ['STAGED_GENERATED'])
for name in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
    sys.path.insert(0,str(GENERATED/name))
from mecanum_wand_4ws8_hardware.suspended_core import SuspendedController,DIRECTIONS
from mecanum_wand_4ws8_hardware.stage0_contract import decode_motion_status,tier_payload
from mecanum_wand_4ws8_hardware.stage0_session import Stage0Session
from mecanum_wand_4ws8_hardware.stages import STAGES
from mecanum_wand_4ws8_hardware.authorization import verify_runtime_profile,verify_trial_review,verify_flash_record
from mecanum_wand_4ws8_hardware.commissioning import AuthoritySigner,AuthorityVerifier,TransportPolicy,Runtime
from mecanum_wand_4ws8_control.core import Velocity,Pose
from mecanum_wand_4ws8_control.safety_core import ZERO
from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig
from mecanum_vicon_control.transport_protocol import decode_frame,unpack_cmd_vel,MessageType,FrameFlags,ProtocolError

@pytest.fixture(autouse=True)
def offline_only(monkeypatch):
    import socket
    def no_socket(*a,**k):raise AssertionError('offline: sockets forbidden')
    monkeypatch.setattr(socket,'socket',no_socket)

class Oracle:
    def __init__(self,period_ms=100):
        self.dll=ctypes.CDLL(str(Path(os.environ['STAGED_NATIVE'])/'libstage0_oracle.so'))
        self.dll.oracle_request.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint32,ctypes.c_void_p]
        self.dll.oracle_request.restype=ctypes.c_size_t
        self.dll.oracle_packet.argtypes=[ctypes.c_uint,ctypes.c_void_p];self.dll.oracle_packet.restype=ctypes.c_size_t
        self.dll.oracle_tick.argtypes=[ctypes.c_uint32]
        self.dll.oracle_init();self.out=ctypes.create_string_buffer(1024)
        self.period_ms=period_ms;self.last_publish=0

    def request(self,raw,now):
        n=self.dll.oracle_request(raw,len(raw),round(now*1000),self.out)
        return self.out.raw[:n] if n else None

    def packets(self,now):
        self.dll.oracle_tick(round(now*1000))
        if round(now*1000)-self.last_publish<self.period_ms:return []
        self.last_publish=round(now*1000);result=[]
        for kind in (0x30,0x31):
            n=self.dll.oracle_packet(kind,self.out)
            if n:result.append(self.out.raw[:n])
        return result

class Harness:
    def __init__(self,stage=0):
        ids=iter(range(17,100));self.core=SuspendedController(lambda:next(ids),stage=stage)
        self.mcu=Oracle();self.now=0.;self.tx=[]

    def step(self,down=False,focus=True,ui=True,receive=True,seconds=.01):
        for _ in range(round(seconds/.01)):
            self.now+=.01
            if ui:self.core.operator(self.now,focus,down)
            for raw in self.core.step(self.now):
                self.tx.append(raw);ack=self.mcu.request(raw,self.now)
                if ack and receive:self.core.receive(ack,self.now)
            for raw in self.mcu.packets(self.now):
                if receive:self.core.receive(raw,self.now)

    def prepare(self,direction='前进'):
        self.step(True);self.step(False)
        assert self.core.unlocked
        assert self.core.prepare(self.now,direction)
        self.step(seconds=1.8)
        assert self.core.state=='READY',(self.core.state,self.core.reason)

    def nonzero(self):
        output=[]
        for raw in self.tx:
            f=decode_frame(raw)
            if f.message_type==MessageType.CMD_VEL:
                v=unpack_cmd_vel(f.payload)
                if any((v.vx_mps,v.vy_mps,v.wz_radps)):output.append((f,v))
        return output

@pytest.mark.parametrize('stage',range(7))
@pytest.mark.parametrize('direction',tuple(DIRECTIONS))
def test_all_stages_all_directions_follow_fixed_caps(stage,direction):
    h=Harness(stage);h.prepare(direction)
    assert not h.nonzero() and not h.core.status['permitted']
    h.step(True,seconds=3.)
    assert h.core.state=='ACTIVE',(stage,direction,h.core.reason)
    assert h.core.status['stage']==stage and h.nonzero()
    for f,v in h.nonzero():
        assert f.flags&FrameFlags.ENABLED
        assert math.hypot(v.vx_mps,v.vy_mps)<=STAGES[stage].speed+1e-9
        assert v.timeout_ms==600 and not v.wz_radps
    assert max(map(abs,h.core.status['raw_duty']))<=STAGES[stage].duty
    h.step(False,seconds=1.8)
    assert h.core.state=='COMPLETE' and not h.core.status['permitted']

@pytest.mark.parametrize('stage',range(7))
@pytest.mark.parametrize('cause',['release','focus','ui_freeze','feedback_loss','five_seconds'])
def test_stop_never_resumes_without_new_manual_session(stage,cause):
    h=Harness(stage);h.prepare();h.step(True,seconds=1.)
    if cause=='release':h.step(False)
    elif cause=='focus':h.step(True,False)
    elif cause=='ui_freeze':h.step(True,ui=False,seconds=.17)
    elif cause=='feedback_loss':h.step(True,receive=False,seconds=.22)
    else:h.step(True,seconds=4.3)
    assert h.core.state in ('STOPPING','COMPLETE')
    before=len(h.nonzero());h.step(True,seconds=2.)
    assert h.core.state in ('COMPLETE','FAULT') and len(h.nonzero())==before
    h.step(False);h.prepare();assert not h.core.status['permitted']
    h.step(True,seconds=.3);assert h.core.state=='ACTIVE'

def profile(stage):
    return json.loads((GENERATED/f'mecanum_wand_4ws8_hardware/config/stage{stage}.NOT_APPROVED.json').read_text())

@pytest.mark.parametrize('stage',range(7))
def test_draft_blocked_before_node_or_socket(stage):
    path=GENERATED/f'mecanum_wand_4ws8_hardware/config/stage{stage}.NOT_APPROVED.json'
    with pytest.raises(ValueError,match='site approval'):Runtime('hardware',str(path))
    with pytest.raises(ValueError,match='site approval'):verify_runtime_profile(profile(stage),'suspended')
    with pytest.raises(ValueError):Runtime('shadow',str(path))

@pytest.mark.parametrize('stage',range(7))
def test_config_and_stage_protocol_are_exact(stage):
    p=profile(stage);cfg=StagedSafetyConfig(**p['safety'])
    assert cfg.stage==stage and cfg.follower_config().max_speed_mps==STAGES[stage].speed
    assert cfg.follower_config().max_accel_mps2==STAGES[stage].accel
    assert (cfg.arena_forward_max_m,cfg.arena_left_max_m,cfg.arena_buffer_m)==(3,3,.5)
    assert p['mapping_scale']==1 and cfg.goal_boundary_policy=='clamp'
    raw=(Path(os.environ['STAGED_NATIVE'])/f'active{stage}.bin').read_bytes()
    assert decode_motion_status(raw)['stage']==stage
    for offset,value in ((0,b'4WS7'),(153,b'\x03'),(178,b'\x07')):
        bad=bytearray(raw);bad[offset:offset+len(value)]=value
        with pytest.raises(ProtocolError):decode_motion_status(bad)
    assert struct.unpack('<BBH',tier_payload(stage))[2]==0x7800+stage

@pytest.mark.parametrize('value',[-1,7,True,1.0,None])
def test_invalid_stage_never_creates_session(value):
    with pytest.raises(ValueError):Stage0Session(stage=value)
    with pytest.raises(ValueError):tier_payload(value)

@pytest.mark.parametrize('speed,accel',[(.70,.08),(.20,2),(.71,2),(.7,2.01),(float('nan'),2)])
def test_unenumerated_pairs_rejected(speed,accel):
    p=profile(6);p['safety'].update(max_speed_mps=speed,max_accel_mps2=accel)
    with pytest.raises(ValueError):StagedSafetyConfig(**p['safety'])

@pytest.mark.parametrize('stage',range(7))
def test_per_stage_signed_authority_and_crosstier_rejection(stage):
    key='23'*32;s=AuthoritySigner(key,stage);v=AuthorityVerifier(key,stage)
    v.decode(s.encode(ZERO,10,100),10,100)
    v.decode(s.encode(ZERO,10.01,100.01,epoch='e',state='ARMED',permit=True),10.01,100.01)
    command=Velocity(STAGES[stage].speed,0,0)
    wire=s.encode(command,10.02,100.02,epoch='e',state='ACTIVE',permit=True)
    assert v.decode(wire,10.02,100.02)[1]==command
    with pytest.raises(ValueError):AuthorityVerifier(key,(stage+1)%7).decode(wire,10.02,100.02)
    with pytest.raises(ValueError):v.decode(wire,10.02,100.02)
    with pytest.raises(ValueError):v.decode(s.encode(Velocity(STAGES[stage].speed+.001,0,0),10.03,100.03,epoch='e',state='ACTIVE',permit=True),10.03,100.03)

@pytest.mark.parametrize('stage',range(7))
def test_signed_timeout_blocks_old_epoch_and_requires_armed_new_epoch(stage):
    s=AuthoritySigner('23'*32,stage);p=TransportPolicy('23'*32,stage)
    def tx(t,state='STANDBY',permit=False,epoch='',cmd=ZERO):
        return p.receive(s.encode(cmd,t,100+t,state=state,permit=permit,epoch=epoch),t,100+t)
    assert tx(10);assert tx(10.01,'ARMED',True,'e');assert tx(10.02,'ACTIVE',True,'e',Velocity(.1,0,0))
    assert p.active(10.02) and not p.active(10.13)
    tx(10.14,'ACTIVE',True,'e',Velocity(.1,0,0));assert not p.active(10.14)
    tx(10.15);tx(10.16,'ARMED',True,'f');tx(10.17,'ACTIVE',True,'f',Velocity(.1,0,0));assert p.active(10.17)

def review(stage=0,purpose='suspended'):
    return dict(kind='BOUNDED_TRIAL_NOT_QUALIFICATION',identity='4WS8',image_sha256='abc',
        authorized_stage=stage,purpose=purpose,human_authorized=True)

def test_review_no_automatic_promote_and_hash_checked_previous_physical_evidence(tmp_path):
    verify_trial_review(review(),0,'suspended','abc')
    r=review(1)
    with pytest.raises(ValueError):verify_trial_review(r,1,'suspended','abc')
    prev=tmp_path/'previous.json';prev.write_text(json.dumps(dict(identity='4WS8',image_sha256='abc',stage=0,
        purpose='suspended',physical_trial=True,action_and_stop_passed=True,reviewed_by_human=True)))
    r.update(previous_stage=0,previous_stage_action_and_stop_passed=True,previous_stage_evidence=str(prev),
        previous_stage_evidence_sha256=hashlib.sha256(prev.read_bytes()).hexdigest())
    verify_trial_review(r,1,'suspended','abc')
    prev.write_text('{}')
    with pytest.raises(ValueError):verify_trial_review(r,1,'suspended','abc')

@pytest.mark.parametrize('speed',[float('nan'),float('inf'),.2,True])
def test_ground_review_cannot_claim_unmeasured_current_stage_stop(tmp_path,speed):
    r=review(1,'sign');prev=tmp_path/'previous.json'
    prev.write_text(json.dumps(dict(identity='4WS8',image_sha256='abc',stage=0,purpose='sign',
        physical_trial=True,action_and_stop_passed=True,reviewed_by_human=True)))
    r.update(previous_stage=0,previous_stage_action_and_stop_passed=True,previous_stage_evidence=str(prev),
        previous_stage_evidence_sha256=hashlib.sha256(prev.read_bytes()).hexdigest(),
        measured_trial_stop_distance_m=.4,stop_test_speed_mps=speed,stop_envelope_includes_fallback_and_footprint=True,
        stop_envelope_fits_original_buffer=True,electrical_thermal_trial_reviewed=True)
    with pytest.raises(ValueError):verify_trial_review(r,1,'sign','abc')

def test_old_real_flash_record_is_not_4ws8_approval(tmp_path):
    manifest=tmp_path/'build.json';manifest.write_text(json.dumps(dict(firmware=dict(identity='4WS7'))))
    record=tmp_path/'record.json';record.write_text(json.dumps(dict(full_readback_path=str(record))))
    with pytest.raises(ValueError):verify_flash_record(record,manifest)

def test_final_gate_and_continuous_servo_algorithm_unchanged():
    import ast
    root=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
    old=root/'docs/wand_4ws7_stage0_2026-10-23/host_fix_20261023/verified_v2/runtime/src/mecanum_wand_4ws7_control/mecanum_wand_4ws7_control'
    new=GENERATED/'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control'
    def gate(folder):
        s=(folder/'safety_core.py').read_text()
        return ast.get_source_segment(s,next(n for n in ast.parse(s).body if isinstance(n,ast.ClassDef) and n.name=='FinalGate'))
    assert gate(old)==gate(new)
    assert (old/'continuous_servo.py').read_bytes()==(new/'continuous_servo.py').read_bytes()

@pytest.mark.parametrize('stage',range(7))
def test_real_safety_node_constructor_accepts_exact_dataclass_without_dds(stage,monkeypatch):
    from types import SimpleNamespace
    from mecanum_wand_4ws8_control.safety_nodes import SafetyNode,Node
    from mecanum_wand_4ws8_control.safety_core import Supervisor,FinalGate
    from mecanum_wand_4ws8_control.continuous_servo import ContinuousSession
    from dataclasses import fields,asdict
    config=StagedSafetyConfig(**profile(stage)['safety']);parameters={}
    assert StagedSafetyConfig(**asdict(config))==config and fields(type(config))
    monkeypatch.setattr(Node,'__init__',lambda self,*a,**k:None)
    monkeypatch.setattr(Node,'declare_parameter',lambda self,n,v:parameters.setdefault(n,v))
    monkeypatch.setattr(Node,'get_parameter',lambda self,n:SimpleNamespace(value=parameters[n]))
    monkeypatch.setattr(Node,'create_timer',lambda *a,**k:None)
    monkeypatch.setattr(Node,'create_subscription',lambda *a,**k:None)
    runtime=SimpleNamespace(namespace='/wand_4ws8_staged',world='map',mode='hardware',config=config,guard=lambda n:None)
    for core in (Supervisor,FinalGate):
        node=object.__new__(SafetyNode);SafetyNode.__init__(node,'offline_constructor',core,runtime)
        assert node.core.c==config and type(node.core.c) is StagedSafetyConfig
        if core is Supervisor:
            assert isinstance(node.core.follower,ContinuousSession)
            assert node.core.follower.config.max_speed_mps==STAGES[stage].speed
            assert node.core.follower.config.max_accel_mps2==STAGES[stage].accel

@pytest.mark.parametrize('stage',range(7))
def test_runtime_validates_synthetic_file_evidence_without_io(stage,tmp_path):
    # Test-only file fixtures, not physical evidence or a runnable approval.
    from mecanum_wand_4ws8_hardware.commissioning import CONSENT
    image=tmp_path/'SYNTHETIC_NOT_FIRMWARE.bin';image.write_bytes(b'SYNTHETIC_TEST_ONLY')
    image_sha=hashlib.sha256(image.read_bytes()).hexdigest()
    readback=tmp_path/'SYNTHETIC_NOT_DEVICE.bin';readback.write_bytes(image.read_bytes()+b'\xff'*(1048576-image.stat().st_size))
    manifest=tmp_path/'SYNTHETIC_build.json';manifest.write_text(json.dumps(dict(firmware=dict(
        identity='4WS8',authority_version=4,path=str(image),sha256=image_sha))))
    record=tmp_path/'SYNTHETIC_record.json';record.write_text(json.dumps(dict(identity='4WS8',flashed=True,
        independent_readback_verified=True,image_sha256=image_sha,full_readback_path=str(readback),
        full_readback_sha256=hashlib.sha256(readback.read_bytes()).hexdigest())))
    p=profile(stage);r=review(stage,'sign');r['image_sha256']=image_sha
    if stage:
        previous=tmp_path/'SYNTHETIC_previous.json';previous.write_text(json.dumps(dict(identity='4WS8',
            image_sha256=image_sha,stage=stage-1,purpose='sign',physical_trial=True,action_and_stop_passed=True,reviewed_by_human=True)))
        r.update(previous_stage=stage-1,previous_stage_action_and_stop_passed=True,previous_stage_evidence=str(previous),
            previous_stage_evidence_sha256=hashlib.sha256(previous.read_bytes()).hexdigest(),
            measured_trial_stop_distance_m=.4,stop_test_speed_mps=STAGES[stage].speed,
            stop_envelope_includes_fallback_and_footprint=True,stop_envelope_fits_original_buffer=True,
            electrical_thermal_trial_reviewed=True)
    p['trial_review']=r;p['staged_runtime_approval']=dict(purpose='sign',issued_utc_s=time.time(),
        clear_and_power_cut_ready_now=True,flash_record=str(record),build_manifest=str(manifest))
    p['confirmations']['fixed_arena_checked']=True
    p['arena_confirmations']={k:True for k in p['arena_confirmations']}
    path=tmp_path/'SYNTHETIC_profile.json';path.write_text(json.dumps(p))
    runtime=Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),CONSENT)
    assert runtime.config.stage==stage and runtime.config.max_accel_mps2==STAGES[stage].accel
    p['staged_runtime_approval']['issued_utc_s']-=601
    with pytest.raises(ValueError):verify_runtime_profile(p,'sign')

def test_original_decoder_and_global_whitelist_reject_new_firmware():
    from mecanum_vicon_control.motion_link_status import MOTION_TAGS
    assert '4WS8' not in MOTION_TAGS
    old=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4ws7_stage0_2026-10-23/host_fix_20261023/verified_v2/runtime/src/mecanum_wand_4ws7_hardware/mecanum_wand_4ws7_hardware/stage0_contract.py')
    spec=importlib.util.spec_from_file_location('unchanged_4ws7_decoder',old)
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    with pytest.raises(ProtocolError):module.decode_motion_status((Path(os.environ['STAGED_NATIVE'])/'active6.bin').read_bytes())

