"""New adapters only; real C state machine, synthetic encoders, no UDP/ROS nodes."""
import ctypes
import importlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
import pytest

GENERATED=Path(os.environ['STAGE0_GENERATED'])
for name in ('mecanum_wand_4ws7_control','mecanum_wand_4ws7_hardware'):
    sys.path.insert(0,str(GENERATED/name))
from mecanum_wand_4ws7_hardware.suspended_core import SuspendedController,DIRECTIONS
from mecanum_wand_4ws7_hardware.stage0_contract import decode_motion_status,disabled_motion_status_ready
from mecanum_wand_4ws7_hardware.stage0_session import Stage0Session
from mecanum_wand_4ws7_hardware.authorization import verify_runtime_profile
from mecanum_vicon_control.transport_protocol import decode_frame,unpack_cmd_vel,MessageType,FrameFlags,ProtocolError

class Oracle:
    def __init__(self,period_ms=100):
        self.dll=ctypes.CDLL(str(Path(os.environ['STAGE0_NATIVE'])/'libstage0_oracle.so'))
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
        self.last_publish=round(now*1000)
        result=[]
        for kind in (0x30,0x31):
            n=self.dll.oracle_packet(kind,self.out)
            if n:result.append(self.out.raw[:n])
        return result

class Harness:
    def __init__(self):
        ids=iter(range(17,100));self.core=SuspendedController(lambda:next(ids))
        self.mcu=Oracle();self.now=0.;self.tx=[]

    def step(self,down=False,focus=True,ui=True,receive=True,seconds=.01):
        for _ in range(round(seconds/.01)):
            self.now+=.01
            if ui:self.core.operator(self.now,focus,down)
            for raw in self.core.step(self.now):
                self.tx.append(raw)
                ack=self.mcu.request(raw,self.now)
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

def test_boot_prepare_and_new_press_do_not_move_automatically():
    h=Harness();h.prepare()
    assert not h.nonzero() and not h.core.status['permitted']
    h.step(True,seconds=.25)
    assert h.core.state=='ACTIVE' and h.nonzero()
    assert all(f.flags&FrameFlags.ENABLED for f,v in h.nonzero())

def test_stop_discards_an_enable_queued_before_worker_send():
    h=Harness();h.prepare()
    h.core.operator(h.now+.001,True,True)
    assert h.core.pending
    h.core.stop(h.now+.002,'stop before queued enable can be sent')
    for raw in h.core.step(h.now+.003):
        f=decode_frame(raw)
        if f.message_type==MessageType.CONTROL_ENABLE:assert f.payload[0]==0
        if f.message_type==MessageType.CMD_VEL:assert not f.flags&FrameFlags.ENABLED

@pytest.mark.parametrize('direction',tuple(DIRECTIONS))
def test_all_eight_directions_are_stage0_and_inward_quantized(direction):
    h=Harness();h.prepare(direction);h.step(True,seconds=3.)
    assert h.core.state=='ACTIVE'
    for _,v in h.nonzero():
        assert math.hypot(v.vx_mps,v.vy_mps)<=.20 and not v.wz_radps and v.timeout_ms==600
    assert max(map(abs,h.core.status['raw_duty']))<=350

@pytest.mark.parametrize('cause',['release','focus','ui_freeze','feedback_loss'])
def test_release_focus_and_independent_leases_revoke_and_never_auto_resume(cause):
    h=Harness();h.prepare();h.step(True,seconds=2.)
    before=len(h.nonzero())
    if cause=='release':h.step(False)
    elif cause=='focus':h.step(True,False)
    elif cause=='ui_freeze':h.step(True,ui=False,seconds=.17)
    else:h.step(True,receive=False,seconds=.22)
    assert h.core.state=='STOPPING'
    h.step(False,seconds=1.8)
    assert h.core.state in ('COMPLETE','FAULT')
    after=len(h.nonzero());h.step(True,seconds=.3)
    assert len(h.nonzero())==after and after>=before

def test_five_seconds_one_shot_and_manual_second_prepare():
    h=Harness();h.prepare();h.step(True,seconds=5.2)
    assert h.core.state in ('STOPPING','COMPLETE')
    after=len(h.nonzero());h.step(True,seconds=1.2)
    assert h.core.state=='COMPLETE' and len(h.nonzero())==after
    h.step(False);assert not h.core.unlocked
    h.prepare();assert not h.core.status['permitted']
    h.step(True,seconds=.3);assert h.core.state=='ACTIVE'

def test_held_key_during_prepare_is_not_deferred_enable():
    h=Harness();h.step(True);h.step(False)
    assert h.core.prepare(h.now,'前进');h.step(True,seconds=.3)
    assert h.core.state in ('STOPPING','FAULT','COMPLETE') and not h.nonzero()

def test_true_mcu_fault_cannot_be_cleared_by_gui():
    h=Harness();h.prepare();h.step(True,seconds=.3)
    h.core.stop(h.now,'synthetic emergency',fault=True,estop=True)
    h.step(False,seconds=2.)
    assert h.core.wheel.fault_bits and h.core.state=='FAULT'
    h.step(True);h.step(False)
    assert not h.core.prepare(h.now,'前进')

@pytest.mark.parametrize('index',range(1,7))
def test_client_has_no_higher_tier(index):
    from mecanum_wand_4ws7_hardware.stage0_contract import tier_payload
    with pytest.raises(ValueError):tier_payload(index)

def test_native_c_v3_fixtures_and_wrong_version_rejected():
    root=Path(os.environ['STAGE0_NATIVE'])
    idle=decode_motion_status((root/'idle.bin').read_bytes())
    active=decode_motion_status((root/'bench_active.bin').read_bytes())
    done=decode_motion_status((root/'bench_completed.bin').read_bytes())
    assert disabled_motion_status_ready(idle)
    assert active['action_mode']==2 and active['permitted']
    assert done['bench_completed'] and not done['permitted']
    raw=bytearray((root/'bench_active.bin').read_bytes());raw[153]=2
    with pytest.raises(ProtocolError):decode_motion_status(raw)

def test_runtime_draft_blocked_before_socket_or_node(monkeypatch):
    from mecanum_wand_4ws7_hardware.commissioning import Runtime
    profile=GENERATED/'mecanum_wand_4ws7_hardware/config/4ws7_stage0.SITE_AND_FLASH_REQUIRED.json'
    with pytest.raises(ValueError,match='site approval'):Runtime('hardware',str(profile))
    value=json.loads(profile.read_text())
    with pytest.raises(ValueError,match='site approval'):verify_runtime_profile(value,'suspended')

def test_generated_sign_adapter_selects_before_ready_without_global_whitelist_changes():
    from mecanum_wand_4ws7_hardware.nodes import HardwareBridge
    import inspect
    from mecanum_vicon_control.motion_link_status import MOTION_TAGS
    assert '4WS7' not in MOTION_TAGS
    assert "status.get('selected')" in inspect.getsource(HardwareBridge.safety_tick)
    assert 'select_mode' in inspect.getsource(HardwareBridge.safety_tick)
    assert "self.last_motion_status.get('selected')" in inspect.getsource(HardwareBridge.queue_control_action)

def test_preserved_key_sampler_is_read_only_and_uses_physical_bitmap():
    from mecanum_wand_4ws7_control.x11_key_state import X11KeyState
    bitmap=bytearray(32);bitmap[65//8]|=1<<(65%8)
    assert X11KeyState.bitmap_down(bitmap,65)
