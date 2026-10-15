"""ROS sender -> actual C motion state machine, entirely in native memory."""
import ctypes
from pathlib import Path
import subprocess

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mecanum_vicon_control.cmd_vel_udp_bridge import CmdVelUdpBridge
from mecanum_vicon_control.motion_link_status import decode_motion_status
from mecanum_vicon_control.transport_protocol import (
    FrameFlags, MessageType, ProtocolError, decode_frame, encode_frame, unpack_cmd_vel,
)

@pytest.fixture(scope='module')
def native(tmp_path_factory):
    root=Path(__file__).resolve().parents[3]/'stm32/mecanum_f407'
    output=tmp_path_factory.mktemp('motion_native')/'motion.so'
    sources=['tests/motion_host_fixture.c','Src/four_link_motion.c','Src/four_drive_group.c',
             'Src/four_wheel_speed.c','Src/drive_bench_control.c','Src/mecanum_drive_plan.c',
             'Src/mecanum_kinematics.c','Src/mc_protocol.c']
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-shared','-fPIC',
        '-DFOUR_LINK_MOTION=1','-DDRIVE_BENCH=1','-DDRIVE_STARTUP_OBSERVE=1',
        '-IInc','-I../../protocol',*sources,'-lm','-o',str(output)],cwd=root,check=True)
    lib=ctypes.CDLL(str(output))
    lib.fixture_receive.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint32]
    lib.fixture_feedback.argtypes=[ctypes.c_void_p,ctypes.c_uint8,ctypes.c_uint32,ctypes.c_uint32]
    lib.fixture_feedback.restype=ctypes.c_size_t
    return lib

class MemoryWire:
    def __init__(self,lib):self.lib=lib;self.ms=0;self.rx=[];self.sent=[];self.drop=False;self.seq=1000
    def sendto(self,raw,_peer):
        f=decode_frame(raw);self.sent.append((self.ms,f))
        if not self.drop:
            ack=self.lib.fixture_receive(raw,len(raw),self.ms)
            assert ack>=0
            if ack:self.rx.append(encode_frame(MessageType.HELLO_ACK,f.sequence,f.session_id,
                                               self.ms,flags=FrameFlags.RESPONSE))
        return len(raw)
    def feedback(self):
        for kind in (MessageType.WHEEL_TELEMETRY,MessageType.LINK_STATUS):
            buffer=ctypes.create_string_buffer(300);self.seq+=1
            n=self.lib.fixture_feedback(buffer,kind,self.seq,self.ms);assert n
            if not self.drop:self.rx.append(buffer.raw[:n])
    def recvfrom(self,_n):
        if not self.rx:raise BlockingIOError
        return self.rx.pop(0),('192.0.2.1',46000)
    def close(self):pass

def test_zero_primer_actual_c_enable_and_loss_never_restarts(native,monkeypatch):
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true','-p','max_vx_mps:=0.05',
                     '-p','max_vy_mps:=0.05','-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge();native.fixture_init();wire=MemoryWire(native)
    node.sock=wire;node.esp_ip='192.0.2.1';node.destination=(node.esp_ip,46000)
    monkeypatch.setattr('mecanum_vicon_control.cmd_vel_udp_bridge.time.monotonic',lambda:10+wire.ms/1000)
    try:
        node.on_enable(Bool(data=True));cmd=Twist();cmd.linear.x=.05;node.on_cmd(cmd)
        node.on_control_enable(Bool(data=True))
        for ms in range(10,1801,10):
            wire.ms=ms;native.fixture_tick(ms)
            if ms%100==0:wire.feedback()
            if ms%20==0:node.tick()
            if ms<600:assert not native.fixture_enabled() and not native.fixture_permitted()
        controls=[(ms,f) for ms,f in wire.sent if f.message_type==MessageType.CONTROL_ENABLE]
        assert controls and controls[0][0]>=600
        primer=[f for ms,f in wire.sent if ms<controls[0][0] and f.message_type==MessageType.CMD_VEL]
        assert len(primer)>=25 and all(not f.flags&FrameFlags.ENABLED and
            unpack_cmd_vel(f.payload).vx_mps==0 for f in primer)
        assert native.fixture_enabled() and native.fixture_permitted() and not native.fixture_fault()
        wire.drop=True
        for ms in range(1810,2501,10):
            wire.ms=ms;native.fixture_tick(ms)
            if ms%20==0:node.tick()
        assert not native.fixture_enabled() and native.fixture_fault() and node.local_stop_latched
        wire.drop=False
        for ms in range(2510,3201,10):
            wire.ms=ms;native.fixture_tick(ms)
            if ms%100==0:wire.feedback()
            if ms%20==0:node.tick()
        assert not native.fixture_enabled() and not native.fixture_permitted() and node.local_stop_latched
    finally:
        node.destroy_node();rclpy.shutdown()

def test_motion_decoder_refuses_readonly_or_bench_identity():
    from mecanum_vicon_control.bench_link_status import STATUS
    for tag in [0x314F5234,0x31425734]:
        values=[0]*35;values[0]=tag;values[3]=values[4]=10
        with pytest.raises(ProtocolError):decode_motion_status(STATUS.pack(*values))
