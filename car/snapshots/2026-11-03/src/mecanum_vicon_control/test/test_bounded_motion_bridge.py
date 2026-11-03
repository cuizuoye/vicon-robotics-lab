"""ROS sender -> actual C motion state machine, entirely in native memory."""
import ctypes
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mecanum_vicon_control.cmd_vel_udp_bridge import CmdVelUdpBridge
from mecanum_vicon_control.motion_link_status import decode_motion_status
from mecanum_vicon_control.transport_protocol import (
    FrameFlags, MessageType, ProtocolError, decode_frame, encode_frame,
    unpack_cmd_vel, unpack_control_enable,
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

def test_confirmed_gate_is_preserved_by_flagged_zero_before_ros_motion(native,monkeypatch):
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true','-p','max_vx_mps:=0.05',
                     '-p','max_vy_mps:=0.05','-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge();native.fixture_init();wire=MemoryWire(native)
    node.sock=wire;node.esp_ip='192.0.2.1';node.destination=(node.esp_ip,46000)
    monkeypatch.setattr('mecanum_vicon_control.cmd_vel_udp_bridge.time.monotonic',lambda:20+wire.ms/1000)
    try:
        # Pose controller is publishing fresh zero, but the ROS safety gate is
        # deliberately still closed while CONTROL_ENABLE is established.
        node.on_cmd(Twist())
        node.on_control_enable(Bool(data=True))
        for ms in range(10,1601,10):
            wire.ms=ms;native.fixture_tick(ms)
            if ms%100==0:wire.feedback()
            if ms%20==0:node.tick()
        assert native.fixture_permitted() and not native.fixture_enabled()
        assert not native.fixture_fault() and not node.safety_enabled
        post_enable=[f for ms,f in wire.sent if ms>=900 and f.message_type==MessageType.CMD_VEL]
        assert post_enable
        assert all(f.flags&FrameFlags.ENABLED and unpack_cmd_vel(f.payload).vx_mps==0
                   for f in post_enable)
        node.on_enable(Bool(data=True))
        for ms in range(1610,1901,10):
            wire.ms=ms;native.fixture_tick(ms)
            if ms%100==0:wire.feedback()
            if ms%20==0:node.tick()
        assert native.fixture_permitted() and not native.fixture_enabled()
        assert not native.fixture_fault()
        safety_zero=[f for ms,f in wire.sent if ms>=1620 and f.message_type==MessageType.CMD_VEL]
        assert safety_zero
        assert all(f.flags&FrameFlags.ENABLED and unpack_cmd_vel(f.payload).vx_mps==0
                   for f in safety_zero)
    finally:
        node.destroy_node();rclpy.shutdown()

def test_motion_decoder_refuses_readonly_or_bench_identity():
    from mecanum_vicon_control.bench_link_status import STATUS
    for tag in [0x314F5234,0x31425734]:
        values=[0]*35;values[0]=tag;values[3]=values[4]=10
        with pytest.raises(ProtocolError):decode_motion_status(STATUS.pack(*values))

def test_motion_decoder_accepts_and_bounds_ground_identity():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x31475734;values[3]=values[4]=10;values[15]=1
    values[8:16]=[100,85,85,100,100,85,85,100]
    values[16:20]=[150,-150,150,-150]
    values[20:24]=[100,100,100,100]
    values[28]=50;values[31]=15;values[34]=1
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WG1' and decoded['startup_mask']==15
    values[16]=160;values[8:10]=[100,84]
    with pytest.raises(ProtocolError):decode_motion_status(STATUS.pack(*values))

def test_motion_decoder_accepts_and_bounds_ground2_identity():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x32475734;values[3]=values[4]=10
    values[8:16]=[100,70,70,100,100,70,70,100]
    values[16:20]=[300,-300,300,-300]
    values[20:24]=[100,100,100,100]
    values[28]=50;values[31]=15;values[34]=1
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WG2' and decoded['startup_mask']==15
    values[16]=360;values[8:10]=[100,64]
    with pytest.raises(ProtocolError):decode_motion_status(STATUS.pack(*values))

def test_motion_decoder_accepts_ground3_identity():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x33475734;values[3]=values[4]=10;values[7]=500
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WG3' and decoded['permitted']==0

def test_motion_decoder_accepts_ground4_identity():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x34475734;values[3]=values[4]=10;values[7]=500
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WG4' and decoded['permitted']==0

def test_motion_decoder_accepts_ground5_identity():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x35475734;values[3]=values[4]=10;values[7]=500
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WG5' and decoded['permitted']==0

def test_motion_decoder_accepts_ground6_identity():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x36475734;values[3]=values[4]=10;values[7]=500
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WG6' and decoded['permitted']==0

def test_motion_decoder_accepts_ground7_variable_launch_duty():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x37475734;values[3]=values[4]=10;values[7]=500
    values[8:10]=[100,98];values[16]=20;values[20]=150
    values[31]=1;values[34]=1
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WG7' and decoded['startup_mask']==1
    values[16]=0;values[8:10]=[0,0]
    with pytest.raises(ProtocolError):decode_motion_status(STATUS.pack(*values))

def test_motion_decoder_accepts_ground8_200mmps_envelope_only():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x38475734;values[3]=values[4]=10;values[7]=500
    values[8:16]=[100,75,75,100,100,75,75,100]
    values[16:20]=[250,-250,250,-250]
    values[20:24]=[1124,1113,1120,1126]
    values[28]=200;values[34]=1
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WG8' and decoded['body_milli'][0]==200
    values[28]=201
    with pytest.raises(ProtocolError):decode_motion_status(STATUS.pack(*values))
    values[28]=200;values[20]=1201
    with pytest.raises(ProtocolError):decode_motion_status(STATUS.pack(*values))

def test_motion_decoder_accepts_ground9_safe_rearm_identity():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x39475734;values[3]=values[4]=10;values[7]=500
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WG9' and decoded['started_ms']==0

def test_motion_decoder_accepts_grounda_recovery_identity_and_100mmps_bound():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x41475734;values[3]=values[4]=10;values[7]=500
    values[8:10]=[100,90];values[16]=100;values[20]=600
    values[28]=100;values[34]=1
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WGA' and decoded['body_milli'][0]==100
    values[28]=101
    with pytest.raises(ProtocolError):decode_motion_status(STATUS.pack(*values))

def test_motion_decoder_accepts_groundb_recovery_identity_and_150mmps_bound():
    from mecanum_vicon_control.bench_link_status import STATUS
    values=[0]*35
    values[0]=0x42475734;values[3]=values[4]=10;values[7]=500
    values[8:10]=[100,85];values[16]=150;values[20]=850
    values[28]=150;values[34]=1
    decoded=decode_motion_status(STATUS.pack(*values))
    assert decoded['tag']=='4WGB' and decoded['body_milli'][0]==150
    values[28]=151
    with pytest.raises(ProtocolError):decode_motion_status(STATUS.pack(*values))

def test_recovery_bridge_requires_exact_grounda_600ms_profile():
    args=['--ros-args','-p','bounded_motion_mode:=true',
          '-p','required_motion_tag:=4WGA','-p','wire_timeout_ms:=600',
          '-p','max_vx_mps:=0.1','-p','max_vy_mps:=0.1',
          '-p','max_wz_radps:=0.2']
    rclpy.init(args=args)
    node=CmdVelUdpBridge()
    try:
        assert node.required_motion_tag=='4WGA' and node.wire_timeout_ms==600
    finally:
        node.destroy_node();rclpy.shutdown()

    for wrong_timeout in (150, 750):
        rclpy.init(args=[*args[:5], f'-p', f'wire_timeout_ms:={wrong_timeout}', *args[7:]])
        try:
            with pytest.raises(ValueError):CmdVelUdpBridge()
        finally:
            rclpy.shutdown()

def test_faster_recovery_bridge_requires_exact_groundb_600ms_profile():
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true',
                     '-p','required_motion_tag:=4WGB','-p','wire_timeout_ms:=600',
                     '-p','max_vx_mps:=0.15','-p','max_vy_mps:=0.15',
                     '-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge()
    try:
        assert node.required_motion_tag=='4WGB'
        assert node.wire_timeout_ms==600 and node.max_vx_mps==0.15
    finally:
        node.destroy_node();rclpy.shutdown()

def test_200mmps_bridge_accepts_exact_ground9_identity():
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true',
                     '-p','required_motion_tag:=4WG9','-p','max_vx_mps:=0.2',
                     '-p','max_vy_mps:=0.2','-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge()
    try:
        assert node.required_motion_tag=='4WG9' and node.max_vx_mps==0.2
    finally:
        node.destroy_node();rclpy.shutdown()

def test_safe_rearm_uses_disabled_zero_primer_before_control_frame():
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true',
                     '-p','required_motion_tag:=4WG9','-p','max_vx_mps:=0.2',
                     '-p','max_vy_mps:=0.2','-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge()
    class Sink:
        def __init__(self):self.frames=[]
        def sendto(self,raw,_peer):self.frames.append(decode_frame(raw));return len(raw)
        def close(self):pass
    try:
        node.sock=Sink();node.destination=('192.0.2.1',46000)
        node.session.connected=True;node.session.session_id=7
        node.pending_control_action='clear_faults';node.pending_control_repeats=1
        node.enable_prime_started=time.monotonic()
        node.send_pending_control_action()
        assert node.sock.frames[-1].message_type==MessageType.CMD_VEL
        assert not node.sock.frames[-1].flags&FrameFlags.ENABLED
        assert unpack_cmd_vel(node.sock.frames[-1].payload).vx_mps==0.0

        node.enable_prime_started=time.monotonic()-0.7
        node.last_motion_status={
            'tag':'4WG9','stationary_ms':500,'permitted':0,
            'compares':[0]*8,'raw_duty':[0]*4,'target_cps':[0]*4,
        }
        node.last_motion_status_monotonic=time.monotonic()
        node.last_mcu_fault_bits=0
        node.send_pending_control_action()
        assert node.sock.frames[-1].message_type==MessageType.CONTROL_ENABLE
        control=unpack_control_enable(node.sock.frames[-1].payload)
        assert not control.enabled and control.clear_latched_faults
        assert node.enable_prime_complete
        assert node.pending_control_action is None and not node.motion_gate_requested
        node.send_command()
        assert node.sock.frames[-1].message_type==MessageType.CMD_VEL
        assert not node.sock.frames[-1].flags&FrameFlags.ENABLED
        assert unpack_cmd_vel(node.sock.frames[-1].payload).vx_mps==0.0
    finally:
        node.destroy_node();rclpy.shutdown()


def test_safe_rearm_request_is_a_single_confirmed_transaction():
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true',
                     '-p','required_motion_tag:=4WG9','-p','max_vx_mps:=0.2',
                     '-p','max_vy_mps:=0.2','-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge()
    try:
        node.on_clear_faults(Bool(data=True))
        assert node.pending_control_action=='clear_faults'
        assert node.pending_control_repeats==1
        assert not node.motion_gate_requested
    finally:
        node.destroy_node();rclpy.shutdown()

def test_200mmps_bridge_requires_exact_ground8_identity():
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true',
                     '-p','required_motion_tag:=4WG8','-p','max_vx_mps:=0.2',
                     '-p','max_vy_mps:=0.2','-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge()
    try:
        assert node.required_motion_tag=='4WG8' and node.max_vx_mps==0.2
    finally:
        node.destroy_node();rclpy.shutdown()

def test_legacy_ground_identity_cannot_open_200mmps_bridge():
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true',
                     '-p','required_motion_tag:=4WG7','-p','max_vx_mps:=0.2',
                     '-p','max_vy_mps:=0.2','-p','max_wz_radps:=0.2'])
    try:
        with pytest.raises(ValueError):CmdVelUdpBridge()
    finally:
        rclpy.shutdown()

def test_ros_safety_true_cannot_emit_enabled_frame_without_fresh_mcu_permit():
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true','-p','max_vx_mps:=0.05',
                     '-p','max_vy_mps:=0.05','-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge()
    class Sink:
        def __init__(self):self.frames=[]
        def sendto(self,raw,_peer):self.frames.append(decode_frame(raw));return len(raw)
        def close(self):pass
    try:
        node.sock=Sink();node.session.connected=True;node.session.session_id=7
        command=Twist();command.linear.x=.03
        node.on_cmd(command);node.on_enable(Bool(data=True))
        node.send_command()
        frame=node.sock.frames[-1]
        assert not frame.flags&FrameFlags.ENABLED
        assert unpack_cmd_vel(frame.payload).vx_mps==0
    finally:
        node.destroy_node();rclpy.shutdown()

def test_post_enable_wait_sends_nothing_until_fresh_permit():
    """A disabled zero after CONTROL_ENABLE would revoke the new MCU gate."""
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true','-p','max_vx_mps:=0.05',
                     '-p','max_vy_mps:=0.05','-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge()
    class Sink:
        def __init__(self):self.frames=[]
        def sendto(self,raw,_peer):self.frames.append(decode_frame(raw));return len(raw)
        def close(self):pass
    try:
        node.sock=Sink();node.session.connected=True;node.session.session_id=7
        node.enable_prime_complete=True
        node.motion_gate_requested=True
        node.last_motion_status={'permitted': 0}
        node.last_motion_status_monotonic=__import__('time').monotonic()
        sequence=node.session.sequence
        node.send_command()
        assert not node.sock.frames
        assert node.session.sequence==sequence

        node.last_motion_status={'permitted': 1}
        node.last_motion_status_monotonic=__import__('time').monotonic()
        node.send_command()
        assert len(node.sock.frames)==1
        frame=node.sock.frames[0]
        assert frame.flags&FrameFlags.ENABLED
        command=unpack_cmd_vel(frame.payload)
        assert (command.vx_mps,command.vy_mps,command.wz_radps)==(0.0,0.0,0.0)
    finally:
        node.destroy_node();rclpy.shutdown()

def test_completed_enable_uses_only_zero_until_mcu_permit_is_confirmed():
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true','-p','max_vx_mps:=0.05',
                     '-p','max_vy_mps:=0.05','-p','max_wz_radps:=0.2'])
    node=CmdVelUdpBridge()
    class Sink:
        def __init__(self):self.frames=[]
        def sendto(self,raw,_peer):self.frames.append(decode_frame(raw));return len(raw)
        def close(self):pass
    try:
        node.sock=Sink();node.session.connected=True;node.session.session_id=7
        node.motion_gate_requested=True
        node.enable_prime_complete=True
        node.enable_command_complete=True
        node.last_motion_status={'permitted': 0}
        node.last_motion_status_monotonic=__import__('time').monotonic()
        command=Twist();command.linear.x=.05
        node.on_cmd(command);node.on_enable(Bool(data=True))
        node.send_command()
        frame=node.sock.frames[-1]
        assert frame.flags&FrameFlags.ENABLED
        velocity=unpack_cmd_vel(frame.payload)
        assert (velocity.vx_mps,velocity.vy_mps,velocity.wz_radps)==(0.0,0.0,0.0)
    finally:
        node.destroy_node();rclpy.shutdown()

def test_installed_chassis_rotates_ros_translation_by_pi_but_preserves_yaw():
    rclpy.init(args=['--ros-args','-p','bounded_motion_mode:=true',
                     '-p','required_motion_tag:=4WG8','-p','max_vx_mps:=0.2',
                     '-p','max_vy_mps:=0.2','-p','max_wz_radps:=0.2',
                     '-p','invert_body_xy:=true'])
    node=CmdVelUdpBridge()
    class Sink:
        def __init__(self):self.frames=[]
        def sendto(self,raw,_peer):self.frames.append(decode_frame(raw));return len(raw)
        def close(self):pass
    try:
        node.sock=Sink();node.session.connected=True;node.session.session_id=7
        node.motion_gate_requested=True
        node.enable_prime_complete=True
        node.enable_command_complete=True
        node.last_motion_status={'permitted': 1}
        node.last_motion_status_monotonic=__import__('time').monotonic()
        command=Twist();command.linear.x=.03;command.linear.y=-.02;command.angular.z=.1
        node.on_cmd(command);node.on_enable(Bool(data=True))
        node.send_command()
        frame=node.sock.frames[-1]
        assert frame.flags&FrameFlags.ENABLED
        velocity=unpack_cmd_vel(frame.payload)
        assert velocity.vx_mps==pytest.approx(-.03)
        assert velocity.vy_mps==pytest.approx(.02)
        assert velocity.wz_radps==pytest.approx(.1)
    finally:
        node.destroy_node();rclpy.shutdown()
