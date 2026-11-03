"""Software-only actuator: exact C controller/protocol plus synthetic encoders.

This is NOT a motor identification or a physical acceleration/braking model.
No UDP/serial/programmer is constructed. The historical oracle's uncalibrated
PWM->speed plant is retained, not adjusted to make this host look better.
"""
import ctypes
import math
from pathlib import Path
import time

from mecanum_wand_4wsb_hardware.stage0_session import Stage0Session
from mecanum_wand_4wsb_hardware.stage0_contract import decode_status_frame
from mecanum_vicon_control.transport_protocol import decode_frame, unpack_wheel_telemetry
from mecanum_vicon_control.mecanum_kinematics import WheelSpeeds, wheel_angular_to_body


class NativeController:
    def __init__(self, library, stage):
        self.lib = ctypes.CDLL(str(library))
        self.lib.oracle_request.argtypes = [ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint32,ctypes.c_void_p]
        self.lib.oracle_request.restype = ctypes.c_size_t
        self.lib.oracle_tick.argtypes = [ctypes.c_uint32]
        self.lib.oracle_packet.argtypes = [ctypes.c_uint,ctypes.c_void_p]
        self.lib.oracle_packet.restype = ctypes.c_size_t
        self.lib.oracle_init()
        self.session = Stage0Session(stage=stage)
        self.now = 0
        self.pose = [0.,0.,0.]
        self.received = self.transmitted = 0
        self.enabled = False
        self.request(self.session.new_hello(0))
        for t in range(10,601,10):
            self.now = t
            self.velocity((0.,0.,0.),False)
            self.lib.oracle_tick(t)
        self.request(self.session.select_mode(600))
        self.snapshot()

    def request(self, raw):
        src = ctypes.create_string_buffer(raw); dst = ctypes.create_string_buffer(512)
        n = self.lib.oracle_request(src,len(raw),self.now,dst)
        self.transmitted += 1
        if n: self.session.accept(dst.raw[:n])

    def velocity(self, command, enabled):
        if enabled != self.enabled:
            self.request(self.session.control_enable(enabled,False,self.now))
            self.enabled = enabled
        self.request(self.session.cmd_vel(*command,600,enabled,self.now))

    def packet(self, kind):
        out = ctypes.create_string_buffer(512)
        n = self.lib.oracle_packet(kind,out)
        if not n: raise ValueError('C oracle failed to produce telemetry')
        self.received += 1
        return out.raw[:n]

    def snapshot(self):
        self.status = decode_status_frame(self.packet(0x31))
        self.wheels = unpack_wheel_telemetry(decode_frame(self.packet(0x30)).payload)
        return self.status, self.wheels

    def step(self, now):
        # All C controller updates remain 10ms. Wall callbacks may be late;
        # do not feed a fabricated large dt into the real firmware algorithm.
        now = max(self.now,int(now))
        while self.now+10 <= now:
            self.now += 10
            self.lib.oracle_tick(self.now)
            self.snapshot()
            v = self.body_velocity()
            x,y,w = v; heading = self.pose[2]
            self.pose[0] += (math.cos(heading)*x-math.sin(heading)*y)*.01
            self.pose[1] += (math.sin(heading)*x+math.cos(heading)*y)*.01
            self.pose[2] += w*.01
        return self.status, self.wheels

    def body_velocity(self):
        speeds = WheelSpeeds(*(v*2*math.pi/60 for v in self.wheels.wheel_rpm))
        return wheel_angular_to_body(speeds,.03,.061,.085)


from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mecanum_wand_4wsb_control.core import Pose
from mecanum_wand_4wsb_control.safety_core import ZERO
from mecanum_wand_4wsb_control.safety_nodes import LATEST, pose_message
from mecanum_wand_4wsb_control.bound_history import HISTORY_QOS
from mecanum_wand_4wsb_control.rotation_source import canonical
from mecanum_wand_4wsb_hardware.commissioning import TransportPolicy


class NativeTwin(Node):
    def __init__(self, runtime, library, key, publish_car=True):
        if not getattr(runtime,'software_only',False):
            raise ValueError('native twin requires explicit software-only runtime')
        super().__init__('safe_native_twin',namespace=runtime.namespace,enable_rosout=False)
        runtime.guard(self)
        self.runtime = runtime
        self.policy = TransportPolicy(key,runtime.config.stage)
        self.controller = NativeController(library,runtime.config.stage)
        self.wall_start = time.monotonic()
        self.last_wire = -math.inf
        self.accepted = self.sequence = 0
        self.enabled_on_wire = self.output_nonzero = False
        self.feedback_on = self.pose_on = self.status_on = True
        self.actuator_fault = 0
        self.first_error = ''
        self.last_signed_at = -math.inf
        self.metrics = dict(peak_command_mps=0.,peak_measured_mps=0.,peak_rpm=0.,peak_duty=0,
                            reversing_samples=0,stage_specific_guard=True)
        self.pose_pub = self.create_publisher(PoseStamped,'car_pose',HISTORY_QOS) if publish_car else None
        self.feedback_pub = self.create_publisher(String,'plant_feedback',LATEST)
        self.status_pub = self.create_publisher(String,'transport_status',LATEST)
        self.create_subscription(String,'actuator_command',self.on_signed,LATEST)
        self.create_timer(.01,self.tick)

    def on_signed(self,msg):
        now = time.monotonic(); ros = self.get_clock().now().nanoseconds*1e-9
        self.policy.receive(msg.data,now,ros)
        if self.policy.fault == 'SIGNED_AUTHORITY_INVALID':
            self.first_error = self.first_error or self.policy.fault
        else:
            # A valid, authenticated ESTOP correctly hard-latches policy; it
            # is not a signature/decoder failure.
            self.accepted += 1
        self.last_signed_at = now

    def tick(self):
        now = time.monotonic(); ros = self.get_clock().now().nanoseconds*1e-9
        c = self.controller
        target_tick = 600+int((now-self.wall_start)*1000)
        # Old command may only reach its ORIGINAL wire lease on delayed ticks.
        # Advance the MCU before applying a newly received command.
        c.step(target_tick)
        active = self.policy.active(now) and not self.actuator_fault and not c.wheels.fault_bits
        cmd = self.policy.command if active else ZERO
        if now-self.last_wire >= .019:
            c.velocity((cmd.vx,cmd.vy,cmd.wz),bool(active))
            self.last_wire = now
            # Production ISR: process request -> control tick -> snapshot.
            # A tiny all-normalized-zero retarget has a legitimate transient
            # between request and tick. NEVER sample there; keep the last
            # completed sample until step() executes the next C control tick.
        status,wheels = c.status,c.wheels
        self.enabled_on_wire = status['permitted']
        self.output_nonzero = bool(any(status['raw_duty']))
        speed = max(abs(v) for v in wheels.wheel_rpm)*2*math.pi*.03/60
        vx,vy,wz = c.body_velocity()
        m = self.metrics
        m['peak_command_mps'] = max(m['peak_command_mps'],math.hypot(cmd.vx,cmd.vy))
        m['peak_measured_mps'] = max(m['peak_measured_mps'],math.hypot(vx,vy))
        m['peak_rpm'] = max(m['peak_rpm'],max(map(abs,wheels.wheel_rpm)))
        m['peak_duty'] = max(m['peak_duty'],max(map(abs,status['raw_duty'])))
        m['reversing_samples'] += int(bool(status['reversing_mask']))
        m['stage_specific_guard'] &= status['wheel_target_slew_guard_cps_s'] == self.runtime.config.limits.wheel_slew_guard
        self.sequence += 1
        if self.pose_on and self.pose_pub is not None:
            p = Pose(*c.pose[:2],self.runtime.config.ground_z_m,math.atan2(math.sin(c.pose[2]),math.cos(c.pose[2])))
            self.pose_pub.publish(pose_message(p,self.get_clock().now().to_msg(),world='map'))
        if self.feedback_on:
            zero = not status['permitted'] and not any(status['raw_duty']+status['target_cps']+status['body_milli'])
            self.feedback_pub.publish(String(data=canonical(dict(sequence=self.sequence,issued_steady=now,
                issued_ros=ros,output_zero=bool(zero),enabled=bool(status['permitted']),
                speed=speed,yaw_rate=speed/.146,fault_bits=wheels.fault_bits|self.actuator_fault))))
        if self.status_on:
            self.status_pub.publish(String(data=canonical(dict(issued_steady=now,issued_ros=ros,
                fresh=True,ready=not status['permitted'] and not wheels.fault_bits,
                fault=self.policy.fault,local_stop=False,armed_epoch=self.policy.epoch,
                baseline_current=True,safety_enabled=status['permitted'],motion_gate_requested=active,
                command_progress_age_s=0.,startup_clock=dict(ready=True,qualified=True,fault=''),
                software_native_controller=True))))

    def close(self):
        self.policy.revoke('SOFTWARE_TEST_END')
        self.controller.velocity((0.,0.,0.),False)
