"""Separate offline ROS processes. Fixed domain118 and /wand_safe_sim only.

There is deliberately no real Vicon bridge or hardware transport in this
entrypoint. The same-host monotonic envelopes are not a cross-machine protocol.
"""

from dataclasses import asdict, fields
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mecanum_vicon_control.math_utils import wrap_angle
from .core import Pose, Velocity
from .offline_suite import Plant
from .ros_nodes import run_node
from .safety_core import Frame, Lease, SafetyConfig, ZERO
from .rotation_core import RotationFinalGate as FinalGate, RotationSupervisor as Supervisor
from .safety_wire import OutputSigner, OutputVerifier, load_json, finite_number

DOMAIN = 96
NAMESPACE = '/wand_safe_sim'
WORLD = 'wand_safe_map'
TOPICS = ('car_pose', 'wand_pose', 'operator_lease', 'candidate', 'cmd_vel',
          'actuator_command', 'plant_feedback', 'supervisor_status', 'gate_status', 'mode_claim', 'inject')
SERVICES = ('start', 'pause', 'stop', 'acknowledge', 'recover', 'estop')
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)
SENSOR = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE)
EXPECTED = {'car_pose': 'safe_sim_plant', 'wand_pose': 'safe_sim_operator',
            'operator_lease': 'safe_sim_operator', 'plant_feedback': 'safe_sim_plant',
            'candidate': 'safe_wand_supervisor', 'cmd_vel': 'safe_wand_gate',
            'actuator_command': 'safe_wand_gate'}


def guard(node):
    if (os.environ.get('ROS_DOMAIN_ID') != str(DOMAIN)
            or node.context.get_domain_id() != DOMAIN or node.get_namespace() != NAMESPACE):
        raise RuntimeError('safe wand nodes require domain118 and /wand_safe_sim')
    if node.get_parameter('use_sim_time').value:
        raise RuntimeError('safety deadline tests require real steady/system clocks')
    for name in TOPICS:
        if node.resolve_topic_name(name) != NAMESPACE+'/'+name:
            raise RuntimeError('all safe simulation topics must stay in /wand_safe_sim')
    for name in SERVICES:
        if node.resolve_service_name(name) != NAMESPACE+'/'+name:
            raise RuntimeError('all safe simulation services must stay in /wand_safe_sim')


def decode_lease(text):
    value = load_json(text)
    for k in ('issued_steady', 'issued_ros'): value[k] = finite_number(value[k])
    lease = Lease(**value)
    if not lease.valid(): raise ValueError('invalid lease fields')
    return lease


def decode_frame(text):
    value = load_json(text)
    for k in ('issued_steady', 'issued_ros'): value[k] = finite_number(value[k])
    value['command'] = Velocity(**{k: finite_number(v) for k, v in value['command'].items()})
    if value['goal'] is not None:
        value['goal'] = Pose(**{k: finite_number(v) for k, v in value['goal'].items()})
    f = Frame(**value)
    if (not isinstance(f.source, str) or not 0 < len(f.source) <= 80
            or type(f.sequence) is not int or f.sequence < 0
            or type(f.enabled) is not bool or type(f.hard_fault) is not bool
            or not isinstance(f.epoch, str) or len(f.epoch) > 80
            or f.state not in ('STANDBY', 'ARMED', 'ACTIVE', 'PAUSED', 'FAULT', *('STOPPING', 'STOP_UNCONFIRMED'))):
        raise ValueError('invalid command fields')
    return f


def pose_message(pose, stamp, roll=0., pitch=0., world=WORLD):
    msg = PoseStamped()
    msg.header.frame_id, msg.header.stamp = world, stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(pose.x), float(pose.y), float(pose.z)
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(pose.yaw/2), math.sin(pose.yaw/2)
    q = msg.pose.orientation
    q.x, q.y = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy
    q.z, q.w = cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
    return msg


def decode_pose(msg, world=WORLD):
    if msg.header.frame_id != world or not 0 <= msg.header.stamp.nanosec < 1_000_000_000:
        raise ValueError('frame or stamp format mismatch')
    stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    if stamp <= 0: raise ValueError('missing source timestamp')
    p, q = msg.pose.position, msg.pose.orientation
    x, y, z, w = (finite_number(v) for v in (q.x, q.y, q.z, q.w))
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not .99 <= norm <= 1.01: raise ValueError('invalid quaternion')
    x, y, z, w = (v/norm for v in (x, y, z, w))
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return Pose(*(finite_number(v) for v in (p.x, p.y, p.z)), yaw), stamp, roll, pitch


class SafetyNode(Node):
    def __init__(self, name, core_type, runtime=None):
        super().__init__(name, **({'namespace': runtime.namespace} if runtime else {}))
        self.runtime = runtime
        self.world = runtime.world if runtime else WORLD
        (runtime.guard if runtime else guard)(self)
        config_type = type(runtime.config) if runtime else SafetyConfig
        values = {}
        for f in fields(config_type):
            self.declare_parameter(f.name, getattr(runtime.config, f.name) if runtime else f.default)
            values[f.name] = self.get_parameter(f.name).value
        self.core = core_type(config_type(**values))
        if runtime and self.core.c != runtime.config:
            raise RuntimeError('acceptance safety parameters must match the reviewed profile exactly')
        from .pose_quality import CarPoseQuality
        self.car_pose_quality = CarPoseQuality(self.core.c)
        self.first_stop_reason = ""
        self.core.inputs.graph_ok = False
        self.create_timer(.10, self.check_graph)
        for source in ('car', 'wand'):
            self.create_subscription(PoseStamped, source+'_pose',
                lambda msg, source=source: self.on_pose(source, msg), SENSOR)
        self.create_subscription(String, 'operator_lease', self.on_lease, LATEST)
        self.create_subscription(String, 'plant_feedback', self.on_feedback, LATEST)
        self.create_subscription(String, 'mode_claim', self.on_mode, LATEST)
        self.source_errors_seen, self.transport_error_seen = {}, ''
        self.transport_report = None
        if runtime:
            self.create_subscription(String, 'source_status', self.on_source_status, LATEST)
            if runtime.mode != 'shadow':
                self.create_subscription(String, 'transport_status', self.on_transport_status, LATEST)

    def clocks(self):
        return time.monotonic(), self.get_clock().now().nanoseconds*1e-9

    def check_graph(self):
        ok = True
        namespace = self.runtime.namespace if self.runtime else NAMESPACE
        for topic, name in (self.runtime.expected if self.runtime else EXPECTED).items():
            pubs = self.get_publishers_info_by_topic(namespace+'/'+topic)
            ok &= (len(pubs) == 1 and pubs[0].node_name == name
                   and pubs[0].node_namespace == namespace)
        self.core.inputs.graph_ok = bool(ok)

    def on_pose(self, source, msg):
        steady, ros = self.clocks()
        try:
            pose, stamp, roll, pitch = decode_pose(msg, self.world)
            self.core.inputs.observe_pose(source, pose, stamp, steady, ros, roll, pitch)
            if source == "car":
                self.car_pose_quality.observe(pose, stamp, steady, ros, roll, pitch)
        except (ValueError, TypeError):
            self.core.inputs.poses.pop(source, None)
            self.core.inputs.event('POSE_DECODE_INVALID')
            if source == 'car':self.car_pose_quality.reject('POSE_DECODE_INVALID', steady)

    def on_lease(self, msg):
        try:
            self.core.inputs.observe_lease(decode_lease(msg.data), *self.clocks())
        except (ValueError, TypeError, KeyError, AttributeError):
            self.core.inputs.lease = None
            self.core.inputs.event('LEASE_DECODE_INVALID')

    def on_feedback(self, msg):
        steady, ros = self.clocks()
        try:
            value = load_json(msg.data)
            self.core.inputs.observe_feedback(steady=steady, ros=ros, **value)
        except (ValueError, TypeError, KeyError):
            self.core.inputs.feedback = None
            self.core.inputs.event('FEEDBACK_DECODE_INVALID')

    def on_mode(self, msg):
        self.core.inputs.mode = msg.data
        if msg.data != 'wand': self.core.inputs.event('CONTROL_MODE_CONFLICT')

    def on_source_status(self, msg):
        try:
            value = load_json(msg.data)
            from .safety_core import fresh_envelope
            if not fresh_envelope(value['issued_steady'], value['issued_ros'], *self.clocks(), .20):
                raise ValueError('stale source status')
            for key, count in value['rejected'].items():
                if type(count) is not int or count < 0: raise ValueError('invalid rejection count')
                if count > self.source_errors_seen.get(key, 0):
                    self.core.inputs.event('RAW_'+key.upper()+'_INVALID')
                self.source_errors_seen[key] = count
        except (ValueError, TypeError, KeyError, AttributeError):
            self.core.inputs.event('SOURCE_STATUS_INVALID')

    def on_transport_status(self, msg):
        try:
            value = load_json(msg.data)
            from .safety_core import fresh_envelope
            if not fresh_envelope(value['issued_steady'], value['issued_ros'], *self.clocks(), .20):
                raise ValueError('stale transport status')
            self.transport_report = value
            error = value['fault']
            if error and error != self.transport_error_seen:
                self.core.inputs.event('TRANSPORT_'+error)
            self.transport_error_seen = error
        except (ValueError, TypeError, KeyError, AttributeError):
            self.core.inputs.event('TRANSPORT_STATUS_INVALID')


class SupervisorNode(SafetyNode):
    def __init__(self, runtime=None):
        super().__init__('safe_wand_supervisor', Supervisor, runtime)
        self.candidate = self.create_publisher(String, 'candidate', LATEST)
        self.status = self.create_publisher(String, 'supervisor_status', LATEST)
        self.create_subscription(String, 'gate_status', self.on_gate, LATEST)
        for name in ('start', 'pause', 'stop', 'acknowledge', 'recover'):
            self.create_service(Trigger, name, lambda req, res, name=name: self.service(name, res))
        self.create_timer(.02, self.tick)

    def on_gate(self, msg):
        try:
            value = load_json(msg.data)
            if type(value['hard_fault']) is not bool: raise ValueError('invalid gate fault')
            self.core.observe_gate(steady=finite_number(value['issued_steady']),
                ros=finite_number(value['issued_ros']), epoch=value['primed_epoch'],
                fault=value['fault'], hard_fault=value['hard_fault'])
        except (ValueError, TypeError, KeyError):
            self.core.inputs.event('GATE_REPORT_INVALID')

    def service(self, name, response):
        steady, ros = self.clocks()
        if name == 'start':
            response.success, response.message = self.core.request_start(steady, ros)
        elif name == 'acknowledge':
            response.success, response.message = self.core.acknowledge(steady, ros)
        elif name == 'recover':
            response.success, response.message = self.core.request_recover(steady, ros)
        else:
            self.core.request_stop(steady, 'operator '+name)
            self.candidate.publish(String(data=json.dumps(asdict(self.core.frame(steady, ros)))))
            response.success, response.message = True, 'stop requested; wait for stop_confirmed feedback'
        return response

    def tick(self):
        steady, ros = self.clocks()
        f = self.core.tick(steady, ros)
        if self.core.state in ('STOPPING','STOP_UNCONFIRMED','FAULT') and not self.first_stop_reason:
            self.first_stop_reason = self.core.reason
        self.candidate.publish(String(data=json.dumps(asdict(f), allow_nan=False)))
        self.status.publish(String(data=json.dumps(dict(
            issued_steady=steady, issued_ros=ros,
            pid=os.getpid(), state=self.core.state, reason=self.core.reason, epoch=self.core.epoch,
            pilot=self.core.pilot_status(steady), first_stop_reason=self.first_stop_reason,
            car_pose_quality=self.car_pose_quality.snapshot(),
            stop_confirmed=self.core.stop_confirmed, hard_fault=self.core.hard_fault,
            recovery_pending=self.core.recovery_deadline is not None,
            ready_to_start=(self.core.ready_since is not None
                and steady-self.core.ready_since >= self.core.c.ready_window_s),
            graph_ok=self.core.inputs.graph_ok,
            input_health=self.core.inputs.health(steady, ros),
            transport_preparation=(self.transport_report or {}).get('preparation_stage'),
            car_speed_estimate_mps=(self.core.inputs.car_speed if math.isfinite(self.core.inputs.car_speed) else None),
            car_yaw_rate_estimate_radps=(self.core.inputs.car_yaw_rate if math.isfinite(self.core.inputs.car_yaw_rate) else None),
            stationary_since=self.core.inputs.stationary_since,
            pose_source_ages_s={source: ros-value['source_ros'] for source, value in self.core.inputs.poses.items()},
            gate_ready=bool(self.core.gate_fresh(steady, ros) and self.core.gate_report['epoch']
                == self.core.epoch and self.core.epoch and not self.core.gate_report['fault']
                and self.operator_gate_ready(steady, ros)),
            scale=self.core.follower.config.scale, speed_limit_mps=self.core.c.max_speed_mps,
            goal_boundary_policy=self.core.c.goal_boundary_policy,
            goal_reference_policy=self.core.c.goal_reference_policy,
            goal_limited=self.core.follower.goal_limited,
            reference_limited=self.core.follower.reference_limited,
            mapped_goal=asdict(self.core.follower.mapped_goal) if self.core.follower.mapped_goal else None,
            raw_goal=asdict(self.core.follower.raw_goal) if self.core.follower.raw_goal else None,
            anchors=asdict(self.core.follower.mapping) if self.core.follower.mapping else None,
            goal=asdict(self.core.follower.goal) if self.core.follower.goal else None))))

    def operator_gate_ready(self, steady, ros): return True


class GateNode(SafetyNode):
    def __init__(self, runtime=None):
        super().__init__('safe_wand_gate', FinalGate, runtime)
        self.commands = self.create_publisher(Twist, 'cmd_vel', LATEST)
        self.signer = OutputSigner(os.environ.get('WAND_4WS8_CHANNEL_KEY', ''))
        self.actuator = self.create_publisher(String, 'actuator_command', LATEST)
        self.status = self.create_publisher(String, 'gate_status', LATEST)
        self.create_subscription(String, 'candidate', self.on_candidate, LATEST)
        self.create_service(Trigger, 'estop', self.estop)
        self.create_timer(.01, self.tick)

    def publish_output(self, output):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = output.vx, output.vy, output.wz
        self.commands.publish(msg)
        self.actuator.publish(String(data=self.signer.encode(output, *self.clocks())))

    def on_candidate(self, msg):
        try:
            self.core.receive(decode_frame(msg.data), *self.clocks())
        except (ValueError, TypeError, KeyError, AttributeError):
            self.core.trip('COMMAND_DECODE_INVALID')
            self.publish_output(ZERO)

    def estop(self, _, response):
        self.core.trip('ESTOP', hard=True)
        self.publish_output(ZERO)
        response.success, response.message = True, 'offline gate ESTOP latched; not a hardware emergency stop'
        return response

    def tick(self):
        steady, ros = self.clocks()
        output = self.core.tick(steady, ros)
        self.publish_output(output)
        self.status.publish(String(data=json.dumps(dict(
            pid=os.getpid(), issued_steady=steady, issued_ros=ros, fault=self.core.fault,
            hard_fault=self.core.hard_fault, primed_epoch=self.core.primed_epoch,
            output_zero=output == ZERO, graph_ok=self.core.inputs.graph_ok,
            speed_mps=math.hypot(output.vx, output.vy),
            car_pose_quality=self.car_pose_quality.snapshot()))))

    def destroy_node(self):
        if rclpy.ok(): self.publish_output(ZERO)
        return super().destroy_node()


class PlantNode(Node):
    """Raw 3D pose + independent feedback for software tests, with 150 ms watchdog."""
    def __init__(self, runtime=None):
        super().__init__('safe_sim_plant', **({'namespace': runtime.namespace} if runtime else {}))
        self.world = runtime.world if runtime else WORLD
        (runtime.guard if runtime else guard)(self)
        self.plant = Plant(runtime.initial_pose if runtime else Pose(0, 0, .10))
        self.command, self.last_command = ZERO, -math.inf
        self.last_tick = time.monotonic()
        self.sequence = 0
        self.pose_on = self.feedback_on = True
        self.roll = self.pitch = 0.
        self.fault_bits = 0
        self.verifier = OutputVerifier(os.environ.get('WAND_4WS8_CHANNEL_KEY', ''))
        self.output_rejection_logged = False
        self.pose_pub = self.create_publisher(PoseStamped, 'car_pose', LATEST)
        self.feedback_pub = self.create_publisher(String, 'plant_feedback', LATEST)
        self.create_subscription(String, 'actuator_command', self.on_command, LATEST)
        self.create_subscription(String, 'inject', self.inject, LATEST)
        self.create_timer(.01, self.tick)

    def on_command(self, msg):
        try:
            self.command = self.verifier.decode(msg.data, time.monotonic(),
                                                 self.get_clock().now().nanoseconds*1e-9)
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self.fault_bits |= 1 << 30
            self.command = ZERO
            if not self.output_rejection_logged:
                self.get_logger().error('Simulation signed output rejected: '+str(exc))
                self.output_rejection_logged = True
            return
        self.last_command = time.monotonic()

    def inject(self, msg):
        try:
            value = load_json(msg.data)
            allowed = {'pose_on', 'feedback_on', 'roll', 'pitch', 'fault_bits', 'dx', 'dy', 'dz'}
            if set(value)-allowed: raise ValueError('unknown injection')
            for k in ('pose_on', 'feedback_on'):
                if k in value:
                    if type(value[k]) is not bool: raise ValueError('boolean required')
                    setattr(self, k, value[k])
            for k in ('roll', 'pitch'):
                if k in value: setattr(self, k, finite_number(value[k]))
            if 'fault_bits' in value:
                if type(value['fault_bits']) is not int or value['fault_bits'] < 0:
                    raise ValueError('fault bit mask required')
                self.fault_bits |= value['fault_bits']  # No clearing actuator faults via injection.
            p = self.plant.pose
            self.plant.pose = Pose(p.x+finite_number(value.get('dx', 0)),
                p.y+finite_number(value.get('dy', 0)), p.z+finite_number(value.get('dz', 0)), p.yaw)
        except (ValueError, TypeError):
            self.get_logger().warn('invalid simulation-only injection rejected')

    def tick(self):
        now = time.monotonic()
        dt = min(.10, max(0., now-self.last_tick))
        self.last_tick = now
        command = self.command if 0 <= now-self.last_command <= .15 and not self.fault_bits else ZERO
        self.plant.step(command, dt)
        self.sequence += 1
        stamp = self.get_clock().now()
        if self.pose_on:
            self.pose_pub.publish(pose_message(self.plant.pose, stamp.to_msg(), self.roll, self.pitch, self.world))
        if self.feedback_on:
            self.feedback_pub.publish(String(data=json.dumps(dict(
                sequence=self.sequence, issued_steady=now, issued_ros=stamp.nanoseconds*1e-9,
                output_zero=command == ZERO, enabled=command != ZERO,
                speed=math.hypot(self.plant.velocity.vx, self.plant.velocity.vy),
                yaw_rate=abs(self.plant.velocity.wz), fault_bits=self.fault_bits))))


def supervisor_main(): run_node(SupervisorNode)
def gate_main(): run_node(GateNode)
def plant_main(): run_node(PlantNode)
