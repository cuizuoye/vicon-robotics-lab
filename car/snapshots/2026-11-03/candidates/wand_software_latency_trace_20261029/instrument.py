"""Process-local instrumentation of the PINNED software-only ROS chain.

Never imported by the original console. Same wire schema, queues, parameters,
source checks, signing/verifying, watchdogs and model. IDs are a sidecar only.
"""
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

PARENT = Path('/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_fast_follow_ros_sim_20261029')
sys.path.insert(0, str(PARENT))
import ros_chain as original
from bootstrap import ZERO
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from trace_log import PublishTap, control_id, output_id

LOG = None
BaseSupervisor, BaseGate = original.Supervisor, original.Gate
BasePlant, BaseSource, BaseChain = original.SoftwarePlant, original.Source, original.Chain


class Supervisor(BaseSupervisor):
    last_control = None
    tick_start = 0

    def tick(self):
        self.tick_start = time.monotonic_ns()
        return super().tick()

    def create_publisher(self, msg_type, topic, *args, **kwargs):
        if topic != 'candidate':
            return super().create_publisher(msg_type, topic, *args, **kwargs)
        # Exact accepted BoundPublisher construction, plus passive sidecar.
        pub = Node.create_publisher(self, msg_type, topic, original.HISTORY)
        def publish(msg):
            frame = json.loads(msg.data)
            packet = self.selected
            car = self.core.inputs.poses.get('car')
            value = dict(frame=frame, sign_digest=packet.digest if packet else None,
                         sign_stamp=packet.stamp if packet else None,
                         car_stamp=car['source_ros'] if car else None)
            value = getattr(self, 'modify_bound', lambda v: v)(value)
            wire = String(data=original.canonical(value))
            begin = time.monotonic_ns()
            pub.publish(wire)
            end = time.monotonic_ns()
            self.last_control = control_id(frame)
            LOG.emit('control_publish', cid=self.last_control, sign=value['sign_digest'],
                     source_stamp=value['sign_stamp'], issued_ns=round(frame['issued_steady']*1e9),
                     tick_ns=self.tick_start, begin_ns=begin, end_ns=end,
                     state=frame['state'], enabled=frame['enabled'], command=frame['command'])
        return PublishTap(pub, publish)


class Gate(BaseGate):
    def __init__(self, runtime):
        super().__init__(runtime)
        self.attach_core()
        signer = self.authority
        gate = self
        class Signer:
            def encode(self, *args, **kwargs):
                begin = time.monotonic_ns()
                cid = control_id(gate.core.frame)
                text = signer.encode(*args, **kwargs)
                end = time.monotonic_ns()
                payload = json.loads(text)['payload']
                LOG.emit('sign_encode', oid=output_id(payload), cid=cid,
                         begin_ns=begin, end_ns=end, permit=payload['permit'],
                         state=payload['state'], fault=gate.core.fault)
                return text
        self.authority = Signer()

    def attach_core(self):
        if getattr(self, '_traced_core', None) is self.core:
            return
        core = self.core
        receive = core.receive
        def traced_receive(frame, now, ros):
            begin = time.monotonic_ns()
            accepted = receive(frame, now, ros)
            end = time.monotonic_ns()
            LOG.emit('gate_accept', cid=control_id(frame), begin_ns=begin,
                     end_ns=end, accepted=bool(accepted), fault=core.fault)
            return accepted
        core.receive = traced_receive
        self._traced_core = core

    def on_candidate(self, msg):
        begin = time.monotonic_ns()
        self.attach_core()
        try:
            cid = control_id(json.loads(msg.data)['frame'])
        except (ValueError, KeyError, TypeError):
            cid = None
        LOG.emit('gate_receive', cid=cid, begin_ns=begin)
        return super().on_candidate(msg)

    def process_pending(self):
        self.attach_core()
        return super().process_pending()

    def create_publisher(self, msg_type, topic, *args, **kwargs):
        pub = super().create_publisher(msg_type, topic, *args, **kwargs)
        if topic != 'actuator_command':
            return pub
        def publish(msg):
            payload = json.loads(msg.data)['payload']
            begin = time.monotonic_ns()
            pub.publish(msg)
            end = time.monotonic_ns()
            LOG.emit('signed_publish', oid=output_id(payload), cid=control_id(self.core.frame),
                     begin_ns=begin, end_ns=end, state=payload['state'],
                     permit=payload['permit'], command=payload['command'])
        return PublishTap(pub, publish)


class Plant(BasePlant):
    last_output = None
    model_meta = None

    def __init__(self, runtime):
        super().__init__(runtime)
        verifier = self.verifier
        plant = self
        class Verifier:
            def decode(self, *args, **kwargs):
                begin = time.monotonic_ns()
                try:
                    payload, command = verifier.decode(*args, **kwargs)
                except Exception as error:
                    LOG.emit('verify_rejected', begin_ns=begin, end_ns=time.monotonic_ns(),
                             reason=str(error))
                    raise
                end = time.monotonic_ns()
                applied = not plant.fault_bits
                if applied:
                    plant.last_output = output_id(payload)
                LOG.emit('verify_accept', oid=output_id(payload), begin_ns=begin,
                         end_ns=end, applied=applied, state=payload['state'])
                return payload, command
        self.verifier = Verifier()

    def on_command(self, msg):
        begin = time.monotonic_ns()
        try:
            oid = output_id(json.loads(msg.data)['payload'])
        except (ValueError, TypeError, KeyError):
            oid = None
        LOG.emit('plant_receive', oid=oid, begin_ns=begin)
        return super().on_command(msg)

    def tick(self):
        # Unchanged SoftwarePlant.tick, with state/command IDs at integration.
        begin = time.monotonic_ns()
        now = time.monotonic()
        dt = min(.10, max(0., now-self.last_tick)); self.last_tick = now
        valid = 0 <= now-self.last_command <= .10 and not self.fault_bits
        command = self.command if valid else ZERO
        oid = self.last_output if valid else None
        self.plant.step(command, dt)
        end = time.monotonic_ns()
        self.sequence += 1
        self.model_meta = dict(model_seq=self.sequence, oid=oid, begin_ns=begin, end_ns=end)
        LOG.emit('model_step', **self.model_meta, command=asdict(command),
                 car=asdict(self.plant.pose), velocity=asdict(self.plant.velocity))
        stamp = self.get_clock().now()
        if self.pose_on:
            self.pose_pub.publish(original.pose_message(self.plant.pose, stamp.to_msg(), world='map'))
        if self.feedback_on:
            self.feedback_pub.publish(String(data=original.canonical(dict(sequence=self.sequence,
                issued_steady=now, issued_ros=stamp.nanoseconds*1e-9,
                output_zero=command == ZERO, enabled=command != ZERO,
                speed=math.hypot(self.plant.velocity.vx, self.plant.velocity.vy),
                yaw_rate=abs(self.plant.velocity.wz), fault_bits=self.fault_bits))))


class Source(BaseSource):
    def create_publisher(self, msg_type, topic, *args, **kwargs):
        pub = super().create_publisher(msg_type, topic, *args, **kwargs)
        if topic != 'rotation_packet':
            return pub
        def publish(msg):
            packet = self.last_packet
            # Clock pair for local PC source stamp, not Vicon exposure time.
            before = time.monotonic_ns()
            ros_ns = self.get_clock().now().nanoseconds
            after = time.monotonic_ns()
            begin = time.monotonic_ns()
            pub.publish(msg)
            end = time.monotonic_ns()
            LOG.emit('source_relay', sign=packet.digest, source_stamp=packet.stamp,
                     source_local_ns=round(packet.stamp*1e9)-(ros_ns-(before+after)//2),
                     clock_pair_span_ns=after-before,
                     raw_first_ns=round(min(s.received for s in packet.samples)*1e9),
                     raw_complete_ns=round(max(s.received for s in packet.samples)*1e9),
                     begin_ns=begin, end_ns=end)
        return PublishTap(pub, publish)


class Chain(BaseChain):
    snapshot_sequence = 0

    def snapshot(self):
        begin = time.monotonic_ns()
        result = super().snapshot()
        self.snapshot_sequence += 1
        meta = dict(snapshot_seq=self.snapshot_sequence, cid=self.sup.last_control,
                    model_seq=(self.plant.model_meta or {}).get('model_seq'),
                    oid=(self.plant.model_meta or {}).get('oid'),
                    begin_ns=begin, end_ns=time.monotonic_ns())
        result['trace'] = meta
        LOG.emit('snapshot', **meta, state=result['state'])
        return result


def install(log):
    """Factory substitution affects this independent process ONLY."""
    global LOG
    LOG = log
    original.Supervisor, original.Gate = Supervisor, Gate
    original.SoftwarePlant, original.Source, original.Chain = Plant, Source, Chain
    return original
