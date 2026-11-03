"""Isolated passive raw Vicon reader. No motor socket or control publisher in domain0."""
from dataclasses import asdict
import multiprocessing as mp
import queue
import time
import math
import copy


def latest(channel, value):
    try:
        channel.put_nowait(value)
    except queue.Full:
        try: channel.get_nowait()
        except queue.Empty: return
        try: channel.put_nowait(value)
        except queue.Full: pass


def passive_child(channel, stop, ready):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from .rotation_nodes import RotationRawSource
    from mecanum_wand_4wsc_control.safety_nodes import decode_pose
    context = Context()
    node = executor = None
    try:
        rclpy.init(context=context, domain_id=0, signal_handler_options=SignalHandlerOptions.NO)
        from .source_pipeline import BufferedRawSource, passive_loop
        node = BufferedRawSource(context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        passive_loop(node, executor, channel, stop, ready)
    except Exception as error:
        latest(channel, dict(kind='error', error=repr(error)))
    finally:
        if executor is not None: executor.shutdown()
        if node is not None: node.destroy_node()
        if context.ok(): context.shutdown()
        channel.cancel_join_thread()


class PassiveReader:
    def __init__(self):
        ctx = mp.get_context('spawn')
        self.channel = ctx.Queue(maxsize=1)
        self.stop, self.ready = ctx.Event(), ctx.Event()
        self.process = ctx.Process(target=passive_child, args=(self.channel,self.stop,self.ready),
                                   name='4wsc-passive-vicon', daemon=True)
        self.value = None
        self.closed = False
        self.ipc_rejected = 0
        self.discarded_ipc = 0

    def start(self):
        self.process.start()
        try:
            first = self.channel.get(timeout=8.)
            if first.get('kind') != 'ready': raise ValueError('passive child failed: '+repr(first))
            self.provenance = first['provenance']
            self.ready.set()
        except Exception:
            self.close()
            raise

    def poll(self):
        from .source_pipeline import poll_latest
        return poll_latest(self)

    def close(self):
        if self.closed: return
        self.closed = True
        self.stop.set(); self.ready.set()
        if self.process.pid:
            self.process.join(timeout=2.)
            if self.process.is_alive():
                self.process.terminate(); self.process.join(timeout=2.)
        self.channel.close(); self.channel.cancel_join_thread()


from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from .commissioning import runtime_from_environment
from .bound_rotation import HistoryQosMixin
from mecanum_wand_4wsc_control.cached_packet import decode_packet
from mecanum_wand_4wsc_control.rotation_source import canonical
from mecanum_wand_4wsc_control.safety_nodes import SENSOR, LATEST, pose_message
from mecanum_wand_4wsc_control.core import Pose


class SourceRelay(HistoryQosMixin, Node):
    def __init__(self, runtime=None, reader=None):
        self.runtime = runtime or runtime_from_environment()
        super().__init__('wand_sign_source', namespace=self.runtime.namespace, enable_rosout=False)
        self.runtime.guard(self)
        self.reader = reader or PassiveReader()
        self.reader.start()
        self.packet_pub = self.create_publisher(String, 'rotation_packet', LATEST)
        self.car_pub = self.create_publisher(PoseStamped, 'car_pose', SENSOR)
        self.wand_pub = self.create_publisher(PoseStamped, 'wand_pose', SENSOR)
        self.mode_pub = self.create_publisher(String, 'mode_claim', LATEST)
        self.info = self.create_publisher(String, 'source_status', LATEST)
        self.last_packet = self.last_car = None
        self.cached_wire = self.cached_packet = None
        self.rejected = 0
        self.last_status, self.status_at = None, -math.inf
        self.create_timer(.005, self.tick)

    def header(self, value):
        stamp = self.get_clock().now().to_msg()
        stamp.sec = int(value); stamp.nanosec = round((value-stamp.sec)*1e9)
        if stamp.nanosec == 1_000_000_000: stamp.sec += 1; stamp.nanosec = 0
        return stamp

    def tick(self):
        # Poll BEFORE sampling clocks. Source/receipt stamps are never renewed.
        conflicts = ['passive source unavailable']
        errors = {}
        pipeline = {}
        validated_cached_reuse = False
        try:
            value = self.reader.poll()
            now, ros = time.monotonic(), self.get_clock().now().nanoseconds*1e-9
            if value:
                conflicts, errors = value['conflicts'], dict(value['rejected'])
                pipeline = dict(value.get('pipeline', {}))
                if not conflicts:
                    # Car tracking is independent of Sign geometry. Preserve
                    # fresh car feedback for STOP confirmation even when the
                    # wand packet is malformed; no permission is granted here.
                    car = value['car']
                    if car and -.005 <= ros-car['stamp'] <= .20 and (self.last_car is None or car['stamp'] > self.last_car):
                        q = Pose(**car['pose'])
                        q = Pose(q.x,q.y,q.z,math.atan2(math.sin(q.yaw+self.runtime.yaw_offset),math.cos(q.yaw+self.runtime.yaw_offset)))
                        self.car_pub.publish(pose_message(q,self.header(car['stamp']),car['roll'],car['pitch'],'map'))
                        self.last_car = car['stamp']
                    p = None
                    if value['packet']:
                        wire = canonical(value['packet'])
                        if wire != self.cached_wire:
                            p = decode_packet(value['packet'], now, ros)
                            self.cached_wire,self.cached_packet = wire,p
                        else:
                            p = self.cached_packet
                            validated_cached_reuse = True
                            p.pose(now,ros)  # ALL original age checks on EVERY reuse.
                    if p is not None and (self.last_packet is None or p.stamp > self.last_packet):
                        self.packet_pub.publish(String(data=canonical(p.payload())))
                        self.wand_pub.publish(pose_message(p.pose(now,ros), self.header(p.stamp), world='map'))
                        self.last_packet = p.stamp
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            if str(error) != 'SIGN_PACKET_AGE_OR_MIXED_FRAME' or not validated_cached_reuse:
                self.rejected += 1
            else:
                # A once-validated immutable relay cache merely expired.
                # Do not manufacture a geometry rejection; unchanged input
                # watchdogs still stop if no fresh source arrives in 200ms.
                pipeline['cached_packet_expired'] = True
            now, ros = time.monotonic(), self.get_clock().now().nanoseconds*1e-9
        errors['ipc_or_geometry'] = self.rejected
        status = (tuple(conflicts), tuple(sorted(errors.items())))
        if status != self.last_status or now-self.status_at >= .05:
            self.mode_pub.publish(String(data='conflict' if conflicts else 'wand'))
            self.info.publish(String(data=canonical(dict(issued_steady=now,issued_ros=ros,
                rejected=errors,conflicts=conflicts,passive_process=True,pipeline=pipeline))))
            self.last_status, self.status_at = status, now

    def destroy_node(self):
        self.reader.close()
        return super().destroy_node()
