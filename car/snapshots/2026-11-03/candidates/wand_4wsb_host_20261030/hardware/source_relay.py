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
    from mecanum_wand_4wsb_control.safety_nodes import decode_pose
    context = Context()
    node = executor = None
    try:
        rclpy.init(context=context, domain_id=0, signal_handler_options=SignalHandlerOptions.NO)
        node = RotationRawSource(context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        channel.put(dict(kind='ready', provenance=node.provenance), timeout=1.)
        if not ready.wait(8.): raise ValueError('SOURCE_PARENT_HANDSHAKE_TIMEOUT')
        last = audit_at = 0.
        conflicts = ['discovery pending']
        discovered = time.monotonic()+2.
        while not stop.is_set():
            executor.spin_once(timeout_sec=.001)
            now = time.monotonic()
            if now-audit_at >= .10:
                conflicts = node.audit()
                if now < discovered: conflicts.append('discovery pending')
                audit_at = now
            if now-last < .005: continue
            car = None
            msg = node.samples.get('car')
            if msg is not None:
                pose, stamp, roll, pitch = decode_pose(msg, 'map')
                car = dict(pose=asdict(pose), stamp=stamp, roll=roll, pitch=pitch)
            latest(channel, dict(kind='data', issued=now,
                packet=node.packet.payload() if node.packet is not None else None,
                car=car, conflicts=conflicts, rejected=dict(node.rejected)))
            last = now
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
                                   name='4wsb-passive-vicon', daemon=True)
        self.value = None
        self.closed = False
        self.ipc_rejected = 0

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
        for _ in range(2):
            try: value = self.channel.get_nowait()
            except queue.Empty: break
            if value.get('kind') != 'data':
                self.ipc_rejected += 1
                self.value = None
                raise ValueError('SOURCE_IPC_ERROR '+repr(value))
            if not 0 <= time.monotonic()-value['issued'] <= .20:
                self.ipc_rejected += 1
                self.value = None
                raise ValueError('SOURCE_IPC_AGE')
            self.value = value
        if not self.process.is_alive() or self.value is None: return None
        if not 0 <= time.monotonic()-self.value['issued'] <= .20: return None
        return self.value

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
from mecanum_wand_4wsb_control.cached_packet import decode_packet
from mecanum_wand_4wsb_control.rotation_source import canonical
from mecanum_wand_4wsb_control.safety_nodes import SENSOR, LATEST, pose_message
from mecanum_wand_4wsb_control.core import Pose


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
        try:
            value = self.reader.poll()
            now, ros = time.monotonic(), self.get_clock().now().nanoseconds*1e-9
            if value:
                conflicts, errors = value['conflicts'], dict(value['rejected'])
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
                            p.pose(now,ros)  # ALL original age checks on EVERY reuse.
                    if p is not None and (self.last_packet is None or p.stamp > self.last_packet):
                        self.packet_pub.publish(String(data=canonical(p.payload())))
                        self.wand_pub.publish(pose_message(p.pose(now,ros), self.header(p.stamp), world='map'))
                        self.last_packet = p.stamp
        except (ValueError, TypeError, KeyError, AttributeError):
            self.rejected += 1
            now, ros = time.monotonic(), self.get_clock().now().nanoseconds*1e-9
        errors['ipc_or_geometry'] = self.rejected
        status = (tuple(conflicts), tuple(sorted(errors.items())))
        if status != self.last_status or now-self.status_at >= .05:
            self.mode_pub.publish(String(data='conflict' if conflicts else 'wand'))
            self.info.publish(String(data=canonical(dict(issued_steady=now,issued_ros=ros,
                rejected=errors,conflicts=conflicts,passive_process=True))))
            self.last_status, self.status_at = status, now

    def destroy_node(self):
        self.reader.close()
        return super().destroy_node()
