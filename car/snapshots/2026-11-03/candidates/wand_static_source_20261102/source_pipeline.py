"""Bounded raw-frame intake. Unselected backlog is not motor authority.

No stamp renewal, position-based heartbeat, selected-source substitution or
deadline relaxation. The unchanged supervisor/gate own the 200ms watchdog.
"""
import copy
import math
import queue
import threading
import time
import multiprocessing as mp
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mecanum_wand_4wsc_control.cached_packet import CachedPacket
from mecanum_wand_4wsc_control.rotation_source import Raw
from mecanum_wand_4wsc_control.rotation_geometry import REQUIRED
from mecanum_wand_4wsc_control.safety_nodes import decode_pose
from .rotation_nodes import RotationRawSource

RAW_HISTORY = QoSProfile(depth=32, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
MAX_AGE = .20


class BufferedRawSource(RotationRawSource):
    def __init__(self, context):
        self.callback_generation = 0
        self.discarded_unselected = 0
        self.completed_frames = 0
        super().__init__(context)

    def create_subscription(self, msg_type, topic, callback, qos, *args, **kw):
        # Retain a bounded SAME-FRAME cohort instead of seven independent
        # depth-one mailboxes overwriting different parts of that cohort.
        return super().create_subscription(msg_type, topic, callback, RAW_HISTORY, *args, **kw)

    def receive(self, source, msg):
        self.callback_generation += 1
        try:
            _, stamp, _, _ = decode_pose(msg, 'map')
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if age > MAX_AGE:
                self.discarded_unselected += 1
                return  # Do not erase a newer validated pose for an old DDS item.
        except (ValueError, TypeError, AttributeError):
            pass  # Original strict validation rejects malformed/future input.
        return super().receive(source, msg)

    def marker(self, name, msg):
        self.callback_generation += 1
        return super().marker(name, msg)

    def raw_sample(self, name, p, q, msg):
        now = time.monotonic()
        ros = self.get_clock().now().nanoseconds*1e-9
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if (msg.header.frame_id != 'map' or not 0 <= msg.header.stamp.nanosec < 1_000_000_000
                or not math.isfinite(stamp) or stamp <= 0 or ros-stamp < -.005):
            raise ValueError('SIGN_RAW_FRAME')
        if ros-stamp > MAX_AGE:
            self.discarded_unselected += 1
            return
        frame = self.frames.setdefault(stamp, {})
        old = frame.get(name)
        if old is not None:
            if (old.position, old.quaternion) != (tuple(p), q):
                raise ValueError('SIGN_SAME_STAMP_CONFLICT')
            return  # A duplicate cannot renew any original receipt timestamp.
        frame[name] = Raw(name, tuple(p), q, stamp, now)
        for key, values in list(self.frames.items()):
            if ros-key > MAX_AGE or now-min(v.received for v in values.values()) > MAX_AGE:
                del self.frames[key]
        while len(self.frames) > 32:
            del self.frames[min(self.frames)]
        if stamp not in self.frames or not all(k in frame for k in REQUIRED):
            return
        packet = CachedPacket(tuple(frame[k] for k in REQUIRED))
        packet.pose(now, ros)  # Original 8mm, rotation, source/receipt age checks.
        if self.packet is None or packet.stamp > self.packet.stamp:
            self.packet = packet
            self.completed_frames += 1


def drain_callbacks(executor, node, budget_s=.003, maximum=256):
    started = time.monotonic()
    count = 0
    while count < maximum and time.monotonic()-started < budget_s:
        previous = node.callback_generation
        executor.spin_once(timeout_sec=0.)
        if node.callback_generation == previous:
            break
        count += 1
    return count, time.monotonic()-started


class GraphAudit:
    """One read-only graph worker; DDS graph queries cannot stall raw intake."""
    def __init__(self, audit):
        self.audit = audit
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.value = None
        self.thread = threading.Thread(target=self.run, daemon=True, name='vicon-graph-audit-only')
        self.thread.start()

    def run(self):
        while not self.stop_event.is_set():
            start = time.monotonic()
            try:
                conflicts = list(self.audit())
            except Exception as error:
                conflicts = ['source graph audit error: '+type(error).__name__]
            finished = time.monotonic()
            with self.lock:
                self.value = (finished, finished-start, conflicts)
            # Leave room for a bounded slow query inside the unchanged 200ms
            # freshness window (100ms sleep + 120ms query would exceed it).
            self.stop_event.wait(max(0., .05-(finished-start)))

    def snapshot(self, now):
        with self.lock:
            value = self.value
        now = max(now, time.monotonic())  # Observe time AFTER receiving the result.
        if value is None:
            return ['discovery pending'], dict(ready=False)
        finished, duration, conflicts = value
        if not 0 <= now-finished <= MAX_AGE or not 0 <= duration <= MAX_AGE:
            return ['source graph audit stale'], dict(ready=False, age_s=now-finished, duration_s=duration)
        return list(conflicts), dict(ready=True, age_s=now-finished, duration_s=duration)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=1.)
        return not self.thread.is_alive()


def graph_inspector(domain, namespace, expected, channel, stop):
    """Readonly separate participant; never owns a control node's DDS locks."""
    import os
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    context = Context();node = None;parent = os.getppid()
    try:
        rclpy.init(context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
        node = Node('wand_graph_observer', namespace=namespace, context=context,
                    use_global_arguments=False, enable_rosout=False)
        while not stop.is_set() and os.getppid() == parent:
            start = time.monotonic();conflicts = []
            try:
                for topic, name in expected.items():
                    pubs = node.get_publishers_info_by_topic(namespace+'/'+topic)
                    if not (len(pubs)==1 and pubs[0].node_name==name and pubs[0].node_namespace==namespace):
                        conflicts.append('source identity '+topic)
            except Exception as error:
                conflicts = ['source graph audit error: '+type(error).__name__]
            finished = time.monotonic()
            from .source_relay import latest
            latest(channel, (finished, finished-start, conflicts))
            stop.wait(max(0., .05-(finished-start)))
    finally:
        if node is not None:node.destroy_node()
        if context.ok():context.shutdown()
        channel.cancel_join_thread()


class GraphProcessAudit:
    def __init__(self, domain, namespace, expected):
        context = mp.get_context('spawn')
        self.channel = context.Queue(maxsize=1)
        self.stop = context.Event();self.value = None
        self.process = context.Process(target=graph_inspector,
            args=(domain,namespace,dict(expected),self.channel,self.stop),daemon=True,
            name='readonly-graph-inspector')
        self.process.start()

    def snapshot(self, now):
        for _ in range(4):
            try:self.value = self.channel.get_nowait()
            except queue.Empty:break
        now = max(now, time.monotonic())
        if not self.process.is_alive():
            return ['source graph inspector exited'], dict(ready=False)
        if self.value is None:
            return ['discovery pending'], dict(ready=False)
        finished, duration, conflicts = self.value
        if not 0<=now-finished<=MAX_AGE or not 0<=duration<=MAX_AGE:
            return ['source graph audit stale'], dict(ready=False,age_s=now-finished,duration_s=duration)
        return list(conflicts), dict(ready=True,age_s=now-finished,duration_s=duration)

    def close(self):
        self.stop.set();self.process.join(timeout=1.)
        if self.process.is_alive():
            self.process.terminate();self.process.join(timeout=1.)
        self.channel.close();self.channel.cancel_join_thread()


class AsyncGraphMixin:
    """Keep synchronous DDS graph enumeration out of motor-control timers.

    Only the main ROS executor writes graph_ok. The background thread has no
    access to authority, commands, leases, pose clocks or watchdog baselines.
    """
    def create_timer(self, period, callback, *args, **kw):
        if period == .10 and getattr(callback, '__name__', '') == 'check_graph':
            period = .02
        return super().create_timer(period, callback, *args, **kw)

    def inspect_graph(self):
        conflicts = []
        for topic, name in self.runtime.expected.items():
            pubs = self.get_publishers_info_by_topic(self.runtime.namespace+'/'+topic)
            if not (len(pubs) == 1 and pubs[0].node_name == name
                    and pubs[0].node_namespace == self.runtime.namespace):
                conflicts.append('source identity '+topic)
        return conflicts

    def check_graph(self):
        if getattr(self, '_independent_graph_audit', None) is None:
            self._independent_graph_audit = GraphProcessAudit(
                self.runtime.domain, self.runtime.namespace, self.runtime.expected)
        conflicts, self.graph_diagnostics = self._independent_graph_audit.snapshot(time.monotonic())
        self.core.inputs.graph_ok = not conflicts

    def destroy_node(self):
        audit = getattr(self, '_independent_graph_audit', None)
        if audit is not None:
            audit.close()
        return super().destroy_node()


def passive_loop(node, executor, channel, stop, ready):
    from .source_relay import latest
    channel.put(dict(kind='ready', provenance=node.provenance), timeout=1.)
    if not ready.wait(8.):
        raise ValueError('SOURCE_PARENT_HANDSHAKE_TIMEOUT')
    audit = GraphAudit(node.audit)
    discovered = time.monotonic()+2.
    last = previous = time.monotonic()
    max_loop_gap = max_batch = 0.
    callbacks = 0
    try:
        while not stop.is_set():
            current = time.monotonic()
            max_loop_gap = max(max_loop_gap, current-previous)
            previous = current
            count, duration = drain_callbacks(executor, node)
            callbacks += count
            max_batch = max(max_batch, duration)
            now = time.monotonic()
            if now-last < .005:
                if not count:
                    stop.wait(.0005)
                continue
            conflicts, graph = audit.snapshot(now)
            if now < discovered:
                conflicts.append('discovery pending')
            car = None
            msg = node.samples.get('car')
            if msg is not None:
                pose, stamp, roll, pitch = decode_pose(msg, 'map')
                from dataclasses import asdict
                car = dict(pose=asdict(pose), stamp=stamp, roll=roll, pitch=pitch)
            latest(channel, dict(kind='data', issued=now,
                packet=node.packet.payload() if node.packet is not None else None,
                car=car, conflicts=conflicts, rejected=dict(node.rejected),
                pipeline=dict(callbacks=callbacks, completed_frames=node.completed_frames,
                    discarded_unselected=node.discarded_unselected,
                    max_loop_gap_s=max_loop_gap, max_batch_s=max_batch, graph=graph)))
            last = now
    finally:
        audit.close()


def poll_latest(reader):
    # An expired UNSELECTED IPC item is not republished and cannot update a
    # lease/source stamp. Drop it and drain a newer item within a bounded read.
    for _ in range(4):
        try:
            value = reader.channel.get_nowait()
        except queue.Empty:
            break
        if value.get('kind') != 'data':
            reader.ipc_rejected += 1
            reader.value = None
            raise ValueError('SOURCE_IPC_ERROR '+repr(value))
        if not 0 <= time.monotonic()-value['issued'] <= MAX_AGE:
            reader.discarded_ipc += 1
            continue
        if reader.value is None or value['issued'] > reader.value['issued']:
            reader.value = value
    if not reader.process.is_alive() or reader.value is None:
        return None
    if not 0 <= time.monotonic()-reader.value['issued'] <= MAX_AGE:
        return None
    return reader.value
