"""One-way real Sign input (domain 0) to simulation only (domain 99)."""

import argparse
import copy
import csv
import json
import math
from pathlib import Path
import signal
import tempfile
import threading
import time

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from mecanum_vicon_control.math_utils import wrap_angle
from .core import Pose
from .readonly_preview import parse_input

SOURCE_DOMAIN = 0
SIM_DOMAIN = 99
SIM_FRAME = 'wand_sim_map'
YAW_OFFSET = 2.356194490192345
OUTPUT_NAMES = ('wand_pose', 'disturbance', 'source_status', 'source_markers')


class LiveInputs:
    """The source header is retained; receiving/relaying old frames cannot renew it."""
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}
        self.counts = {'wand': 0, 'car': 0}
        self.rejected = {'wand': 0, 'car': 0}

    def update(self, source, msg, now_ns, now_s):
        try:
            pose, stamp_ns, age = parse_input(msg, now_ns, 'map')
        except ValueError:
            with self.lock:
                self.rejected[source] += 1
                self.latest.pop(source, None)
            return False
        with self.lock:
            old = self.latest.get(source)
            if old is not None and stamp_ns <= old['stamp_ns']:
                return False
            if source == 'car':
                pose = Pose(pose.x, pose.y, pose.z, wrap_angle(pose.yaw+YAW_OFFSET))
            self.latest[source] = {'message': copy.deepcopy(msg), 'pose': pose,
                                   'stamp_ns': stamp_ns, 'received_s': now_s,
                                   'receiver_age_s': age}
            self.counts[source] += 1
        return True

    def snapshot(self):
        with self.lock:
            return dict(self.latest), dict(self.counts), dict(self.rejected)


def input_fresh(sample, now_s, now_ns):
    return (sample is not None and 0 <= now_s-sample['received_s'] <= .20
            and -.005 <= (now_ns-sample['stamp_ns'])*1e-9 <= .20)


def relay_message(sample):
    # Explicit identity transform: sim world axes/origin equal Vicon map axes/origin.
    # Only the frame's name differs, and the real source timestamp is not changed.
    msg = copy.deepcopy(sample['message'])
    msg.header.frame_id = SIM_FRAME
    return msg


class SourceSubscriber(Node):
    def __init__(self, context, inputs):
        super().__init__('sign_sim_source', namespace='/sign_source',
                         context=context, use_global_arguments=False)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        self.processed_at = {}
        self.inputs = inputs
        for source, topic in (('wand', '/vicon/Sign/Sign'), ('car', '/vicon/car01/car01')):
            self.create_subscription(PoseStamped, topic,
                lambda msg, source=source: self.receive(source, msg), newest)

    def receive(self, source, msg):
        now = time.monotonic()
        # The relay runs at 50 Hz; process the latest wand near 100 Hz and the
        # static, unpowered car reference at 20 Hz instead of deep-copying 660/s.
        period = .008 if source == 'wand' else .05
        if now-self.processed_at.get(source, -math.inf) < period:
            return
        self.processed_at[source] = now
        self.inputs.update(source, msg, self.get_clock().now().nanoseconds, now)


class SimulationRelay(Node):
    def __init__(self, context, inputs, record_dir, auto_engage):
        super().__init__('sign_to_simulation', namespace='/wand_sim',
                         context=context, use_global_arguments=False)
        self.inputs, self.auto_engage = inputs, auto_engage
        self.folder = Path(record_dir) if record_dir else Path(tempfile.mkdtemp(prefix='sign_sim_'))
        if record_dir:
            self.folder.mkdir(parents=True, exist_ok=False)
        self.started = time.monotonic()
        self.hand_pub = self.create_publisher(PoseStamped, 'wand_pose', 10)
        self.seed_pub = self.create_publisher(Pose2D, 'disturbance', 10)
        self.info_pub = self.create_publisher(String, 'source_status', 10)
        self.graphics_pub = self.create_publisher(MarkerArray, 'source_markers', 10)
        for name in OUTPUT_NAMES:
            if self.resolve_topic_name(name) != '/wand_sim/'+name:
                raise RuntimeError('live relay output must stay inside /wand_sim')
        self.engage_client = self.create_client(Trigger, 'engage')
        self.create_service(SetBool, 'simulate_occlusion', self.on_occlusion)
        self.create_subscription(PoseStamped, 'car_pose', self.on_plant, 10)
        self.create_subscription(String, 'status', self.on_status, 10)
        self.plant = None
        self.controller_status = {}
        self.synthetic_occlusion = False
        self.forwarded_stamp = None
        self.forwarded_count = 0
        self.seed = None
        self.seed_requested_s = None
        self.seeded = False
        self.auto_started = False
        self.engage_pending = False
        self.last_state = None
        self.events = []
        self.csv_stream = (self.folder/'live_simulation.csv').open('x', buffering=65536)
        self.csv = csv.writer(self.csv_stream)
        self.csv.writerow(['elapsed_s', 'state', 'synthetic_occlusion', 'real_wand_fresh',
                           'real_wand_x', 'real_wand_y', 'source_stamp_ns',
                           'virtual_x', 'virtual_y', 'wand_forward_m', 'wand_left_m', 'speed_mps'])
        self.create_timer(.02, self.tick)
        self.create_timer(5.0, self.write_report)
        self.get_logger().info(f'真实Sign→仿真域99；记录：{self.folder}；域0节点只订阅')

    def on_plant(self, msg):
        self.plant = msg

    def on_status(self, msg):
        self.controller_status = json.loads(msg.data)

    def on_occlusion(self, request, response):
        self.synthetic_occlusion = bool(request.data)
        self.events.append({'elapsed_s': time.monotonic()-self.started,
                            'simulated_input_blocked': self.synthetic_occlusion})
        response.success = True
        response.message = ('Only the simulation relay is blocked; real Vicon/preview unchanged'
                            if request.data else 'Relay restored; controller does not auto-resume')
        return response

    def engage_result(self, future):
        self.engage_pending = False
        result = future.result()
        if result.success:
            self.auto_started = True
            self.get_logger().info('真实杆仿真跟随已启用；恢复数据不会再次自动启用')

    def tick(self):
        latest, counts, rejected = self.inputs.snapshot()
        now, now_ns = time.monotonic(), self.get_clock().now().nanoseconds
        wand, car = latest.get('wand'), latest.get('car')
        wand_fresh = input_fresh(wand, now, now_ns)
        car_fresh = input_fresh(car, now, now_ns)
        state = self.controller_status.get('state', 'WAITING')
        if (wand_fresh and not self.synthetic_occlusion
                and wand['stamp_ns'] != self.forwarded_stamp):
            self.hand_pub.publish(relay_message(wand))
            self.forwarded_stamp = wand['stamp_ns']
            self.forwarded_count += 1
        if (self.seed_requested_s is None and car_fresh and self.plant is not None
                and state == 'STANDBY' and self.seed_pub.get_subscription_count()):
            self.seed = car['pose']
            current = self.plant.pose
            q = current.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.seed_pub.publish(Pose2D(x=float(self.seed.x-current.position.x),
                                        y=float(self.seed.y-current.position.y),
                                        theta=float(wrap_angle(self.seed.yaw-yaw))))
            self.seed_requested_s = now
        if self.seed is not None and self.plant is not None:
            p, q = self.plant.pose.position, self.plant.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if math.hypot(p.x-self.seed.x, p.y-self.seed.y) < .002 and abs(wrap_angle(yaw-self.seed.yaw)) < .01:
                self.seeded = True
        if (self.auto_engage and self.seeded and not self.auto_started and not self.engage_pending
                and wand_fresh and car_fresh and not self.synthetic_occlusion
                and now-self.seed_requested_s > .5 and state == 'STANDBY'
                and self.engage_client.service_is_ready()):
            self.engage_pending = True
            self.engage_client.call_async(Trigger.Request()).add_done_callback(self.engage_result)
        info = {
            'source_domain': SOURCE_DOMAIN, 'simulation_domain': SIM_DOMAIN,
            'real_wand_fresh': wand_fresh, 'real_car_reference_fresh': car_fresh,
            'synthetic_occlusion': self.synthetic_occlusion,
            'seeded': self.seeded, 'auto_started_once': self.auto_started,
            'forwarded_frames': self.forwarded_count, 'source_counts': counts,
            'rejected_source_samples': rejected, 'controller': self.controller_status,
            'physical_command_publishers_in_this_bridge': 0,
        }
        self.info_pub.publish(String(data=json.dumps(info)))
        if state != self.last_state:
            self.events.append({'elapsed_s': now-self.started, 'controller_state': state,
                                'reason': self.controller_status.get('reason', '')})
            self.last_state = state
        p = self.plant.pose.position if self.plant else None
        self.csv.writerow([now-self.started, state, int(self.synthetic_occlusion), int(wand_fresh),
            wand['pose'].x if wand else '', wand['pose'].y if wand else '',
            wand['stamp_ns'] if wand else '', p.x if p else '', p.y if p else '',
            self.controller_status.get('wand_forward_m', ''),
            self.controller_status.get('wand_left_m', ''), self.controller_status.get('speed_mps', '')])
        if self.graphics_pub.get_subscription_count():
            self.draw_reference(car, state)

    def draw_reference(self, car, state):
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = SIM_FRAME, self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = 'live_source', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 1.9
        marker.scale.z = .11
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = f'REAL Sign -> SIM CAR | 2x | {state}\nPhysical car stays OFF | speed <= 0.15 m/s'
        markers = [marker]
        if car:
            ref = Marker()
            ref.header = marker.header
            ref.ns, ref.id, ref.type, ref.action = 'live_source', 1, Marker.CUBE, Marker.ADD
            ref.pose.position.x, ref.pose.position.y, ref.pose.position.z = (
                float(car['pose'].x), float(car['pose'].y), .10)
            ref.pose.orientation.z = math.sin(car['pose'].yaw/2)
            ref.pose.orientation.w = math.cos(car['pose'].yaw/2)
            ref.scale.x, ref.scale.y, ref.scale.z = .30, .22, .10
            ref.color.r, ref.color.g, ref.color.b, ref.color.a = .8, .8, .8, .3
            markers.append(ref)
        self.graphics_pub.publish(MarkerArray(markers=markers))

    def write_report(self):
        self.csv_stream.flush()
        _, counts, rejected = self.inputs.snapshot()
        report = {'verification': 'REAL SIGN DRIVES SOFTWARE SIMULATION ONLY',
            'source_domain': 0, 'simulation_domain': 99,
            'source_counts': counts, 'rejected_source_samples': rejected,
            'forwarded_frames': self.forwarded_count, 'events': self.events,
            'last_controller_status': self.controller_status,
            'auto_started_once': self.auto_started, 'synthetic_occlusion': self.synthetic_occlusion}
        temporary = self.folder/'summary.json.tmp'
        temporary.write_text(json.dumps(report, indent=2)+'\n')
        temporary.replace(self.folder/'summary.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-dir', default='', nargs='?', const='')
    parser.add_argument('--auto-engage', choices=['true', 'false'], default='true')
    args = parser.parse_args(remove_ros_args()[1:])
    contexts = [Context(), Context()]
    for context, domain in zip(contexts, (SOURCE_DOMAIN, SIM_DOMAIN)):
        rclpy.init(context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    inputs = LiveInputs()
    source = SourceSubscriber(contexts[0], inputs)
    relay = SimulationRelay(contexts[1], inputs, args.record_dir, args.auto_engage == 'true')
    source_executor, sim_executor = (SingleThreadedExecutor(context=c) for c in contexts)
    source_executor.add_node(source)
    sim_executor.add_node(relay)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())

    def read_source():
        while not stopping.is_set() and contexts[0].ok():
            source_executor.spin_once(timeout_sec=.02)

    thread = threading.Thread(target=read_source, daemon=True)
    thread.start()
    try:
        while not stopping.is_set() and contexts[1].ok():
            sim_executor.spin_once(timeout_sec=.02)
    finally:
        stopping.set()
        thread.join(timeout=2)
        relay.write_report()
        relay.csv_stream.close()
        source_executor.shutdown()
        sim_executor.shutdown()
        source.destroy_node()
        relay.destroy_node()
        for context in contexts:
            context.shutdown()
