"""Real Vicon input -> preview poses and markers only. Never drives the car."""

from collections import deque
import csv
import json
import math
from pathlib import Path
import signal
import tempfile
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from mecanum_vicon_control.math_utils import quaternion_from_yaw, wrap_angle, yaw_from_quaternion
from .core import Pose
from .preview_model import ReadonlyMapping, StreamQuality

OUTPUTS = {'target_pose': PoseStamped, 'anchor_pose': PoseStamped,
           'markers': MarkerArray, 'status': String}


def parse_input(msg, now_ns, world_frame):
    p, q = msg.pose.position, msg.pose.orientation
    stamp_ns = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
    age = (now_ns-stamp_ns)*1e-9
    if msg.header.frame_id != world_frame:
        raise ValueError('world frame mismatch')
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError('nonfinite pose')
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not .99 <= norm <= 1.01:
        raise ValueError('invalid quaternion')
    if stamp_ns <= 0 or not -.005 <= age <= .20:
        raise ValueError(f'invalid source timestamp: age={age:.6f}s')
    return Pose(p.x, p.y, p.z, yaw_from_quaternion(
        q.x/norm, q.y/norm, q.z/norm, q.w/norm)), stamp_ns, age


class SignReadonlyPreview(Node):
    def __init__(self):
        super().__init__('sign_readonly_preview')
        if self.get_namespace() != '/wand_preview':
            raise RuntimeError('read-only preview requires namespace /wand_preview')
        for name, value in {
            'wand_topic': '/vicon/Sign/Sign', 'car_topic': '/vicon/car01/car01',
            'world_frame': 'map', 'scale': 2.0, 'auto_capture': True,
            'car_yaw_offset_rad': 2.356194490192345, 'record_dir': '',
        }.items():
            self.declare_parameter(name, value)
        self.frame = str(self.get_parameter('world_frame').value)
        self.yaw_offset = float(self.get_parameter('car_yaw_offset_rad').value)
        self.model = ReadonlyMapping(float(self.get_parameter('scale').value))
        self.started = time.monotonic()
        self.auto_capture = bool(self.get_parameter('auto_capture').value)
        self.ready_since = None
        self.quality = {name: StreamQuality() for name in ('wand', 'car')}
        self.baseline = {name: StreamQuality() for name in ('wand', 'car')}
        self.marker_points = {}
        self.last_source_stamp = {}
        self.trace = deque(maxlen=1500)
        self.max_displacement = 0.0
        self.max_mapping_error = 0.0
        self.last_status = {}
        self.state_events = []
        self.previous_state = None
        self.record_dir = Path(str(self.get_parameter('record_dir').value))
        if str(self.get_parameter('record_dir').value):
            self.record_dir.mkdir(parents=True, exist_ok=False)
        else:
            self.record_dir = Path(tempfile.mkdtemp(prefix='wand_sign_readonly_'))
        self.log_file = (self.record_dir/'mapping.csv').open('x', buffering=65536)
        self.csv = csv.writer(self.log_file)
        self.csv.writerow(['elapsed_s', 'wand_stamp_ns', 'state', 'wand_x', 'wand_y', 'wand_z',
                           'wand_forward_m', 'wand_left_m', 'target_x', 'target_y',
                           'target_forward_m', 'target_left_m', 'markers_visible'])
        self.pubs = {}
        for name, message_type in OUTPUTS.items():
            if self.resolve_topic_name(name) != '/wand_preview/'+name:
                raise RuntimeError('preview output remapping outside /wand_preview is forbidden')
            self.pubs[name] = self.create_publisher(message_type, name, 10)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        for source, parameter in (('wand', 'wand_topic'), ('car', 'car_topic')):
            self.create_subscription(PoseStamped, str(self.get_parameter(parameter).value),
                                     lambda msg, source=source: self.receive(source, msg), newest)
        for number in range(1, 6):
            self.create_subscription(PointStamped, f'/vicon/Sign/markers/Sign{number}',
                                     lambda msg, number=number: self.receive_marker(number, msg), newest)
        self.create_service(Trigger, 'capture_origin', self.capture)
        self.create_service(Trigger, 'pause_preview', self.pause)
        self.create_timer(.05, self.tick)
        self.create_timer(5.0, self.write_report)
        self.get_logger().info(f'Sign只读预览；无速度/使能输出；记录目录：{self.record_dir}')

    def receive(self, source, msg):
        now = time.monotonic()
        try:
            pose, stamp, age = parse_input(msg, self.get_clock().now().nanoseconds, self.frame)
        except ValueError as exc:
            self.quality[source].invalid += 1
            self.model.invalidate(source, str(exc))
            return
        if not self.quality[source].accept(pose, stamp, now, age):
            return
        if now-self.started <= 10:
            self.baseline[source].accept(pose, stamp, now, age)
        if source == 'car':
            # 沿用已有矩形入口的刚体轴→车头修正，不重新标定硬件。
            pose = Pose(pose.x, pose.y, pose.z, wrap_angle(pose.yaw+self.yaw_offset))
        self.last_source_stamp[source] = msg.header.stamp
        self.model.receive(source, pose, now)

    def receive_marker(self, number, msg):
        p = msg.point
        if msg.header.frame_id == self.frame and all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.marker_points[number] = (Pose(p.x, p.y, p.z), time.monotonic())

    def capture(self, _, response):
        response.success, response.message = self.model.capture(time.monotonic())
        if response.success:
            self.trace.clear()
        return response

    def pause(self, _, response):
        self.model.pause()
        response.success, response.message = True, 'preview paused; hardware outputs do not exist'
        return response

    def pose_message(self, pose, stamp):
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = self.frame, stamp
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, (pose.x, pose.y, pose.z))
        q = quaternion_from_yaw(pose.yaw)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = q
        return msg

    def tick(self):
        now = time.monotonic()
        if self.model.mapping is None and self.auto_capture and self.model.fresh(now):
            if self.ready_since is None:
                self.ready_since = now
            elif now-self.ready_since >= 1.0:
                self.model.capture(now)
        elif not self.model.fresh(now):
            self.ready_since = None
        target = self.model.target(now)
        mapping = self.model.mapping
        wand = self.model.samples.get('wand')
        local = mapping.local_displacement(wand[0]) if mapping and wand else None
        error = None
        if target:
            stamp = self.last_source_stamp['wand']
            self.pubs['target_pose'].publish(self.pose_message(target, stamp))
            # 独立地按世界位移核对2倍映射；目标时间戳保留Sign源帧。
            expected_x = mapping.car_origin.x+self.model.scale*(wand[0].x-mapping.wand_origin.x)
            expected_y = mapping.car_origin.y+self.model.scale*(wand[0].y-mapping.wand_origin.y)
            error = math.hypot(target.x-expected_x, target.y-expected_y)
            self.max_mapping_error = max(self.max_mapping_error, error)
            self.max_displacement = max(self.max_displacement, math.hypot(*local))
            self.trace.append((target.x, target.y))
        if mapping:
            self.pubs['anchor_pose'].publish(self.pose_message(
                mapping.car_origin, self.get_clock().now().to_msg()))
        visible = sum(now-t < .20 for _, t in self.marker_points.values())
        self.last_status = {
            'readonly': True, 'hardware_commands_sent': 0,
            'state': self.model.state, 'reason': self.model.reason, 'scale': self.model.scale,
            'capture_count': self.model.capture_count,
            'anchor_yaw_rad': mapping.car_origin.yaw if mapping else None,
            'car_yaw_offset_rad': self.yaw_offset,
            'wand_startframe_xy_m': local,
            'target_startframe_xy_m': [self.model.scale*v for v in local] if target else None,
            'mapping_error_m': error, 'visible_sign_markers': visible,
            'wand_fresh': 'wand' in self.model.samples and now-self.model.samples['wand'][1] <= .20,
            'car_fresh': 'car' in self.model.samples and now-self.model.samples['car'][1] <= .20,
            'source_topics': [str(self.get_parameter(p).value) for p in ('wand_topic', 'car_topic')],
        }
        self.pubs['status'].publish(String(data=json.dumps(self.last_status)))
        if self.model.state != self.previous_state:
            self.state_events.append({'elapsed_s': now-self.started,
                                      'state': self.model.state, 'reason': self.model.reason})
            self.get_logger().info(f'{self.model.state}: {self.model.reason}')
            self.previous_state = self.model.state
        if wand:
            source_stamp = self.last_source_stamp['wand']
            row = [now-self.started, source_stamp.sec*1_000_000_000+source_stamp.nanosec,
                   self.model.state, wand[0].x, wand[0].y, wand[0].z]
            row += list(local) if local else ['', '']
            row += [target.x, target.y, self.model.scale*local[0], self.model.scale*local[1]] if target else ['']*4
            self.csv.writerow(row+[visible])
        if self.pubs['markers'].get_subscription_count():
            self.publish_markers(target, now)

    def publish_markers(self, target, now):
        markers = []

        def add(identifier, kind, pose, color, size):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.frame, self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'sign_readonly', identifier, kind, Marker.ADD
            m.pose = self.pose_message(pose, m.header.stamp).pose
            m.scale.x, m.scale.y, m.scale.z = map(float, size)
            m.color.r, m.color.g, m.color.b, m.color.a = map(float, color)
            markers.append(m)
            return m

        for number in range(1, 6):
            if number in self.marker_points and now-self.marker_points[number][1] < .20:
                add(number, Marker.SPHERE, self.marker_points[number][0],
                    (.1, 1, .2, 1), (.035, .035, .035))
            else:
                m = Marker()
                m.ns, m.id, m.action = 'sign_readonly', number, Marker.DELETE
                markers.append(m)
        if 'car' in self.model.samples:
            car = self.model.samples['car'][0]
            add(10, Marker.CUBE, car, (.1, .6, 1, 1), (.30, .22, .10))
            add(11, Marker.ARROW, car, (.1, .6, 1, 1), (.4, .035, .035))
        if self.model.mapping:
            a = self.model.mapping.car_origin
            add(12, Marker.ARROW, Pose(a.x, a.y, a.z+.14, a.yaw),
                (1, 1, 1, .8), (.6, .025, .025))
        if target:
            add(20, Marker.SPHERE, target, (1, .6, 0, 1), (.12, .12, .12))
        else:
            m = Marker()
            m.ns, m.id, m.action = 'sign_readonly', 20, Marker.DELETE
            markers.append(m)
        line = add(21, Marker.LINE_STRIP, Pose(0, 0), (1, .6, 0, .7), (.012, 0, 0))
        line.points = [Point(x=float(x), y=float(y), z=.05) for x, y in self.trace]
        a = self.model.mapping.car_origin if self.model.mapping else Pose(0, 0)
        text = add(30, Marker.TEXT_VIEW_FACING, Pose(a.x, a.y, 1.5),
                   (1, 1, 1, 1), (0, 0, .10))
        text.text = f'Sign | READ ONLY | 2x | {self.model.state}\nGreen: real markers  Orange: target  Blue: unpowered car'
        self.pubs['markers'].publish(MarkerArray(markers=markers))

    def write_report(self):
        self.log_file.flush()
        baseline = {name: value.snapshot() for name, value in self.baseline.items()}
        report = {
            'verification': 'REAL VICON READONLY PREVIEW; NO PHYSICAL CAR CONTROL',
            'elapsed_s': time.monotonic()-self.started,
            'streams': {name: value.snapshot() for name, value in self.quality.items()},
            'first_10s_baseline': baseline,
            'max_wand_displacement_m': self.max_displacement,
            'max_mapping_error_m': self.max_mapping_error,
            'state_events': self.state_events, 'last_status': self.last_status,
            'output_topics': ['/wand_preview/'+name for name in OUTPUTS],
            'note': 'source timestamp age is receiver-to-callback age, not capture-to-control latency',
        }
        temporary = self.record_dir/'summary.json.tmp'
        temporary.write_text(json.dumps(report, indent=2)+'\n')
        temporary.replace(self.record_dir/'summary.json')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = SignReadonlyPreview()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop:
            executor.spin_once(timeout_sec=.05)
    finally:
        node.write_report()
        node.log_file.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
