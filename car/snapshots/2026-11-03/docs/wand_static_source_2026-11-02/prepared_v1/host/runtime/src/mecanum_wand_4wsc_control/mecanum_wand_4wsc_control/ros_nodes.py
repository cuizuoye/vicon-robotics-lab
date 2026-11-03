"""Simulation adapters: fixed namespace, isolated domains, no hardware bridge."""

from collections import deque
import json
import math
import os
import signal
import time

import rclpy
from geometry_msgs.msg import Point, Pose2D, PoseStamped, Twist
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from mecanum_vicon_control.math_utils import quaternion_from_yaw, yaw_from_quaternion
from mecanum_vicon_control.simulated_vicon_car import SimulatedViconCar
from .core import Config, Pose, WandSession
from .scenario import DEMO_KEYFRAMES, interpolate

FRAME = 'wand_sim_map'
DOMAINS = (97, 98, 99)


def offline_guard(node):
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) not in DOMAINS:
        raise RuntimeError('offline wand nodes require ROS_DOMAIN_ID 97, 98 or 99')
    if node.get_namespace() != '/wand_sim':
        raise RuntimeError('offline wand nodes require namespace /wand_sim')
    for name in ('wand_pose', 'car_pose', 'cmd_vel', 'goal_pose', 'disturbance', 'status'):
        if node.resolve_topic_name(name) != '/wand_sim/' + name:
            raise RuntimeError('offline topics must remain inside /wand_sim')


def pose_message(pose, stamp):
    msg = PoseStamped()
    msg.header.frame_id, msg.header.stamp = FRAME, stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
        float(pose.x), float(pose.y), float(pose.z))
    q = quaternion_from_yaw(pose.yaw)
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = q
    return msg


def parse_pose(msg, now, timeout=0.20):
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if msg.header.frame_id != FRAME:
        raise ValueError('frame mismatch')
    age = now - stamp
    if not -0.005 <= age <= timeout:
        raise ValueError(f'stale or future source timestamp: age={age:.6f}s')
    p, q = msg.pose.position, msg.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite pose')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not 0.99 <= norm <= 1.01:
        raise ValueError('invalid quaternion')
    return Pose(p.x, p.y, p.z, yaw_from_quaternion(
        q.x / norm, q.y / norm, q.z / norm, q.w / norm)), min(stamp, now)


class WandSimController(Node):
    def __init__(self):
        super().__init__('wand_sim_controller')
        offline_guard(self)
        self.declare_parameter('scale', 2.0)
        self.declare_parameter('display_label', 'OFFLINE')
        self.display_label = str(self.get_parameter('display_label').value)
        self.last_control_now = None
        self.max_control_gap = 0.0
        self.session = WandSession(Config(scale=float(self.get_parameter('scale').value)))
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'markers', 10)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseStamped, 'car_pose',
                                 lambda msg: self.receive('car', msg), newest)
        self.create_subscription(PoseStamped, 'wand_pose',
                                 lambda msg: self.receive('wand', msg), newest)
        self.create_service(Trigger, 'engage', self.engage)
        self.create_service(Trigger, 'pause', self.pause)
        self.create_service(Trigger, 'stop', self.stop)
        self.trace = deque(maxlen=1500)
        self.last_state = None
        self.create_timer(0.02, self.tick)
        # 可视化不占用50Hz控制周期；无人订阅时完全不构造点数组。
        self.create_timer(0.10, lambda: self.publish_markers()
                          if self.markers_pub.get_subscription_count() else None)
        self.get_logger().info('离线标定杆控制：/wand_sim；默认待机；位移比例2倍')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def receive(self, source, msg):
        try:
            pose, stamp = parse_pose(msg, self.now_s(), self.session.config.input_timeout_s)
        except ValueError as exc:
            self.session.invalidate(source, str(exc))
            return
        self.session.observe(source, pose, stamp)

    def engage(self, _, response):
        response.success, response.message = self.session.engage(self.now_s())
        if response.success:
            self.trace.clear()
        return response

    def pause(self, _, response):
        self.session.pause()
        self.publish_zero()
        response.success, response.message = True, 'paused; next engage rebases both origins'
        return response

    def stop(self, _, response):
        self.session.pause(stop=True)
        self.publish_zero()
        response.success, response.message = True, 'stopped; no target is retained'
        return response

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def tick(self):
        now = self.now_s()
        if self.last_control_now is not None:
            self.max_control_gap = max(self.max_control_gap, now-self.last_control_now)
        self.last_control_now = now
        command = self.session.tick(now)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = command.vx, command.vy, command.wz
        self.cmd_pub.publish(msg)
        if self.session.goal:
            self.goal_pub.publish(pose_message(
                self.session.goal, self.get_clock().now().to_msg()))
        local = self.session.filtered_local
        mapping = self.session.mapping
        self.status_pub.publish(String(data=json.dumps({
            'state': self.session.state, 'reason': self.session.reason,
            'scale': self.session.config.scale,
            'anchor_yaw_rad': mapping.car_origin.yaw if mapping else None,
            'wand_forward_m': local[0], 'wand_left_m': local[1],
            'speed_mps': math.hypot(command.vx, command.vy),
            'source_age_s': {source: now-self.session.samples[source][1]
                             if source in self.session.samples else None
                             for source in ('car', 'wand')},
            'max_control_timer_gap_s': self.max_control_gap,
        })))
        if self.session.state != self.last_state:
            self.get_logger().info(f'{self.session.state}: {self.session.reason}')
            self.last_state = self.session.state

    def publish_markers(self):
        markers = []

        def marker(identifier, kind, pose, color, size):
            m = Marker()
            m.header.frame_id = FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'wand_sim', identifier, kind, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (
                float(pose.x), float(pose.y), float(pose.z))
            q = quaternion_from_yaw(pose.yaw)
            m.pose.orientation.x, m.pose.orientation.y = q[0], q[1]
            m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
            m.scale.x, m.scale.y, m.scale.z = size
            m.color.r, m.color.g, m.color.b, m.color.a = color
            markers.append(m)
            return m

        samples = self.session.samples
        if 'car' in samples:
            car = samples['car'][0]
            marker(0, Marker.CUBE, Pose(car.x, car.y, 0.10, car.yaw),
                   (0.1, 0.6, 1.0, 1.0), (0.30, 0.22, 0.12))
            marker(1, Marker.ARROW, Pose(car.x, car.y, 0.20, car.yaw),
                   (0.1, 0.6, 1.0, 1.0), (0.40, 0.045, 0.045))
            self.trace.append((car.x, car.y))
        line = marker(2, Marker.LINE_STRIP, Pose(0, 0),
                      (0.1, 0.6, 1.0, 0.8), (0.012, 0.0, 0.0))
        line.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in self.trace]
        if self.session.goal:
            g = self.session.goal
            marker(3, Marker.SPHERE, Pose(g.x, g.y, 0.12),
                   (1.0, 0.6, 0.0, 1.0), (0.12, 0.12, 0.12))
        else:
            m = Marker()
            m.ns, m.id, m.action = 'wand_sim', 3, Marker.DELETE
            markers.append(m)
        if 'wand' in samples:
            wand = samples['wand'][0]
            c, s = math.cos(wand.yaw), math.sin(wand.yaw)
            # 五点T形仅为可视化示意，不代表实物标定尺寸。
            for i, (x, y) in enumerate(((-.16, 0), (-.04, 0), (.16, 0), (0, -.12), (0, -.24))):
                marker(10+i, Marker.SPHERE,
                       Pose(wand.x+c*x-s*y, wand.y+s*x+c*y, wand.z),
                       (0.15, 1.0, 0.3, 1.0), (0.04, 0.04, 0.04))
        text = marker(20, Marker.TEXT_VIEW_FACING, Pose(0, 0, 1.7),
                      (1.0, 1.0, 1.0, 1.0), (0.0, 0.0, 0.12))
        text.text = (f'{self.session.state} | scale {self.session.config.scale:g} | '
                     f'{getattr(self, "display_label", "OFFLINE")}')
        self.markers_pub.publish(MarkerArray(markers=markers))


class WandDemo(Node):
    def __init__(self):
        super().__init__('wand_demo')
        offline_guard(self)
        self.pub = self.create_publisher(PoseStamped, 'wand_pose', 10)
        self.disturbance_pub = self.create_publisher(Pose2D, 'disturbance', 10)
        self.client = self.create_client(Trigger, 'engage')
        self.pause_client = self.create_client(Trigger, 'pause')
        self.car = None
        self.create_subscription(PoseStamped, 'car_pose', self.on_car, qos_profile_sensor_data)
        self.origin_sent = False
        self.engage_requested = False
        self.done = False
        self.origin_time = None
        self.started = None
        self.yaw = math.pi / 3
        self.create_timer(0.02, self.tick)

    def on_car(self, msg):
        self.car = msg

    def engaged(self, future):
        result = future.result()
        if result.success:
            self.started = time.monotonic()
            self.get_logger().info('96秒离线演示开始：前进返回、左移返回、斜移返回；初始车头60度')
        else:
            self.engage_requested = False

    def tick(self):
        elapsed = 0 if self.started is None else time.monotonic() - self.started
        forward, left = interpolate(elapsed)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.pub.publish(pose_message(Pose(
            -0.4+c*forward-s*left, -0.8+s*forward+c*left,
            1.0, 0.3 * math.sin(elapsed)), self.get_clock().now().to_msg()))
        if (not self.origin_sent and self.car is not None
                and self.disturbance_pub.get_subscription_count()):
            self.disturbance_pub.publish(Pose2D(x=1.0, y=-0.5, theta=self.yaw))
            self.origin_sent = True
            self.origin_time = time.monotonic()
        if (self.origin_sent and not self.engage_requested
                and time.monotonic() - self.origin_time > 1.0
                and self.client.service_is_ready()):
            self.engage_requested = True
            self.client.call_async(Trigger.Request()).add_done_callback(self.engaged)
        if elapsed > DEMO_KEYFRAMES[-1][0] and not self.done:
            self.done = True
            self.pause_client.call_async(Trigger.Request())
            self.get_logger().info('离线演示结束，已请求暂停；可重新运行演示')


def run_node(factory):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    # Do not raise a Python interrupt from inside a ROS C-extension callback.
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = factory()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=0.05)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if isinstance(node, WandSimController):
            node.publish_zero()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def controller_main():
    run_node(WandSimController)


def demo_main():
    run_node(WandDemo)


def sim_car_main():
    # Reuse the existing plant without modifying its source or hardware stack.
    def create_plant():
        node = SimulatedViconCar()
        offline_guard(node)
        for parameter, expected in (
                ('cmd_topic', '/wand_sim/cmd_vel'),
                ('pose_topic', '/wand_sim/car_pose'),
                ('disturbance_topic', '/wand_sim/disturbance')):
            actual = str(node.get_parameter(parameter).value)
            if node.resolve_topic_name(actual) != expected:
                raise RuntimeError('simulation plant topics must remain inside /wand_sim')
        return node
    run_node(create_plant)
