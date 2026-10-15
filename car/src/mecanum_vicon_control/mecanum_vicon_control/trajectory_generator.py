#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty

from .math_utils import quaternion_from_yaw, wrap_angle, yaw_from_quaternion


def _append_waypoint(waypoints, x, y, yaw):
    waypoints.append((float(x), float(y), float(yaw)))


def generate_waypoints(
        shape: str,
        center_x: float = 0.0,
        center_y: float = 0.0,
        radius: float = 0.5,
        width: float = 1.0,
        height: float = 0.6,
        waypoint_count: int = 32,
        heading_mode: str = 'tangent',
        fixed_yaw: float = 0.0,
):
    """Generate one closed planar loop, with tangent or constant heading."""
    shape = shape.strip().lower()
    if shape not in ('circle', 'rectangle', 'figure_eight'):
        raise ValueError(f'unsupported trajectory shape: {shape!r}')
    if waypoint_count < 8:
        raise ValueError('waypoint_count must be at least 8')
    if not all(math.isfinite(v) for v in (
            center_x, center_y, radius, width, height, fixed_yaw)):
        raise ValueError('trajectory geometry must be finite')
    if radius <= 0.0 or width <= 0.0 or height <= 0.0:
        raise ValueError('radius, width and height must be positive')
    if heading_mode not in ('tangent', 'fixed'):
        raise ValueError('heading_mode must be tangent or fixed')

    waypoints = []
    if shape == 'circle':
        for index in range(waypoint_count):
            theta = 2.0 * math.pi * index / waypoint_count
            _append_waypoint(
                waypoints,
                center_x + radius * math.cos(theta),
                center_y + radius * math.sin(theta),
                theta + math.pi / 2.0,
            )

    elif shape == 'figure_eight':
        for index in range(waypoint_count):
            theta = 2.0 * math.pi * index / waypoint_count
            x = center_x + radius * math.sin(theta)
            y = center_y + 0.5 * radius * math.sin(2.0 * theta)
            dx = radius * math.cos(theta)
            dy = radius * math.cos(2.0 * theta)
            _append_waypoint(waypoints, x, y, math.atan2(dy, dx))

    else:
        points_per_edge = max(2, int(math.ceil(waypoint_count / 4.0)))
        half_width = 0.5 * width
        half_height = 0.5 * height
        corners = (
            (center_x - half_width, center_y - half_height),
            (center_x + half_width, center_y - half_height),
            (center_x + half_width, center_y + half_height),
            (center_x - half_width, center_y + half_height),
        )
        for edge in range(4):
            start = corners[edge]
            end = corners[(edge + 1) % 4]
            yaw = math.atan2(end[1] - start[1], end[0] - start[0])
            for index in range(points_per_edge):
                ratio = index / points_per_edge
                _append_waypoint(
                    waypoints,
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                    yaw,
                )

    # 麦轮矩形演示保持车头不变；侧移不是在角点转向后再前进。
    if heading_mode == 'fixed':
        waypoints = [(x, y, wrap_angle(fixed_yaw)) for x, y, _yaw in waypoints]
    # 重复起点使 RViz 的计划路径闭合，也让每圈确实回到起点。
    waypoints.append(waypoints[0])
    return waypoints


def anchor_waypoints(waypoints, start_x, start_y, start_yaw):
    """Place the first waypoint at the measured start, using its forward/left axes."""
    if not waypoints or not all(math.isfinite(v) for v in (start_x, start_y, start_yaw)):
        raise ValueError('anchor requires waypoints and a finite start pose')
    origin_x, origin_y, _ = waypoints[0]
    c, s = math.cos(start_yaw), math.sin(start_yaw)
    return [(
        start_x + c * (x - origin_x) - s * (y - origin_y),
        start_y + s * (x - origin_x) + c * (y - origin_y),
        wrap_angle(yaw + start_yaw),
    ) for x, y, yaw in waypoints]


def planar_anchor(msg, world_frame):
    """Reject an invalid or untransformed pose instead of inventing a start origin."""
    if msg.header.frame_id != world_frame:
        raise ValueError('start pose frame does not match world_frame')
    p, q = msg.pose.position, msg.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError('start pose must be finite')
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-9:
        raise ValueError('start pose quaternion has zero length')
    return p.x, p.y, yaw_from_quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)


class TrajectoryGenerator(Node):
    """Advance through a closed waypoint trajectory using goal-reached feedback."""

    def __init__(self):
        super().__init__('trajectory_generator')
        self.declare_parameter('shape', 'circle')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('center_x_m', 0.0)
        self.declare_parameter('center_y_m', 0.0)
        self.declare_parameter('radius_m', 0.5)
        self.declare_parameter('width_m', 1.0)
        self.declare_parameter('height_m', 0.6)
        self.declare_parameter('waypoint_count', 32)
        self.declare_parameter('loop_count', 1)
        self.declare_parameter('heading_mode', 'tangent')
        self.declare_parameter('fixed_yaw_rad', 0.0)
        self.declare_parameter('relative_to_start', False)
        self.declare_parameter('pose_topic', '/car/pose')
        self.declare_parameter('start_pose_timeout_s', 0.2)
        self.declare_parameter('start_delay_s', 1.0)
        self.declare_parameter('minimum_goal_age_s', 0.15)
        self.declare_parameter('start_enabled', True)

        self.shape = str(self.get_parameter('shape').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.loop_count = int(self.get_parameter('loop_count').value)
        self.start_delay_s = max(0.0, float(self.get_parameter('start_delay_s').value))
        self.minimum_goal_age_s = max(
            0.0, float(self.get_parameter('minimum_goal_age_s').value))
        if self.loop_count < 0:
            raise ValueError('loop_count must be nonnegative; 0 means continuous')
        self.relative_to_start = bool(self.get_parameter('relative_to_start').value)
        self.start_pose_timeout_s = float(self.get_parameter('start_pose_timeout_s').value)
        if not math.isfinite(self.start_pose_timeout_s) or self.start_pose_timeout_s <= 0:
            raise ValueError('start_pose_timeout_s must be finite and positive')
        self.latest_anchor = None
        self.last_pose_ns = None
        self.path_anchored = not self.relative_to_start

        self.local_waypoints = generate_waypoints(
            self.shape,
            center_x=float(self.get_parameter('center_x_m').value),
            center_y=float(self.get_parameter('center_y_m').value),
            radius=float(self.get_parameter('radius_m').value),
            width=float(self.get_parameter('width_m').value),
            height=float(self.get_parameter('height_m').value),
            waypoint_count=int(self.get_parameter('waypoint_count').value),
            heading_mode=str(self.get_parameter('heading_mode').value),
            fixed_yaw=float(self.get_parameter('fixed_yaw_rad').value),
        )
        self.waypoints = list(self.local_waypoints)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/car/goal_pose', latched_qos)
        self.path_pub = self.create_publisher(Path, '/car/trajectory_path', latched_qos)
        self.active_pub = self.create_publisher(Bool, '/car/trajectory_active', latched_qos)
        self.create_subscription(Bool, '/car/goal_reached', self.on_goal_reached, 10)
        self.create_subscription(Bool, '/car/trajectory_enable', self.on_enable, 10)
        self.create_subscription(Empty, '/car/trajectory_cancel', self.on_cancel, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('pose_topic').value), self.on_pose, 10)
        # 直接点击 RViz 也会取消自动轨迹，避免下一个航点覆盖手动目标。
        self.create_subscription(PoseStamped, '/goal_pose', self.on_manual_goal, 10)

        self.active = False
        self.start_pending = bool(self.get_parameter('start_enabled').value)
        self.start_at_ns = (
            self.get_clock().now().nanoseconds + int(self.start_delay_s * 1.0e9))
        self.waypoint_index = 0
        self.completed_loops = 0
        self.goal_not_before_ns = 0
        self.create_timer(0.05, self.tick)
        self.publish_path()
        self.publish_active()
        self.get_logger().info(
            f'轨迹发生器: shape={self.shape}, '
            f'waypoints={len(self.waypoints)}, loops={self.loop_count}')

    def make_pose(self, waypoint):
        x, y, yaw = waypoint
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.world_frame
        # 等待真实起点时不把未变换的局部路径冒充map路径。
        path.poses = ([self.make_pose(waypoint) for waypoint in self.waypoints]
                      if self.path_anchored else [])
        self.path_pub.publish(path)

    def publish_active(self):
        self.active_pub.publish(Bool(data=self.active))

    def start(self):
        if self.relative_to_start:
            now_ns = self.get_clock().now().nanoseconds
            if (self.latest_anchor is None or self.last_pose_ns is None
                    or not 0 <= (now_ns - self.last_pose_ns) * 1e-9 <= self.start_pose_timeout_s):
                self.start_pending = True
                self.get_logger().warn('等待新鲜的/car/pose，尚未开始轨迹', throttle_duration_sec=1.0)
                return
            self.waypoints = anchor_waypoints(self.local_waypoints, *self.latest_anchor)
            self.path_anchored = True
            self.publish_path()
        self.start_pending = False
        self.active = True
        self.waypoint_index = 0
        self.completed_loops = 0
        self.publish_active()
        self.publish_current_goal()
        self.get_logger().info(f'开始 {self.shape} 轨迹')

    def stop(self, reason):
        if self.active or self.start_pending:
            self.get_logger().info(f'停止轨迹: {reason}')
        self.active = False
        self.start_pending = False
        self.publish_active()

    def publish_current_goal(self):
        self.goal_pub.publish(self.make_pose(self.waypoints[self.waypoint_index]))
        self.goal_not_before_ns = (
            self.get_clock().now().nanoseconds
            + int(self.minimum_goal_age_s * 1.0e9))

    def advance(self):
        if self.waypoint_index + 1 < len(self.waypoints):
            self.waypoint_index += 1
            self.publish_current_goal()
            return

        self.completed_loops += 1
        if self.loop_count > 0 and self.completed_loops >= self.loop_count:
            self.stop(f'已完成 {self.completed_loops} 圈')
            return

        # 上一圈最后一点已是起点，下一圈从第二点继续。
        self.waypoint_index = 1
        self.publish_current_goal()

    def on_pose(self, msg: PoseStamped):
        if not self.relative_to_start:
            return
        try:
            self.latest_anchor = planar_anchor(msg, self.world_frame)
        except ValueError as error:
            self.latest_anchor = None
            self.last_pose_ns = None
            self.get_logger().warn(str(error), throttle_duration_sec=1.0)
            return
        self.last_pose_ns = self.get_clock().now().nanoseconds
        if not self.path_anchored:
            self.waypoints = anchor_waypoints(self.local_waypoints, *self.latest_anchor)
            self.path_anchored = True
            self.publish_path()

    def on_goal_reached(self, msg: Bool):
        if (
                self.active
                and msg.data
                and self.get_clock().now().nanoseconds >= self.goal_not_before_ns):
            self.advance()

    def on_enable(self, msg: Bool):
        if msg.data:
            self.start()
        else:
            self.stop('操作者取消')

    def on_cancel(self, _msg: Empty):
        self.stop('RViz手动目标接管')

    def on_manual_goal(self, _msg: PoseStamped):
        self.stop('RViz手动目标接管')

    def tick(self):
        if (
                self.start_pending
                and self.get_clock().now().nanoseconds >= self.start_at_ns):
            self.start()


def main():
    rclpy.init()
    try:
        node = TrajectoryGenerator()
    except (TypeError, ValueError) as error:
        rclpy.logging.get_logger('trajectory_generator').error(str(error))
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

