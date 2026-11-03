#!/usr/bin/env python3
"""Continuous active-segment path following with bounded disturbance recovery.

The controller deliberately follows only the segment selected by the existing
trajectory generator.  It never chooses the globally nearest edge, because a
large disturbance on a rectangle could otherwise jump to the opposite edge.
"""

from dataclasses import dataclass
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from .math_utils import (
    clamp,
    quaternion_from_yaw,
    slew,
    world_error_to_body,
    wrap_angle,
    yaw_from_quaternion,
)


WAITING = 'WAITING'
TRACKING = 'TRACKING'
RECOVERY = 'RECOVERY'
HOLD = 'HOLD'
VICON_LOST = 'VICON_LOST'
COMPLETE = 'COMPLETE'


@dataclass(frozen=True)
class SegmentGeometry:
    projection_x: float
    projection_y: float
    target_x: float
    target_y: float
    tangent_x: float
    tangent_y: float
    normal_x: float
    normal_y: float
    length_m: float
    along_m: float
    progress: float
    cross_track_m: float


def segment_geometry(point, start, end, lookahead_m):
    """Project a point onto one directed segment and return signed errors."""
    px, py = point
    ax, ay = start
    bx, by = end
    values = (px, py, ax, ay, bx, by, lookahead_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('segment geometry must be finite')
    if lookahead_m < 0.0:
        raise ValueError('lookahead must be nonnegative')
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return SegmentGeometry(
            ax, ay, ax, ay, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, math.hypot(px - ax, py - ay))
    tx, ty = dx / length, dy / length
    nx, ny = -ty, tx
    along = (px - ax) * tx + (py - ay) * ty
    cross = (px - ax) * nx + (py - ay) * ny
    projected_s = clamp(along, 0.0, length)
    target_s = clamp(projected_s + lookahead_m, 0.0, length)
    return SegmentGeometry(
        ax + projected_s * tx,
        ay + projected_s * ty,
        ax + target_s * tx,
        ay + target_s * ty,
        tx, ty, nx, ny, length, along,
        projected_s / length, cross,
    )


def update_recovery_state(
        state, abs_cross_track_m, enter_m, exit_m, now_s,
        settled_since_s=None):
    """Apply hysteresis and a dwell time before leaving recovery."""
    if not all(math.isfinite(value) for value in (
            abs_cross_track_m, enter_m, exit_m, now_s)):
        raise ValueError('recovery state inputs must be finite')
    if not 0.0 <= exit_m < enter_m:
        raise ValueError('recovery thresholds must satisfy 0 <= exit < enter')
    if state == TRACKING and abs_cross_track_m >= enter_m:
        return RECOVERY, None
    if state != RECOVERY:
        return state, None
    if abs_cross_track_m > exit_m:
        return RECOVERY, None
    if settled_since_s is None:
        return RECOVERY, now_s
    return RECOVERY, settled_since_s


def bounded_world_velocity(
        geometry, state, nominal_speed_mps, recovery_tangent_speed_mps,
        kp_along, kp_cross_track, tracking_cross_speed_mps,
        recovery_cross_speed_mps, max_linear_speed_mps):
    """Compute a bounded world-frame velocity for one active segment."""
    if geometry.length_m < 1e-9:
        return 0.0, 0.0
    tangent_speed = clamp(
        kp_along * (geometry.length_m - geometry.along_m),
        -nominal_speed_mps, nominal_speed_mps)
    cross_limit = tracking_cross_speed_mps
    if state == RECOVERY:
        tangent_speed = clamp(
            tangent_speed,
            -recovery_tangent_speed_mps,
            recovery_tangent_speed_mps)
        cross_limit = recovery_cross_speed_mps
    normal_speed = clamp(
        -kp_cross_track * geometry.cross_track_m,
        -cross_limit, cross_limit)
    vx = tangent_speed * geometry.tangent_x + normal_speed * geometry.normal_x
    vy = tangent_speed * geometry.tangent_y + normal_speed * geometry.normal_y
    magnitude = math.hypot(vx, vy)
    if magnitude > max_linear_speed_mps:
        scale = max_linear_speed_mps / magnitude
        vx *= scale
        vy *= scale
    return vx, vy


class PathRecoveryController(Node):
    """Follow the active path segment and visibly recover after disturbances."""

    def __init__(self):
        super().__init__('path_recovery_controller')
        self.declare_parameter('pose_topic', '/car/pose')
        self.declare_parameter('goal_topic', '/car/goal_pose')
        self.declare_parameter('cmd_topic', '/car/cmd_vel_raw')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('pose_timeout_s', 0.20)
        self.declare_parameter('nominal_speed_mps', 0.10)
        self.declare_parameter('recovery_tangent_speed_mps', 0.03)
        self.declare_parameter('tracking_cross_speed_mps', 0.06)
        self.declare_parameter('recovery_cross_speed_mps', 0.10)
        self.declare_parameter('max_linear_speed_mps', 0.10)
        self.declare_parameter('max_wz_radps', 0.20)
        self.declare_parameter('max_linear_accel_mps2', 0.30)
        self.declare_parameter('max_angular_accel_radps2', 0.80)
        self.declare_parameter('kp_along', 0.9)
        self.declare_parameter('kp_cross_track', 1.2)
        self.declare_parameter('kp_yaw', 1.5)
        self.declare_parameter('lookahead_m', 0.20)
        self.declare_parameter('recovery_enter_m', 0.08)
        self.declare_parameter('recovery_exit_m', 0.03)
        self.declare_parameter('recovery_settle_s', 0.50)
        self.declare_parameter('max_recoverable_deviation_m', 0.45)
        self.declare_parameter('position_tolerance_m', 0.02)
        self.declare_parameter('yaw_tolerance_rad', 0.035)
        self.declare_parameter('world_frame', 'map')

        get = lambda name: self.get_parameter(name).value
        self.rate_hz = float(get('control_rate_hz'))
        self.pose_timeout_s = float(get('pose_timeout_s'))
        self.nominal_speed = float(get('nominal_speed_mps'))
        self.recovery_tangent_speed = float(get('recovery_tangent_speed_mps'))
        self.tracking_cross_speed = float(get('tracking_cross_speed_mps'))
        self.recovery_cross_speed = float(get('recovery_cross_speed_mps'))
        self.max_linear_speed = float(get('max_linear_speed_mps'))
        self.max_wz = float(get('max_wz_radps'))
        self.max_linear_accel = float(get('max_linear_accel_mps2'))
        self.max_angular_accel = float(get('max_angular_accel_radps2'))
        self.kp_along = float(get('kp_along'))
        self.kp_cross = float(get('kp_cross_track'))
        self.kp_yaw = float(get('kp_yaw'))
        self.lookahead = float(get('lookahead_m'))
        self.recovery_enter = float(get('recovery_enter_m'))
        self.recovery_exit = float(get('recovery_exit_m'))
        self.recovery_settle = float(get('recovery_settle_s'))
        self.max_recoverable = float(get('max_recoverable_deviation_m'))
        self.position_tolerance = float(get('position_tolerance_m'))
        self.yaw_tolerance = float(get('yaw_tolerance_rad'))
        self.world_frame = str(get('world_frame'))
        self._validate_parameters()

        self.pose = None
        self.goal = None
        self.segment_start = None
        self.trajectory_active = False
        self.last_pose_receipt_ns = None
        self.state = WAITING
        self.fault_latched = False
        self.settled_since_s = None
        self.command = Twist()

        self.cmd_pub = self.create_publisher(
            Twist, str(get('cmd_topic')), 10)
        self.reached_pub = self.create_publisher(Bool, '/car/goal_reached', 10)
        self.state_pub = self.create_publisher(String, '/car/recovery_state', 10)
        self.cross_pub = self.create_publisher(
            Float64, '/car/cross_track_error', 10)
        self.progress_pub = self.create_publisher(
            Float64, '/car/segment_progress', 10)
        self.projection_pub = self.create_publisher(
            PoseStamped, '/car/path_projection', 10)
        self.target_pub = self.create_publisher(
            PoseStamped, '/car/recovery_target', 10)

        newest = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseStamped, str(get('pose_topic')), self.on_pose, newest)
        self.create_subscription(
            PoseStamped, str(get('goal_topic')), self.on_goal, latched)
        self.create_subscription(
            Bool, '/car/trajectory_active', self.on_trajectory_active, latched)
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            '连续路径恢复控制器已启动；等待轨迹使能和新鲜位姿')

    def _validate_parameters(self):
        positive = (
            self.rate_hz, self.pose_timeout_s, self.nominal_speed,
            self.recovery_cross_speed, self.max_linear_speed,
            self.max_wz, self.max_linear_accel, self.max_angular_accel,
            self.kp_along, self.kp_cross, self.kp_yaw,
            self.recovery_settle, self.max_recoverable,
            self.position_tolerance, self.yaw_tolerance,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('path recovery positive parameters must be finite and positive')
        nonnegative = (
            self.recovery_tangent_speed, self.tracking_cross_speed, self.lookahead,
            self.recovery_exit,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in nonnegative):
            raise ValueError('path recovery nonnegative parameters are invalid')
        if not self.recovery_exit < self.recovery_enter < self.max_recoverable:
            raise ValueError('recovery thresholds must satisfy exit < enter < maximum')
        if self.nominal_speed > self.max_linear_speed:
            raise ValueError('nominal speed must not exceed max linear speed')
        if max(self.tracking_cross_speed, self.recovery_cross_speed) > self.max_linear_speed:
            raise ValueError('cross-track speed limit must not exceed max linear speed')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'路径恢复状态: {self.state} -> {state}')
            self.state = state

    def on_pose(self, msg: PoseStamped):
        if msg.header.frame_id != self.world_frame:
            return
        self.pose = msg
        self.last_pose_receipt_ns = self.get_clock().now().nanoseconds

    def on_goal(self, msg: PoseStamped):
        if msg.header.frame_id != self.world_frame:
            self.get_logger().error('拒绝与world_frame不一致的轨迹目标')
            return
        if self.goal is not None:
            self.segment_start = (
                self.goal.pose.position.x, self.goal.pose.position.y)
        elif self.segment_start is None and self.pose is not None:
            self.segment_start = (
                self.pose.pose.position.x, self.pose.pose.position.y)
        self.goal = msg
        self.settled_since_s = None
        if self.trajectory_active and not self.fault_latched:
            self.set_state(TRACKING)

    def on_trajectory_active(self, msg: Bool):
        active = bool(msg.data)
        if not active:
            self.trajectory_active = False
            self.goal = None
            self.segment_start = None
            self.fault_latched = False
            self.settled_since_s = None
            self.command = Twist()
            self.set_state(WAITING)
            return
        if not self.trajectory_active:
            self.segment_start = None if self.pose is None else (
                self.pose.pose.position.x, self.pose.pose.position.y)
            self.fault_latched = False
            self.settled_since_s = None
        self.trajectory_active = True
        self.set_state(TRACKING)

    def pose_is_fresh(self, now_ns):
        return (
            self.last_pose_receipt_ns is not None
            and 0 <= (now_ns - self.last_pose_receipt_ns) * 1e-9 <= self.pose_timeout_s)

    def publish_debug_pose(self, publisher, x, y, yaw):
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
        publisher.publish(msg)

    def publish_zero(self, reached=False):
        self.command = Twist()
        self.cmd_pub.publish(self.command)
        self.reached_pub.publish(Bool(data=reached))
        self.state_pub.publish(String(data=self.state))

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if not self.trajectory_active:
            self.publish_zero()
            return
        if not self.pose_is_fresh(now_ns):
            self.fault_latched = True
            self.set_state(VICON_LOST)
            self.publish_zero()
            return
        if self.fault_latched:
            self.publish_zero()
            return
        if self.pose is None or self.goal is None or self.segment_start is None:
            self.set_state(WAITING)
            self.publish_zero()
            return

        point = (self.pose.pose.position.x, self.pose.pose.position.y)
        end = (self.goal.pose.position.x, self.goal.pose.position.y)
        pose_yaw = yaw_from_quaternion(
            self.pose.pose.orientation.x, self.pose.pose.orientation.y,
            self.pose.pose.orientation.z, self.pose.pose.orientation.w)
        goal_yaw = yaw_from_quaternion(
            self.goal.pose.orientation.x, self.goal.pose.orientation.y,
            self.goal.pose.orientation.z, self.goal.pose.orientation.w)
        geometry = segment_geometry(
            point, self.segment_start, end, self.lookahead)
        goal_distance = math.dist(point, end)
        yaw_error = wrap_angle(goal_yaw - pose_yaw)
        reached = (
            goal_distance <= self.position_tolerance
            and abs(yaw_error) <= self.yaw_tolerance)
        if reached:
            self.set_state(COMPLETE)
            self.publish_zero(reached=True)
            return

        abs_cross = abs(geometry.cross_track_m)
        if abs_cross > self.max_recoverable:
            self.fault_latched = True
            self.set_state(HOLD)
            self.publish_zero()
            return
        next_state, self.settled_since_s = update_recovery_state(
            self.state if self.state in (TRACKING, RECOVERY) else TRACKING,
            abs_cross, self.recovery_enter, self.recovery_exit,
            now_ns * 1e-9, self.settled_since_s)
        self.set_state(next_state)
        if (
                self.state == RECOVERY
                and self.settled_since_s is not None
                and now_ns * 1e-9 - self.settled_since_s >= self.recovery_settle):
            self.set_state(TRACKING)
            self.settled_since_s = None

        world_vx, world_vy = bounded_world_velocity(
            geometry, self.state, self.nominal_speed,
            self.recovery_tangent_speed, self.kp_along, self.kp_cross,
            self.tracking_cross_speed, self.recovery_cross_speed,
            self.max_linear_speed)
        target_vx, target_vy = world_error_to_body(world_vx, world_vy, pose_yaw)
        target_wz = clamp(self.kp_yaw * yaw_error, -self.max_wz, self.max_wz)
        dt = 1.0 / self.rate_hz
        self.command.linear.x = slew(
            self.command.linear.x, target_vx, self.max_linear_accel * dt)
        self.command.linear.y = slew(
            self.command.linear.y, target_vy, self.max_linear_accel * dt)
        self.command.angular.z = slew(
            self.command.angular.z, target_wz, self.max_angular_accel * dt)

        self.cmd_pub.publish(self.command)
        self.reached_pub.publish(Bool(data=False))
        self.state_pub.publish(String(data=self.state))
        self.cross_pub.publish(Float64(data=geometry.cross_track_m))
        self.progress_pub.publish(Float64(data=geometry.progress))
        segment_yaw = math.atan2(geometry.tangent_y, geometry.tangent_x)
        self.publish_debug_pose(
            self.projection_pub, geometry.projection_x, geometry.projection_y,
            segment_yaw)
        self.publish_debug_pose(
            self.target_pub, geometry.target_x, geometry.target_y, segment_yaw)


def main():
    rclpy.init()
    try:
        node = PathRecoveryController()
    except (TypeError, ValueError) as error:
        rclpy.logging.get_logger('path_recovery_controller').error(str(error))
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
