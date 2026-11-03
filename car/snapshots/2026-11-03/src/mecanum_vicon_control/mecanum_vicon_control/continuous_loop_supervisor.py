#!/usr/bin/env python3
"""Service-controlled, indefinitely repeating hardware trajectory supervisor.

The trajectory itself remains continuous.  At every completed lap this node
briefly closes both motion gates, confirms stationary zero output, safely
rearms the bounded STM32 run budget, and resumes the already anchored path.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32
from std_srvs.srv import Trigger

from .path_recovery_controller import HOLD, VICON_LOST
from .rectangle_run_guard import (
    disabled_motion_status_ready,
    motion_status_needs_rearm,
    rearmed_motion_status_ready,
    vicon_valid_with_grace,
)


STANDBY = 'STANDBY'
PREFLIGHT = 'PREFLIGHT'
WAIT_DISABLED = 'WAIT_DISABLED'
WAIT_REARM = 'WAIT_REARM'
WAIT_MCU_ENABLE = 'WAIT_MCU_ENABLE'
WAIT_SAFETY = 'WAIT_SAFETY'
WAIT_TRAJECTORY = 'WAIT_TRAJECTORY'
RUNNING = 'RUNNING'
LAP_WAIT_DISABLED = 'LAP_WAIT_DISABLED'
STOPPING = 'STOPPING'


def lap_incremented(previous, current):
    """Return true only for a new completed lap, not a new-run reset to zero."""
    return previous is not None and current is not None and current > previous


def recovery_state_allows_motion(state):
    return state not in (HOLD, VICON_LOST)


class ContinuousLoopSupervisor(Node):
    """Safely start, stop and per-lap rearm a continuous rectangle."""

    def __init__(self):
        super().__init__('continuous_loop_supervisor')
        self.declare_parameter('required_motion_tag', '4WGB')
        self.declare_parameter('preflight_timeout_s', 15.0)
        self.declare_parameter('enable_timeout_s', 5.0)
        self.declare_parameter('lap_timeout_s', 140.0)
        self.declare_parameter('vicon_transient_grace_s', 0.15)
        self.declare_parameter('lease_rate_hz', 20.0)

        self.required_motion_tag = str(
            self.get_parameter('required_motion_tag').value).strip()
        self.preflight_timeout_s = float(
            self.get_parameter('preflight_timeout_s').value)
        self.enable_timeout_s = float(
            self.get_parameter('enable_timeout_s').value)
        self.lap_timeout_s = float(
            self.get_parameter('lap_timeout_s').value)
        self.vicon_transient_grace_s = float(
            self.get_parameter('vicon_transient_grace_s').value)
        self.lease_rate_hz = float(self.get_parameter('lease_rate_hz').value)
        if self.required_motion_tag not in ('4WG9', '4WGA', '4WGB'):
            raise ValueError('continuous looping requires a safe-rearm motion image')
        if not 0.0 < self.lap_timeout_s <= 140.0:
            raise ValueError('lap_timeout_s must be in (0, 140] for the 150 s MCU limit')
        if not 0.0 <= self.vicon_transient_grace_s < 0.20:
            raise ValueError('vicon_transient_grace_s must be in [0, 0.20)')
        if self.lease_rate_hz < 10.0:
            raise ValueError('lease_rate_hz must be at least 10 Hz')

        self.control_pub = self.create_publisher(Bool, '/car/control_enable', 10)
        self.rearm_pub = self.create_publisher(Bool, '/car/clear_faults', 10)
        self.safety_pub = self.create_publisher(Bool, '/car/enable', 10)
        self.trajectory_pub = self.create_publisher(
            Bool, '/car/trajectory_enable', 10)
        self.lease_pub = self.create_publisher(Bool, '/car/run_lease', 10)
        self.status_pub = self.create_publisher(
            String, '/car/loop_supervisor_status', 10)
        self.running_pub = self.create_publisher(Bool, '/car/loop_running', 10)

        self.create_service(Trigger, '/car/start_loop', self.on_start)
        self.create_service(Trigger, '/car/stop', self.on_stop)

        newest = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/car/vicon_valid', self.on_vicon, newest)
        self.create_subscription(Bool, '/car/mc_link_connected', self.on_link, 10)
        self.create_subscription(
            UInt32, '/car/motor_fault_bits', self.on_fault, 10)
        self.create_subscription(
            String, '/car/mc_link_diagnostics', self.on_diagnostics, 10)
        self.create_subscription(
            Bool, '/car/trajectory_active', self.on_trajectory, latched)
        self.create_subscription(
            UInt32, '/car/completed_loops', self.on_completed_loops, latched)
        self.create_subscription(
            Bool, '/car/safety_enabled', self.on_safety, 10)
        self.create_subscription(
            String, '/car/recovery_state', self.on_recovery_state, 10)

        self.vicon = self.link = self.fault = self.diagnostics = None
        self.trajectory_active = self.safety_enabled = None
        self.completed_loops = None
        self.recovery_state = None
        self.last_vicon_true_monotonic = None
        self.stamps = {}

        self.desired_run = False
        self.resume_after_rearm = False
        self.state = STANDBY
        self.state_started = time.monotonic()
        self.lap_started = None
        self.last_lap_seen = None
        self.last_stop_reason = 'not started'
        self.create_timer(1.0 / self.lease_rate_hz, self.tick)
        self.get_logger().info(
            '连续矩形监督器已启动；默认失能，等待/car/start_loop服务')

    def stamp(self, name):
        self.stamps[name] = time.monotonic()

    def fresh(self, name, maximum_age):
        return name in self.stamps and time.monotonic() - self.stamps[name] <= maximum_age

    def on_vicon(self, msg):
        self.vicon = bool(msg.data)
        if self.vicon:
            self.last_vicon_true_monotonic = time.monotonic()
        self.stamp('vicon')

    def on_link(self, msg):
        self.link = bool(msg.data)
        self.stamp('link')

    def on_fault(self, msg):
        self.fault = int(msg.data)
        self.stamp('fault')

    def on_diagnostics(self, msg):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(value, dict):
            self.diagnostics = value
            self.stamp('diagnostics')

    def on_trajectory(self, msg):
        self.trajectory_active = bool(msg.data)
        self.stamp('trajectory')

    def on_completed_loops(self, msg):
        self.completed_loops = int(msg.data)
        self.stamp('loops')

    def on_safety(self, msg):
        self.safety_enabled = bool(msg.data)
        self.stamp('safety')

    def on_recovery_state(self, msg):
        self.recovery_state = str(msg.data)
        self.stamp('recovery')

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'连续运行状态: {self.state} -> {state}')
            self.state = state
        self.state_started = time.monotonic()

    @staticmethod
    def publish(publisher, value):
        publisher.publish(Bool(data=value))

    def publish_stop(self):
        self.publish(self.safety_pub, False)
        self.publish(self.trajectory_pub, False)
        self.publish(self.control_pub, False)

    def publish_lease_and_status(self):
        lease = self.desired_run and self.state not in (STANDBY, STOPPING)
        self.publish(self.lease_pub, lease)
        self.publish(self.running_pub, self.state == RUNNING)
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state,
            'desired_run': self.desired_run,
            'completed_loops': self.completed_loops,
            'last_stop_reason': self.last_stop_reason,
        }, separators=(',', ':'))))

    def on_start(self, _request, response):
        if self.desired_run:
            response.success = True
            response.message = f'already active: {self.state}'
            return response
        if self.state == STOPPING:
            response.success = False
            response.message = 'stop sequence is still settling; retry shortly'
            return response
        self.desired_run = True
        self.resume_after_rearm = False
        self.last_lap_seen = None
        self.last_stop_reason = ''
        self.publish_stop()
        self.set_state(PREFLIGHT)
        response.success = True
        response.message = 'start accepted; asynchronous safety preflight has begun'
        return response

    def on_stop(self, _request, response):
        if not self.desired_run and self.state == STANDBY:
            self.publish_stop()
            self.publish(self.lease_pub, False)
            response.success = True
            response.message = 'already stopped; stop outputs repeated'
            return response
        self.begin_stop('operator stop command')
        response.success = True
        response.message = 'stop accepted; all motion gates are closing'
        return response

    def vicon_healthy(self):
        now = time.monotonic()
        return vicon_valid_with_grace(
            self.vicon, self.fresh('vicon', 0.30),
            self.last_vicon_true_monotonic, now,
            self.vicon_transient_grace_s)

    def preflight_ready(self):
        return (
            self.vicon is True and self.fresh('vicon', 0.30)
            and self.link is True and self.fresh('link', 0.60)
            and self.fault == 0 and self.fresh('fault', 0.60)
            and self.fresh('diagnostics', 0.35)
            and self.diagnostics is not None
            and self.diagnostics.get('tag') == self.required_motion_tag
            and disabled_motion_status_ready(self.diagnostics)
            and self.safety_enabled is False and self.fresh('safety', 0.30)
        )

    def rearm_ready(self):
        return self.preflight_ready() and rearmed_motion_status_ready(self.diagnostics)

    def enable_confirmed(self):
        return (
            self.link is True and self.fresh('link', 0.60)
            and self.fault == 0 and self.fresh('fault', 0.60)
            and self.fresh('diagnostics', 0.35)
            and self.diagnostics is not None
            and self.diagnostics.get('tag') == self.required_motion_tag
            and bool(self.diagnostics.get('permitted'))
        )

    def runtime_healthy(self):
        return (
            self.vicon_healthy()
            and self.enable_confirmed()
            and self.safety_enabled is True and self.fresh('safety', 0.30)
            and self.trajectory_active is True
            and self.fresh('recovery', 0.30)
            and recovery_state_allows_motion(self.recovery_state)
        )

    def begin_stop(self, reason, fault=False):
        if self.state == STOPPING:
            return
        self.desired_run = False
        self.resume_after_rearm = False
        self.last_stop_reason = reason
        self.publish_stop()
        self.publish(self.lease_pub, False)
        self.set_state(STOPPING)
        message = f'停止连续矩形并关闭全部运动门: {reason}'
        # rcutils caches a logging call site's severity.  Calling a bound
        # logger method selected at runtime from this same line can therefore
        # raise when a later stop uses the other severity.  Keep distinct call
        # sites for normal stops and fault stops.
        if fault:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def request_rearm_or_enable(self):
        if motion_status_needs_rearm(self.diagnostics):
            self.publish(self.rearm_pub, True)
            self.set_state(WAIT_REARM)
            self.get_logger().info('静止零输出已确认；请求安全重装填STM32运行额度')
        else:
            self.publish(self.control_pub, True)
            self.set_state(WAIT_MCU_ENABLE)
            self.get_logger().info('当前运行额度未使用；请求STM32运动门使能')

    def pause_at_lap_boundary(self):
        self.resume_after_rearm = True
        self.publish(self.safety_pub, False)
        self.publish(self.control_pub, False)
        self.set_state(LAP_WAIT_DISABLED)
        self.get_logger().info(
            f'第{self.completed_loops}圈完成；保持原轨迹锚点并安全重装填')

    def tick(self):
        now = time.monotonic()
        self.publish_lease_and_status()

        if self.state == STANDBY:
            return
        if self.state == STOPPING:
            if now - self.state_started >= 1.0:
                self.set_state(STANDBY)
            return
        if not self.desired_run:
            self.begin_stop('run request withdrawn')
            return

        if self.state == PREFLIGHT:
            if self.preflight_ready():
                self.publish(self.control_pub, False)
                self.set_state(WAIT_DISABLED)
            elif now - self.state_started > self.preflight_timeout_s:
                self.begin_stop('preflight timeout', fault=True)
            return

        if self.state in (WAIT_DISABLED, LAP_WAIT_DISABLED):
            if not self.vicon_healthy():
                self.begin_stop('Vicon invalid while confirming disabled state', fault=True)
            elif self.fault not in (None, 0):
                self.begin_stop(f'STM32 fault while disabled: {self.fault:#x}', fault=True)
            elif self.state == LAP_WAIT_DISABLED and self.trajectory_active is not True:
                self.begin_stop('trajectory unexpectedly stopped at lap boundary', fault=True)
            elif now - self.state_started >= 0.30 and self.preflight_ready():
                self.request_rearm_or_enable()
            elif now - self.state_started > self.enable_timeout_s:
                self.begin_stop('disabled zero-output confirmation timeout', fault=True)
            return

        if self.state == WAIT_REARM:
            if not self.vicon_healthy():
                self.begin_stop('Vicon invalid during safe rearm', fault=True)
            elif self.fault not in (None, 0):
                self.begin_stop(f'STM32 fault during safe rearm: {self.fault:#x}', fault=True)
            elif now - self.state_started >= 1.0 and self.rearm_ready():
                self.publish(self.control_pub, True)
                self.set_state(WAIT_MCU_ENABLE)
            elif now - self.state_started > self.enable_timeout_s:
                self.begin_stop('safe rearm timeout', fault=True)
            return

        if self.state == WAIT_MCU_ENABLE:
            if not self.vicon_healthy():
                self.begin_stop('Vicon invalid while enabling MCU gate', fault=True)
            elif self.fault not in (None, 0):
                self.begin_stop(f'STM32 fault while enabling: {self.fault:#x}', fault=True)
            elif self.enable_confirmed():
                self.publish(self.safety_pub, True)
                self.set_state(WAIT_SAFETY)
            elif now - self.state_started > self.enable_timeout_s:
                self.begin_stop('STM32 motion-gate enable timeout', fault=True)
            return

        if self.state == WAIT_SAFETY:
            if not self.vicon_healthy() or not self.enable_confirmed():
                self.begin_stop('health lost before ROS speed gate opened', fault=True)
            elif self.safety_enabled is True:
                if self.resume_after_rearm:
                    if self.trajectory_active is not True:
                        self.begin_stop('trajectory inactive while resuming lap', fault=True)
                        return
                    self.resume_after_rearm = False
                    self.lap_started = now
                    self.last_lap_seen = self.completed_loops
                    self.set_state(RUNNING)
                else:
                    self.last_lap_seen = 0
                    self.publish(self.trajectory_pub, True)
                    self.set_state(WAIT_TRAJECTORY)
            elif now - self.state_started > 1.0:
                self.begin_stop('ROS speed gate refused enable', fault=True)
            return

        if self.state == WAIT_TRAJECTORY:
            if not self.vicon_healthy() or not self.enable_confirmed():
                self.begin_stop('health lost while starting trajectory', fault=True)
            elif self.trajectory_active is True:
                self.lap_started = now
                self.last_lap_seen = self.completed_loops
                self.set_state(RUNNING)
            elif now - self.state_started > 2.0:
                self.begin_stop('trajectory did not activate', fault=True)
            return

        if self.state == RUNNING:
            if not self.runtime_healthy():
                self.begin_stop('Vicon, link, MCU, trajectory or recovery state invalid', fault=True)
            elif (self.completed_loops is not None
                    and (self.last_lap_seen is None
                         or self.completed_loops < self.last_lap_seen)):
                # A new operator-started trajectory resets its latched count
                # to zero.  Treat that as a baseline, never as a completed lap.
                self.last_lap_seen = self.completed_loops
            elif lap_incremented(self.last_lap_seen, self.completed_loops):
                self.last_lap_seen = self.completed_loops
                self.pause_at_lap_boundary()
            elif self.lap_started is not None and now - self.lap_started > self.lap_timeout_s:
                self.begin_stop(
                    f'lap exceeded {self.lap_timeout_s:.1f} s safety limit', fault=True)


def main():
    rclpy.init()
    node = ContinuousLoopSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # A launch-wide SIGINT may already have invalidated the ROS context.
        # Only publish the redundant stop sequence while publishers remain
        # usable; the bridge and MCU leases independently expire on shutdown.
        if rclpy.ok():
            try:
                node.begin_stop('supervisor interrupted')
                deadline = time.monotonic() + 0.75
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
            except Exception:
                # launch can invalidate the publisher between rclpy.ok() and
                # publish(); shutdown leases still force independent zero.
                pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
