#!/usr/bin/env python3
"""One-shot guarded start/stop sequence for the real Vicon rectangle run."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32
from .motion_link_status import MOTION_TAGS


def disabled_motion_status_ready(status):
    """Return true only for a stationary, output-zero bounded-motion image."""
    if not isinstance(status, dict) or status.get('tag') not in MOTION_TAGS:
        return False
    numeric_lists = ('compares', 'raw_duty', 'target_cps', 'body_milli')
    try:
        return (
            not bool(status['permitted'])
            and int(status['stationary_ms']) >= 500
            and all(not any(status[name]) for name in numeric_lists)
        )
    except (KeyError, TypeError, ValueError):
        return False


def rearmed_motion_status_ready(status):
    """Require a safe zero image whose per-run budget is reset."""
    try:
        return disabled_motion_status_ready(status) and int(status['started_ms']) == 0
    except (KeyError, TypeError, ValueError):
        return False


def motion_status_needs_rearm(status):
    """A never-used or already-rearmed 4WG9 image can enable directly."""
    try:
        return int(status['started_ms']) != 0
    except (KeyError, TypeError, ValueError):
        return True


def vicon_valid_with_grace(current_valid, current_fresh, last_true, now, grace_s):
    """Tolerate only a sub-watchdog transient invalid sample after preflight."""
    if current_valid is True and current_fresh:
        return True
    return last_true is not None and 0.0 <= now - last_true <= grace_s


class RectangleRunGuard(Node):
    """Wait for live Vicon/bounded motion, run one armed trajectory, then disarm."""

    def __init__(self):
        super().__init__('rectangle_run_guard')
        self.declare_parameter('preflight_timeout_s', 15.0)
        self.declare_parameter('enable_timeout_s', 5.0)
        self.declare_parameter('run_timeout_s', 100.0)
        self.declare_parameter('required_motion_tag', '4WG7')
        self.declare_parameter('vicon_transient_grace_s', 0.15)

        self.preflight_timeout_s = float(self.get_parameter('preflight_timeout_s').value)
        self.enable_timeout_s = float(self.get_parameter('enable_timeout_s').value)
        self.run_timeout_s = float(self.get_parameter('run_timeout_s').value)
        self.required_motion_tag = str(
            self.get_parameter('required_motion_tag').value).strip()
        self.vicon_transient_grace_s = float(
            self.get_parameter('vicon_transient_grace_s').value)
        if not 0.0 <= self.vicon_transient_grace_s < 0.20:
            raise ValueError('vicon_transient_grace_s must be in [0, 0.20)')
        if self.required_motion_tag not in MOTION_TAGS:
            raise ValueError('required_motion_tag is not a known bounded-motion image')
        # 4WG9 and its recovery-demo derivative are the images whose
        # clear-latched-faults bit has the
        # deliberately narrow "rearm an already healthy run budget" meaning.
        # Older images must keep their established one-shot start sequence.
        self.safe_rearm_supported = self.required_motion_tag in ('4WG9', '4WGA', '4WGB')

        self.control_pub = self.create_publisher(Bool, '/car/control_enable', 10)
        self.rearm_pub = self.create_publisher(Bool, '/car/clear_faults', 10)
        self.safety_pub = self.create_publisher(Bool, '/car/enable', 10)
        self.trajectory_pub = self.create_publisher(Bool, '/car/trajectory_enable', 10)

        latest_state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        # Keep only the newest health bit.  car_state_bridge publishes normal
        # health at 20 Hz and publishes invalid input immediately.
        self.create_subscription(
            Bool, '/car/vicon_valid', self.on_vicon, latest_state_qos)
        self.create_subscription(Bool, '/car/mc_link_connected', self.on_link, 10)
        self.create_subscription(UInt32, '/car/motor_fault_bits', self.on_fault, 10)
        self.create_subscription(String, '/car/mc_link_diagnostics', self.on_diagnostics, 10)
        self.create_subscription(Bool, '/car/trajectory_active', self.on_trajectory, 10)
        self.create_subscription(Bool, '/car/safety_enabled', self.on_safety, 10)

        self.vicon = self.link = self.fault = self.diagnostics = None
        self.last_vicon_true_monotonic = None
        self.trajectory_active = self.safety_enabled = None
        self.stamps = {}
        self.state = 'preflight'
        self.state_started = time.monotonic()
        self.run_started = None
        self.exit_ok = False
        self.stop_reason = ''
        self.create_timer(0.05, self.tick)
        start_mode = '先安全重新布防再执行一次轨迹' if self.safe_rearm_supported \
            else '按旧版一次性运动门流程执行轨迹'
        self.get_logger().info(
            '等待新鲜Vicon、MC-Link、零故障和受限运动固件静止零输出；'
            + start_mode)

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
        if isinstance(value, dict) and value.get('tag') in MOTION_TAGS:
            self.diagnostics = value
            self.stamp('diagnostics')

    def on_trajectory(self, msg):
        self.trajectory_active = bool(msg.data)
        self.stamp('trajectory')

    def on_safety(self, msg):
        self.safety_enabled = bool(msg.data)
        self.stamp('safety')

    def preflight_ready(self):
        return (
            self.vicon is True and self.fresh('vicon', 0.30)
            and self.link is True and self.fresh('link', 0.60)
            and self.fault == 0 and self.fresh('fault', 0.60)
            and self.fresh('diagnostics', 0.35)
            and self.diagnostics.get('tag') == self.required_motion_tag
            and disabled_motion_status_ready(self.diagnostics)
            and self.safety_enabled is False and self.fresh('safety', 0.30)
        )

    def rearm_ready(self):
        return self.preflight_ready() and rearmed_motion_status_ready(self.diagnostics)

    def runtime_healthy(self):
        return (
            self.vicon_healthy()
            and self.link is True and self.fresh('link', 0.60)
            and self.fault == 0 and self.fresh('fault', 0.60)
            and self.fresh('diagnostics', 0.35)
            and self.diagnostics is not None
            and self.diagnostics.get('tag') == self.required_motion_tag
            and bool(self.diagnostics.get('permitted'))
        )

    def vicon_healthy(self):
        now = time.monotonic()
        return vicon_valid_with_grace(
            self.vicon, self.fresh('vicon', 0.30),
            self.last_vicon_true_monotonic, now,
            self.vicon_transient_grace_s)

    def enable_confirmed(self):
        return (
            self.link is True and self.fresh('link', 0.60)
            and self.fault == 0 and self.fresh('fault', 0.60)
            and self.fresh('diagnostics', 0.35)
            and self.diagnostics.get('tag') == self.required_motion_tag
            and bool(self.diagnostics.get('permitted'))
        )

    def set_state(self, state):
        self.state = state
        self.state_started = time.monotonic()

    def publish(self, publisher, value):
        publisher.publish(Bool(data=value))

    def begin_stop(self, reason, success=False):
        if self.state in ('stopping', 'finished'):
            return
        self.stop_reason = reason
        self.exit_ok = success
        self.set_state('stopping')
        self.get_logger().info(f'停止并失能: {reason}') if success else self.get_logger().error(
            f'中止并失能: {reason}')
        self.publish_stop()

    def publish_stop(self):
        self.publish(self.safety_pub, False)
        self.publish(self.trajectory_pub, False)
        self.publish(self.control_pub, False)

    def finish(self):
        self.state = 'finished'
        if self.exit_ok:
            self.get_logger().info('矩形单圈完成，ROS输出和STM32运动门均已请求失能')
        rclpy.shutdown()

    def tick(self):
        now = time.monotonic()

        if self.state == 'preflight':
            if self.preflight_ready():
                # Always establish an explicit disabled boundary first.  This
                # also acknowledges a host-side stop latch caused by an idle
                # transport-session expiry; fresh zero/fault-free feedback is
                # still required before the later enable can clear that latch.
                self.publish(self.control_pub, False)
                self.set_state('wait_disabled')
                self.get_logger().info(
                    '预检通过；正在显式失能并确认零输出边界')
            elif now - self.state_started > self.preflight_timeout_s:
                self.begin_stop('预检超时')
            return

        if self.state == 'wait_disabled':
            if not self.vicon_healthy():
                self.begin_stop('显式失能确认时Vicon状态失效')
            elif self.fault not in (None, 0):
                self.begin_stop(f'显式失能确认时STM32故障位={self.fault:#x}')
            elif now - self.state_started >= .30 and self.preflight_ready():
                if (self.safe_rearm_supported
                        and motion_status_needs_rearm(self.diagnostics)):
                    self.publish(self.rearm_pub, True)
                    self.set_state('wait_rearm')
                    self.get_logger().info(
                        '显式失能和零输出已确认；正在请求安全重新布防')
                else:
                    self.publish(self.control_pub, True)
                    self.set_state('wait_mcu_enable')
                    reason = ('当前运行额度尚未使用；' if self.safe_rearm_supported
                              else '旧版固件按既有流程；')
                    self.get_logger().info(
                        '显式失能和零输出已确认；' + reason
                        + '请求STM32受限运动门使能')
            elif now - self.state_started > self.enable_timeout_s:
                self.begin_stop('显式失能零输出确认超时')
            return

        if self.state == 'wait_rearm':
            if not self.vicon_healthy():
                self.begin_stop('重新布防时Vicon状态失效')
            elif self.fault not in (None, 0):
                self.begin_stop(f'重新布防时STM32故障位={self.fault:#x}')
            elif now - self.state_started >= 1.0 and self.rearm_ready():
                self.publish(self.control_pub, True)
                self.set_state('wait_mcu_enable')
                self.get_logger().info('安全重新布防已确认；正在请求STM32受限运动门使能')
            elif now - self.state_started > self.enable_timeout_s:
                self.begin_stop('安全重新布防超时')
            return

        if self.state == 'wait_mcu_enable':
            # CONTROL_ENABLE intentionally starts a fresh UDP session.  The
            # resulting brief link/diagnostic gap is expected here, but Vicon
            # loss or an explicit MCU fault still aborts immediately.
            if not self.vicon_healthy():
                self.begin_stop('等待使能时Vicon状态失效')
            elif self.fault not in (None, 0):
                self.begin_stop(f'等待使能时STM32故障位={self.fault:#x}')
            elif self.enable_confirmed():
                # Keep the pose controller at its pre-goal zero while opening
                # the ROS gate.  This removes any gap between the MCU permit
                # and the first flagged-zero lease, without allowing motion.
                self.publish(self.safety_pub, True)
                self.set_state('wait_safety')
                self.get_logger().info('STM32运动门已使能；正在以零目标打开ROS速度安全门')
            elif now - self.state_started > self.enable_timeout_s:
                self.begin_stop('STM32运动门使能超时')
            return

        if self.state == 'wait_trajectory':
            if not self.runtime_healthy():
                self.begin_stop('轨迹启动前Vicon、链路或故障状态失效')
            elif self.trajectory_active is True:
                self.run_started = now
                self.set_state('running')
                self.get_logger().info('轨迹已激活；矩形运动已开始')
            elif now - self.state_started > 2.0:
                self.begin_stop('轨迹未能锚定新鲜Vicon位姿')
            return

        if self.state == 'wait_safety':
            if not self.runtime_healthy():
                self.begin_stop('ROS安全门使能前Vicon、链路或故障状态失效')
            elif self.safety_enabled is True:
                self.publish(self.trajectory_pub, True)
                self.set_state('wait_trajectory')
                self.get_logger().info('ROS速度安全门已确认零输出；正在锚定并启动矩形目标')
            elif now - self.state_started > 1.0:
                self.begin_stop('ROS速度安全门拒绝使能')
            return

        if self.state == 'running':
            if not self.runtime_healthy():
                self.begin_stop('运行中Vicon、链路或故障状态失效')
            elif self.safety_enabled is not True:
                self.begin_stop('运行中ROS速度安全门关闭')
            elif self.trajectory_active is False:
                self.begin_stop('轨迹报告单圈完成', success=True)
            elif now - self.run_started > self.run_timeout_s:
                self.begin_stop(f'矩形运行超过{self.run_timeout_s:.1f}秒硬截止')
            return

        if self.state == 'stopping':
            # The bridge itself repeats the disable frame five times.  Do not
            # republish here: each new request intentionally starts a fresh UDP
            # session and would otherwise keep restarting that sequence.
            if now - self.state_started >= 1.0:
                self.finish()


def main():
    rclpy.init()
    node = RectangleRunGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.begin_stop('操作者按下Ctrl+C')
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
