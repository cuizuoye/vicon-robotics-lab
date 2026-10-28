#!/usr/bin/env python3
"""Independent bounded cutoff for suspended rectangle actuation."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray, UInt32


def startup_deadline_expired(motion_started, now, deadline):
    """The startup deadline must never truncate an already-started motion."""
    return motion_started is None and now >= deadline


class Cutoff(Node):
    def __init__(self):
        super().__init__('suspended_rectangle_cutoff')
        self.declare_parameter('motion_duration_s', 1.0)
        self.declare_parameter('startup_timeout_s', 8.0)
        self.motion_duration_s = float(self.get_parameter('motion_duration_s').value)
        self.startup_timeout_s = float(self.get_parameter('startup_timeout_s').value)
        if not 0.1 <= self.motion_duration_s <= 90.0:
            raise ValueError('motion_duration_s must be between 0.1 and 90.0 seconds')
        if self.startup_timeout_s <= 0.0:
            raise ValueError('startup_timeout_s must be positive')
        self.safety = False
        self.fault = None
        self.encoders = None
        self.start_encoders = None
        self.last_encoders = None
        self.max_abs_rpm = [0.0] * 4
        self.motion_started = None
        self.absolute_deadline = time.monotonic() + self.startup_timeout_s
        self.stop_started = None
        self.prestart_abort = False
        self.reason = ''

        self.safety_pub = self.create_publisher(Bool, '/car/enable', 10)
        self.trajectory_pub = self.create_publisher(Bool, '/car/trajectory_enable', 10)
        self.control_pub = self.create_publisher(Bool, '/car/control_enable', 10)
        self.create_subscription(Bool, '/car/safety_enabled', self.on_safety, 10)
        self.create_subscription(UInt32, '/car/motor_fault_bits', self.on_fault, 10)
        self.create_subscription(Int32MultiArray, '/car/encoder_counts', self.on_encoders, 10)
        self.create_subscription(Float32MultiArray, '/car/wheel_rpm', self.on_rpm, 10)
        self.create_timer(0.02, self.tick)

    def on_safety(self, msg):
        was_enabled = self.safety
        self.safety = bool(msg.data)
        if was_enabled and not self.safety and self.motion_started is not None:
            self.stop('ROS安全门关闭')
        if self.safety and self.prestart_abort:
            self.stop('预检故障后仍检测到运动门打开')
        elif self.safety and self.motion_started is None:
            self.motion_started = time.monotonic()
            self.start_encoders = list(self.encoders) if self.encoders is not None else None
            self.get_logger().warn(
                f'检测到运动门打开；独立{self.motion_duration_s:g}秒截止计时开始')

    def on_fault(self, msg):
        self.fault = int(msg.data)

    def on_encoders(self, msg):
        self.encoders = list(msg.data)
        self.last_encoders = list(msg.data)

    def on_rpm(self, msg):
        for index, value in enumerate(msg.data[:4]):
            self.max_abs_rpm[index] = max(self.max_abs_rpm[index], abs(float(value)))

    def stop(self, reason):
        if self.stop_started is not None:
            return
        self.reason = reason
        self.stop_started = time.monotonic()
        self.safety_pub.publish(Bool(data=False))
        self.trajectory_pub.publish(Bool(data=False))
        self.control_pub.publish(Bool(data=False))
        self.get_logger().warn(f'已下发双重失能: {reason}')

    def tick(self):
        now = time.monotonic()
        if self.stop_started is None:
            if self.fault not in (None, 0):
                if self.motion_started is not None:
                    self.stop(f'STM32故障位={self.fault:#x}')
                elif not self.prestart_abort:
                    self.prestart_abort = True
                    self.safety_pub.publish(Bool(data=False))
                    self.trajectory_pub.publish(Bool(data=False))
                    self.control_pub.publish(Bool(data=False))
                    self.get_logger().error(
                        f'动作前STM32故障位={self.fault:#x}；锁定本次悬空验收')
            elif (self.motion_started is not None
                  and now - self.motion_started >= self.motion_duration_s):
                self.stop(f'悬空动作达到{self.motion_duration_s:g}秒')
            # The absolute deadline protects only the pre-motion enable
            # sequence.  Once the safety gate has opened, the independent
            # motion-duration timer is the sole normal cutoff.  Keeping the
            # startup deadline active here can truncate a correctly started
            # run when DDS discovery or preflight used part of that budget.
            elif startup_deadline_expired(
                    self.motion_started, now, self.absolute_deadline):
                self.stop(f'{self.startup_timeout_s:g}秒内未开始动作')
        elif now - self.stop_started >= 1.0:
            deltas = None
            if self.start_encoders is not None and self.last_encoders is not None:
                deltas = [b - a for a, b in zip(self.start_encoders, self.last_encoders)]
            print(json.dumps({
                'reason': self.reason,
                'motion_started': self.motion_started is not None,
                'fault_bits': self.fault,
                'start_encoders': self.start_encoders,
                'last_encoders': self.last_encoders,
                'encoder_deltas': deltas,
                'max_abs_rpm': self.max_abs_rpm,
            }, ensure_ascii=False), flush=True)
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Cutoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop('操作者中断')
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
