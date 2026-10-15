#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class SafetyWatchdog(Node):
    """位姿或指令超时时锁止零速度；恢复数据后必须重新使能。"""

    def __init__(self):
        super().__init__('safety_watchdog')
        self.declare_parameter('pose_topic', '/car/pose')
        self.declare_parameter('input_cmd_topic', '/car/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/car/cmd_vel')
        self.declare_parameter('pose_timeout_s', 0.20)
        self.declare_parameter('command_timeout_s', 0.20)
        self.declare_parameter('check_rate_hz', 50.0)
        self.declare_parameter('startup_grace_s', 2.0)
        self.declare_parameter('start_enabled', False)

        pose_topic = str(self.get_parameter('pose_topic').value)
        input_cmd_topic = str(self.get_parameter('input_cmd_topic').value)
        output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        self.pose_timeout_s = float(self.get_parameter('pose_timeout_s').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)
        self.rate_hz = float(self.get_parameter('check_rate_hz').value)
        self.startup_grace_s = float(self.get_parameter('startup_grace_s').value)
        self.enabled = bool(self.get_parameter('start_enabled').value)

        self.started_ns = self.get_clock().now().nanoseconds
        self.last_pose_ns = None
        self.last_cmd_ns = None
        self.latest_cmd = Twist()
        self.last_fault = ''

        self.cmd_pub = self.create_publisher(Twist, output_cmd_topic, 10)
        self.enabled_pub = self.create_publisher(Bool, '/car/safety_enabled', 10)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        self.create_subscription(Twist, input_cmd_topic, self.on_cmd, 10)
        self.create_subscription(Bool, '/car/enable', self.on_enable, 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(
            f'安全看门狗: pose_timeout={self.pose_timeout_s:.3f}s, '
            f'cmd_timeout={self.command_timeout_s:.3f}s, enabled={self.enabled}')

    def on_pose(self, _msg: PoseStamped):
        self.last_pose_ns = self.get_clock().now().nanoseconds

    def on_cmd(self, msg: Twist):
        self.latest_cmd = msg
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def on_enable(self, msg: Bool):
        if not msg.data:
            self.enabled = False
            self.last_fault = 'operator_disabled'
            self.get_logger().warn('控制输出已由操作者禁用')
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.is_fresh(now_ns):
            self.enabled = True
            self.last_fault = ''
            self.get_logger().info('控制输出已重新使能')
        else:
            self.get_logger().error('拒绝使能：位姿或控制指令尚未稳定更新')

    def is_fresh(self, now_ns: int) -> bool:
        if self.last_pose_ns is None or self.last_cmd_ns is None:
            return False
        pose_age = (now_ns - self.last_pose_ns) * 1e-9
        cmd_age = (now_ns - self.last_cmd_ns) * 1e-9
        return pose_age <= self.pose_timeout_s and cmd_age <= self.command_timeout_s

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        fresh = self.is_fresh(now_ns)
        grace_elapsed = (now_ns - self.started_ns) * 1e-9 >= self.startup_grace_s

        if self.enabled and grace_elapsed and not fresh:
            self.enabled = False
            self.last_fault = 'pose_or_command_timeout'
            self.get_logger().error('位姿或控制命令超时：锁止零速度，必须重新使能')

        self.cmd_pub.publish(self.latest_cmd if self.enabled and fresh else Twist())
        self.enabled_pub.publish(Bool(data=self.enabled))


def main():
    rclpy.init()
    node = SafetyWatchdog()
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

