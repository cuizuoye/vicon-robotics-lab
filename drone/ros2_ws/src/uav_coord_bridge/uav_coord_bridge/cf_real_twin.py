#!/usr/bin/env python3
"""把【真机】Crazyflie 的姿态/电池状态发布到 ROS 2，用于在 RViz2 里实时观察真机。

用途：没有 Vicon/LPS/Flow deck 时，飞机还不能自主飞行，但我们可以实时看到它的
      真实姿态（你用手转动飞机，RViz 里的模型会跟着转）与电池电压 ——
      这也是"真机 → ROS 2 → RViz"整条链路的验证。

数据来源：cflib 的日志变量（不是参数！）
    stabilizer.roll / pitch / yaw   [deg]
    pm.vbat                         [V]
    stateEstimate.x/y/z             [m]（无位置传感器时是积分出来的，会漂，仅供参考）

注意：
    1. USB 口同一时刻只能被一个程序占用 —— 跑本节点时不要再跑别的 cflib 程序
       （包括 Crazyswarm2 的 server）。
    2. 没有外部位姿时，"位置"没有物理意义，RViz 里请主要看姿态。

用法：
    ros2 run uav_coord_bridge cf_real_twin
    ros2 run uav_coord_bridge cf_real_twin --ros-args -p uri:=radio://0/80/2M/E7E7E7E7E7
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster


class CfRealTwin(Node):

    def __init__(self):
        super().__init__('cf_real_twin')

        self.declare_parameter('uri', 'usb://0')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'cf_real')
        self.declare_parameter('publish_tf', True)

        self.uri = self.get_parameter('uri').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.pose_pub = self.create_publisher(PoseStamped, '/cf_real/pose', 10)
        self.batt_pub = self.create_publisher(BatteryState, '/cf_real/battery', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.cf = None
        self.scf = None
        self.connected = False
        self.last = {}

        threading.Thread(target=self._connect_and_log, daemon=True).start()
        self.get_logger().info(f'正在连接真机 {self.uri} …')

    # ---------- cflib 侧：连接 + 订阅日志 ----------
    def _connect_and_log(self):
        try:
            import cflib.crtp
            from cflib.crazyflie import Crazyflie
            from cflib.crazyflie.log import LogConfig
            from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        except ImportError as exc:
            self.get_logger().error(f'缺少 cflib: {exc}')
            return

        cflib.crtp.init_drivers()
        self.cf = Crazyflie(rw_cache='/tmp/cf_cache')
        try:
            with SyncCrazyflie(self.uri, cf=self.cf) as scf:
                self.scf = scf
                self.connected = True
                self.get_logger().info('✅ 已连接真机，开始读取姿态/电池')

                # CRTP 单个日志块上限 26 字节，所以拆成两块订阅
                self._start_log('att', {
                    'stabilizer.roll': 'float',
                    'stabilizer.pitch': 'float',
                    'stabilizer.yaw': 'float',
                    'pm.vbat': 'float',
                }, period_ms=50)
                self._start_log('pos', {
                    'stateEstimate.x': 'float',
                    'stateEstimate.y': 'float',
                    'stateEstimate.z': 'float',
                }, period_ms=100)

                # 保持连接（在 with 块里循环，否则会立刻断开）
                while rclpy.ok():
                    time.sleep(0.2)
        except Exception as exc:                               # noqa: BLE001
            self.get_logger().error(f'连接失败: {type(exc).__name__}: {exc}')
            self.connected = False

    def _start_log(self, name, variables, period_ms):
        from cflib.crazyflie.log import LogConfig
        conf = LogConfig(name=name, period_in_ms=period_ms)
        for var, vtype in variables.items():
            try:
                conf.add_variable(var, vtype)
            except Exception:
                pass

        def callback(_ts, data, _cfg):
            self.last.update(data)

        conf.data_received_cb.add_callback(callback)
        self.cf.log.add_config(conf)
        conf.start()

    # ---------- ROS 侧 ----------
    @staticmethod
    def _quat_from_rpy_deg(roll, pitch, yaw):
        r, p, y = (math.radians(v) for v in (roll, pitch, yaw))
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        return (sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy)

    def publish(self):
        if not self.connected or not self.last:
            return
        d = self.last
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self._quat_from_rpy_deg(d.get('stabilizer.roll', 0.0),
                                                 d.get('stabilizer.pitch', 0.0),
                                                 d.get('stabilizer.yaw', 0.0))
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(d.get('stateEstimate.x', 0.0))
        pose.pose.position.y = float(d.get('stateEstimate.y', 0.0))
        pose.pose.position.z = float(d.get('stateEstimate.z', 0.0))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = pose.header
            tf.child_frame_id = self.child_frame_id
            tf.transform.translation.x = pose.pose.position.x
            tf.transform.translation.y = pose.pose.position.y
            tf.transform.translation.z = pose.pose.position.z
            tf.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

        if 'pm.vbat' in d:
            batt = BatteryState()
            batt.header.stamp = now
            batt.voltage = float(d['pm.vbat'])
            batt.percentage = float('nan')
            batt.present = True
            self.batt_pub.publish(batt)


def main():
    rclpy.init()
    node = CfRealTwin()
    node.create_timer(1.0 / 20.0, node.publish)
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
