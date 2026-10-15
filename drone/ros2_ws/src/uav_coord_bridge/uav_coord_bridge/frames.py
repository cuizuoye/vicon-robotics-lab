"""ROS(ENU/FLU) <-> PX4(NED/FRD) 坐标与姿态转换。

背景
----
* ROS / RViz / Vicon 习惯: 世界系 ENU (x=East, y=North, z=Up)，
  机体系 FLU (x=Forward, y=Left, z=Up)。
* PX4 内部: 世界系 NED (x=North, y=East, z=Down)，
  机体系 FRD (x=Forward, y=Right, z=Down)。

本模块只做数学换算，不涉及任何话题/时间处理，方便单元测试。
所有向量均为长度 3 的序列，四元数为 (x, y, z, w)。
"""

from __future__ import annotations

import math

import numpy as np

# 向量基变换: v_ned = _M_ENU_TO_NED @ v_enu
#   N =  y_enu
#   E =  x_enu
#   D = -z_enu
_M_ENU_TO_NED = np.array([[0.0, 1.0, 0.0],
                          [1.0, 0.0, 0.0],
                          [0.0, 0.0, -1.0]])

# 机体系基变换: v_frd = _F_FLU_TO_FRD @ v_flu
#   f =  x_flu,  r = -y_flu,  d = -z_flu
_F_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def enu_to_ned_vector(vec):
    """ENU 坐标 -> NED 坐标 (位置/世界系速度)。"""
    return (_M_ENU_TO_NED @ np.asarray(vec, dtype=float)).tolist()


def ned_to_enu_vector(vec):
    """NED 坐标 -> ENU 坐标。"""
    return (_M_ENU_TO_NED.T @ np.asarray(vec, dtype=float)).tolist()


def flu_to_frd_vector(vec):
    """机体系 FLU -> FRD (角速度/机体轴速度)。"""
    return (_F_FLU_TO_FRD @ np.asarray(vec, dtype=float)).tolist()


def frd_to_flu_vector(vec):
    """机体系 FRD -> FLU。"""
    return (_F_FLU_TO_FRD.T @ np.asarray(vec, dtype=float)).tolist()


def quaternion_to_matrix(quat):
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵。"""
    x, y, z, w = (float(v) for v in quat)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('零四元数无法转换为旋转矩阵')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def matrix_to_quaternion(matrix):
    """3x3 旋转矩阵 -> 四元数 (x, y, z, w)。"""
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w])
    quat /= np.linalg.norm(quat)
    # 统一取 w >= 0，便于比较与调试
    if quat[3] < 0.0:
        quat = -quat
    return quat.tolist()


def quat_enu_flu_to_ned_frd(quat):
    """姿态四元数: (ENU 世界系中的 FLU 机体) -> (NED 世界系中的 FRD 机体)。

    R_ned = M * R_enu * F  (M: ENU->NED, F: FLU->FRD)
    """
    rot = _M_ENU_TO_NED @ quaternion_to_matrix(quat) @ _F_FLU_TO_FRD
    return matrix_to_quaternion(rot)


def quat_ned_frd_to_enu_flu(quat):
    """姿态四元数的逆变换: PX4(NED/FRD) -> ROS(ENU/FLU)。"""
    rot = _M_ENU_TO_NED.T @ quaternion_to_matrix(quat) @ _F_FLU_TO_FRD.T
    return matrix_to_quaternion(rot)


def quat_flu_to_frd_local(quat):
    """只把机体系 FLU 换成 FRD，世界系保持不变。

    用于 PX4 `POSE_FRAME_FRD`：直接把动捕世界系当作 PX4 的局部 FRD 系，
    此时允许动捕世界系与真北存在任意(固定)偏航偏置。
    R_frd = F * R_flu * F
    """
    rot = _F_FLU_TO_FRD @ quaternion_to_matrix(quat) @ _F_FLU_TO_FRD
    return matrix_to_quaternion(rot)


def yaw_from_quaternion(quat):
    """从四元数 (x, y, z, w) 提取偏航角 [rad]。"""
    x, y, z, w = (float(v) for v in quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_px4_to_ros(quat_wxyz):
    """px4_msgs 的四元数顺序 (w, x, y, z) -> 本项目内部/ROS 的 (x, y, z, w)。

    px4_msgs 的 VehicleOdometry.q、VehicleAttitude.q 都是 (w, x, y, z)，
    而 ROS（geometry_msgs、本模块其他函数）用 (x, y, z, w)。
    顺序弄错会得到"看起来像大幅度滚转"的错误姿态，务必走这个函数转换。
    """
    w, x, y, z = (float(v) for v in quat_wxyz)
    return [x, y, z, w]


def quat_ros_to_px4(quat_xyzw):
    """ROS (x, y, z, w) -> px4_msgs (w, x, y, z)。"""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return [w, x, y, z]
