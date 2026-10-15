import math

import numpy as np
import pytest

from uav_coord_bridge import frames


def _rotz(angle):
    """ENU/FLU 约定下绕 z 轴的旋转 -> 四元数 (x, y, z, w)。"""
    quat = [0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0)]
    return quat


def _quat_mul(q1, q2):
    """四元数乘法 (x, y, z, w)，结果为先 q2 后 q1 的旋转复合。"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def test_north_facing_is_zero_yaw_in_ned():
    """ENU 下机头由东转向北(yaw=+90°) 时，NED 下航向应为 0。"""
    quat_ned = frames.quat_enu_flu_to_ned_frd(_rotz(math.pi / 2.0))
    assert frames.yaw_from_quaternion(quat_ned) == pytest.approx(0.0, abs=1e-9)


def test_enu_identity_means_nose_east():
    """ENU 与机体系重合(机头朝东) 时，NED 下航向为 +90°(北向东为正)。"""
    quat_ned = frames.quat_enu_flu_to_ned_frd([0.0, 0.0, 0.0, 1.0])
    assert frames.yaw_from_quaternion(quat_ned) == pytest.approx(math.pi / 2.0, abs=1e-9)


def test_full_circle_roundtrip():
    for deg in range(0, 360, 15):
        quat_enu = _rotz(math.radians(deg))
        quat_ned = frames.quat_enu_flu_to_ned_frd(quat_enu)
        back = frames.quat_ned_frd_to_enu_flu(quat_ned)
        # 四元数 q 与 -q 等价
        direct = min(
            max(abs(a - b) for a, b in zip(quat_enu, back)),
            max(abs(a + b) for a, b in zip(quat_enu, back)),
        )
        assert direct < 1e-9


def test_position_conversion():
    # ENU: 东 1m, 北 2m, 上 3m  ->  NED: 北 2m, 东 1m, 下 -3m
    assert frames.enu_to_ned_vector([1.0, 2.0, 3.0]) == pytest.approx([2.0, 1.0, -3.0])
    assert frames.ned_to_enu_vector([2.0, 1.0, -3.0]) == pytest.approx([1.0, 2.0, 3.0])


def test_body_vector_conversion():
    assert frames.flu_to_frd_vector([1.0, 2.0, 3.0]) == pytest.approx([1.0, -2.0, -3.0])


def test_px4_quaternion_order():
    """px4_msgs 用 (w,x,y,z)，ROS 用 (x,y,z,w)，顺序必须显式转换。"""
    assert frames.quat_px4_to_ros([0.5, 0.1, 0.2, 0.3]) == pytest.approx([0.1, 0.2, 0.3, 0.5])
    assert frames.quat_ros_to_px4([0.1, 0.2, 0.3, 0.5]) == pytest.approx([0.5, 0.1, 0.2, 0.3])


def test_px4_yaw_to_enu_yaw():
    """PX4 NED 下偏航 96°(机头基本朝东) -> ROS ENU 下偏航约为 -6°。"""
    yaw_ned = math.radians(96.0)
    quat_ned_xyzw = [0.0, 0.0, math.sin(yaw_ned / 2.0), math.cos(yaw_ned / 2.0)]
    quat_enu = frames.quat_ned_frd_to_enu_flu(quat_ned_xyzw)
    assert frames.yaw_from_quaternion(quat_enu) == pytest.approx(
        math.radians(-6.0), abs=1e-9)


def test_frd_local_yaw_flips_sign():
    """FRD 局部系模式下，偏航角符号翻转(右手系改到 y 向右、z 向下)。"""
    quat_frd = frames.quat_flu_to_frd_local(_rotz(math.radians(30.0)))
    assert frames.yaw_from_quaternion(quat_frd) == pytest.approx(math.radians(-30.0), abs=1e-9)


def test_frd_local_roll_keeps_sign():
    """滚转轴就是机体 x 轴，在 FLU/FRD 之间保持不变。"""
    roll = [math.sin(math.radians(15.0)), 0.0, 0.0, math.cos(math.radians(15.0))]
    assert frames.quat_flu_to_frd_local(roll) == pytest.approx(roll, abs=1e-9)


def test_roll_about_north_axis_is_preserved():
    """滚转轴正好是北向(x_frd 与 NED 的 x 重合) 时，滚转角保持不变。"""
    # 先绕 ENU 竖直轴转 +90°(机头由东转向北)，再绕机体 x 轴滚 90°
    yaw = _rotz(math.pi / 2.0)
    roll = [math.sin(math.pi / 4.0), 0.0, 0.0, math.cos(math.pi / 4.0)]
    quat_enu = _quat_mul(yaw, roll)
    quat_ned = frames.quat_enu_flu_to_ned_frd(quat_enu)
    assert quat_ned == pytest.approx(roll, abs=1e-9)


def test_body_axes_consistency():
    """核心不变式: R_ned = M * R_enu * F，
    即同一机体向量在两种约定下指向同一物理方向。
    """
    m = frames._M_ENU_TO_NED
    f = frames._F_FLU_TO_FRD
    quat_enu = frames.matrix_to_quaternion(
        np.array([[0.36, -0.48, 0.8],
                  [0.8, 0.60, 0.0],
                  [-0.48, 0.64, 0.60]]))
    quat_ned = frames.quat_enu_flu_to_ned_frd(quat_enu)
    r_enu = frames.quaternion_to_matrix(quat_enu)
    r_ned = frames.quaternion_to_matrix(quat_ned)

    assert np.allclose(r_ned, m @ r_enu @ f, atol=1e-9)

    for vec_flu in ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.3, -0.5, 0.8]):
        in_enu = r_enu @ np.array(vec_flu)
        in_ned = r_ned @ (f @ np.array(vec_flu))
        assert np.allclose(in_ned, m @ in_enu, atol=1e-9)
