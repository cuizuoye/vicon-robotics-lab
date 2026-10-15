"""Vicon刚体位姿到MAVLink预览；纯数学和新鲜度检查，不接触飞控。"""

from dataclasses import dataclass
import math

import numpy as np
from pymavlink.dialects.v20 import common as m

from uav_coord_bridge.frames import matrix_to_quaternion, quaternion_to_matrix

F = np.diag([1.0, -1.0, -1.0])


def finite_vector(values, length):
    array = np.asarray(values, dtype=float)
    if array.shape != (length,) or not np.isfinite(array).all():
        raise ValueError('non_finite_or_wrong_shape')
    return array


def checked_quaternion(values):
    q = finite_vector(values, 4)
    norm = float(np.linalg.norm(q))
    if not 0.95 <= norm <= 1.05:
        raise ValueError('invalid_quaternion_norm')
    return q / norm


def transform_rigid_pose(position, quaternion, body_origin_in_rigid=(0, 0, 0),
                         body_orientation_in_rigid=(0, 0, 0, 1)):
    """输入z向上的世界系/刚体系，输出任意航向LOCAL_FRD/机体FRD。

    外参T_rigid_body只补偿一次：位移是刚体原点→机体/IMU原点、在刚体系中表示；
    四元数是机体FLU→刚体系。不能把未知的刚体朝向默认当成真实机头朝向。
    """
    p = finite_vector(position, 3)
    rigid_rotation = quaternion_to_matrix(checked_quaternion(quaternion))
    offset = finite_vector(body_origin_in_rigid, 3)
    mounting = quaternion_to_matrix(checked_quaternion(body_orientation_in_rigid))
    body_position = F @ (p + rigid_rotation @ offset)
    body_rotation = F @ rigid_rotation @ mounting @ F
    xyzw = matrix_to_quaternion(body_rotation)
    wxyz = [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]
    return body_position.tolist(), wxyz


def pose_covariance(position_sigma=0.02, attitude_sigma=0.1):
    if not all(math.isfinite(s) and s > 0 for s in (position_sigma, attitude_sigma)):
        raise ValueError('invalid_covariance')
    covariance = [0.0] * 21
    for index in (0, 6, 11):
        covariance[index] = position_sigma**2
    for index in (15, 18, 20):
        covariance[index] = attitude_sigma**2
    return covariance


def odometry_message(encoder, stamp_us, position, quaternion_wxyz, reset_counter=0):
    if stamp_us <= 0 or not 0 <= reset_counter <= 255:
        raise ValueError('invalid_timestamp_or_reset_counter')
    p = finite_vector(position, 3)
    q = checked_quaternion(quaternion_wxyz)
    # 无速度测量，不伪造零速度或有限差分速度。quality=0表示质量未知，不冒充满分。
    unknown_cov = [math.nan] + [0.0] * 20
    return encoder.odometry_encode(
        stamp_us, m.MAV_FRAME_LOCAL_FRD, m.MAV_FRAME_BODY_FRD,
        *p.tolist(), q.tolist(), *([math.nan] * 6),
        pose_covariance(), unknown_cov, reset_counter, m.MAV_ESTIMATOR_TYPE_VISION, 0)


@dataclass
class PoseSample:
    stamp_ns: int
    received_monotonic_ns: int
    position: list
    quaternion: list


class PoseGate:
    """重复/倒序/过期/非法位姿不变成新测量，定时器不重复发送缓存值。"""

    def __init__(self, expected_frame='map', max_age_ms=150, max_speed_mps=5):
        if not math.isfinite(max_age_ms) or not 1 <= max_age_ms <= 1000:
            raise ValueError('invalid_max_age')
        if not math.isfinite(max_speed_mps) or not 0 < max_speed_mps <= 20:
            raise ValueError('invalid_max_speed')
        self.frame = expected_frame
        self.max_age_ns = int(max_age_ms * 1e6)
        self.max_speed_mps = max_speed_mps
        self.last = self.pending = None
        self.accepted = self.rejected = self.expired = 0
        self.reason = 'waiting_for_pose'

    def ingest(self, stamp_ns, now_ros_ns, received_monotonic_ns, frame, position, quaternion):
        try:
            if frame != self.frame:
                raise ValueError('frame_mismatch')
            if stamp_ns <= 0 or now_ros_ns < stamp_ns or now_ros_ns - stamp_ns > self.max_age_ns:
                raise ValueError('stale_or_future_stamp')
            p, q = finite_vector(position, 3), checked_quaternion(quaternion)
            if self.last is not None:
                dt = (stamp_ns - self.last.stamp_ns) / 1e9
                if dt <= 0:
                    raise ValueError('duplicate_or_out_of_order')
                # 遮挡后的重获不靠无限外推；这里仍要求位移在物理速度边界内。
                if np.linalg.norm(p - np.asarray(self.last.position)) / dt > self.max_speed_mps:
                    raise ValueError('implausible_position_jump')
            sample = PoseSample(int(stamp_ns), int(received_monotonic_ns), p.tolist(), q.tolist())
        except ValueError as exc:
            self.rejected += 1
            self.reason = str(exc)
            return False
        self.last = self.pending = sample
        self.accepted += 1
        self.reason = 'fresh'
        return True

    def take_new(self, now_ros_ns, now_monotonic_ns):
        sample, self.pending = self.pending, None
        if sample is None:
            return None
        ages = (now_ros_ns - sample.stamp_ns, now_monotonic_ns - sample.received_monotonic_ns)
        if any(age < 0 or age > self.max_age_ns for age in ages):
            self.expired += 1
            self.reason = 'expired_before_encoding'
            return None
        return sample

    def fresh(self, now_ros_ns, now_monotonic_ns):
        if self.last is None:
            return False
        return all(0 <= age <= self.max_age_ns for age in (
            now_ros_ns - self.last.stamp_ns,
            now_monotonic_ns - self.last.received_monotonic_ns))
