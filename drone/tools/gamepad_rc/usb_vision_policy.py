"""未解锁且视觉融合关闭时的USB接收台架白名单；无启用融合/飞行入口。"""

import math
import re

import numpy as np
from pymavlink.dialects.v20 import common as m

from mavlink_readonly_probe import readonly_message

PARAMS = ('EKF2_EV_CTRL', 'EKF2_AID_MASK', 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z')
COMMANDS = {
    'actuator_armed': b'listener actuator_armed -n 1\n',
    'vehicle_visual_odometry': b'listener vehicle_visual_odometry -n 1\n',
    'timesync_status': b'listener timesync_status -n 1\n',
}


def shell_message(encoder, topic=None, close=False):
    payload = COMMANDS[topic] if topic else b''
    return encoder.serial_control_encode(10, 0 if close else 2, 0, 0,
                                         len(payload), payload.ljust(70, b'\0'))


def shell_allowed(message):
    if message.get_type() != 'SERIAL_CONTROL' or (message.device, message.timeout, message.baudrate) != (10, 0, 0):
        return False
    data = bytes(message.data)
    # 已保留mavlink_if3崩溃证据：所有派生策略均禁止销毁旧版Shell。
    if message.flags == 0:
        return False
    return message.flags == 2 and any(message.count == len(p) and data == p.ljust(70, b'\0')
                                    for p in (b'', *COMMANDS.values()))


def fusion_disabled(parameters):
    return (parameters.get('EKF2_EV_CTRL') == 0 and parameters.get('EKF2_AID_MASK') == 0
            and all(parameters.get(name) == 0 for name in PARAMS[2:]))


def send_allowed(message, *, bench_authorized=False, safe=False):
    if readonly_message(message):
        return True
    kind = message.get_type()
    if kind == 'PARAM_REQUEST_READ':
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\0')
        return ((message.target_system, message.target_component) == (1, 1)
                and message.param_index == -1 and name in PARAMS)
    if shell_allowed(message):
        return safe
    if kind == 'TIMESYNC':
        return safe and message.tc1 > 0 and message.ts1 > 0
    if kind != 'ODOMETRY' or not bench_authorized or not safe:
        return False
    fields = [message.x, message.y, message.z, *message.q]
    return (message.frame_id == m.MAV_FRAME_LOCAL_FRD and message.child_frame_id == m.MAV_FRAME_BODY_FRD
            and message.estimator_type == m.MAV_ESTIMATOR_TYPE_VISION and message.quality == 0
            and message.time_usec > 0 and message.reset_counter == 0
            and all(math.isfinite(v) for v in fields)
            and .95 <= np.linalg.norm(message.q) <= 1.05
            and all(math.isnan(v) for v in (message.vx, message.vy, message.vz,
                                            message.rollspeed, message.pitchspeed, message.yawspeed))
            and math.isnan(message.velocity_covariance[0])
            and all(math.isfinite(v) for v in message.pose_covariance)
            and all(message.pose_covariance[i] > 0 for i in (0, 6, 11, 15, 18, 20)))


def parse_visual_snapshot(text):
    blocks = re.split(r'TOPIC:\s*vehicle_visual_odometry[^\n]*\n', text)
    for block in reversed(blocks[1:]):
        result = {}
        for name in ('timestamp', 'timestamp_sample', 'pose_frame', 'velocity_frame', 'reset_counter', 'quality'):
            found = re.search(rf'^\s*{name}:\s*(-?\d+)\b', block, re.M)
            if found:
                result[name] = int(found.group(1))
        for name, size in (('position', 3), ('q', 4)):
            found = re.search(rf'^\s*{name}:\s*\[([^\]]+)\]', block, re.M)
            if found:
                try:
                    values = [float(v.strip()) for v in found.group(1).split(',')]
                except ValueError:
                    continue
                if len(values) == size and all(math.isfinite(v) for v in values):
                    result[name] = values
        if len(result) == 8:
            return result
    return None


def match_snapshot(snapshot, sent):
    """飞控内部uORB数据须匹配实际发送样本；非ACK/本机写成功即通过。"""
    if snapshot.get('pose_frame') != 2 or snapshot.get('quality') != 0 or snapshot.get('reset_counter') != 0:
        return None
    q = np.asarray(snapshot['q'])
    norm = np.linalg.norm(q)
    if not .95 <= norm <= 1.05:
        return None
    q = q / norm
    for row in reversed(sent):
        angle = math.degrees(2 * math.acos(float(np.clip(abs(q @ np.asarray(row['q'])), 0, 1))))
        distance = float(np.linalg.norm(np.asarray(snapshot['position']) - row['position']))
        if distance <= .002 and angle <= .5:
            return {'source_stamp_ns': row['source_stamp_ns'], 'position_error_m': distance,
                    'quaternion_difference_deg': angle}
    return None


def parse_timesync_snapshot(text):
    fields = ('timestamp', 'source_protocol', 'remote_timestamp', 'observed_offset',
              'estimated_offset', 'round_trip_time')
    for block in reversed(re.split(r'TOPIC:\s*timesync_status[^\n]*\n', text)[1:]):
        result = {}
        for name in fields:
            found = re.search(rf'^\s*{name}:\s*(-?\d+)\b', block, re.M)
            if found:
                result[name] = int(found.group(1))
        # v1.14.3 MavlinkTimesync使用Timesync{}默认构造，_source可保持UNKNOWN(0)。
        # UNKNOWN不等于同步无效；必须再用本机时域和内部采样时间映射验收。
        if len(result) == len(fields) and result['source_protocol'] in (0, 1):
            return result
    return None


def check_clock_probe(status, snapshot, row, pc_now_us):
    """真实样本暂存50ms：区分采样钟转换与未收敛的接收时刻回退。"""
    if not status or not snapshot or not match_snapshot(snapshot, [row]):
        return {'verified': False, 'reason': 'missing_or_unmatched_internal_evidence'}
    error_us = snapshot['timestamp_sample'] - (row['wire_time_usec'] + status['estimated_offset'])
    age_us = snapshot['timestamp'] - snapshot['timestamp_sample']
    own_clock = 0 <= pc_now_us - status['remote_timestamp'] < 2000000
    good = (own_clock and 0 <= status['round_trip_time'] < 10000
            and abs(error_us) <= 2000 and 30000 <= age_us <= 150000)
    residual_us = status['observed_offset'] - status['estimated_offset']
    return {'verified': good, 'own_pc_clock': own_clock, 'mapping_error_us': error_us,
            'fc_sample_age_us': age_us, 'round_trip_time_us': status['round_trip_time'],
            'observed_minus_estimated_offset_us': residual_us,
            'clock_accuracy_candidate_pass': good and abs(residual_us) <= 2000,
            'mapping_error_is_not_independent_clock_accuracy': True}


class SourceLossLatch:
    """失去新鲜输入后锁存停止；恢复输入也不自动重发。"""
    def __init__(self):
        self.blocked = False

    def permits(self, fresh):
        self.blocked = self.blocked or not fresh
        return not self.blocked


class ClockReadiness:
    """三次新鲜内部状态连续满足门限才就绪；低RTT/回复总数都不能单独放行。"""
    def __init__(self, required=3, tolerance_us=2000):
        self.required, self.tolerance_us = required, tolerance_us
        self.count = 0
        self.last_timestamp = None

    def update(self, status, pc_now_us):
        fresh = bool(status and 0 <= pc_now_us - status['remote_timestamp'] < 1000000)
        increasing = bool(status and (self.last_timestamp is None or status['timestamp'] > self.last_timestamp))
        good = bool(fresh and increasing and 0 <= status['round_trip_time'] < 10000
                    and abs(status['observed_offset'] - status['estimated_offset']) <= self.tolerance_us)
        self.count = self.count + 1 if good else 0
        self.last_timestamp = status['timestamp'] if status else None
        return self.count >= self.required


def assess_clock_series(status_rows, request_rows):
    """到达时差只用于诊断钟速变化；不是独立绝对同步/相机曝光精度。"""
    residuals = [r['status']['observed_offset'] - r['status']['estimated_offset'] for r in status_rows]
    valid = [r for r in request_rows if r['pc_receive_ns'] > 0 and r['fc_originate_ns'] > 0]
    # 长间隔差分减少单次USB到达抖动影响；保留原始记录，不用结果回写飞控。
    slopes, raw_slopes, pc_raw_slopes = [], [], []
    if len(valid) > 10:
        for first, last in zip(valid[:len(valid)//3], valid[-len(valid)//3:]):
            dt = last['pc_receive_ns'] - first['pc_receive_ns']
            if dt > 5_000_000_000:
                slopes.append(((last['fc_originate_ns'] - first['fc_originate_ns']) / dt - 1) * 1e6)
                raw_dt = last.get('pc_raw_ns', 0) - first.get('pc_raw_ns', 0)
                if first.get('pc_raw_ns', 0) > 0 and raw_dt > 5_000_000_000:
                    raw_slopes.append(((last['fc_originate_ns'] - first['fc_originate_ns']) / raw_dt - 1) * 1e6)
                    pc_raw_slopes.append((dt / raw_dt - 1) * 1e6)
    return {'status_count': len(status_rows), 'request_count': len(valid),
            'first_residual_us': residuals[0] if residuals else None,
            'last_residual_us': residuals[-1] if residuals else None,
            'max_absolute_residual_us': max(map(abs, residuals)) if residuals else None,
            'last_three_within_2ms': len(residuals) >= 3 and all(abs(v) <= 2000 for v in residuals[-3:]),
            'fc_vs_pc_arrival_clock_rate_ppm': float(np.median(slopes)) if slopes else None,
            'fc_vs_pc_raw_arrival_clock_rate_ppm': float(np.median(raw_slopes)) if raw_slopes else None,
            'pc_monotonic_vs_raw_clock_rate_ppm': float(np.median(pc_raw_slopes)) if pc_raw_slopes else None,
            'absolute_clock_accuracy_independently_verified': False}
