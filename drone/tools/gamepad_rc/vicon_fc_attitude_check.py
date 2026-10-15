#!/usr/bin/env python3
"""USB/Vicon姿态只读核对：仅发GCS心跳和版本请求，不写参/发送定位/控制。"""

import argparse
from collections import Counter
import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import serial
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import quaternion_to_matrix  # noqa: E402
from uav_coord_bridge.mavlink_vision import checked_quaternion, transform_rigid_pose  # noqa: E402
from mavlink_readonly_probe import Evidence, send_readonly  # noqa: E402


def attitude_comparison(vicon_wxyz, fc_wxyz):
    """都为FRD机体/z向下世界；航向偏置仅观察，不冒充真北标定。"""
    def rotation(wxyz):
        w, x, y, z = wxyz
        return quaternion_to_matrix(checked_quaternion([x, y, z, w]))

    def euler(matrix):
        return np.array([math.atan2(matrix[2, 1], matrix[2, 2]),
                         math.asin(float(np.clip(-matrix[2, 0], -1, 1))),
                         math.atan2(matrix[1, 0], matrix[0, 0])])

    rv, rf = rotation(vicon_wxyz), rotation(fc_wxyz)
    ev, ef = euler(rv), euler(rf)
    delta = np.arctan2(np.sin(ef - ev), np.cos(ef - ev))
    # 在机体系比较重力向下轴，绕世界z的任意航向偏置不影响这个量。
    tilt = math.degrees(math.acos(float(np.clip((rv.T @ [0, 0, 1]) @
                                                    (rf.T @ [0, 0, 1]), -1, 1))))
    return {'vicon_roll_pitch_yaw_deg': np.degrees(ev).tolist(),
            'fc_roll_pitch_yaw_deg': np.degrees(ef).tolist(),
            'fc_minus_vicon_wrapped_deg': np.degrees(delta).tolist(),
            'gravity_axis_difference_deg': tilt}


def nose_up_observation(baseline, raised):
    """保存端点俯仰符号观测；小动作不自动升级为完整安装标定。"""
    for record in (baseline, raised):
        if record.get('static_observation_passed') is not True or not record.get('pairs'):
            raise ValueError('valid_stationary_capture_required')
    if baseline['identity'] != raised['identity'] or baseline['heading_candidate'] != raised['heading_candidate']:
        raise ValueError('identity_or_mounting_candidate_changed')
    t0 = datetime.datetime.fromisoformat(baseline['captured_at_utc'])
    t1 = datetime.datetime.fromisoformat(raised['captured_at_utc'])
    if t0.utcoffset() is None or t1.utcoffset() is None or not 0 < (t1 - t0).total_seconds() <= 1800:
        raise ValueError('invalid_capture_order_or_age')
    if raised['last_pair']['fc_boot_ms'] <= baseline['last_pair']['fc_boot_ms']:
        raise ValueError('fc_reboot_or_capture_order')
    changes = {}
    for source in ('vicon', 'fc'):
        medians = []
        for record in (baseline, raised):
            rp = np.asarray([p[source + '_roll_pitch_yaw_deg'][:2] for p in record['pairs']], dtype=float)
            if rp.ndim != 2 or rp.shape[1] != 2 or not np.isfinite(rp).all():
                raise ValueError('invalid_capture_angles')
            medians.append(np.degrees(np.median(np.unwrap(np.radians(rp), axis=0), axis=0)))
        changes[source] = {'baseline_roll_pitch_deg': medians[0].tolist(),
                           'raised_roll_pitch_deg': medians[1].tolist(),
                           'delta_roll_pitch_deg': (medians[1] - medians[0]).tolist()}
    v, f = changes['vicon']['delta_roll_pitch_deg'], changes['fc']['delta_roll_pitch_deg']
    checks = {'both_pitch_increase': 1.5 <= v[1] <= 30 and 1.5 <= f[1] <= 30,
              'pitch_change_difference_within_2_deg': abs(v[1] - f[1]) <= 2,
              'roll_change_smaller_than_pitch': abs(v[0]) < v[1] and abs(f[0]) < f[1]}
    stronger = min(v[1], f[1]) >= 5
    return {'scope': 'Held nose-up endpoints; qualitative pitch-sign check, not time-synchronized dynamics',
            'changes': changes, 'checks': checks, 'pitch_sign_observation_passed': all(checks.values()),
            'both_pitch_changes_at_least_5_deg': stronger,
            'quality': 'direction_observation' if stronger else 'limited_small_angle_observation',
            'dynamic_axes_verified': False, 'extrinsics_verified': False,
            'north_alignment_verified': False, 'imu_origin_verified': False,
            'ready_to_send_to_fc': False, 'actual_sent_to_fc': 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--uid2', required=True)
    parser.add_argument('--heading-candidate', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or not 7 <= args.seconds <= 30:
        parser.error('Use new output and 7..30 seconds')
    if subprocess.run(['fuser', str(Path(args.port).resolve())], capture_output=True).returncode == 0:
        parser.error('Serial port occupied; no process will be killed')
    heading = json.loads(args.heading_candidate.read_text())
    if heading.get('manual_forward_horizontal_confirmed') is not True:
        parser.error('Confirmed heading candidate required')
    mount = checked_quaternion(heading['body_orientation_in_rigid_xyzw_candidate'])

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    class PoseCapture(Node):
        def __init__(self):
            super().__init__('vicon_fc_attitude_readonly')
            self.last = None
            self.last_stamp = 0
            self.rejected = 0
            self.samples = 0
            self.kept = self.create_subscription(
                PoseStamped, '/vicon/MircoAir/MircoAir', self.receive,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE))

        def receive(self, msg):
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e6
            if msg.header.frame_id != 'map' or stamp <= self.last_stamp or not 0 <= age <= 150:
                self.rejected += 1
                return
            p, q = msg.pose.position, msg.pose.orientation
            try:
                _, wxyz = transform_rigid_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
                                               body_orientation_in_rigid=mount)
            except ValueError:
                self.rejected += 1
                return
            self.last_stamp = stamp
            self.last = {'wxyz': wxyz, 'stamp_ns': stamp, 'arrival': time.monotonic(),
                         'position_m': [p.x, p.y, p.z]}
            self.samples += 1

    rclpy.init()
    node = PoseCapture()
    evidence = Evidence(expected_parameters=())
    encoder = m.MAVLink(None, srcSystem=250, srcComponent=190)
    decoder = m.MAVLink(None)
    decoder.robust_parsing = True
    rows, positions, times, boot_times = [], [], [], []
    counts = Counter()
    last_boot = None
    rejected_fc = stale_pair = 0
    start = time.monotonic()
    next_heartbeat = 0
    error = None
    try:
        with serial.Serial(args.port, 115200, timeout=0, write_timeout=1, exclusive=True) as port:
            while time.monotonic() - start < args.seconds:
                rclpy.spin_once(node, timeout_sec=.001)
                now = time.monotonic() - start
                for msg in decoder.parse_buffer(port.read(min(port.in_waiting, 65536))) or []:
                    evidence.ingest(msg, now)
                    if evidence.fatal:
                        raise RuntimeError(evidence.fatal)
                    identity = evidence.identity
                    if identity and (identity['uid2'] != args.uid2 or identity['firmware'] != '1.14.3'):
                        raise RuntimeError('Flight-controller identity/version mismatch')
                    if msg.get_type() != 'ATTITUDE_QUATERNION' or (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                        continue
                    # USB启动缓存不参与核对；未确认身份/新鲜未解锁心跳也不配对。
                    fresh_hb = bool(evidence.heartbeats) and now - evidence.heartbeats[-1]['elapsed_s'] < 2
                    if now < 2 or identity is None or not fresh_hb:
                        continue
                    if last_boot is not None and msg.time_boot_ms <= last_boot:
                        rejected_fc += 1
                        continue
                    last_boot = msg.time_boot_ms
                    pose = node.last
                    age_ms = (node.get_clock().now().nanoseconds - pose['stamp_ns']) / 1e6 if pose else None
                    if pose is None or not 0 <= age_ms <= 150 or time.monotonic() - pose['arrival'] > .15:
                        stale_pair += 1
                        continue
                    try:
                        row = attitude_comparison(pose['wxyz'], [msg.q1, msg.q2, msg.q3, msg.q4])
                    except ValueError:
                        rejected_fc += 1
                        continue
                    row.update({'fc_boot_ms': msg.time_boot_ms, 'vicon_stamp_ns': pose['stamp_ns'],
                                'vicon_ros_age_ms': age_ms, 'elapsed_s': now})
                    rows.append(row)
                    positions.append(pose['position_m'])
                    times.append(time.monotonic())
                    boot_times.append(msg.time_boot_ms)
                if now >= next_heartbeat:
                    send_readonly(encoder, port, encoder.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
                    counts['HEARTBEAT'] += 1
                    next_heartbeat = now + 1
                if evidence.heartbeats and evidence.version_requested_at is None and now - evidence.heartbeats[-1]['elapsed_s'] < 2:
                    evidence.version_requested_at = now
                    send_readonly(encoder, port, encoder.command_long_encode(
                        1, 1, m.MAV_CMD_REQUEST_MESSAGE, 0, m.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0))
                    counts['AUTOPILOT_VERSION_REQUEST'] += 1
    except (OSError, ValueError, RuntimeError, serial.SerialException) as exc:
        error = str(exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    gap = float(max(np.diff(times)) * 1000) if len(times) > 1 else None
    rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else None
    tilts = [row['gravity_axis_difference_deg'] for row in rows]
    std = (np.std(positions, axis=0) * 1000).tolist() if rows else None
    checks = {'same_uid_and_px4_1143': bool(evidence.identity) and evidence.identity['uid2'] == args.uid2 and evidence.identity['firmware'] == '1.14.3',
              'all_heartbeats_disarmed': bool(evidence.heartbeats) and all(not h['armed'] for h in evidence.heartbeats),
              'at_least_20_pairs': len(rows) >= 20,
              'fc_rate_at_least_3_hz': rate is not None and rate >= 3,
              'pair_gap_at_most_500_ms': gap is not None and gap <= 500,
              'latest_pair_fresh': bool(times) and time.monotonic() - times[-1] <= .5,
              'no_invalid_or_stale_data': node.rejected == rejected_fc == stale_pair == 0,
              'stationary_position_within_3_mm': std is not None and max(std) <= 3,
              'tilt_difference_within_5_deg': bool(tilts) and max(tilts) <= 5,
              'no_error': error is None}
    result = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope': 'USB/ROS read-only static attitude observation; no position/control/parameter writes',
              'pairing': 'Latest fresh ROS sample at FC message PC arrival; NOT synchronized exposure/boot clocks',
              'port': args.port, 'identity': evidence.identity,
              'heading_candidate': str(args.heading_candidate.resolve()),
              'checks': checks, 'static_observation_passed': all(checks.values()),
              'fc_pair_rate_hz': rate, 'maximum_pair_gap_ms': gap, 'position_std_mm': std,
              'vicon_samples': node.samples, 'rejected_vicon': node.rejected,
              'rejected_fc': rejected_fc, 'stale_pair_count': stale_pair,
              'median_gravity_axis_difference_deg': float(np.median(tilts)) if rows else None,
              'median_fc_minus_vicon_roll_pitch_deg': np.median([r['fc_minus_vicon_wrapped_deg'][:2] for r in rows], axis=0).tolist() if rows else None,
              'last_pair': rows[-1] if rows else None, 'pairs': rows,
              'heartbeats': evidence.heartbeats, 'tx_messages': dict(counts), 'error': error,
              'north_alignment_verified': False, 'dynamic_axes_verified': False,
              'imu_origin_verified': False, 'ready_to_send_to_fc': False, 'actual_sent_to_fc': 0}
    encoded = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)
    with args.output.open('x') as stream:
        stream.write(encoded + '\n')
    summary = {k: v for k, v in result.items() if k not in ('pairs', 'heartbeats')}
    print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))
    return 0 if result['static_observation_passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
