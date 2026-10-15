#!/usr/bin/env python3
"""只在内存编码报文并计算TRS带宽，不打开串口、发控制报文或改无线模式。"""

import argparse
import datetime
import json
import math
from pathlib import Path
import statistics

from pymavlink.dialects.v20 import common as m


CAPACITIES = {'long-range_57600': 2400, 'standard_57600': 4000,
              'high-speed_115200': 8000}
SOURCES = {
    'trs_modes': 'https://micoair.cn/zh/docs/telemetry/trs/trs',
    'external_vision': 'https://docs.px4.io/v1.14/en/ros/external_position_estimation.html',
    'failsafe_parameters': 'https://docs.px4.io/v1.14/en/advanced_config/parameter_reference.html',
}


def packet_sizes():
    encoder = m.MAVLink(None, srcSystem=250, srcComponent=197)
    pose_cov = [0.0] * 21
    for i in (0, 6, 11):
        pose_cov[i] = 0.02**2
    for i in (15, 18, 20):
        pose_cov[i] = 0.1**2
    mask = (m.POSITION_TARGET_TYPEMASK_VX_IGNORE | m.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | m.POSITION_TARGET_TYPEMASK_VZ_IGNORE | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AY_IGNORE | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    # 目标位置/停止指令只做字节预算，绝不通过send函数输出。
    messages = {
        'ODOMETRY': encoder.odometry_encode(
            1700000000000000, m.MAV_FRAME_LOCAL_FRD, m.MAV_FRAME_BODY_FRD,
            1, -2, -1, [1, 0, 0, 0], *([math.nan] * 6), pose_cov,
            [math.nan] + [0.0] * 20, 0, m.MAV_ESTIMATOR_TYPE_VISION, 0),
        'VISION_POSITION_ESTIMATE_covariance': encoder.vision_position_estimate_encode(
            1700000000000000, 1, -2, -1, 0, 0, 0, pose_cov, 0),
        'SET_POSITION_TARGET_LOCAL_NED': encoder.set_position_target_local_ned_encode(
            1000, 1, 1, m.MAV_FRAME_LOCAL_NED, mask, 1, 2, -1, 0, 0, 0, 0, 0, 0, 0, 0),
        'HEARTBEAT': encoder.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0),
        'TIMESYNC': encoder.timesync_encode(0, 1700000000000000000),
        'STOP_COMMAND_LONG': encoder.command_long_encode(1, 1, 185, 0, 2, 0, 0, 0, 0, 0, 0),
    }
    return {name: len(msg.pack(encoder)) for name, msg in messages.items()}


def budget(sizes, vision_message='ODOMETRY', signing=False, control_rate_hz=20,
           vision_rate_hz=30):
    if not math.isfinite(control_rate_hz) or not 5 <= control_rate_hz <= 50:
        raise ValueError('Planning control rate must be 5..50Hz, not the Offboard survival threshold')
    if not math.isfinite(vision_rate_hz) or not 30 <= vision_rate_hz <= 50:
        raise ValueError('Keep covariance-bearing external-vision planning rate at 30..50Hz')
    # 20%空余是规划余量，不是厂商保证；另保留5次/秒的停止指令突发空间。
    rates = {vision_message: vision_rate_hz, 'SET_POSITION_TARGET_LOCAL_NED': control_rate_hz,
             'HEARTBEAT': 1, 'TIMESYNC': 10, 'STOP_COMMAND_LONG': 5}
    parts = {name: (sizes[name] + (13 if signing else 0)) * rate
             for name, rate in rates.items()}
    total = sum(parts.values())
    return {'rates_hz': rates, 'parts_bytes_per_second': parts,
            'planned_peak_uplink_bytes_per_second': total,
            'mavlink2_signing': signing,
            'capacity_comparison': {
                mode: {'advertised_bytes_per_second': capacity,
                       'planning_limit_with_20_percent_headroom': capacity * 0.8,
                       'fits_even_without_headroom': total <= capacity,
                       'fits_planning_limit': total <= capacity * 0.8}
                for mode, capacity in CAPACITIES.items()},
            'execution_verified': False}


def frequency_options(sizes):
    # 回传遥测按MAVLink2完整载荷上界预算，避免尾零截断使估算过于乐观。
    downlink_rates = {'HEARTBEAT': (0, 1), 'SYS_STATUS': (1, 1),
                      'LOCAL_POSITION_NED': (32, 5), 'ATTITUDE_QUATERNION': (31, 5),
                      'BATTERY_STATUS': (147, 1), 'ESTIMATOR_STATUS': (230, 1),
                      'EXTENDED_SYS_STATE': (245, 1), 'ALTITUDE': (141, 1)}
    downlink = {
        name: {'rate_hz': hz, 'maximum_unsigned_mavlink2_bytes': m.mavlink_map[msgid].unpacker.size + 12,
               'bytes_per_second': (m.mavlink_map[msgid].unpacker.size + 12) * hz}
        for name, (msgid, hz) in downlink_rates.items()}
    total_downlink = sum(item['bytes_per_second'] for item in downlink.values())
    profiles = {
        'full_odometry_control_10hz': budget(sizes, control_rate_hz=10),
        'compact_covariance_control_10hz_candidate': budget(
            sizes, 'VISION_POSITION_ESTIMATE_covariance', control_rate_hz=10),
    }
    for profile in profiles.values():
        combined = profile['planned_peak_uplink_bytes_per_second'] + total_downlink
        profile['if_radio_capacity_is_shared_both_directions'] = {
            'combined_planned_bytes_per_second': combined,
            'fits_high_speed_20_percent_headroom': combined <= CAPACITIES['high-speed_115200'] * 0.8,
        }
    return {'scope': 'Offline candidate rates only; no SET_MESSAGE_INTERVAL or parameter writes',
            'source_sampling': 'Keep Vicon wired input unchanged; select only newest valid sample for 30Hz preview',
            'proposed_downlink': downlink, 'planned_periodic_downlink_bytes_per_second': total_downlink,
            'event_messages': 'ACK and STATUSTEXT stay event-driven; their bursts are not bounded by this table',
            'profiles': profiles, 'radio_mode_changes': [], 'flight_controller_rate_changes': []}


def safety_findings(parameters):
    findings = []
    if parameters.get('COM_OBL_RC_ACT') == 0:
        findings.append('Offboard loss action is Position, not an established indoor autonomous-Land policy.')
    if int(parameters.get('COM_RCL_EXCEPT', 0)) & 4:
        findings.append('RC loss is ignored while in Offboard; RC-loss settings alone do not protect PC/TRS setpoint loss.')
    if parameters.get('NAV_DLL_ACT') == 3:
        findings.append('Data-link loss is configured as Land; parameter readback is NOT an executed failsafe test.')
    if parameters.get('NAV_RCL_ACT') == 3:
        findings.append('RC loss is configured as Land outside applicable exception modes; not an executed test.')
    return findings


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--audit', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        ap.error('Use a new output filename')
    audit = json.loads(args.audit.read_text())
    if not (audit.get('all_parameters_received') and audit.get('round_trip_verified')
            and audit.get('all_fc_heartbeats_disarmed') and audit.get('error') is None):
        ap.error('Need a complete, disarmed, successful read-only audit')
    sizes = packet_sizes()
    rtt = audit['ping_rtt_ms']
    result = {
        'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'scope': 'Offline packet budget + read-only safety-parameter interpretation, not flight readiness',
        'source_audit': {'path': str(args.audit.resolve()),
                         'captured_at_utc': audit.get('captured_at_utc'),
                         'identity': audit.get('identity')},
        'sources': SOURCES, 'actual_fc_messages_sent_by_this_tool': 0,
        'packet_sizes_bytes_unsigned_mavlink2': sizes,
        'serial_baud_at_source_audit': audit['baud'],
        'radio_mode_verified': False,
        'capacity_assumptions': [
            'Advertised KB/s treated as 1000 bytes/s and optimistically all available to uplink.',
            '57600 cannot distinguish standard versus long-range; mode was not changed or queried.',
            'Measured downlink, air retries, framing and scheduling may further reduce available uplink.',
            'Sample MAVLink2 trailing-zero truncation can vary; these are measured sample packet sizes, not maximum payload sizes.',
            'Compact VISION_POSITION_ESTIMATE is only a candidate: PX4 1.14 tags it as NED; yaw/frame alignment needs a separate review.',
        ],
        'full_odometry_profile': budget(sizes),
        'compact_covariance_candidate_not_implemented': budget(sizes, 'VISION_POSITION_ESTIMATE_covariance'),
        'full_odometry_signed_profile': budget(sizes, signing=True),
        'frequency_optimization_candidates': frequency_options(sizes),
        'read_only_latency': {'matched_replies': len(rtt), 'minimum_ms': min(rtt),
                              'median_ms': statistics.median(rtt), 'maximum_ms': max(rtt),
                              'mean_ms': statistics.mean(rtt),
                              'tx_bytes_per_second': audit['tx_bytes'] / audit['duration_s'],
                              'rx_bytes_per_second': audit['rx_bytes'] / audit['duration_s'],
                              'scope': 'Low-load two-way PING, not one-way or loaded control delay'},
        'parameters': audit['requested_parameters'],
        'safety_findings': safety_findings(audit['requested_parameters']),
        'parameter_changes': [], 'physical_link_loss_test_executed': False,
        'flight_ready': False,
    }
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
