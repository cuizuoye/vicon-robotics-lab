"""指定PX4 1.14.3的独立EV航向台架；仅EV_CTRL 0/8，无位置/速度融合或飞行。"""

import math
import struct

from pymavlink.dialects.v20 import common as m
import usb_fusion_policy as horizontal
import usb_height_fusion_policy as height
from usb_vision_policy import send_allowed as receive_allowed
from manual_translation import rotation

READ_PARAMS = height.READ_PARAMS + ('EKF2_EVA_NOISE', 'EKF2_MAG_TYPE', 'EKF2_MAG_CHECK')
COMMANDS = dict(horizontal.COMMANDS)
for instance in range(9):
    COMMANDS[f'yaw_aid_{instance}'] = f'listener estimator_aid_src_ev_yaw -i {instance} -n 1\n'.encode()


def fusion_scope_ok(params):
    return (params.get('EKF2_EV_CTRL') in (0, 8) and params.get('EKF2_HGT_REF') == 0
            and params.get('EKF2_BARO_CTRL') == 1 and params.get('EKF2_AID_MASK') == 0
            and params.get('EKF2_EV_QMIN') == 0
            and all(params.get(n) == 0 for n in ('EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z')))


def control_message(encoder, value):
    if type(value) is not int or value not in (0, 8):
        raise ValueError('Only yaw-only enable=8 and restore=0 permitted')
    wire = struct.unpack('<f', struct.pack('<i', value))[0]
    return encoder.param_set_encode(1, 1, b'EKF2_EV_CTRL', wire, m.MAV_PARAM_TYPE_INT32)


def fusion_shell_message(encoder, topic):
    payload = COMMANDS[topic]
    return encoder.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0'))


def allowed(message, *, authorized=False, safe=False, restore_safe=False,
            backup_verified=False, clock_verified=False, position_authorized=False):
    kind = message.get_type()
    if kind == 'PARAM_SET':
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\0')
        if not (authorized and backup_verified and name == 'EKF2_EV_CTRL'
                and message.param_type == m.MAV_PARAM_TYPE_INT32
                and (message.target_system, message.target_component) == (1, 1)):
            return False
        value = struct.unpack('<i', struct.pack('<f', message.param_value))[0]
        return (value == 0 and restore_safe) or (value == 8 and safe and clock_verified)
    if kind == 'PARAM_REQUEST_READ':
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\0')
        if name in READ_PARAMS:
            return (message.target_system, message.target_component, message.param_index) == (1, 1, -1)
    if kind == 'SERIAL_CONTROL' and (message.device, message.flags, message.timeout, message.baudrate) == (10, 2, 0, 0):
        if any(message.count == len(p) and bytes(message.data) == p.ljust(70, b'\0') for p in COMMANDS.values()):
            return safe
    return receive_allowed(message, bench_authorized=authorized and position_authorized, safe=safe)


parse_selector = horizontal.parse_selector
parse_aid = horizontal.parse_aid


def parse_flags(text):
    core = height.parse_flags(text)
    more = horizontal.parse_topic(text, 'estimator_status_flags',
                                  bool_fields=('cs_gps', 'cs_mag_hdg', 'cs_mag_3d', 'cs_gps_yaw', 'fs_bad_hdg', 'reject_yaw'))
    return {**core, **more} if core and more else None


def parse_local_position(text):
    core = horizontal.parse_local_position(text)
    more = horizontal.parse_topic(text, 'vehicle_local_position', bool_fields=('heading_good_for_control',))
    return {**core, **more} if core and more else None


def parse_yaw_aid(text):
    return horizontal.parse_topic(text, 'estimator_aid_src_ev_yaw',
                                  ('timestamp', 'timestamp_sample', 'estimator_instance', 'time_last_fuse'),
                                  ('fusion_enabled', 'innovation_rejected', 'fused'),
                                  ('observation', 'observation_variance', 'innovation', 'innovation_variance', 'test_ratio'))


def inactive_other_heading_sources(flags):
    return bool(flags and not any(flags[n] for n in ('cs_gps', 'cs_mag_hdg', 'cs_mag_3d', 'cs_gps_yaw')))


def compare_heading(vicon_wxyz, fc_wxyz, *, attitude_age_ms, sent_age_ms):
    rv, rf = rotation(vicon_wxyz), rotation(fc_wxyz)
    yv, yf = math.atan2(rv[1, 0], rv[0, 0]), math.atan2(rf[1, 0], rf[0, 0])
    delta = math.degrees(math.atan2(math.sin(yf-yv), math.cos(yf-yv)))
    return {'vicon_yaw_deg': math.degrees(yv), 'fc_yaw_deg': math.degrees(yf),
            'fc_minus_vicon_wrapped_deg': delta, 'attitude_age_ms': attitude_age_ms,
            'sent_age_ms': sent_age_ms,
            'fresh': 0 <= attitude_age_ms <= 300 and 0 <= sent_age_ms <= 150,
            'earth_north_alignment_verified': False}


def fusion_evidence_ok(rows, primary):
    if len(rows) < 2:
        return False
    for row in rows:
        f, a, local, q = row['flags'], row.get('yaw_aid'), row['local'], row.get('yaw_comparison')
        if not (f and a and local and q and f['cs_tilt_align'] and f['cs_ev_yaw'] and f['cs_baro_hgt']
                and inactive_other_heading_sources(f)
                and not any(f[n] for n in ('cs_ev_pos', 'cs_ev_hgt', 'cs_ev_vel', 'cs_in_air', 'fs_bad_hdg', 'reject_yaw'))
                and a['estimator_instance'] == primary and a['fusion_enabled'] and a['fused']
                and not a['innovation_rejected'] and a['time_last_fuse'] > 0
                and a['observation_variance'] > 0 and a['innovation_variance'] > 0
                and 0 <= a['test_ratio'] <= 1 and abs(local['timestamp'] - a['timestamp']) < 2000000
                and q['fresh'] and abs(q['fc_minus_vicon_wrapped_deg']) <= 5):
            return False
        # LOCAL_FRD EV yaw刻意令地理北向yaw_align=False；不通过伪改标志“修复”。
    return all(b['yaw_aid']['time_last_fuse'] > a['yaw_aid']['time_last_fuse']
               and b['yaw_aid']['timestamp_sample'] > a['yaw_aid']['timestamp_sample']
               for a, b in zip(rows, rows[1:]))
