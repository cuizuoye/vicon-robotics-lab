"""PX4 1.14.3高度辅助台架的独立写入边界：仅EV_CTRL 0/3，保留气压参考。"""

import struct

from pymavlink.dialects.v20 import common as m
import usb_fusion_policy as horizontal
import height_readonly_policy as height
from usb_vision_policy import send_allowed as receive_allowed

READ_PARAMS = tuple(dict.fromkeys(horizontal.READ_PARAMS + height.PARAMS))
COMMANDS = dict(horizontal.COMMANDS)
for instance in range(9):
    COMMANDS[f'height_aid_{instance}'] = height.COMMANDS[f'ev_{instance}']


def fusion_scope_ok(params):
    return (params.get('EKF2_EV_CTRL') in (0, 3) and params.get('EKF2_HGT_REF') == 0
            and params.get('EKF2_BARO_CTRL') == 1 and params.get('EKF2_AID_MASK') == 0
            and params.get('EKF2_EV_QMIN') == 0
            and all(params.get(n) == 0 for n in ('EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z')))


def control_message(encoder, value):
    if type(value) is not int or value not in (0, 3):
        raise ValueError('Only XYZ aiding=3 and restore=0 are permitted; no reference changes')
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
        return (value == 0 and restore_safe) or (value == 3 and safe and clock_verified)
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
parse_local_position = horizontal.parse_local_position
parse_aid = horizontal.parse_aid


def parse_flags(text):
    xy, z = horizontal.parse_flags(text), height.parse_flags(text)
    return {**xy, **z} if xy and z else None


def parse_height_aid(text):
    return height.parse_aid(text, 'ev')


def fusion_evidence_ok(rows, primary):
    if len(rows) < 2:
        return False
    for row in rows:
        f, a, h, local = row['flags'], row['aid'], row.get('height_aid'), row['local']
        if not (f and a and h and local and f['cs_tilt_align'] and f['cs_ev_pos'] and f['cs_ev_hgt']
                and f['cs_baro_hgt'] and local['xy_valid'] and local['z_valid']
                and h['observation_variance'] > 0 and h['innovation_variance'] > 0
                and not any(f[n] for n in ('cs_ev_yaw', 'cs_ev_vel', 'cs_in_air',
                                            'fs_bad_pos_n', 'fs_bad_pos_e', 'fs_bad_pos_d',
                                            'reject_hor_pos', 'reject_ver_pos'))):
            return False
        for aid in (a, h):
            ratios = aid['test_ratio'] if isinstance(aid['test_ratio'], list) else [aid['test_ratio']]
            if not (aid['estimator_instance'] == primary and aid['fusion_enabled'] and aid['fused']
                    and not aid['innovation_rejected'] and aid['time_last_fuse'] > 0
                    and abs(local['timestamp'] - aid['timestamp']) < 2000000
                    and all(0 <= v <= 1 for v in ratios)):
                return False
    return all(b[k]['time_last_fuse'] > a[k]['time_last_fuse']
               and b[k]['timestamp_sample'] > a[k]['timestamp_sample']
               for a, b in zip(rows, rows[1:]) for k in ('aid', 'height_aid'))
