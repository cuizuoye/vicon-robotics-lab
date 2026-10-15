"""单项水平融合台架的独立白名单；不放宽原先融合关闭的USB接收策略。"""

import math
import re
import struct

from pymavlink.dialects.v20 import common as m
from usb_vision_policy import PARAMS, send_allowed as receive_allowed

READ_PARAMS = PARAMS + ('EKF2_EV_QMIN', 'EKF2_HGT_REF', 'EKF2_EV_DELAY',
                      'EKF2_IMU_POS_X', 'EKF2_IMU_POS_Y', 'EKF2_IMU_POS_Z')
COMMANDS = {'selector': b'listener estimator_selector_status -n 1\n',
            'local_position': b'listener vehicle_local_position -n 1\n'}
for instance in range(9):
    COMMANDS[f'flags_{instance}'] = f'listener estimator_status_flags -i {instance} -n 1\n'.encode()
    COMMANDS[f'aid_{instance}'] = f'listener estimator_aid_src_ev_pos -i {instance} -n 1\n'.encode()


def fusion_scope_ok(params):
    return (params.get('EKF2_EV_CTRL') in (0, 1) and params.get('EKF2_AID_MASK') == 0
            and params.get('EKF2_EV_QMIN') == 0
            and all(params.get(n) == 0 for n in PARAMS[2:]))


def control_message(encoder, value):
    if type(value) is not int or value not in (0, 1):
        raise ValueError('Only horizontal enable=1 and restore=0 are permitted')
    # PX4 PARAM整数走字节编码。直接传float(1)会被误解为1065353216！
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
        return (value == 0 and restore_safe) or (value == 1 and safe and clock_verified)
    if kind == 'PARAM_REQUEST_READ':
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\0')
        if name in READ_PARAMS:
            return (message.target_system, message.target_component, message.param_index) == (1, 1, -1)
    if kind == 'SERIAL_CONTROL':
        if ((message.device, message.flags, message.timeout, message.baudrate) == (10, 2, 0, 0)
                and any(message.count == len(p) and bytes(message.data) == p.ljust(70, b'\0')
                        for p in COMMANDS.values())):
            return safe
    return receive_allowed(message, bench_authorized=authorized and position_authorized, safe=safe)


def parse_topic(text, topic, integer_fields=(), bool_fields=(), float_fields=(), vectors=()):
    blocks = re.split(rf'TOPIC:\s*{re.escape(topic)}[^\n]*\n', text)
    for block in reversed(blocks[1:]):
        result = {}
        for name in integer_fields:
            found = re.search(rf'^\s*{name}:\s*(-?\d+)\b', block, re.M)
            if found:
                result[name] = int(found.group(1))
        for name in bool_fields:
            found = re.search(rf'^\s*{name}:\s*(True|False|true|false)\b', block, re.M)
            if found:
                result[name] = found.group(1).lower() == 'true'
        for name in float_fields:
            found = re.search(rf'^\s*{name}:\s*([^\s]+)', block, re.M)
            if found:
                try:
                    value = float(found.group(1))
                    if math.isfinite(value):
                        result[name] = value
                except ValueError:
                    pass
        for name, length in vectors:
            found = re.search(rf'^\s*{name}:\s*\[([^\]]+)\]', block, re.M)
            if found:
                try:
                    value = [float(v.strip()) for v in found.group(1).split(',')]
                    if len(value) == length and all(math.isfinite(v) for v in value):
                        result[name] = value
                except ValueError:
                    pass
        if len(result) == len(integer_fields)+len(bool_fields)+len(float_fields)+len(vectors):
            return result
    return None


def parse_selector(text):
    return parse_topic(text, 'estimator_selector_status',
                       ('timestamp', 'primary_instance', 'instances_available', 'instance_changed_count',
                        'accel_device_id', 'gyro_device_id'), ('gyro_fault_detected', 'accel_fault_detected'))


def parse_flags(text):
    return parse_topic(text, 'estimator_status_flags', ('timestamp', 'timestamp_sample'),
                       ('cs_tilt_align', 'cs_ev_pos', 'cs_ev_yaw', 'cs_ev_vel', 'cs_ev_hgt', 'cs_in_air',
                        'fs_bad_pos_n', 'fs_bad_pos_e', 'reject_hor_pos', 'reject_optflow_y'))


def parse_aid(text):
    return parse_topic(text, 'estimator_aid_src_ev_pos',
                       ('timestamp', 'timestamp_sample', 'estimator_instance', 'time_last_fuse'),
                       ('fusion_enabled', 'innovation_rejected', 'fused'),
                       vectors=tuple((n, 2) for n in ('observation', 'observation_variance', 'innovation',
                                                     'innovation_variance', 'test_ratio')))


def aid_topic_for_estimator(topics, primary):
    """uORB topic slots may differ from EKF IDs when EV is enabled after boot.

    Match the explicit payload estimator_instance; never infer from slot number.
    Missing/ambiguous mappings fail closed. This does not prove actual fusion.
    """
    matches = [slot for slot, aid in topics.items()
               if type(slot) is int and 0 <= slot <= 8 and aid
               and type(aid.get('estimator_instance')) is int
               and aid['estimator_instance'] == primary]
    if type(primary) is not int or not 0 <= primary <= 8 or len(matches) != 1:
        raise ValueError('No unique aid topic explicitly belonging to the primary EKF')
    return matches[0]


def parse_local_position(text):
    return parse_topic(text, 'vehicle_local_position', ('timestamp', 'timestamp_sample'),
                       ('xy_valid', 'z_valid'), ('x', 'y', 'z', 'heading'))


def fusion_evidence_ok(rows, primary):
    if len(rows) < 2:
        return False
    for row in rows:
        flags, aid, local = row['flags'], row['aid'], row['local']
        if not (flags and aid and local and flags['cs_tilt_align'] and flags['cs_ev_pos']
                and not any(flags[n] for n in ('cs_ev_yaw', 'cs_ev_vel', 'cs_ev_hgt', 'cs_in_air',
                                                'fs_bad_pos_n', 'fs_bad_pos_e', 'reject_hor_pos'))
                and aid['estimator_instance'] == primary and aid['fusion_enabled'] and aid['fused']
                and not aid['innovation_rejected'] and aid['time_last_fuse'] > 0 and local['xy_valid']
                and all(0 <= value <= 1 for value in aid['test_ratio'])):
            return False
    return all(b['aid']['time_last_fuse'] > a['aid']['time_last_fuse']
               and b['aid']['timestamp_sample'] > a['aid']['timestamp_sample']
               for a, b in zip(rows, rows[1:]))
