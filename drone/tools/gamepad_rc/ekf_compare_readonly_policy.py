"""双EKF对照的只读边界；不允许固定实例、改多IMU或启用融合。"""

import math
import re

import height_readonly_policy as height
import usb_fusion_policy as horizontal

PARAMS = height.PARAMS + ('EKF2_MULTI_IMU', 'SENS_IMU_MODE', 'EKF2_SEL_ERR_RED',
                         'EKF2_SEL_IMU_ACC', 'EKF2_SEL_IMU_ANG', 'EKF2_SEL_IMU_RAT', 'EKF2_SEL_IMU_VEL')
COMMANDS = dict(height.COMMANDS)
for instance in range(9):
    COMMANDS[f'xy_{instance}'] = horizontal.COMMANDS[f'aid_{instance}']

SOURCES = height.SOURCES
parse_aid = height.parse_aid
parse_xy = horizontal.parse_aid
absent_reply = height.absent_reply
assess_sources = height.assess_sources


def shell_message(encoder, key):
    payload = COMMANDS[key]
    return encoder.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0'))


def allowed(message, *, safe=False):
    if message.get_type() == 'PARAM_REQUEST_READ':
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\0')
        return (name in PARAMS and message.param_index == -1
                and (message.target_system, message.target_component) == (1, 1))
    if message.get_type() == 'SERIAL_CONTROL' and (message.device, message.flags, message.timeout, message.baudrate) == (10, 2, 0, 0):
        if any(message.count == len(p) and bytes(message.data) == p.ljust(70, b'\0') for p in COMMANDS.values()):
            return safe
    return height.allowed(message, safe=safe)


def parse_flags(text):
    xy, z = horizontal.parse_flags(text), height.parse_flags(text)
    return {**xy, **z} if xy and z else None


def parse_selector(text):
    """保留未运行实例的NaN为null，绝不冒充零误差/健康实例。"""
    blocks = re.split(r'TOPIC:\s*estimator_selector_status[^\n]*\n', text)
    for block in reversed(blocks[1:]):
        full = 'TOPIC: estimator_selector_status\n' + block
        result = horizontal.parse_selector(full)
        extra = horizontal.parse_topic(full, 'estimator_selector_status', ('last_instance_change', 'baro_device_id', 'mag_device_id'))
        if not result or not extra or not 1 <= result['instances_available'] <= 9:
            continue
        result.update(extra)
        for name in ('combined_test_ratio', 'relative_test_ratio', 'healthy'):
            match = re.search(rf'^\s*{name}:\s*\[([^\]]+)\]', block, re.M)
            if not match:
                break
            values = [v.strip() for v in match.group(1).split(',')]
            if len(values) != 9:
                break
            try:
                if name == 'healthy':
                    if any(v.lower() not in ('true', 'false') for v in values):
                        break
                    result[name] = [v.lower() == 'true' for v in values]
                else:
                    floats = [float(v) for v in values]
                    if any(math.isinf(v) for v in floats):
                        break
                    if not all(math.isfinite(v) for v in floats[:result['instances_available']]):
                        break
                    result[name] = [v if math.isfinite(v) else None for v in floats]
            except ValueError:
                break
        else:
            if 0 <= result['primary_instance'] < result['instances_available']:
                return result
    return None


def assess_comparisons(rows):
    if len(rows) < 2:
        return {'valid': False, 'reason': 'need_multiple_readonly_observations'}
    per_instance = {}
    for instance in (0, 1):
        observations = [r['instances'].get(str(instance)) for r in rows]
        if not all(observations):
            return {'valid': False, 'reason': 'missing_instance_data'}
        baro = [o['baro'] for o in observations]
        baro_live = all(a and a['estimator_instance'] == instance and a['fusion_enabled'] and a['fused']
                        and not a['innovation_rejected'] and a['time_last_fuse'] > 0
                        and abs(r['local']['timestamp'] - a['timestamp']) < 2000000
                        for r, a in zip(rows, baro))
        baro_live = baro_live and all(b['time_last_fuse'] > a['time_last_fuse']
                                    and b['timestamp_sample'] > a['timestamp_sample']
                                    for a, b in zip(baro, baro[1:]))
        old_ev = [o['ev'] for o in observations]
        ev_stamps = {a['timestamp_sample'] for a in old_ev if a}
        per_instance[str(instance)] = {
            'selector_healthy_all_observations': all(r['selector']['healthy'][instance] for r in rows),
            'flags_tilt_aligned_all_observations': all(o['flags']['cs_tilt_align'] for o in observations),
            'flags_yaw_aligned_all_observations': all(o['flags']['cs_yaw_align'] for o in observations),
            'ev_pos_and_height_disabled_all_observations': all(not o['flags']['cs_ev_pos'] and not o['flags']['cs_ev_hgt'] for o in observations),
            'vertical_numerical_fault_observed': any(o['flags']['fs_bad_pos_d'] for o in observations),
            'baro_successful_fusion_time_advances': bool(baro_live),
            'last_ev_height_sample_is_unchanged_cache': len(ev_stamps) == 1 and all(old_ev),
            'historical_ev_height_fusion_record_present': any(a and a['time_last_fuse'] > 0 for a in old_ev),
            'combined_test_ratios': [r['selector']['combined_test_ratio'][instance] for r in rows],
            'relative_test_ratios': [r['selector']['relative_test_ratio'][instance] for r in rows],
        }
    return {'valid': True, 'instances': per_instance,
            'primary_instances': [r['selector']['primary_instance'] for r in rows],
            'change_counts': [r['selector']['instance_changed_count'] for r in rows],
            'historical_switch_cause_independently_verified': False,
            'current_healthy_flags_do_not_exclude_a_past_transient': True}
