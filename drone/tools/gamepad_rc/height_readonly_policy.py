"""高度源审查的独立只读白名单；不放宽任何融合写入工具。"""

from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import readonly_message
from usb_fusion_policy import parse_topic
from usb_vision_policy import shell_allowed

PARAMS = ('EKF2_EV_CTRL', 'EKF2_AID_MASK', 'EKF2_HGT_REF', 'EKF2_BARO_CTRL',
          'EKF2_RNG_CTRL', 'EKF2_GPS_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_QMIN',
          'EKF2_EVP_NOISE', 'EKF2_EV_NOISE_MD', 'EKF2_EVP_GATE',
          'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z',
          'EKF2_IMU_POS_X', 'EKF2_IMU_POS_Y', 'EKF2_IMU_POS_Z')
SOURCES = ('baro', 'ev', 'gnss', 'rng', 'fake')
COMMANDS = {'actuator': b'listener actuator_armed -n 1\n',
            'selector': b'listener estimator_selector_status -n 1\n',
            'local': b'listener vehicle_local_position -n 1\n',
            'distance': b'listener distance_sensor -n 1\n',
            'flow': b'listener sensor_optical_flow -n 1\n'}
for instance in range(9):
    COMMANDS[f'flags_{instance}'] = f'listener estimator_status_flags -i {instance} -n 1\n'.encode()
    for source in SOURCES:
        COMMANDS[f'{source}_{instance}'] = f'listener estimator_aid_src_{source}_hgt -i {instance} -n 1\n'.encode()


def shell_message(encoder, key):
    payload = COMMANDS[key]
    return encoder.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0'))


def allowed(message, *, safe=False):
    kind = message.get_type()
    # 防止将通用诊断策略未来的改变带入此工具。
    if kind in ('HEARTBEAT', 'COMMAND_LONG'):
        return readonly_message(message)
    if kind == 'PARAM_REQUEST_READ':
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\0')
        return (name in PARAMS and message.param_index == -1
                and (message.target_system, message.target_component) == (1, 1))
    if kind == 'SERIAL_CONTROL':
        if shell_allowed(message) and message.count == 0:
            return message.flags == 0 or safe
        return bool(safe and (message.device, message.flags, message.timeout, message.baudrate) == (10, 2, 0, 0)
                    and any(message.count == len(p) and bytes(message.data) == p.ljust(70, b'\0')
                            for p in COMMANDS.values()))
    # 包括PARAM_SET、ODOMETRY、RC、模式、解锁、重启、频率命令，一律拒绝。
    return False


def parse_flags(text):
    return parse_topic(text, 'estimator_status_flags', ('timestamp', 'timestamp_sample'),
                       ('cs_tilt_align', 'cs_yaw_align', 'cs_in_air', 'cs_baro_hgt',
                        'cs_rng_hgt', 'cs_gps_hgt', 'cs_ev_hgt', 'cs_fake_hgt',
                        'cs_ev_pos', 'cs_ev_yaw', 'cs_ev_vel', 'fs_bad_pos_d', 'reject_ver_pos'))


def parse_aid(text, source):
    if source not in SOURCES:
        raise ValueError('Unknown height source')
    return parse_topic(text, f'estimator_aid_src_{source}_hgt',
                       ('timestamp', 'timestamp_sample', 'estimator_instance', 'time_last_fuse'),
                       ('fusion_enabled', 'innovation_rejected', 'fused'),
                       ('observation', 'observation_variance', 'innovation', 'innovation_variance', 'test_ratio'))


def absent_reply(text):
    # listener对从未有数据的已注册topic也可能返回“等待2秒无消息”，不是链路故障。
    if 'never published' in text:
        return 'never_published'
    if 'Waited for 2.0 seconds without a message. Giving up.' in text:
        return 'no_message_within_listener_wait'
    return None


def assess_sources(rows, primary):
    """状态位与递增内部融合时间一起验收；配置使能不等于实测融合。"""
    result = {}
    for source in SOURCES:
        samples = [r['aids'].get(source) for r in rows]
        bit = {'gnss': 'gps'}.get(source, source)
        good = bool(len(samples) >= 2 and all(s and s['estimator_instance'] == primary
                    and s['fusion_enabled'] and s['fused'] and not s['innovation_rejected']
                    and s['time_last_fuse'] > 0 and s['observation_variance'] > 0
                    and s['innovation_variance'] > 0 and 0 <= s['test_ratio'] <= 1
                    for s in samples))
        good = good and all(r['selector']['primary_instance'] == primary
                            and r['flags'][f'cs_{bit}_hgt'] and r['local']['z_valid']
                            and abs(r['local']['timestamp'] - s['timestamp']) < 2000000
                            for r, s in zip(rows, samples))
        increasing = good and all(b['timestamp_sample'] > a['timestamp_sample']
                                 and b['time_last_fuse'] > a['time_last_fuse']
                                 for a, b in zip(samples, samples[1:]))
        result[source] = {'sample_count': sum(s is not None for s in samples),
                          'continuous_internal_fusion_observed': bool(increasing)}
    return result
