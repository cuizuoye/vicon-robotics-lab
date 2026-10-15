"""仅断动力静止台架的TRS到达时刻融合例外；不改变旧USB时钟门限。"""
import math
import struct

from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import readonly_message, decode_parameter
import compact_vision as compact
import usb_combined_fusion_policy as combined
from usb_vision_policy import COMMANDS as RECEIVE_COMMANDS

DELAY_MS = 65.0  # ROS发布到FC约61ms的台架候选；不是曝光时间标定。
READ_PARAMS = tuple(dict.fromkeys(combined.READ_PARAMS + ('EKF2_EV_DELAY',)))
COMMANDS = {**RECEIVE_COMMANDS, **combined.COMMANDS,
            'visual_series': compact.SERIES_COMMAND}


def parameter_message(encoder, name, value):
    if name == 'EKF2_EV_CTRL' and type(value) is int and value in (0, 11):
        wire = struct.unpack('<f', struct.pack('<i', value))[0]
        return encoder.param_set_encode(1, 1, name.encode(), wire, m.MAV_PARAM_TYPE_INT32)
    if name == 'EKF2_EV_DELAY' and type(value) in (int, float) and value in (0, DELAY_MS):
        return encoder.param_set_encode(1, 1, name.encode(), float(value), m.MAV_PARAM_TYPE_REAL32)
    raise ValueError('Only EV_CTRL 0/11 and provisional EV_DELAY 0/65 permitted')


def scope_ok(parameters, baseline):
    return (combined.fusion_scope_ok(parameters)
            and parameters.get('EKF2_EV_DELAY') in (0, DELAY_MS)
            and all(parameters.get(n) == baseline[n] for n in READ_PARAMS
                    if n not in ('EKF2_EV_CTRL', 'EKF2_EV_DELAY')))


def stationary(p, q, origin_p, origin_q):
    import numpy as np
    arrays = [np.asarray(v, dtype=float) for v in (p, q, origin_p, origin_q)]
    if any(not np.all(np.isfinite(v)) for v in arrays):
        return False
    if any(not .999 <= np.linalg.norm(v) <= 1.001 for v in (arrays[1], arrays[3])):
        return False
    angle = 2 * math.acos(float(np.clip(abs(arrays[1] @ arrays[3]), 0, 1)))
    return np.linalg.norm(arrays[0] - arrays[2]) <= .01 and angle <= math.radians(2)


def receive_time_mode(snapshot):
    # 必须观察本链路仍为接收时刻回退，不能把USB同步冒充TRS同步。
    return bool(snapshot and 0 <= snapshot['timestamp'] - snapshot['timestamp_sample'] <= 1000)


def allowed(message, *, route, safe=False, authorized=False, backup_verified=False,
            locked_static=False, receive_time_verified=False, restore_safe=False):
    if route not in ('usb', 'trs'):
        return False
    kind = message.get_type()
    if kind == 'PARAM_SET':
        if not (route == 'usb' and authorized and backup_verified
                and (message.target_system, message.target_component) == (1, 1)):
            return False
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\0')
        value = decode_parameter(message)
        expected_type = {'EKF2_EV_CTRL': m.MAV_PARAM_TYPE_INT32,
                         'EKF2_EV_DELAY': m.MAV_PARAM_TYPE_REAL32}.get(name)
        if expected_type is None or message.param_type != expected_type:
            return False
        if value == 0:
            return bool(restore_safe)  # 回退不依赖Vicon或时钟，仍需同机未解锁身份。
        expected = 11 if name == 'EKF2_EV_CTRL' else DELAY_MS
        return bool(value == expected and safe and locked_static and receive_time_verified)
    if kind == 'VISION_POSITION_ESTIMATE':
        return bool(route == 'trs' and authorized and backup_verified and safe
                    and locked_static and compact.pose_allowed(message))
    if kind == 'PARAM_REQUEST_READ':
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii').rstrip('\0')
        return bool(route == 'usb' and name in READ_PARAMS
                    and (message.target_system, message.target_component, message.param_index) == (1, 1, -1))
    if kind == 'SERIAL_CONTROL':
        if route != 'usb' or (message.device, message.timeout, message.baudrate) != (10, 0, 0):
            return False
        data = bytes(message.data)
        if message.flags == 0:
            return False
        return bool(safe and message.flags == 2 and any(
            message.count == len(p) and data == p.ljust(70, b'\0') for p in (b'', *COMMANDS.values())))
    # 不响应TRS TIMESYNC：这是显式到达时刻补偿台架，不声称采样时间同步。
    return readonly_message(message) if kind != 'PARAM_REQUEST_READ' else False
