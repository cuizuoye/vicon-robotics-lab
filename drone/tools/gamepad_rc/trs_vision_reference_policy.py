"""Static TRS XYZ/yaw under Vision height reference; restore EV_CTRL=2.

No arming, mode, unlock or motion targets; retains the existing 65ms bench
delay candidate, not a newly measured camera or radio calibration.
"""
from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import decode_parameter
import trs_static_fusion_policy as old
import vision_reference_xyz_policy as xyz

DELAY_MS = old.DELAY_MS
READ_PARAMS = tuple(dict.fromkeys(old.READ_PARAMS + xyz.READ_PARAMS))
COMMANDS = old.COMMANDS
stationary = old.stationary
receive_time_mode = old.receive_time_mode


def parameter_message(enc, name, value):
    if name == 'EKF2_EV_CTRL' and type(value) is int and value in (2, 11):
        import struct
        wire = struct.unpack('<f', struct.pack('<i', value))[0]
        return enc.param_set_encode(1, 1, name.encode(), wire, m.MAV_PARAM_TYPE_INT32)
    if name == 'EKF2_EV_DELAY' and type(value) in (int, float) and value in (0, DELAY_MS):
        return enc.param_set_encode(1, 1, name.encode(), float(value), m.MAV_PARAM_TYPE_REAL32)
    raise ValueError('Only EV_CTRL 2/11 and EV_DELAY 0/65 are permitted')


def scope_ok(parameters, baseline):
    return bool(parameters.get('EKF2_EV_CTRL') in (2, 11)
        and xyz.fusion_scope_ok({**parameters, 'EKF2_EV_CTRL': 2})
        and parameters.get('EKF2_EV_DELAY') in (0, DELAY_MS)
        and all(parameters.get(n) == baseline.get(n) for n in READ_PARAMS
                if n not in ('EKF2_EV_CTRL', 'EKF2_EV_DELAY')))


def allowed(msg, *, route, safe=False, authorized=False, backup_verified=False,
            locked_static=False, receive_time_verified=False, restore_safe=False):
    if msg.get_type() == 'PARAM_SET':
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        value = decode_parameter(msg)
        types = {'EKF2_EV_CTRL': m.MAV_PARAM_TYPE_INT32, 'EKF2_EV_DELAY': m.MAV_PARAM_TYPE_REAL32}
        if not (route == 'usb' and authorized and backup_verified and name in types
                and msg.param_type == types[name] and (msg.target_system, msg.target_component) == (1, 1)):
            return False
        restore = 2 if name == 'EKF2_EV_CTRL' else 0
        candidate = 11 if name == 'EKF2_EV_CTRL' else DELAY_MS
        return bool((value == restore and restore_safe)
                    or (value == candidate and safe and locked_static and receive_time_verified))
    if msg.get_type() == 'PARAM_REQUEST_READ':
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        return bool(route == 'usb' and name in READ_PARAMS
                    and (msg.target_system, msg.target_component, msg.param_index) == (1, 1, -1))
    return old.allowed(msg, route=route, safe=safe, authorized=authorized,
        backup_verified=backup_verified, locked_static=locked_static,
        receive_time_verified=receive_time_verified, restore_safe=restore_safe)
