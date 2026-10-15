"""Vision-reference static XYZ exception: EV_CTRL 2->3->2, never arm/reboot."""
import struct
from pymavlink.dialects.v20 import common as m
import usb_height_fusion_policy as height
from usb_vision_policy import send_allowed as receive_allowed

READ_PARAMS = tuple(dict.fromkeys(height.READ_PARAMS + ('EKF2_OF_CTRL', 'COM_RC_IN_MODE', 'COM_OBL_RC_ACT')))
COMMANDS = height.COMMANDS


def fusion_scope_ok(params):
    expected = {'EKF2_HGT_REF': 3, 'EKF2_RNG_CTRL': 0, 'EKF2_BARO_CTRL': 1,
                'EKF2_OF_CTRL': 0, 'EKF2_AID_MASK': 0, 'EKF2_EV_QMIN': 0,
                'COM_RC_IN_MODE': 4, 'COM_OBL_RC_ACT': 4,
                'EKF2_EV_POS_X': 0, 'EKF2_EV_POS_Y': 0, 'EKF2_EV_POS_Z': 0,
                'EKF2_IMU_POS_X': 0, 'EKF2_IMU_POS_Y': 0, 'EKF2_IMU_POS_Z': 0}
    return params.get('EKF2_EV_CTRL') in (2, 3) and all(params.get(k) == v for k, v in expected.items())


def control_message(enc, value):
    if type(value) is not int or value not in (2, 3):
        raise ValueError('Vision reference permits only XYZ=3 and baseline restore=2, never zero')
    wire = struct.unpack('<f', struct.pack('<i', value))[0]
    return enc.param_set_encode(1, 1, b'EKF2_EV_CTRL', wire, m.MAV_PARAM_TYPE_INT32)


def allowed(msg, *, authorized=False, safe=False, restore_safe=False, backup_verified=False,
            clock_verified=False, position_authorized=False):
    kind = msg.get_type()
    if kind == 'PARAM_SET':
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        value = struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
        return bool(authorized and backup_verified and name == 'EKF2_EV_CTRL'
                    and msg.param_type == m.MAV_PARAM_TYPE_INT32
                    and (msg.target_system, msg.target_component) == (1, 1)
                    and ((value == 2 and restore_safe) or (value == 3 and safe and clock_verified)))
    if kind == 'PARAM_REQUEST_READ':
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        return name in READ_PARAMS and (msg.target_system, msg.target_component, msg.param_index) == (1, 1, -1)
    if kind == 'SERIAL_CONTROL' and (msg.device, msg.flags, msg.timeout, msg.baudrate) == (10, 2, 0, 0):
        if any(msg.count == len(p) and bytes(msg.data) == p.ljust(70, b'\0') for p in COMMANDS.values()):
            return safe
    return receive_allowed(msg, bench_authorized=authorized and position_authorized, safe=safe)


fusion_shell_message = height.fusion_shell_message
parse_selector = height.parse_selector
parse_local_position = height.parse_local_position
parse_flags = height.parse_flags
parse_aid = height.parse_aid
parse_height_aid = height.parse_height_aid
fusion_evidence_ok = height.fusion_evidence_ok
