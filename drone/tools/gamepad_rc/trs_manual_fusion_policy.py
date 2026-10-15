"""独立断动力手动位移台架；静止台架与旧USB时钟门限保持不变。"""
import math
import re
import numpy as np
import trs_static_fusion_policy as static
from manual_translation import rotation, quaternion_angle

DELAY_MS = static.DELAY_MS
READ_PARAMS = static.READ_PARAMS
MANUAL_SERIES_COMMAND = b'listener vehicle_visual_odometry -n 48\n'
COMMANDS = {**static.COMMANDS, 'manual_visual_series': MANUAL_SERIES_COMMAND}
parameter_message = static.parameter_message
scope_ok = static.scope_ok
receive_time_mode = static.receive_time_mode


def visual_series(text):
    from usb_vision_policy import parse_visual_snapshot
    blocks=re.split(r'TOPIC:\s*vehicle_visual_odometry[^\n]*\n',text)[1:]
    rows=[parse_visual_snapshot('TOPIC: vehicle_visual_odometry\n'+block) for block in blocks]
    rows=[r for r in rows if r]
    return rows if len(rows)==48 else None


def within_envelope(p, q, origin_p, origin_q, axis):
    if axis not in ('translation', 'vertical'):
        return False
    try:
        a, b = np.asarray(p, dtype=float), np.asarray(origin_p, dtype=float)
        if a.shape != (3,) or b.shape != (3,) or not np.isfinite([*a,*b]).all():
            return False
        delta = a - b
        angle = quaternion_angle(q, origin_q)
        gravity = [0, 0, 1]
        tilt = math.degrees(math.acos(float(np.clip(
            (rotation(q).T @ gravity) @ (rotation(origin_q).T @ gravity), -1, 1))))
    except (ValueError, TypeError):
        return False
    if angle > 15 or tilt > 5:
        return False
    if axis == 'translation':
        return np.linalg.norm(delta[:2]) <= .6 and abs(delta[2]) <= .05
    return np.linalg.norm(delta[:2]) <= .08 and -.45 <= delta[2] <= .05


def allowed(message, *, route, safe=False, authorized=False, backup_verified=False,
            locked_manual=False, receive_time_verified=False, restore_safe=False):
    if message.get_type()=='SERIAL_CONTROL' and route=='usb':
        p=MANUAL_SERIES_COMMAND
        if ((message.device,message.flags,message.timeout,message.baudrate)==(10,2,0,0)
                and message.count==len(p) and bytes(message.data)==p.ljust(70,b'\0')):
            return bool(safe)
    # 重用同一两参数/固定诊断白名单；新例外仅在显式手动范围内，不能发控制指令。
    return static.allowed(message, route=route, safe=safe, authorized=authorized,
                          backup_verified=backup_verified, locked_static=locked_manual,
                          receive_time_verified=receive_time_verified, restore_safe=restore_safe)
