"""Strict Sign-only 4WGC v1 decoder. Legacy rectangle decoder is untouched.

156 bytes: original requested-target fields, 32-byte first-fault snapshot,
then current effective float targets + granted directions + version/mask.
The older, never-flashed 132-byte offline draft is deliberately rejected.
"""
import math
import struct

from mecanum_vicon_control.bench_link_status import STATUS
from mecanum_vicon_control.motion_link_status import decode_motion_status as decode_legacy_status
from mecanum_vicon_control.rectangle_run_guard import disabled_motion_status_ready as legacy_ready
from mecanum_vicon_control.transport_protocol import ProtocolError

TAG = 0x43475734
SNAPSHOT = struct.Struct('<IIiiiiHHhH')
AUTHORITY = struct.Struct('<4f4bBBH')
SIZE = STATUS.size + SNAPSHOT.size + AUTHORITY.size
MOTION_TAGS = frozenset(('4WGB', '4WGC'))
MOTION_LINEAR_LIMITS = dict.fromkeys(MOTION_TAGS, .15)
MOTION_WIRE_TIMEOUTS = dict.fromkeys(MOTION_TAGS, 600)


def decode_motion_status(payload):
    if len(payload) == STATUS.size:
        status = decode_legacy_status(payload)
        if status['tag'] != '4WGB':
            raise ProtocolError('Sign adapter accepts legacy 4WGB only')
        return status
    if len(payload) != SIZE:
        raise ProtocolError('4WGC requires exactly 156-byte v1 diagnostics')
    v = STATUS.unpack_from(payload)
    if v[0] != TAG or not 1 <= v[3] <= v[4] <= 30 or v[5] > 1000:
        raise ProtocolError('wrong 4WGC identity or sampling deadline')
    cc, duty = list(v[8:16]), list(v[16:20])
    target, filtered, body = list(v[20:24]), list(v[24:28]), list(v[28:31])
    mask, wheel, watchdog, permit = v[31:35]
    a = AUTHORITY.unpack_from(payload, STATUS.size+SNAPSHOT.size)
    effective, directions = list(a[:4]), list(a[4:8])
    reversing, version, reserved = a[8:]
    if (mask > 15 or wheel > 4 or watchdog or permit not in (0, 1) or v[7] > 500
            or reversing > 15 or version != 1 or reserved):
        raise ProtocolError('invalid 4WGC safety state or authority version')
    if (any(abs(b) > limit for b, limit in zip(body, (150, 150, 200)))
            or any(abs(t) > 1200 for t in target)
            or any(abs(f) > 10000 for f in filtered)
            or any(not math.isfinite(e) or abs(e) > 1200 for e in effective)
            or any(g not in (-1, 0, 1) for g in directions)):
        raise ProtocolError('4WGC velocity or output authority outside envelope')
    if not permit and any(cc+duty+target+filtered+body+effective+directions+[mask, reversing]):
        raise ProtocolError('4WGC output/authority without permit')
    for i, (d, e, g, motor_sign) in enumerate(zip(duty, effective, directions, (1, -1, 1, -1))):
        if abs(d) > 350 or d % 10:
            raise ProtocolError('invalid 4WGC bounded PWM limit/resolution')
        expected = ([100, 100-d//10] if d > 0 else
                    [100-abs(d)//10, 100] if d < 0 else [0, 0])
        if cc[2*i:2*i+2] != expected:
            raise ProtocolError('4WGC actual CCR mapping mismatch')
        if (e and (1 if e > 0 else -1) != g) or (d and (not e or d*e*motor_sign <= 0)):
            raise ProtocolError('4WGC PWM differs from effective granted direction')
        if e*target[i] < 0 and not reversing & (1 << i):
            raise ProtocolError('4WGC opposite effective target without reversal authority')
        if not target[i] and (e or d):
            raise ProtocolError('4WGC output despite zero requested target')
        if mask & (1 << i) and (not d or abs(d) > 300 or reversing & (1 << i)):
            raise ProtocolError('4WGC invalid startup output')
    f = SNAPSHOT.unpack_from(payload, STATUS.size)
    if (f[6] not in range(7) or (f[6] == 0 and any(f))
            or (f[6] and not 1 <= f[7] <= 4) or abs(f[8]) > 350 or f[8] % 10):
        raise ProtocolError('invalid 4WGC first-feedback-fault snapshot')
    snapshot = dict(zip(('ms', 'dt_ms', 'raw_delta', 'measured_milli_cps',
                        'requested_milli_cps', 'effective_milli_cps', 'reason',
                        'wheel', 'duty_permille', 'opposite_counts'), f))
    return dict(tag='4WGC', ticks=v[1], updates=v[2], dt_min=v[3], dt_max=v[4], irq_us_max=v[5],
        started_ms=v[6], stationary_ms=v[7], compares=cc, raw_duty=duty, target_cps=target,
        filtered_cps=filtered, body_milli=body, startup_mask=mask, fault_wheel=wheel,
        watchdog_reset=watchdog, permitted=permit, first_feedback_fault=snapshot,
        effective_cps=effective, granted_direction=directions, reversing_mask=reversing,
        output_authority_version=version)


def disabled_motion_status_ready(status):
    if not isinstance(status, dict) or status.get('tag') != '4WGC':
        return legacy_ready(status)
    try:
        return (status['permitted'] == 0 and status['stationary_ms'] >= 500
            and status['output_authority_version'] == 1 and status['reversing_mask'] == 0
            and all(not any(status[key]) for key in ('compares', 'raw_duty', 'target_cps',
                'body_milli', 'effective_cps', 'granted_direction')))
    except (KeyError, TypeError, ValueError):
        return False
