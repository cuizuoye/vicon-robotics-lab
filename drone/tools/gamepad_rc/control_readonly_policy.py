"""位置控制前置状态只读审查；不改坐标标签、不写参数或发送目标。"""

from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import readonly_message, SAFETY_PARAMETERS
from usb_fusion_policy import parse_topic
from usb_vision_policy import shell_allowed
import usb_yaw_fusion_policy as yaw

PARAMS = tuple(dict.fromkeys(yaw.READ_PARAMS + SAFETY_PARAMETERS + ('COM_ARM_HFLT_CHK',)))
COMMANDS = {'actuator': b'listener actuator_armed -n 1\n',
            'selector': b'listener estimator_selector_status -n 1\n',
            'local': b'listener vehicle_local_position -n 1\n',
            'failsafe': b'listener failsafe_flags -n 1\n'}
for i in range(9):
    COMMANDS[f'flags_{i}']=f'listener estimator_status_flags -i {i} -n 1\n'.encode()
    COMMANDS[f'status_{i}']=f'listener estimator_status -i {i} -n 1\n'.encode()

STATUS_FLAGS=('pre_flt_fail_innov_heading','pre_flt_fail_innov_vel_horiz',
              'pre_flt_fail_innov_vel_vert','pre_flt_fail_innov_height','pre_flt_fail_mag_field_disturbed')
FAILSAFE_FLAGS=('attitude_invalid','angular_velocity_invalid','local_position_invalid',
                'local_velocity_invalid','local_altitude_invalid','offboard_control_signal_lost',
                'manual_control_signal_lost','gcs_connection_lost','home_position_invalid',
                'global_position_invalid')


def shell_message(encoder,key):
    payload=COMMANDS[key]
    return encoder.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0'))


def allowed(message, *, safe=False):
    kind=message.get_type()
    if kind in ('HEARTBEAT','COMMAND_LONG'): return readonly_message(message)
    if kind=='PARAM_REQUEST_READ':
        name=message.param_id
        if isinstance(name,bytes): name=name.decode('ascii').rstrip('\0')
        return name in PARAMS and (message.target_system,message.target_component,message.param_index)==(1,1,-1)
    if kind=='SERIAL_CONTROL':
        if shell_allowed(message) and message.count==0: return message.flags==0 or safe
        return bool(safe and (message.device,message.flags,message.timeout,message.baudrate)==(10,2,0,0)
                    and any(message.count==len(p) and bytes(message.data)==p.ljust(70,b'\0') for p in COMMANDS.values()))
    return False


parse_flags=yaw.parse_flags


def parse_local_position(text):
    base=yaw.parse_local_position(text)
    extra=parse_topic(text,'vehicle_local_position',bool_fields=('v_xy_valid','v_z_valid'),
                      float_fields=('vx','vy','vz','eph','epv','evh','evv'))
    return {**base,**extra} if base and extra else None


def parse_status(text):
    return parse_topic(text,'estimator_status',('timestamp','timestamp_sample'),STATUS_FLAGS)


def parse_failsafe(text):
    return parse_topic(text,'failsafe_flags',('timestamp',),FAILSAFE_FLAGS)


def assess(rows, primary):
    complete=len(rows)>=3 and all(r.get('status') and r.get('failsafe') and r.get('local') and r.get('flags') for r in rows)
    fresh=bool(complete and all(abs(r['local']['timestamp']-r[k]['timestamp'])<2_000_000
                               for r in rows for k in ('flags','status','failsafe'))
               and all(b[k]['timestamp']>a[k]['timestamp'] for a,b in zip(rows,rows[1:])
                       for k in ('local','status','failsafe')))
    stable=bool(fresh and all(r['selector']['primary_instance']==primary for r in rows))
    position=bool(stable and all(r['local']['xy_valid'] and r['local']['z_valid'] for r in rows))
    velocity=bool(stable and all(r['local']['v_xy_valid'] and r['local']['v_z_valid'] for r in rows))
    innovations=bool(stable and all(not any(r['status'][k] for k in STATUS_FLAGS) for r in rows))
    valid=bool(stable and all(not any(r['failsafe'][k] for k in
                  ('attitude_invalid','angular_velocity_invalid','local_position_invalid',
                   'local_velocity_invalid','local_altitude_invalid')) for r in rows))
    return {'complete_fresh_three_snapshot_audit':stable,
            'position_valid_observed':position,'velocity_valid_observed':velocity,
            'preflight_estimator_innovations_clear':innovations,
            'commander_estimator_invalid_flags_clear':valid,
            'heading_good_for_control_observed':bool(stable and all(r['local']['heading_good_for_control'] for r in rows)),
            'heading_flag_is_not_the_only_position_control_gate':True,
            'estimator_prerequisites_observed':position and velocity and innovations and valid,
            'offboard_signal_missing_expected_without_setpoints':bool(stable and all(r['failsafe']['offboard_control_signal_lost'] for r in rows)),
            'no_mode_entry_or_arming_checked':True,'independent_emergency_channel_verified':False,
            'flight_ready':False}
