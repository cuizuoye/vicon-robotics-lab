"""独立LOCAL_FRD联合XYZ/航向台架边界：仅EV_CTRL 0/11，不启用速度或飞行。"""

import struct
from pymavlink.dialects.v20 import common as m
import usb_yaw_fusion_policy as yaw
import usb_height_fusion_policy as height
import control_readonly_policy as control
from usb_vision_policy import send_allowed as receive_allowed

READ_PARAMS=control.PARAMS
COMMANDS={**height.COMMANDS, **yaw.COMMANDS, 'failsafe':control.COMMANDS['failsafe']}
for i in range(9): COMMANDS[f'status_{i}']=control.COMMANDS[f'status_{i}']


def fusion_scope_ok(params):
    return params.get('EKF2_EV_CTRL') in (0,11) and height.fusion_scope_ok({**params,'EKF2_EV_CTRL':0})


def control_message(encoder,value):
    if type(value) is not int or value not in (0,11):
        raise ValueError('Only combined XYZ/yaw enable=11 or restore=0 permitted')
    wire=struct.unpack('<f',struct.pack('<i',value))[0]
    return encoder.param_set_encode(1,1,b'EKF2_EV_CTRL',wire,m.MAV_PARAM_TYPE_INT32)


def fusion_shell_message(encoder,topic):
    payload=COMMANDS[topic]
    return encoder.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0'))


def allowed(message, *, authorized=False,safe=False,restore_safe=False,
            backup_verified=False,clock_verified=False,position_authorized=False):
    kind=message.get_type()
    name=getattr(message,'param_id',None)
    if isinstance(name,bytes): name=name.decode('ascii').rstrip('\0')
    if kind=='PARAM_SET':
        if not (authorized and backup_verified and name=='EKF2_EV_CTRL'
                and message.param_type==m.MAV_PARAM_TYPE_INT32
                and (message.target_system,message.target_component)==(1,1)): return False
        value=struct.unpack('<i',struct.pack('<f',message.param_value))[0]
        return (value==0 and restore_safe) or (value==11 and safe and clock_verified)
    if kind=='PARAM_REQUEST_READ' and name in READ_PARAMS:
        return (message.target_system,message.target_component,message.param_index)==(1,1,-1)
    if kind=='SERIAL_CONTROL' and (message.device,message.flags,message.timeout,message.baudrate)==(10,2,0,0):
        if any(message.count==len(p) and bytes(message.data)==p.ljust(70,b'\0') for p in COMMANDS.values()): return safe
    return receive_allowed(message,bench_authorized=authorized and position_authorized,safe=safe)


parse_selector=yaw.parse_selector
parse_flags=yaw.parse_flags
parse_local_position=control.parse_local_position
parse_aid=yaw.parse_aid
parse_height_aid=height.parse_height_aid
parse_yaw_aid=yaw.parse_yaw_aid
parse_status=control.parse_status
parse_failsafe=control.parse_failsafe
inactive_other_heading_sources=yaw.inactive_other_heading_sources
compare_heading=yaw.compare_heading


def fusion_evidence_ok(rows,primary):
    if len(rows)<2: return False
    for row in rows:
        f,local,comparison=row['flags'],row['local'],row.get('yaw_comparison')
        if not (f and local and comparison and f['cs_tilt_align'] and f['cs_baro_hgt']
                and all(f[k] for k in ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw'))
                and inactive_other_heading_sources(f) and local['xy_valid'] and local['z_valid']
                and not any(f[k] for k in ('cs_ev_vel','cs_in_air','fs_bad_pos_n','fs_bad_pos_e',
                                           'fs_bad_pos_d','fs_bad_hdg','reject_hor_pos','reject_ver_pos','reject_yaw'))
                and comparison['fresh'] and abs(comparison['fc_minus_vicon_wrapped_deg'])<=5): return False
        for key in ('aid','height_aid','yaw_aid'):
            aid=row.get(key)
            if not aid: return False
            ratios=aid['test_ratio'] if isinstance(aid['test_ratio'],list) else [aid['test_ratio']]
            variances=aid['observation_variance'] if isinstance(aid['observation_variance'],list) else [aid['observation_variance']]
            if not (aid['estimator_instance']==primary and aid['fusion_enabled'] and aid['fused']
                    and not aid['innovation_rejected'] and aid['time_last_fuse']>0
                    and abs(local['timestamp']-aid['timestamp'])<2_000_000
                    and all(0<=v<=1 for v in ratios) and all(v>0 for v in variances)): return False
    # 成功融合与Commander/Offboard许可分开；False地理航向标志不伪改为True。
    return all(b[k]['time_last_fuse']>a[k]['time_last_fuse'] and b[k]['timestamp_sample']>a[k]['timestamp_sample']
               for a,b in zip(rows,rows[1:]) for k in ('aid','height_aid','yaw_aid'))
