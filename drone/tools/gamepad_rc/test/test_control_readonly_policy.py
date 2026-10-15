import copy
from pathlib import Path
import subprocess
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import control_readonly_policy as p
from usb_vision_policy import shell_message as empty_shell
from uav_coord_bridge.mavlink_vision import odometry_message


def enc(): return m.MAVLink(None,srcSystem=250,srcComponent=190)


@pytest.mark.parametrize('command',[400,176,185,241,246,511])
def test_no_arm_mode_flight_or_rate_commands(command):
    assert not p.allowed(enc().command_long_encode(1,1,command,0,1,0,0,0,0,0,0),safe=True)


def test_no_writes_or_local_frd_ned_targets():
    assert not p.allowed(enc().param_set_encode(1,1,b'EKF2_EV_CTRL',0,m.MAV_PARAM_TYPE_INT32),safe=True)
    assert not p.allowed(odometry_message(enc(),123,[0,0,0],[1,0,0,0]),safe=True)
    assert p.allowed(enc().param_request_read_encode(1,1,b'EKF2_GPS_CTRL',-1))
    assert not p.allowed(enc().param_request_read_encode(2,1,b'EKF2_GPS_CTRL',-1))
    assert not p.allowed(enc().param_request_read_encode(1,1,b'UNKNOWN',-1))


def test_historical_fault_protection_can_be_read_but_never_disabled():
    assert p.allowed(enc().param_request_read_encode(1,1,b'COM_ARM_HFLT_CHK',-1))
    assert not p.allowed(enc().param_set_encode(1,1,b'COM_ARM_HFLT_CHK',0,m.MAV_PARAM_TYPE_INT32),safe=True)


def test_only_fixed_listeners_and_unconditional_close():
    assert p.allowed(p.shell_message(enc(),'status_1'),safe=True)
    assert not p.allowed(p.shell_message(enc(),'status_1'))
    assert not p.allowed(empty_shell(enc(),close=True))
    with pytest.raises(KeyError): p.shell_message(enc(),'status_9')
    data=b'param set EKF2_GPS_CTRL 0\n'
    assert not p.allowed(enc().serial_control_encode(10,2,0,0,len(data),data.ljust(70,b'\0')),safe=True)


def rows():
    output=[]
    for stamp in (1_000_000,2_000_000,3_000_000):
        output.append({'selector':{'primary_instance':1},'flags':{'timestamp':stamp},
                       'local':{'timestamp':stamp,'xy_valid':True,'z_valid':True,
                                'v_xy_valid':True,'v_z_valid':True,'heading_good_for_control':False},
                       'status':{'timestamp':stamp,**dict.fromkeys(p.STATUS_FLAGS,False)},
                       'failsafe':{'timestamp':stamp,**dict.fromkeys(p.FAILSAFE_FLAGS,False)}})
        output[-1]['failsafe']['offboard_control_signal_lost']=True
    return output


def test_false_heading_flag_alone_is_not_false_position_gate_or_flight_permission():
    result=p.assess(rows(),1)
    assert result['estimator_prerequisites_observed']
    assert not result['heading_good_for_control_observed'] and not result['flight_ready']
    assert result['offboard_signal_missing_expected_without_setpoints']
    assert not result['independent_emergency_channel_verified']


@pytest.mark.parametrize('case',['stale','cached','primary','xy','velocity','heading_unstable','invalid_attitude','missing'])
def test_multiple_prerequisites_and_freshness_not_just_heading(case):
    data=copy.deepcopy(rows())
    if case=='stale': data[0]['failsafe']['timestamp']=4_000_000
    if case=='cached': data[1]['status']['timestamp']=data[0]['status']['timestamp']
    if case=='primary': data[0]['selector']['primary_instance']=0
    if case=='xy': data[0]['local']['xy_valid']=False
    if case=='velocity': data[0]['local']['v_xy_valid']=False
    if case=='heading_unstable': data[0]['status']['pre_flt_fail_innov_heading']=True
    if case=='invalid_attitude': data[0]['failsafe']['attitude_invalid']=True
    if case=='missing': data[0]['status']=None
    assert not p.assess(data,1)['estimator_prerequisites_observed']


def test_parse_status_requires_exact_complete_boolean_fields():
    text='TOPIC: estimator_status instance 1\n timestamp: 100\n timestamp_sample: 90\n'
    text+=''.join(f' {key}: False\n' for key in p.STATUS_FLAGS)
    assert p.parse_status(text)['pre_flt_fail_innov_heading'] is False
    assert p.parse_status(text.replace('pre_flt_fail_innov_heading: False','pre_flt_fail_innov_heading: unknown')) is None


def test_readonly_modes_mutually_exclusive_before_serial(tmp_path):
    tool=Path(__file__).resolve().parents[1]/'height_readonly_check.py'
    result=subprocess.run([sys.executable,str(tool),'--port','/dev/doesnotexist','--uid2','test',
                           '--output',str(tmp_path/'new.json'),'--control-readiness','--compare-ekfs'],capture_output=True,text=True)
    assert result.returncode==2 and 'Select one read-only audit' in result.stderr
    assert not (tmp_path/'new.json').exists()
