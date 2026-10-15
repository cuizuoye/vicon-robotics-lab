from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from trs_vicon_receive_bench import allowed, odometry_message
from usb_vision_policy import PARAMS, shell_message


def test_real_pose_only_on_authorized_safe_trs_not_usb():
    enc=m.MAVLink(None)
    msg=odometry_message(enc,10000,[1,2,3],[1,0,0,0])
    assert allowed(msg,route='trs',safe=True,bench_confirmed=True)
    for route,safe,confirmed in [('usb',True,True),('trs',False,True),('trs',True,False),('other',True,True)]:
        assert not allowed(msg,route=route,safe=safe,bench_confirmed=confirmed)
    assert (msg.frame_id,msg.child_frame_id)==(m.MAV_FRAME_LOCAL_FRD,m.MAV_FRAME_BODY_FRD)


@pytest.mark.parametrize('route',['trs','usb'])
@pytest.mark.parametrize('command',[400,176,21,22,185,246,241,511])
def test_no_arming_mode_stop_reboot_calibration_or_rate_commands(route,command):
    enc=m.MAVLink(None)
    assert not allowed(enc.command_long_encode(1,1,command,0,1,0,0,0,0,0,0),
                       route=route,safe=True,bench_confirmed=True)


@pytest.mark.parametrize('route',['trs','usb'])
def test_no_parameter_write_rc_or_setpoint(route):
    enc=m.MAVLink(None)
    msgs=[enc.param_set_encode(1,1,b'EKF2_EV_CTRL',0,6),
          enc.rc_channels_override_encode(1,1,*([1500]*8)),
          enc.set_position_target_local_ned_encode(1,1,1,1,0,*([0.]*11))]
    assert all(not allowed(msg,route=route,safe=True,bench_confirmed=True) for msg in msgs)


def test_diagnostic_shell_never_uses_wireless_bandwidth_and_usb_timesync_is_blocked():
    enc=m.MAVLink(None)
    for key in ('actuator_armed','vehicle_visual_odometry'):
        msg=shell_message(enc,key)
        assert allowed(msg,route='usb',safe=True)
        assert not allowed(msg,route='trs',safe=True)
    assert not allowed(enc.timesync_encode(100,200),route='usb',safe=True)
    assert allowed(enc.timesync_encode(100,200),route='trs',safe=True)
    assert not allowed(enc.timesync_encode(0,200),route='trs',safe=True)


def test_usb_monitor_reads_only_named_parameters():
    enc=m.MAVLink(None)
    for name in PARAMS:
        assert allowed(enc.param_request_read_encode(1,1,name.encode(),-1),route='usb',safe=True)
    assert not allowed(enc.param_request_read_encode(1,1,b'MAV_0_MODE',-1),route='usb',safe=True)
