from pathlib import Path
import sys
import math
import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import trs_manual_fusion_policy as p
import trs_static_fusion_policy as static


def gate(route='usb'):
    return dict(route=route,safe=True,authorized=True,backup_verified=True,
                locked_manual=True,receive_time_verified=True)


def test_independent_manual_envelope_leaves_static_gate_unchanged():
    args=([.25,0,0],[1,0,0,0],[0,0,0],[1,0,0,0])
    assert p.within_envelope(*args,'translation')
    assert not static.stationary(*args)
    assert not p.within_envelope(*args,'vertical')
    args=([0,0,-.2],[1,0,0,0],[0,0,0],[1,0,0,0])
    assert p.within_envelope(*args,'vertical')
    assert not p.within_envelope(*args,'translation')


@pytest.mark.parametrize('pos,axis', [([.61,0,0],'translation'),([.1,0,.06],'translation'),
    ([0,0,-.46],'vertical'),([.09,0,-.2],'vertical'),([0,0,0],'unknown'),
    ([float('nan'),0,0],'vertical')])
def test_out_of_scope_movement_rejected(pos,axis):
    assert not p.within_envelope(pos,[1,0,0,0],[0,0,0],[1,0,0,0],axis)


def test_large_yaw_and_tilt_not_authorized_by_manual_mode():
    for q in ([math.cos(.2),0,0,math.sin(.2)],[math.cos(.1),math.sin(.1),0,0]):
        assert not p.within_envelope([0,0,0],q,[0,0,0],[1,0,0,0],'translation')


def test_manual_only_two_parameters_and_no_motion_unlock_reboot():
    enc=m.MAVLink(None)
    for name,value in (('EKF2_EV_CTRL',11),('EKF2_EV_DELAY',65.)):
        msg=p.parameter_message(enc,name,value)
        assert p.allowed(msg,**gate())
        assert not p.allowed(msg,**{**gate(),'locked_manual':False})
        assert not p.allowed(msg,**gate('trs'))
    for command in (22,21,400,176,185,246,241,511):
        assert not p.allowed(enc.command_long_encode(1,1,command,0,1,0,0,0,0,0,0),**gate('trs'))


def test_cancel_restore_not_dependent_on_gamepad_or_ros():
    msg=p.parameter_message(m.MAVLink(None),'EKF2_EV_CTRL',0)
    assert p.allowed(msg,route='usb',authorized=True,backup_verified=True,restore_safe=True)
    assert not p.allowed(msg,route='usb',authorized=True,backup_verified=True)


def test_extended_read_window_usb_only_without_weakening_old_policy():
    enc=m.MAVLink(None)
    payload=p.MANUAL_SERIES_COMMAND
    msg=enc.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0'))
    assert p.allowed(msg,**gate())
    assert not p.allowed(msg,**gate('trs'))
    assert not static.allowed(msg,route='usb',safe=True)
    assert not p.allowed(msg,**{**gate(),'safe':False})
    assert p.visual_series('') is None


def test_full_48_fresh_samples_required_before_rate_assessment():
    def block(i):
        return ('TOPIC: vehicle_visual_odometry\n'
                f'timestamp: {1_000_000+i*31_250}\ntimestamp_sample: {999_998+i*31_250}\n'
                'pose_frame: 1\nvelocity_frame: 0\nreset_counter: 0\nquality: 0\n'
                'position: [0, 0, 0]\nq: [1, 0, 0, 0]\n')
    assert p.visual_series(''.join(block(i) for i in range(47))) is None
    series=p.visual_series(''.join(block(i) for i in range(48)))
    assert len(series)==48
    from compact_vision import series_timing
    assert series_timing(series)['rate_hz']==32
