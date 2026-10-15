from dataclasses import replace
from pathlib import Path
import sys
import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from disarmed_offboard_policy import GroundObservation, DisarmedHoldGate, POSITION_YAW_MASK
from receive_only_permit import UID


def observation(t=0):
    return GroundObservation(t,UID,'1.14.3',(.1,.2,-.05),.3,(1,1,1),0,
                             estimator_ready=True,gamepad_available=True)


def enc():return m.MAVLink(None,srcSystem=250,srcComponent=190)


def test_frozen_current_position_not_absolute_zero_or_upward_target():
    o=observation();g=DisarmedHoldGate(o,operator_authorized=True)
    msg=g.hold_message(enc(),123,o)
    assert (msg.x,msg.y,msg.z,msg.yaw)==(*o.position,o.yaw)
    assert msg.type_mask==POSITION_YAW_MASK
    assert g.permits(msg,o)
    msg.z-=.5
    assert not g.permits(msg,o)


@pytest.mark.parametrize('change',[{'armed':True},{'lockdown':False},{'pose_age_s':.151},
    {'status_age_s':.51},{'gamepad_available':False},{'home_latched':True},
    {'estimator_ready':False},{'uid2':'another'},{'firmware':'1.17.0'}])
def test_invalid_start_rejected(change):
    with pytest.raises(ValueError):DisarmedHoldGate(replace(observation(),**change),operator_authorized=True)


def test_reset_or_primary_change_latches_and_old_state_cannot_resume():
    o=observation();g=DisarmedHoldGate(o,operator_authorized=True)
    assert not g.valid(replace(o,t=1,reset_counters=(2,1,1)))
    assert not g.valid(replace(o,t=2))


def test_actual_send_history_required_and_single_mode_attempt():
    o=observation();g=DisarmedHoldGate(o,operator_authorized=True)
    with pytest.raises(ValueError):g.mode_message(enc(),o)
    for i in range(41):
        current=replace(o,t=i*.05)
        msg=g.hold_message(enc(),i*50,current)
        assert g.permits(msg,current)
        g.record_actual_hold_send(current.t)
    msg=g.mode_message(enc(),current)
    assert msg.command==176 and msg.param1==1 and msg.param2==6
    assert g.permits(msg,current)
    g.record_actual_mode_attempt()
    assert not g.permits(msg,current)


@pytest.mark.parametrize('command',[185,21,22,241,246,400,511])
def test_no_arm_unlock_takeoff_land_or_calibration_in_ground_check(command):
    o=observation();g=DisarmedHoldGate(o,operator_authorized=True)
    msg=enc().command_long_encode(1,1,command,0,1,0,0,0,0,0,0)
    assert not g.permits(msg,o)


def test_actual_stream_gap_latches_abort():
    o=observation();g=DisarmedHoldGate(o,operator_authorized=True)
    g.record_actual_hold_send(0)
    with pytest.raises(ValueError):g.record_actual_hold_send(.21)
    assert not g.valid(replace(o,t=.22))


def test_exact_auto_loiter_restore_is_not_a_takeoff_or_mission_command():
    from disarmed_offboard_policy import restorable_mode,restore_mode_allowed
    assert restorable_mode(4,3) and restorable_mode(4,6)
    assert not restorable_mode(4,2) and not restorable_mode(4,4)
    msg=enc().command_long_encode(1,1,176,0,1,4,3,0,0,0,0)
    assert restore_mode_allowed(msg,4,3,safe=True,locked=True)
    assert not restore_mode_allowed(msg,4,6,safe=True,locked=True)
    msg.param3=2
    assert not restore_mode_allowed(msg,4,2,safe=True,locked=True)
