from pathlib import Path
import sys
import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import trs_runtime_monitor as p
from estop_only_policy import shell_message


@pytest.mark.parametrize('command,param1',[(400,1),(400,0),(176,1),(21,0),(22,0),(185,0),
    (185,1),(185,3),(246,1),(511,1)])
def test_all_flight_changes_unlock_and_rates_blocked(command,param1):
    msg=m.MAVLink(None).command_long_encode(1,1,command,0,param1,0,0,0,0,0,0)
    assert not p.allowed(msg,identity_fresh=True,home_latched=True)


def test_stop_requires_current_identity_and_home_latch():
    msg=m.MAVLink(None).command_long_encode(1,1,185,0,2,0,0,0,0,0,0)
    assert p.allowed(msg,identity_fresh=True,home_latched=True)
    assert not p.allowed(msg,identity_fresh=True)
    assert not p.allowed(msg,home_latched=True)


def test_fixed_reads_and_no_injection():
    e=m.MAVLink(None)
    assert p.allowed(shell_message(e),identity_fresh=True)
    assert not p.allowed(shell_message(e))
    for name in p.PARAMS:
        assert p.allowed(e.param_request_read_encode(1,1,name.encode(),-1),identity_fresh=True)
    assert not p.allowed(e.param_request_read_encode(1,1,b'UNRELATED',-1),identity_fresh=True)
    assert not p.allowed(e.param_set_encode(1,1,b'EKF2_EV_CTRL',11,6),identity_fresh=True,home_latched=True)
    assert not p.allowed(e.vision_position_estimate_encode(1,0,0,0,0,0,0),identity_fresh=True,home_latched=True)
    assert not p.allowed(e.set_position_target_local_ned_encode(1,1,1,1,2552,*([0]*11)),identity_fresh=True,home_latched=True)


def test_freshness_and_rate_are_observations_not_assumed():
    assert p.observation_age(1,None) is None
    assert p.observation_age(1,2) is None
    assert p.observation_age(1,.7)==pytest.approx(.3)
    assert p.observed_rate([]) is None
    assert p.observed_rate([0,1,2])==1
    assert p.observed_rate([0,.1,.2])==10


def test_physical_mode_mismatch_rejected(tmp_path,monkeypatch):
    monkeypatch.setattr(p,'FC_USB',tmp_path/'missing')
    p.check_physical('battery-only')
    with pytest.raises(RuntimeError):p.check_physical('usb-only')
    (tmp_path/'missing').touch()
    p.check_physical('usb-only')
    with pytest.raises(RuntimeError):p.check_physical('battery-only')


def test_recovered_heartbeat_or_source_does_not_clear_readiness_fault():
    args=dict(baseline_seen=True,joint_seen=True,identity_fresh=True,source_fresh=True)
    assert p.readiness_fault(None,**args) is None
    fault=p.readiness_fault(None,**{**args,'identity_fresh':False})
    assert fault=='fc_heartbeat_stale_after_ready'
    assert p.readiness_fault(fault,**args)==fault
    source=p.readiness_fault(None,**{**args,'source_fresh':False})
    assert source=='vicon_source_lost_after_ready'
    assert p.readiness_fault(source,**args)==source
    assert p.readiness_fault(None,baseline_seen=False,joint_seen=False,
                             identity_fresh=False,source_fresh=False) is None
