from pathlib import Path
import sys
import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from trs_fusion_restore_only import allowed
from trs_static_fusion_policy import parameter_message


@pytest.mark.parametrize('name',['EKF2_EV_CTRL','EKF2_EV_DELAY'])
def test_restore_zero_only_and_same_identity_gate(name):
    msg=parameter_message(m.MAVLink(None),name,0)
    assert allowed(msg,safe=True,backup_verified=True)
    assert not allowed(msg,safe=False,backup_verified=True)
    assert not allowed(msg,safe=True,backup_verified=False)
    msg.target_system=2
    assert not allowed(msg,safe=True,backup_verified=True)


def test_no_enable_modes_reboot_lock_commands():
    enc=m.MAVLink(None)
    for name,value in [('EKF2_EV_CTRL',11),('EKF2_EV_DELAY',65.)]:
        assert not allowed(parameter_message(enc,name,value),safe=True,backup_verified=True)
    for c in (400,176,22,21,185,241,246,511):
        assert not allowed(enc.command_long_encode(1,1,c,0,1,0,0,0,0,0,0),safe=True,backup_verified=True)
