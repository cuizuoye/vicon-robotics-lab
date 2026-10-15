from pathlib import Path
import sys
import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import trs_battery_readiness as p
from estop_only_policy import shell_message


@pytest.mark.parametrize('command,param1', [(400, 1), (400, 0), (176, 1), (21, 0),
                                          (22, 0), (185, 0), (185, 2), (246, 1)])
def test_all_control_including_stop_changes_blocked(command, param1):
    msg = m.MAVLink(None).command_long_encode(1, 1, command, 0, param1, 0, 0, 0, 0, 0, 0)
    assert not p.allowed(msg, identity_fresh=True)


def test_no_parameter_write_pose_or_targets():
    enc = m.MAVLink(None)
    assert not p.allowed(enc.param_set_encode(1, 1, b'EKF2_EV_CTRL', 11, 6), identity_fresh=True)
    assert not p.allowed(enc.vision_position_estimate_encode(1000, 0, 0, 0, 0, 0, 0), identity_fresh=True)
    assert not p.allowed(enc.set_position_target_local_ned_encode(1, 1, 1, 1, 2552,
                        *([0] * 11)), identity_fresh=True)


def test_fixed_reads_and_shell_require_identity():
    enc = m.MAVLink(None)
    for name in p.PARAMS:
        msg = enc.param_request_read_encode(1, 1, name.encode(), -1)
        assert p.allowed(msg, identity_fresh=True)
        assert not p.allowed(msg)
    assert not p.allowed(enc.param_request_read_encode(1, 1, b'OTHER', -1), identity_fresh=True)
    assert not p.allowed(enc.param_request_read_encode(2, 1, b'EKF2_EV_CTRL', -1), identity_fresh=True)
    assert p.allowed(shell_message(enc), identity_fresh=True)
    assert p.allowed(shell_message(enc, poll=True), identity_fresh=True)
    assert not p.allowed(shell_message(enc))
    arbitrary = b'commander arm\n'
    assert not p.allowed(enc.serial_control_encode(10, 2, 0, 0, len(arbitrary),
                         arbitrary.ljust(70, b'\0')), identity_fresh=True)


def test_maintenance_configuration_must_remain_exact():
    assert p.parameters_match(p.PARAMS)
    assert not p.parameters_match({})
    for name in p.PARAMS:
        assert not p.parameters_match({**p.PARAMS, name: 100})


def test_live_lock_required_not_just_old_report_or_disarmed():
    state = dict(timestamp_us=1_000_000, armed=False, lockdown=True,
                 force_failsafe=False, manual_lockdown=False)
    assert p.locked_fresh(state, 1100)
    assert not p.locked_fresh({**state, 'lockdown': False}, 1100)
    assert not p.locked_fresh({**state, 'armed': True}, 1100)
    assert not p.locked_fresh(state, 5000)
    assert not p.locked_fresh(None, 1100)
