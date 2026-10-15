from pathlib import Path
import sys
import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import trs_only_estop_bench as p
from estop_only_policy import shell_message


@pytest.mark.parametrize('command', [400, 176, 21, 22, 241, 246])
def test_arm_modes_motion_calibration_reboot_blocked(command):
    enc = m.MAVLink(None)
    msg = enc.command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0)
    assert not p.allowed(msg, safe=True, authorized=True, latched=True)


@pytest.mark.parametrize('param1', [0, 1, 1.5, 3, float('nan')])
def test_unlock_real_termination_other_stop_values_blocked(param1):
    msg = m.MAVLink(None).command_long_encode(1, 1, 185, 0, param1, 0, 0, 0, 0, 0, 0)
    assert not p.allowed(msg, safe=True, authorized=True, latched=True)


def test_stop_requires_all_gates_and_fixed_shell():
    enc = m.MAVLink(None)
    msg = enc.command_long_encode(1, 1, 185, 0, 2, 0, 0, 0, 0, 0, 0)
    assert p.allowed(msg, safe=True, authorized=True, latched=True)
    for gate in ('safe', 'authorized', 'latched'):
        kw = dict(safe=True, authorized=True, latched=True)
        kw[gate] = False
        assert not p.allowed(msg, **kw)
    assert p.allowed(shell_message(enc), safe=True, authorized=True)
    assert not p.allowed(shell_message(enc), authorized=True)
    assert not p.allowed(enc.param_set_encode(1, 1, b'COM_ARM_HFLT_CHK', 0, 6), safe=True, authorized=True)
    assert not p.allowed(enc.rc_channels_override_encode(1, 1, *([1500]*8)), safe=True, authorized=True)
    assert not p.allowed(enc.serial_control_encode(10, 0, 0, 0, 0, bytes(70)), safe=True, authorized=True)


def test_state_requires_fresh_disarmed_nontermination_snapshot_but_keeps_prior_lock():
    state = dict(timestamp_us=1_000_000, armed=False, lockdown=False, force_failsafe=False, manual_lockdown=False)
    assert p.state_valid(state, 1100)
    assert p.state_valid(dict(state, lockdown=True), 1100)
    assert not p.state_valid(state, None)
    assert not p.state_valid(state, 5000)
    for field in ('armed', 'force_failsafe', 'manual_lockdown'):
        assert not p.state_valid(dict(state, **{field: True}), 1100)


def test_conflicting_physical_modes_rejected_before_opening_devices(tmp_path):
    import subprocess
    tool = Path(__file__).resolve().parents[1] / 'trs_only_estop_bench.py'
    result = subprocess.run([sys.executable, str(tool), '--trs-port', '/not-a-device',
        '--output', str(tmp_path/'result.json'), '--latch-file', str(tmp_path/'latch.json'),
        '--props-off-battery-confirmed', '--guarded-props-installed-stop-only-confirmed'],
        capture_output=True, text=True, timeout=5)
    assert result.returncode == 2 and 'not allowed with argument' in result.stderr
    assert not (tmp_path/'result.json').exists()
    assert not (tmp_path/'latch.json').exists()


def test_no_physical_confirmation_rejected_before_devices(tmp_path):
    import subprocess
    tool = Path(__file__).resolve().parents[1] / 'trs_only_estop_bench.py'
    result = subprocess.run([sys.executable, str(tool), '--trs-port', '/not-a-device',
        '--output', str(tmp_path/'result.json'), '--latch-file', str(tmp_path/'latch.json')],
        capture_output=True, text=True, timeout=5)
    assert result.returncode == 2 and 'required' in result.stderr
    assert not (tmp_path/'result.json').exists()
