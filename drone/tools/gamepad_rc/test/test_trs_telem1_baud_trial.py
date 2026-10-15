import sys
from pathlib import Path
import struct

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trs_telem1_baud_trial import allowed, write_message


def gates():
    return dict(safe=True, authorized=True, backup_verified=True, readback_verified=True)


@pytest.mark.parametrize('value', [57600, 115200])
def test_only_two_exact_integer_baud_values(value):
    msg = write_message(m.MAVLink(None), value)
    assert struct.unpack('<i', struct.pack('<f', msg.param_value))[0] == value
    assert allowed(msg, **gates())
    for key in ('safe', 'authorized', 'backup_verified'):
        gate = gates()
        gate[key] = False
        assert not allowed(msg, **gate)


@pytest.mark.parametrize('value', [9600, 921600, 0, 115200.0, True])
def test_other_baud_values_rejected(value):
    with pytest.raises(ValueError):
        write_message(m.MAVLink(None), value)


def test_wrong_parameter_target_and_type_rejected():
    enc = m.MAVLink(None)
    for name in (b'EKF2_EV_CTRL', b'SER_TEL2_BAUD'):
        assert not allowed(enc.param_set_encode(1, 1, name, 0, 6), **gates())
    msg = write_message(enc, 115200)
    msg.target_system = 2
    assert not allowed(msg, **gates())
    assert not allowed(enc.param_set_encode(1, 1, b'SER_TEL1_BAUD', 115200., 9), **gates())


@pytest.mark.parametrize('command', [400, 176, 21, 22, 185, 241, 511])
def test_control_and_stream_commands_blocked(command):
    assert not allowed(m.MAVLink(None).command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0), **gates())


def test_reboot_requires_verified_readback_and_no_extra_actions():
    enc = m.MAVLink(None)
    msg = enc.command_long_encode(1, 1, 246, 0, 1, 0, 0, 0, 0, 0, 0)
    assert allowed(msg, **gates())
    for key in gates():
        gate = gates()
        gate[key] = False
        assert not allowed(msg, **gate)
    msg.param2 = 1
    assert not allowed(msg, **gates())
    msg.param2, msg.param1 = 0, 3
    assert not allowed(msg, **gates())
