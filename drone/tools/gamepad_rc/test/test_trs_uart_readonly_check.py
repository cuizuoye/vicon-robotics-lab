import sys
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trs_uart_readonly_check import PARAMS, STATUS, allowed


def test_fixed_serial_parameter_reads_require_gate():
    enc = m.MAVLink(None)
    for name in PARAMS:
        msg = enc.param_request_read_encode(1, 1, name.encode(), -1)
        assert allowed(msg, safe=True)
        assert not allowed(msg)
    assert not allowed(enc.param_request_read_encode(1, 1, b'EKF2_EV_CTRL', -1), safe=True)


@pytest.mark.parametrize('command', [400, 176, 21, 22, 185, 246, 241, 511])
def test_mutating_commands_blocked(command):
    enc = m.MAVLink(None)
    assert not allowed(enc.command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0), safe=True)


def test_no_writes_or_arbitrary_shell():
    enc = m.MAVLink(None)
    assert not allowed(enc.param_set_encode(1, 1, b'SER_TEL1_BAUD', 115200, 6), safe=True)
    for payload in (b'', STATUS):
        msg = enc.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0'))
        assert allowed(msg, safe=True)
        assert not allowed(msg)
    payload = b'mavlink stop-all\n'
    assert not allowed(enc.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0')), safe=True)
    assert not allowed(enc.serial_control_encode(10, 0, 0, 0, 0, bytes(70)))
