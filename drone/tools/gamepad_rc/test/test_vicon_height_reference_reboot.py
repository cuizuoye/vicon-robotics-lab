from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vicon_height_reference_reboot import reboot_allowed


def message(command=246):
    return m.MAVLink(None).command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0)


GATES = dict(identity_safe=True, locked=True, endpoint_verified=True, backup_verified=True)


def test_requires_all_preconditions_and_single_attempt():
    assert reboot_allowed(message(), **GATES)
    for key in GATES:
        assert not reboot_allowed(message(), **{**GATES, key:False})
    assert not reboot_allowed(message(), **GATES, already_attempted=True)


@pytest.mark.parametrize('field,value', [('param1', 2), ('param2', 1), ('param3', 1),
    ('param4', 1), ('param5', 1), ('param6', 1), ('param7', 1),
    ('target_system', 2), ('target_component', 0), ('confirmation', 1)])
def test_rejects_other_reboot_modes_or_targets(field, value):
    msg = message()
    setattr(msg, field, value)
    assert not reboot_allowed(msg, **GATES)


@pytest.mark.parametrize('command', [176, 185, 241, 400, 511, 512])
def test_no_other_commands_through_reboot_gate(command):
    assert not reboot_allowed(message(command), **GATES)
