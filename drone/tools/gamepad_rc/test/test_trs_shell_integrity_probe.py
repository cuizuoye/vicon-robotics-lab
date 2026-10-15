from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trs_shell_integrity_probe import COMMANDS, allowed, sequence_observation, shell_fragment


def test_fixed_reads_only():
    enc = m.MAVLink(None)
    for payload in (b'', *COMMANDS.values()):
        msg = enc.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0'))
        assert allowed(msg, safe=True)
        assert not allowed(msg)
    assert not allowed(enc.serial_control_encode(10, 0, 0, 0, 0, bytes(70)), safe=True)
    assert not allowed(enc.param_set_encode(1, 1, b'EKF2_EV_CTRL', 11, 6), safe=True)
    payload = b'mavlink stop-all\n'
    assert not allowed(enc.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0')), safe=True)


@pytest.mark.parametrize('command', [400, 176, 21, 22, 185, 241, 246, 511])
def test_mutating_commands_never_allowed(command):
    msg = m.MAVLink(None).command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0)
    assert not allowed(msg, safe=True)


def test_modulo_sequence_gaps_not_mislabeled_radio_loss():
    assert sequence_observation(None, 8)['kind'] == 'first'
    assert sequence_observation(255, 0)['kind'] == 'contiguous'
    assert sequence_observation(8, 8)['kind'] == 'duplicate'
    assert sequence_observation(8, 10) == {'kind': 'gap_or_reordering', 'unseen_sequence_slots': 1}


def test_legacy_serial_control_roundtrip_without_target_extensions():
    enc = m.MAVLink(None, srcSystem=1, srcComponent=1)
    text = b'TOPIC: actuator_armed\n'
    original = enc.serial_control_encode(10, 1, 0, 0, len(text), text.ljust(70, b'\0'))
    message = m.MAVLink(None).parse_buffer(original.pack(enc))[0]
    assert not hasattr(message, 'target_system')
    row = shell_fragment(message, .7)
    assert row['text'] == text.decode()
    assert row['target'] == [None, None]
    assert not row['target_fields_available']


def test_optional_target_metadata_does_not_change_send_gate():
    enc = m.MAVLink(None)
    text = b'nsh> '
    message = enc.serial_control_encode(10, 1, 0, 0, len(text), text.ljust(70, b'\0'))
    message.target_system, message.target_component = 250, 198
    row = shell_fragment(message, 1.)
    assert row['target_fields_available'] and row['target'] == [250, 198]
    assert not allowed(message, safe=True)  # Replies are not outbound commands.
