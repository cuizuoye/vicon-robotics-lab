"""停止专用发送边界、持久锁存与内部状态解析；不连接实机。"""

import io
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from estop_only_policy import (  # noqa: E402
    DurableStopLatch, LISTENER, LOCK_COMMAND, lock_message,
    parse_actuator_snapshot, send_gated, shell_message, shell_message_allowed,
)
from gamepad_estop_monitor import HOME  # noqa: E402

UID = '000000000000000000000000000000000000'
IDENTITY = {'uid2': UID, 'firmware': '1.14.3'}


def encoder():
    return m.MAVLink(None, srcSystem=250, srcComponent=190)


def latch(tmp_path, active=True):
    item = DurableStopLatch(tmp_path / 'stop.json', UID)
    if active:
        item.event(1, HOME, 1)
    return item


def test_home_release_and_restart_never_clear_latch(tmp_path):
    item = latch(tmp_path)
    item.event(1, HOME, 0)
    assert item.latched and not item.home_pressed
    assert DurableStopLatch(item.path, UID).latched


def test_corrupted_latch_file_is_also_inhibited(tmp_path):
    item = latch(tmp_path)
    item.path.write_text('damaged', encoding='utf-8')
    restored = DurableStopLatch(item.path, UID)
    assert restored.restored and restored.latched


@pytest.mark.parametrize('field', ['authorized', 'disarmed', 'identity', 'latched', 'version', 'uid'])
def test_command_requires_all_gates(tmp_path, field):
    item = latch(tmp_path, active=field != 'latched')
    identity = dict(IDENTITY)
    if field == 'version':
        identity['firmware'] = '1.17.0'
    if field == 'uid':
        identity['uid2'] = 'another aircraft'
    with pytest.raises(ValueError):
        lock_message(encoder(), identity if field != 'identity' else None, UID, item,
                     field != 'authorized', field != 'disarmed')


@pytest.mark.parametrize('param1', [0, 1, 1.5, 3, float('nan')])
def test_unlock_and_actual_termination_are_blocked(tmp_path, param1):
    msg = encoder().command_long_encode(1, 1, LOCK_COMMAND, 0, param1, 0, 0, 0, 0, 0, 0)
    with pytest.raises(ValueError):
        send_gated(encoder(), io.BytesIO(), msg, route='trs', latch=latch(tmp_path), authorized=True)


@pytest.mark.parametrize('command', [400, 176, 21, 22, 241, 246])
def test_arm_mode_land_takeoff_calibration_reboot_are_blocked(tmp_path, command):
    msg = encoder().command_long_encode(1, 1, command, 0, 0, 0, 0, 0, 0, 0, 0)
    with pytest.raises(ValueError):
        send_gated(encoder(), io.BytesIO(), msg, route='trs', latch=latch(tmp_path), authorized=True)


def test_rc_and_parameter_write_are_blocked(tmp_path):
    item = latch(tmp_path)
    enc = encoder()
    for msg in (enc.rc_channels_override_encode(1, 1, *([1500] * 8)),
                enc.param_set_encode(1, 1, b'COM_RC_IN_MODE', 0, 6)):
        with pytest.raises(ValueError):
            send_gated(enc, io.BytesIO(), msg, route='trs', latch=item, authorized=True)


def test_motor_lock_only_on_trs_and_exact_v2_payload(tmp_path):
    enc, item = encoder(), latch(tmp_path)
    msg = lock_message(enc, IDENTITY, UID, item, True, True)
    with pytest.raises(ValueError):
        send_gated(enc, io.BytesIO(), msg, route='usb', latch=item, authorized=True)
    port = io.BytesIO()
    send_gated(enc, port, msg, route='trs', latch=item, authorized=True)
    assert port.getvalue()[0] == 0xfd
    decoded = m.MAVLink(None).parse_char(port.getvalue())
    assert decoded.command == 185 and decoded.param1 == 2


def test_missing_authorization_blocks_motor_lock(tmp_path):
    enc, item = encoder(), latch(tmp_path)
    msg = lock_message(enc, IDENTITY, UID, item, True, True)
    with pytest.raises(ValueError):
        send_gated(enc, io.BytesIO(), msg, route='trs', latch=item, authorized=False)


def test_shell_is_fixed_listener_on_usb_only(tmp_path):
    enc, item = encoder(), latch(tmp_path, active=False)
    for poll in (False, True):
        msg = shell_message(enc, poll=poll)
        assert shell_message_allowed(msg)
        send_gated(enc, io.BytesIO(), msg, route='usb', latch=item, authorized=False)
        with pytest.raises(ValueError):
            send_gated(enc, io.BytesIO(), msg, route='trs', latch=item, authorized=True)
    unsafe = b'commander arm\n'
    msg = enc.serial_control_encode(10, 2, 0, 0, len(unsafe), unsafe.ljust(70, b'\0'))
    assert not shell_message_allowed(msg)


def test_parse_complete_state_and_not_just_ack():
    text = ('nsh> listener actuator_armed -n 1\nTOPIC: actuator_armed\n'
            ' timestamp: 123456 (0.1 seconds ago)\n armed: False\n lockdown: True\n'
            ' manual_lockdown: False\n force_failsafe: False\n')
    snapshot = parse_actuator_snapshot(text)
    assert snapshot['lockdown'] and not snapshot['armed']
    assert snapshot['timestamp_us'] == 123456
    assert parse_actuator_snapshot('COMMAND_ACK: accepted') is None
    assert parse_actuator_snapshot(text.split('force_failsafe')[0]) is None


def test_latest_complete_state_is_selected():
    def block(timestamp, locked):
        return (f'TOPIC: actuator_armed\n timestamp: {timestamp}\n armed: False\n'
                f' lockdown: {locked}\n manual_lockdown: False\n force_failsafe: False\n')
    result = parse_actuator_snapshot(block(10, False) + block(20, True))
    assert result['lockdown'] and result['timestamp_us'] == 20
