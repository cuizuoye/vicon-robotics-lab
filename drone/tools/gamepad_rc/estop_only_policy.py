"""PX4 1.14.3专用停止边界：不发RC、不解锁、不解除锁定。"""

import json
import os
from pathlib import Path
import re

from pymavlink.dialects.v20 import common as m

from gamepad_estop_monitor import StopLatch
from mavlink_readonly_probe import readonly_message

FIRMWARE = '1.14.3'
LOCK_COMMAND = m.MAV_CMD_DO_FLIGHTTERMINATION
LISTENER = b'listener actuator_armed -n 1\n'


class DurableStopLatch(StopLatch):
    """文件存在即保持停止；损坏文件也不能默默恢复运行。无清除入口。"""

    def __init__(self, path, uid2):
        super().__init__()
        self.path, self.uid2 = Path(path), uid2
        self.restored = self.path.exists()
        if self.restored:
            self.latched, self.reason = True, 'retained_stop_latch'

    def fail(self, reason):
        super().fail(reason)
        record = {'latched': True, 'uid2': self.uid2, 'reason': self.reason,
                  'scope': 'No automatic clear; retained even after HOME release/process exit'}
        try:
            fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            return
        with os.fdopen(fd, 'w') as stream:
            json.dump(record, stream, ensure_ascii=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())


def lock_message(encoder, identity, expected_uid2, latch, authorized, disarmed, confirmation=0):
    """param1=2是这个版本的lockdown，不是param1=1的终止/降落伞动作。"""
    if not (authorized and disarmed and latch.latched
            and identity and identity.get('firmware') == FIRMWARE
            and identity.get('uid2') == expected_uid2):
        raise ValueError('Stop command gate not satisfied')
    if confirmation not in (0, 1, 2):
        raise ValueError('Bounded retry only')
    return encoder.command_long_encode(1, 1, LOCK_COMMAND, confirmation, 2, 0, 0, 0, 0, 0, 0)


def stop_message_allowed(message, latched, authorized):
    return (latched and authorized and message.get_type() == 'COMMAND_LONG'
            and (message.target_system, message.target_component) == (1, 1)
            and message.command == LOCK_COMMAND and message.param1 == 2
            and message.confirmation in (0, 1, 2)
            and all(getattr(message, f'param{i}') == 0 for i in range(2, 8)))


def shell_message(encoder, poll=False):
    payload = b'' if poll else LISTENER
    return encoder.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0'))


def shell_message_allowed(message):
    return (message.get_type() == 'SERIAL_CONTROL' and message.device == 10
            and message.flags == 2 and message.timeout == 0 and message.baudrate == 0
            and message.count in (0, len(LISTENER))
            and bytes(message.data) == (b'' if message.count == 0 else LISTENER).ljust(70, b'\0'))


def send_gated(encoder, port, message, *, route, latch, authorized):
    # 停止命令只能走TRS；USB只接受信息读取和固定listener，无任意shell入口。
    allowed = readonly_message(message)
    if route == 'trs':
        allowed |= stop_message_allowed(message, latch.latched, authorized)
    elif route == 'usb':
        allowed |= shell_message_allowed(message)
    else:
        allowed = False
    if not allowed:
        raise ValueError(f'Refused message on {route}: {message.get_type()}')
    packet = message.pack(encoder)
    if port.write(packet) != len(packet):
        raise OSError('Short serial write')
    encoder.seq = (encoder.seq + 1) % 256
    return len(packet)


def parse_actuator_snapshot(text):
    """只接受完整actuator_armed块；ACK不能替代状态证据。"""
    blocks = re.split(r'TOPIC:\s*actuator_armed[^\n]*\n', text)
    for block in reversed(blocks[1:]):
        timestamp = re.search(r'^\s*timestamp:\s*(\d+)', block, re.M)
        fields = {}
        for name in ('armed', 'lockdown', 'manual_lockdown', 'force_failsafe'):
            match = re.search(rf'^\s*{name}:\s*(True|False|true|false)\b', block, re.M)
            if match:
                fields[name] = match.group(1).lower() == 'true'
        if timestamp and len(fields) == 4:
            return {'timestamp_us': int(timestamp.group(1)), **fields}
    return None
