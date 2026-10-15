import struct
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import fault_archive_service as p


def test_crc_matches_uncomplemented_nuttx_reference():
    crc = 0
    data = b'123456789'
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xedb88320 if crc & 1 else 0)
    assert p.nuttx_crc(data) == crc == 0x2dfd2d88


@pytest.mark.parametrize('opcode', [0, 2, 6, 7, 8, 10, 11, 12, 15])
def test_no_reset_remove_write_truncate_or_burst(opcode):
    assert not p.ftp_allowed(opcode, b'', archive_authorized=True, backed_up=True)


def test_exact_archive_pairs_only_and_backup_before_move():
    for name in p.NAMES:
        pair = (p.ROOT + '/' + name).encode() + b'\0' + (p.ARCHIVE + '/' + name).encode()
        assert p.ftp_allowed(13, pair, archive_authorized=True, backed_up=True)
        assert not p.ftp_allowed(13, pair, archive_authorized=False, backed_up=True)
        assert not p.ftp_allowed(13, pair, archive_authorized=True, backed_up=False)
        assert not p.ftp_allowed(13, pair.replace(b'/svc_20261014/', b'/log/'), archive_authorized=True, backed_up=True)
    assert not p.ftp_allowed(4, b'/fs/microsd/parameters_backup.bson')
    assert not p.ftp_allowed(9, p.ROOT.encode(), archive_authorized=True, backed_up=True)
    assert p.ftp_allowed(9, p.ARCHIVE.encode(), archive_authorized=True, backed_up=True)


def test_read_bounds_and_payload_layout():
    assert p.ftp_allowed(5, b'', size=239, offset=10)
    assert not p.ftp_allowed(5, b'', size=240, offset=0)
    assert not p.ftp_allowed(5, b'', size=1, offset=65536)
    assert not p.ftp_allowed(1, b'', session=1)
    raw = p.payload(100, 5, offset=239, size=100)
    assert len(raw) == 251
    assert struct.unpack('<HBBBBBBI', raw[:12]) == (100, 0, 5, 100, 0, 0, 0, 239)


def test_byte_file_and_shell_fault_body_compare():
    raw = '[hardfault_log] -- start --\nrecord\n'
    transcript = '\x1b[Kcat /fs/microsd/log\r\n' + raw.replace('\n', '\r\n') + 'nsh> '
    assert p.normalized_fault(raw) == p.normalized_fault(transcript)


def test_reboot_not_permitted_without_archived_receipt():
    from pymavlink.dialects.v20 import common as m
    c = p.Connection(None)
    c.safe = lambda: True
    msg = c.enc.command_long_encode(1, 1, m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
    with pytest.raises(RuntimeError, match='send gate'):
        c.send(msg)
