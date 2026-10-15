#!/usr/bin/env python3
"""One authorized normal USB reboot after verifying Vision-reference parameters.

Battery disconnected, props installed, no parameter writes/poses/arm/unlock.
Sending a reboot is NOT proof it succeeded; inspect fresh boot state afterwards.
"""
import argparse
import datetime
import json
from pathlib import Path
import subprocess
import time

import serial
from pymavlink.dialects.v20 import common as m
import vicon_height_reference_setup as setup
from gamepad_estop_monitor import find_pad


def reboot_allowed(msg, *, identity_safe=False, locked=False, endpoint_verified=False,
                   backup_verified=False, already_attempted=False):
    return bool(msg.get_type() == 'COMMAND_LONG'
                and msg.command == m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
                and identity_safe and locked and endpoint_verified and backup_verified
                and not already_attempted
                and (msg.target_system, msg.target_component, msg.confirmation) == (1, 1, 0)
                and msg.param1 == 1
                and all(getattr(msg, f'param{i}') == 0 for i in range(2, 8)))


class RebootSession(setup.Session):
    def __init__(self, port):
        super().__init__(port, writes_enabled=False)
        self.endpoint_verified = False
        self.reboot_attempted = False

    def send(self, msg):
        if msg.get_type() != 'COMMAND_LONG' or msg.command != m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            return super().send(msg)
        if not reboot_allowed(msg, identity_safe=self.safe(),
                              locked=time.monotonic()-self.locked_at < 1,
                              endpoint_verified=self.endpoint_verified, backup_verified=True,
                              already_attempted=self.reboot_attempted):
            raise RuntimeError('One-reboot-only gate rejected request')
        self.reboot_attempted = True  # Any write ambiguity consumes the single attempt.
        packet = msg.pack(self.enc)
        if self.port.write(packet) != len(packet):
            raise OSError('Short reboot write; do not retry automatically')
        self.enc.seq = (self.enc.seq + 1) % 256
        self.sent['COMMAND_LONG_REBOOT'] += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--backup-dir', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--battery-off-usb-only-reboot-authorized', action='store_true')
    args = ap.parse_args()
    if not args.battery_off_usb_only_reboot_authorized or args.output.exists() or not args.output.parent.is_dir():
        ap.error('Explicit one-reboot authorization and new output required')
    hashes = setup.verify_backup(args.backup_dir)
    pad = find_pad()
    trs = Path('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    if not trs.exists():
        ap.error('TRS absent; cannot prepare HOME relock')
    for path in (setup.FC_USB, trs):
        if subprocess.run(['fuser', str(path.resolve())], capture_output=True).returncode == 0:
            ap.error('Serial port occupied; no process will be killed')
    result = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope': __doc__, 'backup_hashes': hashes, 'gamepad_device': pad,
              'parameters_before': {}, 'reboot_sent': False, 'reboot_success_verified': False,
              'parameter_writes': 0, 'pose_packets_sent': 0, 'flight_ready': False, 'error': None}
    session = None
    try:
        with serial.Serial(str(setup.FC_USB), 115200, timeout=.02, write_timeout=1, exclusive=True) as port:
            session = RebootSession(port)
            session.identity()
            session.check_lock()
            for name, value in setup.ENDPOINT.items():
                got = session.read(name)
                result['parameters_before'][name] = got
                if got != (value, 6):
                    raise RuntimeError('Endpoint mismatch; no reboot: '+name)
            session.check_lock()
            session.endpoint_verified = True
            session.send(session.enc.command_long_encode(1, 1, m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                                        0, 1, 0, 0, 0, 0, 0, 0))
            result['reboot_sent'] = True
            # Do not retry or treat the transport write/USB disappearance as boot verification.
    except Exception as exc:
        result['error'] = str(exc)
    if session:
        result.update(identity=session.ev.identity, tx_messages=dict(session.sent),
                      reboot_attempted=session.reboot_attempted, lock_snapshots=session.lock_snapshots)
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'lock_snapshots'}, indent=2))
    return 0 if result['reboot_sent'] and not result['error'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
