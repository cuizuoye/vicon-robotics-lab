#!/usr/bin/env python3
"""One authorized ESP-only boot after full programming/readback proof.

Run with the ESP-IDF Python environment. Battery, external 5V and both UART
lines must remain disconnected. This does not open UDP or issue MCU commands.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import serial
from esptool.reset import HardReset

PORT='/dev/serial/by-id/usb-1a86_USB_Single_Serial_00000000-if00'
EXPECTED='1185a34026b7d15ff24eb871a699d8f6049e7e492150f15f002bfed2e2304c4f'

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--backup',type=Path,required=True)
    ap.add_argument('--confirm-battery-and-external-5v-and-uart-disconnected',action='store_true',required=True)
    args=ap.parse_args();folder=args.backup.resolve()
    proof=json.loads((folder/'POST_FLASH_VERIFIED.json').read_text())
    if not proof['passed'] or proof['candidate_sha256']!=EXPECTED or not proof['readback_byte_identical']:
        raise ValueError('missing verified candidate/full readback')
    log=folder/'candidate_boot.log';result_path=folder/'BOOT_LOG_RESULT.json'
    if log.exists() or result_path.exists():raise ValueError('boot already attempted; no automatic retry')
    os.umask(0o077)
    port=serial.Serial(port=None,baudrate=115200,timeout=.1)
    port.dtr=False;port.rts=False;port.port=PORT
    chunks=[]
    with log.open('xb') as stream:
        try:
            port.open();port.reset_input_buffer()
            HardReset(port)()
            until=time.monotonic()+15
            while time.monotonic()<until:
                data=port.read(4096)
                if data:chunks.append(data);stream.write(data);stream.flush()
        finally:port.close()
    content=b''.join(chunks).decode(errors='replace')
    version='eventio-20261026' in content
    failed=any(x in content.lower() for x in ('forward wake initialization failed',
        'event forwarder stopped','forwarding select failed','guru meditation','abort() was called'))
    result=dict(passed=version and not failed,app_version_confirmed=version,
        explicit_event_init_or_crash_error=failed,boot_log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
        intentional_esp_only_reset_count=1,uart_physically_disconnected=True,
        stm32_reset_or_written=False,udp_created=False,motion_started=False,
        mcu_handshake_validated=False,requires_reconnect_before_handshake=True)
    result_path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)

if __name__=='__main__':main()
