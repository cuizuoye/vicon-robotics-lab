#!/usr/bin/env python3
"""Bounded battery-disconnected timing run; verified no-PWM maintenance reset."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import time
from types import SimpleNamespace

from four_link_ro_probe import run
from link_timing import SerialLogCapture, esp_ipv4_from_logs

CLI='/home/PRIMARY_USER/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI'
CONNECTION=['-q','-c','port=SWD','sn=000000000000000000000000','mode=HOTPLUG','freq=1000']
EXPECTED='12d4224f5337b42d3bdc3eef30f4142e957d65d900d61b64d8b8fb22033e9aa6'


def cli(args, log):
    p=subprocess.run([CLI,*CONNECTION,*args],capture_output=True,text=True,timeout=15)
    log.write_text(p.stdout+p.stderr)
    if p.returncode: raise RuntimeError('SWD operation failed: '+str(log))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--esp-ip',required=True,help='Station IPv4, or auto from COM startup log')
    p.add_argument('--bind-ip',required=True)
    p.add_argument('--interface',required=True)
    p.add_argument('--esp-log-serial',required=True)
    p.add_argument('--result-dir',type=Path,required=True)
    p.add_argument('--confirm-battery-disconnected',action='store_true',required=True)
    a=p.parse_args()
    a.result_dir.mkdir(mode=0o700)  # Refuse to overwrite any previous run.
    image=a.result_dir/'stm32_readback.bin'
    cli(['-u','0x08000000','26988',str(image)],a.result_dir/'verify.log')
    if hashlib.sha256(image.read_bytes()).hexdigest()!=EXPECTED:
        raise RuntimeError('Not the verified no-PWM image; reset and probe refused')
    started=time.monotonic()
    capture=SerialLogCapture(a.esp_log_serial,started)
    handed_off=False
    try:
        # Opening COM can cause a board-specific boot even with DTR/RTS false.
        # Never interpret that startup window as healthy command transport.
        time.sleep(3)
        if capture.error: raise RuntimeError(capture.error)
        boot_seen=any('ESP-ROM:' in x['text'] for x in capture.lines)
        if boot_seen:
            until=time.monotonic()+7
            while not any('UDP command listener ready' in x['text'] for x in capture.lines):
                if time.monotonic()>=until: raise RuntimeError('ESP did not finish network startup')
                time.sleep(0.05)
        # Allow association/DHCP and startup ARP traffic to settle before HELLO.
        # Do not treat a startup-only result as a steady-state acceptance test.
        time.sleep(5)
        if a.esp_ip == 'auto':
            a.esp_ip = esp_ipv4_from_logs(capture.lines)
        else:
            # An exact packet-capture filter must not follow a stale DHCP IP.
            observed_ip = esp_ipv4_from_logs(capture.lines)
            if observed_ip != a.esp_ip:
                raise RuntimeError('ESP station IP changed; exact-peer probe refused')
        (a.result_dir/'setup.json').write_text(json.dumps(dict(
            esp_ip=a.esp_ip,pc_bind_ip=a.bind_ip,interface=a.interface,
            battery_disconnected_confirmed=True,firmware_sha256=EXPECTED),indent=2)+'\n')
        cli(['-rst'],a.result_dir/'maintenance_reset.log')
        time.sleep(0.25)
        args=SimpleNamespace(serial=None,esp_ip=a.esp_ip,bind_ip=a.bind_ip,
             interface=a.interface,seconds=15,check_timeout=False,
             output=a.result_dir/'probe.json',esp_log_serial=None)
        handed_off=True
        run(args,capture=capture,started=started)
    finally:
        if not handed_off:
            (a.result_dir/'setup_serial.json').write_text(json.dumps(capture.finish(),indent=2))
        # Exact-image HOTPLUG snapshot, no halt/reset/write, after the trace.
        ram=a.result_dir/'ram_after.bin'
        cli(['-u','0x20000000','8192',str(ram)],a.result_dir/'ram_after.log')
        raw=ram.read_bytes()
        if len(raw)!=8192: raise RuntimeError('Incomplete RAM snapshot')
        # Fixed offsets are valid ONLY for EXPECTED, checked above.
        sid,seq,cmd,cmd_ms,fault=struct.unpack_from('<5I',raw,0x30)
        snapshot=dict(firmware_sha256=EXPECTED,mode='HOTPLUG read only',
            mcu_tick_ms=struct.unpack_from('<I',raw,0x2c)[0],session_id=sid,
            last_sequence=seq,last_command_sequence=cmd,last_command_ms=cmd_ms,
            fault=fault,session_active=bool(raw[0x44]),command_seen=bool(raw[0x45]),
            expired=bool(raw[0x46]),uart_fault=bool(raw[0x48]),
            note='Snapshot is after test cleanup, not the exact fault instant')
        (a.result_dir/'mcu_after.json').write_text(json.dumps(snapshot,indent=2)+'\n')

if __name__=='__main__': main()
