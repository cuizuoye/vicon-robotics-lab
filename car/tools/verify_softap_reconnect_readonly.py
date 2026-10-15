#!/usr/bin/env python3
"""One staged no-power test; only disabled zero CMD_VEL, no COM/reset/flash.

May disconnect ONLY the explicitly approved Tenda profile. An exact verified
no-PWM SWD precheck is mandatory. Reconnection never issues an enable.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import secrets
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/mecanum_vicon_control'))
from mecanum_vicon_control.transport_protocol import (
    MessageType, FrameFlags, encode_frame, decode_frame, pack_cmd_vel, unpack_wheel_telemetry,
)
from mecanum_vicon_control.readonly_link_status import decode_readonly_status

DEVICE='wlx000000000000'
UUID='00000000-0000-4000-8000-000000000000'
EXPECTED='12d4224f5337b42d3bdc3eef30f4142e957d65d900d61b64d8b8fb22033e9aa6'

def command(*argv):
    return subprocess.check_output(argv, text=True, timeout=25).strip()

def network():
    active=command('nmcli','-t','-f','UUID,DEVICE','connection','show','--active').splitlines()
    return dict(other_active=sorted(x for x in active if x!=f'{UUID}:{DEVICE}'),
        default_route=command('ip','-4','route','show','default'),
        policy_rules=command('ip','rule','show'),
        resolv_sha256=hashlib.sha256(Path('/etc/resolv.conf').read_bytes()).hexdigest())

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--precheck', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--confirm-battery-disconnected',action='store_true',required=True)
    p.add_argument('--allow-tenda-reconnect',action='store_true',required=True)
    a=p.parse_args()
    if a.output.exists():p.error('output already exists')
    proof=json.loads(a.precheck.read_text())
    assert proof['passed'] and proof['firmware_sha256']==EXPECTED
    assert all(x['readonly'] and not x['pi_enabled'] and not x['run_used']
               for x in proof['samples'])
    assert time.time()-a.precheck.stat().st_mtime < 1800, 'need fresh physical SWD check'
    assert command('nmcli','-g','connection.interface-name','connection','show',UUID)==DEVICE
    assert command('nmcli','-g','ipv4.never-default','connection','show',UUID)=='yes'
    assert command('nmcli','-g','ipv4.gateway','connection','show',UUID)==''
    assert command('nmcli','-g','ipv4.ignore-auto-dns','connection','show',UUID)=='yes'
    before=network(); started=time.monotonic(); seq=0; sid=secrets.randbits(32) or 1
    record=dict(scope='REAL disabled-zero MC-Link, battery disconnected, no motion image',
        enabled_or_nonzero_sent=False,COM_opened=False,reset_or_flash=False,
        phases=[],network_before=before,success=False)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,DEVICE.encode()+b'\0')
    sock.bind(('192.168.50.2',46001));sock.setblocking(False)
    profile_down=False
    def send(kind):
        nonlocal seq
        seq=(seq+1)&0xffffffff
        payload=pack_cmd_vel(0,0,0,timeout_ms=150) if kind==MessageType.CMD_VEL else b''
        flags=FrameFlags.ACK_REQUIRED if kind==MessageType.HELLO else 0
        sock.sendto(encode_frame(kind,seq,sid,int((time.monotonic()-started)*1000),payload,flags),
                    ('192.168.50.1',46000))
        return seq
    def collect(name, seconds, stream=True):
        phase=dict(name=name,telemetry=[],diagnostics=[],acks=[])
        record['phases'].append(phase); until=time.monotonic()+seconds;next_send=0
        while time.monotonic()<until:
            now=time.monotonic()
            if stream and now>=next_send:send(MessageType.CMD_VEL);next_send=now+.02
            for _ in range(100):
                try:raw,peer=sock.recvfrom(1024)
                except BlockingIOError:break
                f=decode_frame(raw)
                if peer[0]!='192.168.50.1' or f.session_id!=sid:continue
                if f.message_type==MessageType.HELLO_ACK:
                    assert f.flags==FrameFlags.RESPONSE and not f.payload
                    phase['acks'].append(f.sequence)
                elif f.message_type==MessageType.WHEEL_TELEMETRY:
                    t=asdict(unpack_wheel_telemetry(f.payload))
                    assert not f.flags&FrameFlags.ENABLED and not t['state_flags']&2
                    assert t['state_flags']&0x6d==0x6d and not t['fault_bits']&~1
                    t['host_s']=time.monotonic()-started;phase['telemetry'].append(t)
                elif f.message_type==MessageType.LINK_STATUS:
                    d=decode_readonly_status(f.payload)
                    assert not any(d['compares'])
                    phase['diagnostics'].append(d)
            time.sleep(.001)
        print(name, 'wheel=',len(phase['telemetry']),'diag=',len(phase['diagnostics']),flush=True)
        return phase
    def healthy(name,seconds):
        nonlocal sid,seq
        sid=secrets.randbits(32) or 1;seq=0
        hello=send(MessageType.HELLO)
        phase=collect(name,seconds)
        assert hello in phase['acks'] and len(phase['telemetry'])>=10 and len(phase['diagnostics'])>=10
        assert all(t['fault_bits']==0 for t in phase['telemetry'])
        assert len({t['last_command_sequence'] for t in phase['telemetry']})>=5
    try:
        healthy('normal_30s',30)
        phase=collect('sender_silence_450ms',.45,False)
        assert phase['telemetry'] and phase['telemetry'][-1]['fault_bits']==1
        record['mcu_lease_timeout_seen']=True
        healthy('new_hello_after_silence',3)
        command('nmcli','--wait','10','connection','down',UUID);profile_down=True
        time.sleep(1)
        record['network_during_tenda_outage']=network()
        assert record['network_during_tenda_outage']==before
        command('nmcli','--wait','20','connection','up',UUID,'ifname',DEVICE);profile_down=False
        # Drain any pre-outage packets without treating them as new-session evidence.
        until=time.monotonic()+.3
        while time.monotonic()<until:
            try:sock.recvfrom(1024)
            except BlockingIOError:time.sleep(.001)
        phase=collect('old_session_without_hello',1)
        assert not phase['acks'] and not phase['telemetry'] and not phase['diagnostics'], \
            'old session unexpectedly forwarded after AP client disconnection'
        record['new_hello_required_after_ap_reconnect']=True
        healthy('fresh_hello_after_ap_reconnect',5)
        phase=collect('final_sender_exit_timeout',.45,False)
        assert phase['telemetry'] and phase['telemetry'][-1]['fault_bits']==1
        record['success']=True
    except Exception as error:
        record['error']=str(error)
        raise
    finally:
        sock.close()
        if profile_down:
            try:command('nmcli','--wait','20','connection','up',UUID,'ifname',DEVICE)
            except Exception as error:record['restore_error']=str(error)
        record['network_after']=network()
        record['original_network_preserved']=record['network_after']==before
        record['success'] &= record['original_network_preserved']
        record['elapsed_s']=time.monotonic()-started
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with a.output.open('x') as out:json.dump(record,out,indent=2)
        print(json.dumps({k:v for k,v in record.items() if k!='phases'}),flush=True)
    return 0 if record['success'] else 1

if __name__=='__main__':raise SystemExit(main())
