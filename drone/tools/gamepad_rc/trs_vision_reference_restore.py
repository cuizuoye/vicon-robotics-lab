#!/usr/bin/env python3
"""Recovery ONLY: same locked USB aircraft EV_CTRL 11->2; no poses/reboot/unlock."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import serial
from pymavlink.dialects.v20 import common as m
import vicon_height_reference_setup as setup
import vision_reference_xyz_policy as xyz
from trs_static_fusion_bench import load_backup


def allowed(msg, safe=False, locked=False):
    if msg.get_type() == 'PARAM_SET':
        from mavlink_readonly_probe import decode_parameter
        name=msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id,bytes) else msg.param_id
        return bool(name=='EKF2_EV_CTRL' and decode_parameter(msg)==2 and locked
            and xyz.allowed(msg,authorized=True,backup_verified=True,restore_safe=safe))
    return setup.allowed(msg,identity_safe=safe,write_authorized=False)


class Session(setup.Session):
    def send(self,msg):
        if not allowed(msg,self.safe(),time.monotonic()-self.locked_at<1):
            raise RuntimeError('Restore-only policy rejected '+msg.get_type())
        packet=msg.pack(self.enc)
        if self.port.write(packet)!=len(packet):raise OSError('Short restore write')
        self.enc.seq=(self.enc.seq+1)%256
        self.sent[msg.get_type()]+=1


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--backup',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    load_backup(a.backup,vision_reference_static=True)
    if a.output.exists() or subprocess.run(['fuser',str(setup.FC_USB.resolve())],capture_output=True).returncode==0:
        ap.error('New output and free known USB required')
    r={'scope':__doc__,'write_attempted':False,'restore_verified':False,'error':None,'pose_packets_sent':0,'flight_ready':False}
    s=None
    try:
        with serial.Serial(str(setup.FC_USB),115200,timeout=.02,write_timeout=1,exclusive=True) as port:
            s=Session(port);s.identity();s.check_lock()
            before={name:s.read(name) for name in setup.ENDPOINT}
            r['before']=before
            if any(before[k]!=(v,6) for k,v in setup.ENDPOINT.items() if k!='EKF2_EV_CTRL') or before['EKF2_EV_CTRL'] not in ((2,6),(11,6)):
                raise RuntimeError('Unsupported recovery baseline')
            if before['EKF2_EV_CTRL']==(11,6):
                s.check_lock();r['write_attempted']=True
                s.send(xyz.control_message(s.enc,2))
            after={name:s.read(name) for name in setup.ENDPOINT}
            r['after']=after;s.check_lock()
            if any(after[k]!=(v,6) for k,v in setup.ENDPOINT.items()):raise RuntimeError('Recovery readback mismatch')
            r['restore_verified']=True
    except Exception as exc:r['error']=str(exc)
    if s:r.update(identity=s.ev.identity,tx_messages=dict(s.sent),lock_snapshots=s.lock_snapshots)
    with a.output.open('x') as stream:json.dump(r,stream,indent=2);stream.write('\n')
    print(json.dumps({k:v for k,v in r.items() if k!='lock_snapshots'},indent=2))
    return 0 if r['restore_verified'] else 2


if __name__=='__main__':raise SystemExit(main())
