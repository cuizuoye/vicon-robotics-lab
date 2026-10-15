#!/usr/bin/env python3
"""故障后的同机未解锁回退：只将EV_CTRL与EV_DELAY恢复0，无定位/控制入口。"""
import argparse
import json
from pathlib import Path
import subprocess
import time
import serial
from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import Evidence,decode_parameter,readonly_message
from trs_static_fusion_bench import load_backup
from trs_static_fusion_policy import parameter_message,READ_PARAMS
from trs_uart_readonly_check import UID2


def allowed(msg,*,safe=False,backup_verified=False):
    kind=msg.get_type()
    name=getattr(msg,'param_id','')
    if isinstance(name,bytes): name=name.decode('ascii').rstrip('\0')
    if kind=='PARAM_SET':
        types={'EKF2_EV_CTRL':m.MAV_PARAM_TYPE_INT32,'EKF2_EV_DELAY':m.MAV_PARAM_TYPE_REAL32}
        return bool(safe and backup_verified and name in types and msg.param_type==types[name]
                    and (msg.target_system,msg.target_component)==(1,1) and decode_parameter(msg)==0)
    if kind=='PARAM_REQUEST_READ':
        return name in READ_PARAMS and (msg.target_system,msg.target_component,msg.param_index)==(1,1,-1)
    return readonly_message(msg)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',required=True)
    ap.add_argument('--backup',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    # Both reviewed baselines may be restored to zero; this never enables fusion.
    try:
        baseline=load_backup(args.backup)
    except ValueError:
        baseline=load_backup(args.backup, range_reference_static=True)
    if args.output.exists() or subprocess.run(['fuser',str(Path(args.port).resolve())],capture_output=True).returncode==0:
        ap.error('New output and free port required')
    enc=m.MAVLink(None,srcSystem=250,srcComponent=196)
    dec=m.MAVLink(None);dec.robust_parsing=True
    ev=Evidence(source_component=196,expected_parameters=READ_PARAMS)
    start=time.monotonic()
    times,writes={},[]
    next_hb=next_request=0
    error=None
    verified=False
    before={}

    def safe():
        now=time.monotonic()-start
        return bool(ev.identity and ev.identity['uid2']==UID2 and ev.identity['firmware']=='1.14.3'
                    and not ev.fatal and ev.heartbeats and not ev.heartbeats[-1]['armed']
                    and now-ev.heartbeats[-1]['elapsed_s']<2)

    def send(msg):
        if not allowed(msg,safe=safe(),backup_verified=True):
            raise RuntimeError('Restore-only gate rejected '+msg.get_type())
        packet=msg.pack(enc)
        if port.write(packet)!=len(packet): raise OSError('Short write')
        enc.seq=(enc.seq+1)%256

    def pump():
        nonlocal next_hb,next_request
        now=time.monotonic()-start
        if now>35: raise RuntimeError('Restore global deadline')
        for msg in dec.parse_buffer(port.read(min(port.in_waiting,65536))) or []:
            ev.ingest(msg,now)
            if ev.fatal: raise RuntimeError(ev.fatal)
            if ev.identity and (ev.identity['uid2']!=UID2 or ev.identity['firmware']!='1.14.3'):
                raise RuntimeError('Wrong aircraft or firmware')
            if (msg.get_srcSystem(),msg.get_srcComponent())==(1,1) and msg.get_type()=='PARAM_VALUE':
                name=str(msg.param_id)
                if name in ev.requests: times[name]=now
        if now>=next_hb:
            send(enc.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0));next_hb=now+1
        if ev.heartbeats and ev.identity is None and now>=next_request:
            ev.version_requested_at=now
            send(enc.command_long_encode(1,1,512,0,148,0,0,0,0,0,0));next_request=now+2
        time.sleep(.002)

    try:
        with serial.Serial(args.port,115200,timeout=0,write_timeout=.5,exclusive=True) as port:
            while not safe():
                pump()
                if time.monotonic()-start>8: raise RuntimeError('No same-aircraft disarmed identity')
            for name in READ_PARAMS:
                ev.requests[name]=time.monotonic()-start
                send(enc.param_request_read_encode(1,1,name.encode(),-1))
            until=time.monotonic()+4
            while not all(n in ev.parameters for n in READ_PARAMS):
                pump()
                if time.monotonic()>until: raise RuntimeError('Incomplete restoration baseline')
            before=dict(ev.parameters)
            if (any(before[n]!=baseline[n] for n in READ_PARAMS if n not in ('EKF2_EV_CTRL','EKF2_EV_DELAY'))
                    or before['EKF2_EV_CTRL'] not in (0,11) or before['EKF2_EV_DELAY'] not in (0,65)):
                raise RuntimeError('Unexpected baseline; no blind restoration')
            for name in ('EKF2_EV_CTRL','EKF2_EV_DELAY'):
                for attempt in range(3):
                    when=time.monotonic()-start
                    send(parameter_message(enc,name,0))
                    writes.append({'name':name,'value':0,'attempt':attempt,'elapsed_s':when})
                    ev.requests[name]=when
                    send(enc.param_request_read_encode(1,1,name.encode(),-1))
                    until=time.monotonic()+2
                    while time.monotonic()<until:
                        pump()
                        if times.get(name,-100)>=when and ev.parameters.get(name)==0: break
                    if times.get(name,-100)>=when and ev.parameters.get(name)==0: break
                else: raise RuntimeError('Restore readback failed: '+name)
            verified=bool(safe() and all(ev.parameters[n]==0 for n in ('EKF2_EV_CTRL','EKF2_EV_DELAY')))
    except (Exception,KeyboardInterrupt) as exc:
        error=str(exc)
    result={'scope':'Same-aircraft disarmed restore-only maintenance; no poses, modes, arming, lock clearing or reboot',
            'identity':ev.identity,'heartbeats':ev.heartbeats,'before':before,'after':ev.parameters,
            'writes':writes,'restore_verified':verified and error is None,'error':error,'serial_port_released':True}
    with args.output.open('x') as f: json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['restore_verified'] else 2


if __name__=='__main__': raise SystemExit(main())
