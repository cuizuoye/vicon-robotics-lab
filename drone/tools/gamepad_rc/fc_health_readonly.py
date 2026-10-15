#!/usr/bin/env python3
"""同机未解锁飞控的固定健康诊断；不写参数、不测试hardfault、不重启。"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import time
import serial
from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import Evidence,readonly_message
from estop_only_policy import parse_actuator_snapshot
from trs_uart_readonly_check import UID2
from persistent_shell import reply_complete
from collections import Counter

PARAMS=('EKF2_EV_CTRL','EKF2_EV_DELAY','EKF2_HGT_REF')

COMMANDS={
    'actuator':b'listener actuator_armed -n 1\n',
    'health_report':b'listener health_report -n 1\n',
    'attitude':b'listener vehicle_attitude -n 1\n',
    'version':b'ver all\n',
    'boot_log':b'dmesg\n',
    'sd_listing':b'ls /fs/microsd\n',
    'board_sensors':b'cat /etc/init.d/rc.board_sensors\n',
    'board_defaults':b'cat /etc/init.d/rc.board_defaults\n',
    'board_extras':b'cat /etc/init.d/rc.board_extras\n',
    'sensor_combined':b'listener sensor_combined -n 1\n',
    'sensor_selection':b'listener sensor_selection -n 1\n',
    'sensor_estimator_selector':b'listener estimator_selector_status -n 1\n',
    'sensor_range':b'listener distance_sensor -n 1\n',
    'sensor_magnetometer':b'listener vehicle_magnetometer -n 1\n',
    'sensor_local_position':b'listener vehicle_local_position -n 1\n',
    'fault_first':b'cat /fs/microsd/fault_2000_01_01_00_31_10.log\n',
    'fault_second':b'cat /fs/microsd/fault_2000_01_01_00_46_57.log\n'}


def allowed(msg,safe=False):
    if msg.get_type()=='PARAM_REQUEST_READ':
        name=msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id,bytes) else msg.param_id.rstrip('\0')
        return bool(safe and name in PARAMS and (msg.target_system,msg.target_component,msg.param_index)==(1,1,-1))
    if msg.get_type()!='SERIAL_CONTROL': return readonly_message(msg)
    if (msg.device,msg.timeout,msg.baudrate)!=(10,0,0): return False
    # 旧版Shell关闭竞态待确认：诊断不再发送flags=0，不频繁销毁Shell对象。
    if msg.flags==0: return False
    return bool(safe and msg.flags==2 and any(msg.count==len(p) and bytes(msg.data)==p.ljust(70,b'\0')
                                             for p in (b'',*COMMANDS.values())))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--regression-seconds',type=int,default=0)
    ap.add_argument('--physical-confirmed',action='store_true',help='Battery off, props off, USB only, QGC exited')
    ap.add_argument('--board-config',action='store_true',help='Fixed readonly vendor startup scripts only; no execution')
    ap.add_argument('--sensor-status',action='store_true',help='Fixed readonly sensor/estimator snapshots; no writes or control')
    args=ap.parse_args()
    if args.regression_seconds and (not 20<=args.regression_seconds<=60 or not args.physical_confirmed):
        ap.error('Readonly regression requires physical confirmation and 20..60 seconds')
    if args.board_config and (not args.physical_confirmed or args.regression_seconds):
        ap.error('Board-config reads require physical confirmation and no regression loop')
    if args.sensor_status and (not args.physical_confirmed or args.regression_seconds or args.board_config):
        ap.error('Sensor-status reads require physical confirmation and no other profile')
    if args.output.exists() or subprocess.run(['fuser',str(Path(args.port).resolve())],capture_output=True).returncode==0:
        ap.error('New output and free port required')
    ev=Evidence(source_component=195,expected_parameters=PARAMS)
    enc=m.MAVLink(None,srcSystem=250,srcComponent=195)
    dec=m.MAVLink(None);dec.robust_parsing=True
    started=time.monotonic();next_hb=next_version=0
    buffer='';records={};boot_ms=[];error=None;tx=Counter();repeats=[]
    parameters_before=parameters_after=None;faults_before=faults_after=None
    def safe():
        now=time.monotonic()-started
        return bool(ev.identity and ev.identity['uid2']==UID2 and ev.identity['firmware']=='1.14.3'
                    and not ev.fatal and ev.heartbeats and not ev.heartbeats[-1]['armed']
                    and now-ev.heartbeats[-1]['elapsed_s']<2)
    def send(msg):
        if not allowed(msg,safe()): raise RuntimeError('Health readonly gate rejected '+msg.get_type())
        packet=msg.pack(enc)
        if port.write(packet)!=len(packet): raise OSError('Short write')
        enc.seq=(enc.seq+1)%256
        tx[msg.get_type()]+=1
    def shell(payload=b''):
        send(enc.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0')))
    def pump():
        nonlocal next_hb,next_version,buffer
        now=time.monotonic()-started
        if now>50+args.regression_seconds: raise RuntimeError('Bounded health session expired')
        for msg in dec.parse_buffer(port.read(min(port.in_waiting,65536))) or []:
            ev.ingest(msg,now)
            if ev.fatal: raise RuntimeError(ev.fatal)
            if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1): continue
            if msg.get_type()=='ATTITUDE_QUATERNION':
                if boot_ms and msg.time_boot_ms<boot_ms[-1]: raise RuntimeError('FC boot clock regressed: possible reboot')
                boot_ms.append(msg.time_boot_ms)
            if msg.get_type()=='SERIAL_CONTROL':
                buffer+=bytes(msg.data[:msg.count]).decode('utf8','replace')
                if len(buffer)>65536: raise RuntimeError('Health observer overflow')
        if ev.identity and (ev.identity['uid2']!=UID2 or ev.identity['firmware']!='1.14.3'):
            raise RuntimeError('Wrong aircraft or firmware')
        if now>=next_hb:
            send(enc.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0));next_hb=now+1
        if ev.heartbeats and ev.identity is None and now>=next_version:
            ev.version_requested_at=now
            send(enc.command_long_encode(1,1,512,0,148,0,0,0,0,0,0));next_version=now+2
        time.sleep(.002)

    def observe(payload):
        nonlocal buffer
        # 每条命令保持同一Shell，必须收完提示符，超时即结束而不是销毁会话。
        buffer='';shell(payload)
        until=time.monotonic()+6;poll=time.monotonic()+.25
        while time.monotonic()<until:
            pump()
            if not safe(): raise RuntimeError('Disarmed identity freshness lost')
            if reply_complete(buffer,payload): return buffer
            if time.monotonic()>=poll: shell();poll=time.monotonic()+.25
        raise RuntimeError('Incomplete persistent Shell reply: '+payload.decode().strip())

    def read_baseline():
        ev.parameters.clear()
        now=time.monotonic()-started
        for name in PARAMS:
            ev.requests[name]=now;send(enc.param_request_read_encode(1,1,name.encode(),-1))
        until=time.monotonic()+3
        while set(ev.parameters)!=set(PARAMS):
            pump()
            if not safe() or time.monotonic()>until: raise RuntimeError('Fresh parameter read failed')
        if any(ev.parameters[n]!=0 for n in ('EKF2_EV_CTRL','EKF2_EV_DELAY')):
            raise RuntimeError('Fusion/delay not disabled')
        return dict(ev.parameters)

    def fault_names(text):
        return sorted(set(re.findall(r'\bfault_[A-Za-z0-9_]+\.log\b',text)))
    try:
        with serial.Serial(args.port,115200,timeout=0,write_timeout=.5,exclusive=True) as port:
            while not safe():
                pump()
                if time.monotonic()-started>8: raise RuntimeError('No fresh same-aircraft disarmed identity')
            parameters_before=read_baseline()
            for key,payload in COMMANDS.items():
                if args.board_config and key not in ('actuator','version','board_sensors','board_defaults','board_extras'): continue
                if args.sensor_status and key not in ('actuator','version') and not key.startswith('sensor_'): continue
                if not args.board_config and key.startswith('board_'): continue
                if not args.sensor_status and key.startswith('sensor_'): continue
                if args.regression_seconds and key.startswith('fault_'): continue
                records[key]=observe(payload)
            if args.regression_seconds:
                faults_before=fault_names(records['sd_listing'])
                end=time.monotonic()+args.regression_seconds
                while time.monotonic()<end:
                    for key in ('actuator','attitude'):
                        text=observe(COMMANDS[key])
                        if key=='actuator':
                            state=parse_actuator_snapshot(text)
                            if not state or state['armed']: raise RuntimeError('Missing/disarmed internal state lost')
                        repeats.append({'key':key,'elapsed_s':time.monotonic()-started,'text':text})
                    idle_until=min(end,time.monotonic()+.25)
                    while time.monotonic()<idle_until:
                        pump()
                        if not safe(): raise RuntimeError('Identity freshness lost during regression')
                records['sd_listing_after']=observe(COMMANDS['sd_listing'])
                faults_after=fault_names(records['sd_listing_after'])
                if faults_after!=faults_before: raise RuntimeError('SD fault log list changed')
            parameters_after=read_baseline()
    except (Exception,KeyboardInterrupt) as exc: error=str(exc)
    result={'scope':'Fixed readonly health commands; no motion, writes, fault tests or reboot',
            'identity':ev.identity,'heartbeats':ev.heartbeats,'command_records':records,
            'fc_boot_ms_first_last':[boot_ms[0],boot_ms[-1]] if boot_ms else None,
            'actuator':parse_actuator_snapshot(records.get('actuator','')),
            'fault_related_log_lines':[l for l in records.get('boot_log','').splitlines()
                                      if re.search(r'\b(?:hardfault|fault|reset|reboot|brownout|panic|assert)\b|Crash dumps',l,re.I)],
            'error':error,'serial_port_released':True,
            'explicit_shell_close_disabled':True,'firmware_repaired':False,
            'tx_messages':dict(tx),'regression_seconds_requested':args.regression_seconds,
            'regression_replies':repeats,'parameters_before':parameters_before,'parameters_after':parameters_after,
            'fault_names_before':faults_before,'fault_names_after':faults_after,
            'regression_passed':bool(args.regression_seconds and error is None and repeats
                                      and len(boot_ms)>1 and faults_before==faults_after
                                      and parameters_before==parameters_after)}
    with args.output.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps({'output':str(args.output),'error':error,
                      'regression_passed':result['regression_passed'],
                      'completed_regression_replies':len(repeats),
                      'firmware_repaired':False},ensure_ascii=False,indent=2))
    return 0 if error is None else 2


if __name__=='__main__': raise SystemExit(main())
