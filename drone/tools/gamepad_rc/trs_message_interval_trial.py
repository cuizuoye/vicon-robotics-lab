#!/usr/bin/env python3
"""Temporary TRS telemetry rates ONLY, battery OFF/USB power, locked/disarmed.

Read original link intervals, try local position10Hz/estimator5Hz/landed5Hz,
measure actual arrivals, then restore exactly. No parameter, pose, mode, arm,
unlock, calibration, reboot or motor commands. PX41.14.3 rejects heartbeat
rate writes; heartbeat is queried/observed, NEVER changed.
"""
import argparse
import datetime
import json
from pathlib import Path
import subprocess
import time
import serial
from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import Evidence,FC_USB,PassivePowerEvidence,readonly_message
from estop_only_policy import LISTENER,parse_actuator_snapshot,shell_message,shell_message_allowed
from persistent_shell import reply_complete
from trs_only_estop_bench import state_valid
from trs_uart_readonly_check import UID2

CANDIDATES={32:100000,230:200000,245:200000}


def interval_message(enc,msgid,interval=None):
    if msgid not in (0,*CANDIDATES):raise ValueError('Fixed telemetry IDs only')
    return enc.command_long_encode(1,1,510 if interval is None else 511,0,
        msgid,0 if interval is None else interval,0,0,0,0,0)


def allowed(msg,*,safe=False,locked=False,original=None):
    if msg.get_type()=='SERIAL_CONTROL':return bool(safe and shell_message_allowed(msg))
    if readonly_message(msg):return True
    if msg.get_type()!='COMMAND_LONG' or (msg.target_system,msg.target_component)!=(1,1):return False
    if any(getattr(msg,f'param{i}')!=0 for i in range(3,8)) or msg.confirmation!=0:return False
    msgid=msg.param1
    if msg.command==510:return bool(safe and msgid in (0,*CANDIDATES) and msg.param2==0)
    if msg.command!=511 or msgid not in CANDIDATES or not safe or not locked or original is None:return False
    # 0 from this firmware's GET means no configured stream; -1 disables it.
    restore=original if original>0 else -1
    return bool(msg.param2 in (CANDIDATES[msgid],restore))


def rate_stats(times):
    gaps=[b-a for a,b in zip(times,times[1:])]
    return {'samples':len(times),'rate_hz':(len(times)-1)/(times[-1]-times[0]) if gaps else None,
            'max_arrival_gap_ms':max(gaps)*1000 if gaps else None}


def run(args):
    if not args.battery_off_usb_only_locked_confirmed or not FC_USB.exists() or args.output.exists():
        raise ValueError('Battery-off USB-only/locked confirmation, known USB and NEW output required')
    if subprocess.run(['fuser',str(Path(args.trs_port).resolve())],capture_output=True).returncode==0:
        raise ValueError('TRS occupied; no process killed')
    enc,dec=m.MAVLink(None,srcSystem=250,srcComponent=203),m.MAVLink(None)
    dec.robust_parsing=True
    ev,power=Evidence(250,203,expected_parameters=()),PassivePowerEvidence()
    start=time.monotonic();next_hb=next_version=0
    intervals={};acks=[];snapshots=[];tx=[];arrivals={i:[] for i in (0,*CANDIDATES)}
    original={};possibly_changed=[];restored={};error=None;restore_errors=[]
    buffer='';measuring=False;initial_heartbeat_interval=None
    def elapsed():return time.monotonic()-start
    def safe():
        return bool(ev.identity and ev.identity['uid2']==UID2 and ev.identity['firmware']=='1.14.3'
            and not ev.fatal and not power.reboot_observed and ev.heartbeats
            and not ev.heartbeats[-1]['armed'] and elapsed()-ev.heartbeats[-1]['elapsed_s']<2)
    def send(msg):
        orig=original.get(int(msg.param1)) if msg.get_type()=='COMMAND_LONG' else None
        locked=bool(snapshots and elapsed()-snapshots[-1]['at']<2 and snapshots[-1]['lockdown'])
        if not allowed(msg,safe=safe(),locked=locked,original=orig):raise RuntimeError('Telemetry-only send policy rejected message')
        packet=msg.pack(enc)
        if port.write(packet)!=len(packet):raise OSError('Short serial write')
        enc.seq=(enc.seq+1)%256
        tx.append({'at':elapsed(),'kind':msg.get_type(),'command':getattr(msg,'command',None),
                   'param1':getattr(msg,'param1',None),'param2':getattr(msg,'param2',None)})
    def pump():
        nonlocal next_hb,next_version,buffer
        t=elapsed()
        if not FC_USB.exists():raise RuntimeError('USB power mode changed')
        for msg in dec.parse_buffer(port.read(4096)) or []:
            ev.ingest(msg,t);power.ingest(msg,t)
            if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):continue
            if measuring and msg.get_msgId() in arrivals:arrivals[msg.get_msgId()].append(t)
            if msg.get_type()=='MESSAGE_INTERVAL':intervals[int(msg.message_id)]={'at':t,'interval':int(msg.interval_us)}
            elif msg.get_type()=='COMMAND_ACK' and msg.command in (510,511) and (msg.target_system,msg.target_component)==(250,203):
                acks.append({'at':t,'command':msg.command,'result':msg.result})
            elif msg.get_type()=='SERIAL_CONTROL':buffer+=bytes(msg.data[:msg.count]).decode('utf8','replace')
        if ev.fatal or power.reboot_observed:raise RuntimeError(ev.fatal or 'FC reboot')
        if ev.identity and (ev.identity['uid2']!=UID2 or ev.identity['firmware']!='1.14.3'):raise RuntimeError('Wrong FC')
        if t>=next_hb:
            send(enc.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0));next_hb=t+1
        if ev.heartbeats and ev.identity is None and t>=next_version:
            ev.version_requested_at=t;send(enc.command_long_encode(1,1,512,0,148,0,0,0,0,0,0));next_version=t+3
    def wait_safe():
        deadline=time.monotonic()+8
        while not safe():
            pump()
            if time.monotonic()>deadline:raise RuntimeError('No fresh disarmed identity')
    def check_lock():
        nonlocal buffer
        wait_safe();buffer='';send(shell_message(enc));deadline=time.monotonic()+4;poll=time.monotonic()+.3
        while time.monotonic()<deadline:
            pump()
            snap=parse_actuator_snapshot(buffer)
            if snap and reply_complete(buffer,LISTENER):
                boot=power.boot_samples[-1]['time_boot_ms'] if power.boot_samples else None
                if not state_valid(snap,boot) or not snap['lockdown']:raise RuntimeError('Expected fresh disarmed output lock')
                snapshots.append({**snap,'at':elapsed()});return
            if time.monotonic()>=poll:send(shell_message(enc,poll=True));poll=time.monotonic()+.3
        raise RuntimeError('Incomplete lock read')
    def get_interval(msgid):
        wait_safe();requested=elapsed();send(interval_message(enc,msgid));deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            pump();row=intervals.get(msgid)
            if row and row['at']>=requested:
                if not 0<=row['interval']<=60_000_000:raise RuntimeError('Unsupported original interval')
                return row['interval']
        raise RuntimeError('Interval read timeout: '+str(msgid))
    def set_interval(msgid,value):
        check_lock();requested=elapsed();send(interval_message(enc,msgid,value));deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            pump();matches=[x for x in acks if x['command']==511 and x['at']>=requested]
            if matches:
                if matches[-1]['result']!=m.MAV_RESULT_ACCEPTED:raise RuntimeError('Stream interval command rejected')
                break
        else:raise RuntimeError('Stream interval ACK timeout; not retried')
        # configure_stream_threadsafe is applied asynchronously; do not resend SET.
        until=time.monotonic()+.15
        while time.monotonic()<until:pump()
        actual=get_interval(msgid)
        if actual != (value if value>0 else 0):raise RuntimeError('Stream interval readback mismatch')
    try:
        with serial.Serial(args.trs_port,115200,timeout=.002,write_timeout=.3,exclusive=True) as port:
            port.reset_input_buffer()
            try:
                wait_safe();check_lock()
                initial_heartbeat_interval=get_interval(0)
                for msgid in CANDIDATES:original[msgid]=get_interval(msgid)
                for msgid,value in CANDIDATES.items():
                    possibly_changed.append(msgid);set_interval(msgid,value)
                measuring=True;until=time.monotonic()+10
                while time.monotonic()<until:
                    pump()
                    if not safe():raise RuntimeError('Heartbeat freshness lost; trial ends, no threshold relaxation')
                measuring=False;check_lock()
            except (Exception,KeyboardInterrupt) as exc:error=str(exc) or 'Interrupted'
            finally:
                measuring=False
                for msgid in reversed(possibly_changed):
                    try:
                        set_interval(msgid,original[msgid] if original[msgid]>0 else -1)
                        restored[msgid]=True
                    except Exception as exc:restore_errors.append(str(msgid)+': '+str(exc))
    except (Exception,KeyboardInterrupt) as exc:error=str(exc) or 'Interrupted'
    stats={str(i):rate_stats(t) for i,t in arrivals.items()}
    passed=bool(not error and not restore_errors and len(restored)==3 and
        all(stats[str(i)]['rate_hz'] is not None and stats[str(i)]['rate_hz']>=4 and
            stats[str(i)]['max_arrival_gap_ms']<=450 for i in CANDIDATES))
    result={'scope':__doc__,'captured_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'identity':ev.identity,'original_intervals_us':original,'requested_intervals_us':CANDIDATES,
        'heartbeat_interval_us_observed_only':initial_heartbeat_interval,'heartbeat_rate_writes':0,
        'observed_after_rate_changes':stats,'heartbeats':ev.heartbeats,'lock_snapshots':snapshots,
        'tx_messages':tx,'acks':acks,'restored':restored,'error':error,'restore_errors':restore_errors,
        'temporary_rate_trial_verified':passed,'parameter_writes':0,'pose_or_control_commands_sent':0,
        'flight_authorized':False,'serial_port_released':True}
    with args.output.open('x') as stream:json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({k:result[k] for k in ('original_intervals_us','observed_after_rate_changes',
        'restored','error','restore_errors','temporary_rate_trial_verified')},ensure_ascii=False,indent=2))
    return 0 if passed else 2


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trs-port',required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--battery-off-usb-only-locked-confirmed',action='store_true',required=True)
    a=ap.parse_args()
    try:return run(a)
    except (ValueError,OSError,RuntimeError) as exc:ap.exit(2,str(exc)+'\n')


if __name__=='__main__':raise SystemExit(main())
