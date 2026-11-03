#!/usr/bin/env python3
"""Wi-Fi-only zero-frame echo; never emits MCU motor commands or fake feedback."""
import argparse
import json
from pathlib import Path
import secrets
import struct
import time

from four_link_ro_probe import Transport
from link_timing import SerialLogCapture, esp_ipv4_from_logs
from mecanum_vicon_control.transport_protocol import encode_frame, pack_cmd_vel, MessageType, FrameFlags

STATUS=struct.Struct('<7I')
TAG=0x31454657
HEALTH_STATUS=struct.Struct('<11IiI')
HEALTH_TAG=0x32454657
RADIO_STATUS=struct.Struct('<11IiI7I')
RADIO_TAG=0x33454657

def wait_for_hello_ack(transport,sid,sequence,timeout=4.0,clock=time.monotonic,sleep=time.sleep):
    start=clock();deadline=start+timeout
    while clock()<deadline:
        for frame in transport.receive():
            if frame.session_id!=sid:continue
            if (frame.message_type!=MessageType.HELLO_ACK or frame.sequence!=sequence
                or frame.flags!=FrameFlags.RESPONSE or frame.payload):
                raise ValueError('Invalid local echo HELLO_ACK before command stream')
            return (clock()-start)*1000
        sleep(0.001)
    raise TimeoutError('HELLO_ACK missing; no commands sent')

def decode_echo(frame):
    schemas={STATUS.size:(STATUS,TAG),HEALTH_STATUS.size:(HEALTH_STATUS,HEALTH_TAG),
             RADIO_STATUS.size:(RADIO_STATUS,RADIO_TAG)}
    if frame.message_type!=MessageType.LINK_STATUS or frame.flags!=FrameFlags.RESPONSE or len(frame.payload) not in schemas:
        raise ValueError('Not Wi-Fi-only echo feedback')
    schema,expected_tag=schemas[len(frame.payload)];values=schema.unpack(frame.payload)
    tag,seq,pc_ms,rx_ms,gap,loop_max,failures=values[:7]
    if tag!=expected_tag or seq!=frame.sequence or rx_ms!=frame.sender_time_ms:
        raise ValueError('Invalid Wi-Fi-only diagnostic schema')
    result=dict(sequence=seq,pc_sender_ms=pc_ms,esp_rx_ms=rx_ms,
                esp_rx_gap_ms=gap,esp_loop_max_gap_ms=loop_max,send_failures=failures)
    if tag in (HEALTH_TAG,RADIO_TAG):
        result.update(zip(('last_errno','internal_free','internal_largest','ps_type','rssi','disconnects'),values[7:]))
    if tag==RADIO_TAG:
        result.update(zip(('radio_count','radio_last_callback_ms','radio_max_callback_gap_ms',
            'radio_last_hardware_us','radio_max_hardware_gap_us','radio_last_sequence','radio_retries'),values[13:]))
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bind-ip',required=True);p.add_argument('--interface',required=True)
    p.add_argument('--esp-log-serial',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wait-hello-ack',action='store_true',help='Do not stream until validated local echo ACK')
    p.add_argument('--seconds',type=float,default=15)
    p.add_argument('--confirm-battery-disconnected',action='store_true',required=True)
    a=p.parse_args()
    if not 2<=a.seconds<=30:p.error('--seconds must be within 2..30')
    if a.output.exists():raise ValueError('Refuse to overwrite prior evidence')
    origin=time.monotonic();log=SerialLogCapture(a.esp_log_serial,origin);transport=None
    record=dict(mode='Wi-Fi-only WFE diagnostics, no MCU telemetry',motor_commands_enabled=False,
                battery_disconnected_confirmed=True,sent=[],echo=[],hello_ack=False,
                wait_hello_ack=a.wait_hello_ack,requested_duration_s=a.seconds)
    try:
        until=time.monotonic()+12
        while not any('WIFI_ECHO_ONLY_READY' in l['text'] for l in log.lines):
            if log.error:raise RuntimeError(log.error)
            if time.monotonic()>=until:raise RuntimeError('Echo-only firmware not ready; no UDP sent')
            time.sleep(0.05)
        time.sleep(5)
        a.esp_ip=esp_ipv4_from_logs(log.lines);a.serial=None
        record.update(esp_ip=a.esp_ip,pc_ip=a.bind_ip)
        transport=Transport(a)
        sid=secrets.randbits(32) or 1;record['session_id']=sid;seq=0;sent_by_seq={}
        def send(kind,payload=b'',flags=0):
            nonlocal seq
            seq+=1;started=time.monotonic()-origin;ms=int(started*1000)&0xffffffff
            transport.send(encode_frame(kind,seq,sid,ms,payload,flags))
            s=dict(sequence=seq,kind=int(kind),start_s=started,end_s=time.monotonic()-origin,sender_ms=ms)
            record['sent'].append(s);sent_by_seq[seq]=s
        send(MessageType.HELLO,flags=FrameFlags.ACK_REQUIRED)
        if a.wait_hello_ack:
            record['hello_ack_wait_ms']=wait_for_hello_ack(transport,sid,seq)
            record['hello_ack']=True
        end=time.monotonic()+a.seconds;next_send=time.monotonic()
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_send:
                send(MessageType.CMD_VEL,pack_cmd_vel(0,0,0,timeout_ms=150));next_send=now+0.02
            for frame in transport.receive():
                if frame.session_id!=sid:continue
                if frame.message_type==MessageType.HELLO_ACK:
                    if frame.sequence!=1 or frame.flags!=FrameFlags.RESPONSE or frame.payload:
                        raise ValueError('Invalid local echo HELLO_ACK')
                    record['hello_ack']=True;continue
                e=decode_echo(frame);s=sent_by_seq.get(e['sequence'])
                if not s or s['sender_ms']!=e['pc_sender_ms']:raise ValueError('Echo is not a sent command')
                e.update(host_receive_s=time.monotonic()-origin,
                         rtt_ms=(time.monotonic()-origin-s['start_s'])*1000)
                record['echo'].append(e)
            time.sleep(0.001)
        commands=[s for s in record['sent'] if s['kind']==int(MessageType.CMD_VEL)]
        echoes=record['echo'];seen={e['sequence'] for e in echoes}
        gaps=[(b['start_s']-a['start_s'])*1000 for a,b in zip(commands,commands[1:])]
        record['summary']=dict(sent_commands=len(commands),echo_frames=len(echoes),
            missing_echo_sequences=[s['sequence'] for s in commands if s['sequence'] not in seen],
            max_host_send_gap_ms=max(gaps,default=None),
            max_esp_rx_gap_ms=max((e['esp_rx_gap_ms'] for e in echoes),default=None),
            max_esp_loop_gap_ms=max((e['esp_loop_max_gap_ms'] for e in echoes),default=None),
            max_rtt_ms=max((e['rtt_ms'] for e in echoes),default=None),
            send_failures=max((e['send_failures'] for e in echoes),default=None))
        health=[e for e in echoes if 'internal_free' in e]
        if health:
            record['summary']['health']=dict(
                min_internal_free=min(e['internal_free'] for e in health),
                min_internal_largest=min(e['internal_largest'] for e in health),
                observed_errno=sorted({e['last_errno'] for e in health}),
                observed_ps=sorted({e['ps_type'] for e in health}),
                rssi_range=[min(e['rssi'] for e in health),max(e['rssi'] for e in health)],
                disconnects=max(e['disconnects'] for e in health))
        radio=[e for e in echoes if 'radio_count' in e]
        if radio:
            record['summary']['radio']=dict(
                frame_count=radio[-1]['radio_count'],retries=radio[-1]['radio_retries'],
                max_callback_gap_ms=max(e['radio_max_callback_gap_ms'] for e in radio),
                max_hardware_gap_us=max(e['radio_max_hardware_gap_us'] for e in radio),
                note='Own PC to STA data metadata, not one-to-one UDP mapping; observer may affect timing')
        record['roundtrip_completed']=bool(record['hello_ack'] and len(echoes)>=10)
        record['receive_gap_below_150ms']=bool(echoes and all(e['esp_rx_gap_ms']<150 for e in echoes))
        record['motion_acceptance']=False
        print(json.dumps({k:v for k,v in record.items() if k not in ('sent','echo')}))
    except Exception as error:
        record['error']=str(error);raise
    finally:
        if transport:transport.close()
        record['esp_serial']=log.finish()
        a.output.write_text(json.dumps(record,indent=2)+'\n')

if __name__=='__main__':main()
