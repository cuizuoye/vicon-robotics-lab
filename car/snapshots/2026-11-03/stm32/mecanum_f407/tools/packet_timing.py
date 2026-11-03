#!/usr/bin/env python3
"""Parse bounded classic PCAP and correlate MC-Link sequences, no capture/writes."""
import argparse
import ipaddress
import json
from pathlib import Path
import struct
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/mecanum_vicon_control'))
from mecanum_vicon_control.transport_protocol import (
    decode_frame, unpack_cmd_vel, MessageType, FrameFlags, ProtocolError,
)


def pcap_records(data):
    formats={b'\xd4\xc3\xb2\xa1':('<',1e6),b'\xa1\xb2\xc3\xd4':('>',1e6),
             b'\x4d\x3c\xb2\xa1':('<',1e9),b'\xa1\xb2\x3c\x4d':('>',1e9)}
    if len(data)<24 or data[:4] not in formats: raise ValueError('Unsupported PCAP header')
    endian,scale=formats[data[:4]]
    major,minor,_,_,snap,link=struct.unpack_from(endian+'HHIIII',data,4)
    if (major,minor)!=(2,4) or link not in (1,101,113,276):
        raise ValueError('Unsupported PCAP version/link type')
    offset=24
    while offset<len(data):
        if len(data)-offset<16: raise ValueError('Truncated PCAP record header')
        sec,sub,n,original=struct.unpack_from(endian+'IIII',data,offset);offset+=16
        if n>snap or n>original or sub>=scale or len(data)-offset<n:
            raise ValueError('Invalid or truncated PCAP record')
        yield sec+sub/scale,link,data[offset:offset+n]
        offset+=n


def ipv4_udp(link, packet):
    if link==1:
        if len(packet)<14: return None
        protocol=struct.unpack_from('!H',packet,12)[0];off=14
        while protocol in (0x8100,0x88a8):
            if len(packet)<off+4: return None
            protocol=struct.unpack_from('!H',packet,off+2)[0];off+=4
    elif link==113:
        if len(packet)<16:return None
        protocol=struct.unpack_from('!H',packet,14)[0];off=16
    elif link==276:
        if len(packet)<20:return None
        protocol=struct.unpack_from('!H',packet,0)[0];off=20
    else:
        protocol=0x800;off=0
    if protocol!=0x800 or len(packet)<off+20:return None
    ip=packet[off:];ihl=(ip[0]&15)*4
    if ip[0]>>4!=4 or ihl<20 or len(ip)<ihl+8 or ip[9]!=17:return None
    # No fragment reassembly: an incomplete datagram is not valid MC-Link evidence.
    if struct.unpack_from('!H',ip,6)[0]&0x3fff:return None
    total=struct.unpack_from('!H',ip,2)[0]
    src=str(ipaddress.IPv4Address(ip[12:16]));dst=str(ipaddress.IPv4Address(ip[16:20]))
    sport,dport,length=struct.unpack_from('!HHH',ip,ihl)
    if length<8 or total<ihl+length or len(ip)<ihl+length:return None
    return src,dst,sport,dport,ip[ihl+8:ihl+length]


def analyze(data, probe, pc_ip, esp_ip):
    packets=[];invalid=0
    sessions={x['session_id'] for x in probe['sent']}
    for at,link,packet in pcap_records(data):
        udp=ipv4_udp(link,packet)
        if udp is None:continue
        src,dst,sport,dport,payload=udp
        if {src,dst}!={pc_ip,esp_ip} or not ({sport,dport}&{46000,46001}):continue
        try:f=decode_frame(payload)
        except ProtocolError:invalid+=1;continue
        if f.session_id not in sessions:continue
        packets.append(dict(at_epoch_s=at,direction='out' if src==pc_ip else 'in',
                            kind=int(f.message_type),sequence=f.sequence,session_id=f.session_id))
        if src==pc_ip:
            if f.message_type not in (MessageType.HELLO,MessageType.CMD_VEL):
                raise ValueError('Unexpected outbound command type in readonly capture')
            if f.message_type==MessageType.CMD_VEL:
                c=unpack_cmd_vel(f.payload)
                if f.flags or c.vx_mps or c.vy_mps or c.wz_radps or c.control_mode:
                    raise ValueError('Outbound command was not disabled zero velocity')
    commands=[p for p in packets if p['direction']=='out' and p['kind']==int(MessageType.CMD_VEL)]
    gaps=[b['at_epoch_s']-a['at_epoch_s'] for a,b in zip(commands,commands[1:])
          if a['session_id']==b['session_id']]
    sent={(s['session_id'],s['sequence']):s for s in probe['sent'] if s['kind']==int(MessageType.CMD_VEL)}
    seen={(s['session_id'],s['sequence']):s for s in commands}
    pairs=[]
    events=probe['timing']['esp_events'];rx=[e for e in events if e['kind']==1]
    fault=probe['timing'].get('esp_fault')
    if fault:
        sid=fault['session_id']
        for a,b in zip(rx,rx[1:]):
            pa,pb=seen.get((sid,a['sequence'])),seen.get((sid,b['sequence']))
            if pa and pb:
                pairs.append(dict(from_sequence=a['sequence'],to_sequence=b['sequence'],
                    capture_gap_ms=(pb['at_epoch_s']-pa['at_epoch_s'])*1000,
                    esp_gap_ms=(b['at_ms']-a['at_ms'])&0xffffffff))
    return dict(captured_packets=packets,invalid_protocol_packets=invalid,
                captured_zero_commands=len(commands),
                max_capture_command_gap_ms=max(gaps,default=0)*1000 if gaps else None,
                sent_but_not_captured_sequences=[list(k) for k in sent if k not in seen],
                matched_receive_intervals=pairs,
                capture_available=bool(commands),
                interpretation='PC interface capture is a kernel observation point, NOT proof of radio transmission or AP receipt. Compare same-device intervals and matching sequences; clocks are not synchronized.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pcap',type=Path,required=True)
    p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--pc-ip',required=True);p.add_argument('--esp-ip',required=True)
    a=p.parse_args()
    if a.pcap.stat().st_size>20*1024*1024:raise ValueError('Capture exceeds bounded size')
    result=analyze(a.pcap.read_bytes(),json.loads(a.probe.read_text()),a.pc_ip,a.esp_ip)
    print(json.dumps(result,indent=2))
    if not result['capture_available']:raise SystemExit(2)

if __name__=='__main__':main()
