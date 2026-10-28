#!/usr/bin/env python3
"""OFFLINE usbmon/native/socket correlation; opens no device or network.

RX descriptor interpretation checked against upstream Linux v7.0 rx.c and
rx.h, and the installed 7.0 headers' mactime contract. TSF differences are
relative device-clock measurements, NOT absolute host-clock timestamps.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import statistics
import struct
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0,str(ROOT/'src/mecanum_vicon_control'))
from mecanum_vicon_control.transport_protocol import decode_frame,encode_frame,MessageType,ProtocolError

USB=struct.Struct('<QBBBBHBBqiiII8siiII')
SRC=bytes((192,168,50,1));DST=bytes((192,168,50,2))
PC_MAC=bytes.fromhex('502b73a508c9')
AP_MAC=bytes.fromhex('98c377ad5db9')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')


def records(path):
    with Path(path).open('rb') as stream:
        header=stream.read(24)
        if len(header)!=24 or struct.unpack('<I',header[:4])[0]!=0xa1b2c3d4:
            raise ValueError('expected little-endian microsecond classic pcap')
        if struct.unpack_from('<I',header,20)[0]!=220:raise ValueError('USB mmap DLT required')
        number=0
        while True:
            h=stream.read(16)
            if not h:return
            if len(h)!=16:raise ValueError('partial record header; capture not finalized')
            sec,usec,size,original=struct.unpack('<IIII',h)
            if size>245824 or size>original or not 0<=usec<1000000:raise ValueError('pcap length/timestamp')
            data=stream.read(size)
            if len(data)!=size or len(data)<64:raise ValueError('truncated USB capture')
            u=USB.unpack_from(data);number+=1
            if 64+u[12]>len(data):raise ValueError('truncated USB transfer payload')
            yield dict(record=number,real_ns=sec*10**9+usec*1000,urb_id=u[0],event=chr(u[1]),
                transfer=u[2],endpoint=u[3],device=u[4],bus=u[5],status=u[10],
                usb_header_ns=u[8]*10**9+u[9]*1000,payload=data[64:64+u[12]])


def checksum(data):
    if len(data)%2:data+=b'\0'
    value=sum(struct.unpack('!'+str(len(data)//2)+'H',data))
    while value>>16:value=(value&65535)+(value>>16)
    return (~value)&65535


def incoming_mc(row):
    if row['event']!='C' or row['transfer']!=3 or row['endpoint']!=0x84 or row['status']:
        return []
    data=row['payload']
    if len(data)<24:return []
    w0,w1,w2,w3,w4,tsf=struct.unpack_from('<6I',data)
    if w0&(3<<14) or w2&(1<<28):return [] # CRC/ICV failure or firmware event.
    packet_len=w0&0x3fff;head=24+((w0>>16)&15)*8+((w0>>24)&3)
    end=head+packet_len
    if end>len(data) or head+24>end:return []
    fc=struct.unpack_from('<H',data,head)[0]
    if (fc&0x0c)!=8 or fc&0x0300!=0x0200:return []
    if data[head+4:head+10]!=PC_MAC or data[head+10:head+16]!=AP_MAC:return []
    out=[];at=head+24
    while True:
        at=data.find(b'MC\x01',at,end)
        if at<0:break
        start=at;at+=1
        if start+20>end or start<28:continue
        size=struct.unpack_from('<H',data,start+18)[0]+24
        if start+size>end:continue
        raw=data[start:start+size]
        try:frame=decode_frame(raw)
        except (ProtocolError,ValueError):continue
        # Enforce actual direction, complete IPv4/UDP and both checksums. This
        # excludes ICMP quotations and accidental magic/CRC byte patterns.
        udp_start=start-8
        sport,dport,udp_size,udp_check=struct.unpack_from('!HHHH',data,udp_start)
        if (sport,dport,udp_size)!=(46000,46001,size+8):continue
        ip_start=start-28;ip=data[ip_start:udp_start]
        if (ip[0]!=0x45 or ip[9]!=17 or ip[12:16]!=SRC or ip[16:20]!=DST or checksum(ip)
            or struct.unpack_from('!H',ip,2)[0]!=20+udp_size):continue
        pseudo=SRC+DST+bytes((0,17))+struct.pack('!H',udp_size)
        if udp_check and checksum(pseudo+data[udp_start:start+size]):continue
        out.append(dict(record=row['record'],usb_real_ns=row['real_ns'],
            usb_header_ns=row['usb_header_ns'],urb_id=row['urb_id'],kind=MessageType(frame.message_type).name,
            sequence=frame.sequence,session=frame.session_id,source_ms=frame.sender_time_ms,
            raw_hex=raw.hex(),tsf_low_us=tsf,radio_retry=bool(fc&0x0800),
            wifi_sequence=struct.unpack_from('<H',data,head+22)[0]>>4,
            radio_rate_code=w3&127,descriptor_payload_offset=start,
            descriptor_crc_error=False,descriptor_icv_error=False))
    return out


def delta32(new,old):
    value=(new-old)&0xffffffff
    if value>=0x80000000:raise ValueError('device TSF reordered/reset beyond wrap-safe interval')
    return value


def classify(first_late,gaps,missing):
    if missing or len(first_late)!=2:return 'INSUFFICIENT_CORRELATION'
    selected=[g for g in gaps if any(g['sequence']==p['sequence'] and g['kind']==p['kind'] for p in first_late)]
    if len(selected)!=2:return 'INSUFFICIENT_RADIO_GAPS'
    if all(p['usb_relative_to_trip_ms']>0 and p['usb_to_socket_ms']<5 for p in first_late) and all(
        g['source_step_ms']==100 and g['radio_rx_gap_ms']>150 and abs(g['radio_to_usb_gap_difference_ms'])<5 for g in selected):
        return 'DELAY_VISIBLE_AT_RADIO_RX_NOT_ADDED_BY_HOST_USB_DELIVERY'
    return 'INDETERMINATE_DO_NOT_INFER_COMPONENT'


def analyse(capture,diagnostic,out):
    capture=Path(capture);diagnostic=Path(diagnostic);out=Path(out)
    out.mkdir(exist_ok=False,parents=True)
    inputs={str(capture):sha(capture)}
    for name in ('kernel_receive.json','RESULT.json','outgoing.json'):
        inputs[str(diagnostic/name)]=sha(diagnostic/name)
    native=json.loads((diagnostic/'kernel_receive.json').read_text())
    result=json.loads((diagnostic/'RESULT.json').read_text())
    offsets=[r['realtime_monotonic_offset_ns'] for r in native]
    clock_offset=statistics.median(offsets);trip_ns=round(result['trip']['at']*1e9)+clock_offset
    rows=list(records(capture));hits=collections.defaultdict(list);all_mc=[]
    errors=[];identity=set();pending={};pending_at_trip=None;near_counts=collections.Counter()
    total=collections.Counter()
    for r in rows:
        identity.add((r['bus'],r['device']))
        if (r['bus'],r['device'])!=(3,7):raise ValueError('unexpected USB device in private capture')
        if r['real_ns']!=r['usb_header_ns']:raise ValueError('pcap/usbmon time mismatch')
        total[(r['event'],r['transfer'],r['endpoint'],r['status'])]+=1
        if r['event']=='C' and r['status']:errors.append({k:v for k,v in r.items() if k!='payload'})
        if abs(r['real_ns']-trip_ns)<200000000:
            near_counts[(r['event'],r['transfer'],r['endpoint'],r['status'])]+=1
        if pending_at_trip is None and r['real_ns']>trip_ns:pending_at_trip=len(pending)
        if r['transfer']==3 and r['endpoint']==0x84:
            if r['event']=='S':pending[r['urb_id']]=r['real_ns']
            elif r['event']=='C':pending.pop(r['urb_id'],None)
        for f in incoming_mc(r):
            all_mc.append(f);hits[f['raw_hex']].append(f)
    aligned=[];missing=[]
    for r in native:
        raw=encode_frame(MessageType[r['kind']],r['sequence'],r['session'],r['source_ms'],
            bytes.fromhex(r['payload_hex']),r['flags'])
        matches=hits.get(raw.hex(),[])
        if len(matches)!=1:
            missing.append(dict(kind=r['kind'],sequence=r['sequence'],matches=len(matches)));continue
        usb=matches[0]
        aligned.append(dict(usb,linux_kernel_real_ns=r['kernel_real_ns'],
            usb_to_socket_ms=(r['kernel_real_ns']-usb['usb_real_ns'])/1e6,
            socket_to_ros_ms=r['kernel_to_ros_ms'],
            usb_relative_to_trip_ms=(usb['usb_real_ns']-trip_ns)/1e6,
            socket_relative_to_trip_ms=(r['kernel_real_ns']-trip_ns)/1e6,
            after_tx_stop=r['after_tx_stop']))
    aligned.sort(key=lambda r:r['usb_real_ns'])
    gaps=[]
    for kind in ('WHEEL_TELEMETRY','LINK_STATUS'):
        stream=[r for r in aligned if r['kind']==kind]
        for a,b in zip(stream,stream[1:]):
            radio=delta32(b['tsf_low_us'],a['tsf_low_us'])/1000
            usb=(b['usb_real_ns']-a['usb_real_ns'])/1e6
            gaps.append(dict(kind=kind,previous_sequence=a['sequence'],sequence=b['sequence'],
                source_step_ms=delta32(b['source_ms'],a['source_ms']),radio_rx_gap_ms=radio,
                usb_rx_gap_ms=usb,radio_to_usb_gap_difference_ms=usb-radio,
                usb_relative_to_trip_ms=b['usb_relative_to_trip_ms'],
                radio_retry=b['radio_retry'],wifi_sequence=b['wifi_sequence']))
    near=[r for r in aligned if -300<=r['usb_relative_to_trip_ms']<=300]
    late=[r for r in aligned if r['after_tx_stop']]
    first_late=late[:2]
    near_gap=[g for g in gaps if -250<=g['usb_relative_to_trip_ms']<=150]
    deliveries=[r['usb_to_socket_ms'] for r in aligned]
    classification=classify(first_late,gaps,missing)
    summary=dict(correlation_complete=not missing,diagnostic_window_passed=result['completed'],
        diagnostic_termination=result['error'],diagnostic_elapsed_s=result['trip']['elapsed_s'],
        classification=classification,scope='OFFLINE correlation; no additional motion or live diagnostic',
        capture=str(capture),diagnostic=str(diagnostic),input_sha256=inputs,
        capture_records=len(rows),device_identities=[list(x) for x in sorted(identity)],
        usb_error_completions=len(errors),note='SUBMIT status -115 is pending, not a USB completion failure',
        actual_linux_packets=len(native),uniquely_matched_usb_packets=len(aligned),missing_or_ambiguous=missing,
        socket_to_ros_max_ms=max(r['socket_to_ros_ms'] for r in aligned),
        usb_to_socket_median_ms=statistics.median(deliveries),usb_to_socket_max_ms=max(deliveries),
        usb_to_socket_min_ms=min(deliveries),pending_rx_urbs_at_trip=pending_at_trip,
        trip_source_ages_ms=result['trip']['source_ages_ms'],
        first_late_packets=first_late,trip_near_gaps=near_gap,
        hardware_clock_interpretation=dict(
            source='https://github.com/torvalds/linux/blob/v7.0/drivers/net/wireless/realtek/rtw88/rx.c',
            descriptor_header_source='https://github.com/torvalds/linux/blob/v7.0/drivers/net/wireless/realtek/rtw88/rx.h',
            installed_contract='/usr/src/linux-hwe-7.0-headers-7.0.0-38/include/net/mac80211.h',
            field='descriptor word5 TSFL, used as MAC receive-start time in microseconds',
            limits='Only modulo-safe relative gaps used. No absolute TSF-to-host or MCU-to-radio one-way latency is claimed. Upstream v7.0 ABI is consistent with captured framing; Ubuntu-specific binary is not source-attested.'),
        usb_events_near_trip=[dict(event=k[0],transfer=k[1],endpoint=hex(k[2]),status=k[3],count=v)
            for k,v in sorted(near_counts.items())],
        preserved=dict(feedback_protection_ms=200,firmware_changed=False,network_changed=False,
            motion_started=False,operator_console_started=False),
        conclusions=([
            'Delayed packets completed USB reception only after the safety trip.',
            'Device RX timestamp gaps track USB gaps at failure; no comparable extra 100ms USB-after-radio hold is visible in this sample.',
            'Further separation needs ESP UART/UDP submission and air-delivery evidence. This does not prove that every part of the Tenda hardware/radio is fault-free.'
        ] if classification=='DELAY_VISIBLE_AT_RADIO_RX_NOT_ADDED_BY_HOST_USB_DELIVERY' else [
            'The timing data do not meet the conservative component-boundary classification rule; inspect raw evidence before drawing a conclusion.'
        ]))
    save(out/'USB_NATIVE_MATCHES.json',aligned);save(out/'RADIO_USB_GAPS.json',gaps)
    save(out/'TRIP_PACKETS.json',near);save(out/'USB_ERRORS.json',errors);save(out/'USB_CORRELATION_RESULT.json',summary)
    if any(sha(p)!=v for p,v in inputs.items()):raise ValueError('capture inputs changed during analysis')
    return summary


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--capture',type=Path,required=True)
    ap.add_argument('--diagnostic',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();result=analyse(args.capture,args.diagnostic,args.output)
    print(json.dumps({k:v for k,v in result.items() if k not in
        ('first_late_packets','trip_near_gaps','usb_events_near_trip','input_sha256')},ensure_ascii=False,indent=2))
