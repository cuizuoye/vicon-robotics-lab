"""Offline decoder/classification tests. No USB, socket, ROS or root actions."""
import json
from pathlib import Path
import socket
import struct
import pytest
import analyse_usb as a
from mecanum_vicon_control.transport_protocol import MessageType as M,encode_frame


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    def denied(*args,**kwargs):pytest.fail('offline USB analysis attempted device/network I/O')
    monkeypatch.setattr(socket,'socket',denied)


def frame_row():
    raw=encode_frame(M.HELLO_ACK,9,123,400,b'',2)
    udp=struct.pack('!HHHH',46000,46001,len(raw)+8,0)+raw
    ip=struct.pack('!BBHHHBBH4s4s',0x45,0,20+len(udp),1,0,64,17,0,a.SRC,a.DST)
    ip=ip[:10]+struct.pack('!H',a.checksum(ip))+ip[12:]
    mac=struct.pack('<HH',0x4288,0)+a.PC_MAC+a.AP_MAC+a.AP_MAC+struct.pack('<HH',32,0)
    mpdu=mac+b'\0'*8+b'\xaa\xaa\x03\0\0\0\x08\0'+ip+udp
    desc=struct.pack('<6I',len(mpdu)|(4<<16)|(4<<20),0,0,0,0,0xfffffff0)
    return dict(event='C',transfer=3,endpoint=0x84,status=0,payload=desc+b'\0'*32+mpdu,
        real_ns=1000000000,usb_header_ns=1000000000,record=1,urb_id=7)


def test_exact_inbound_packet_with_descriptor_and_checksums():
    r=a.incoming_mc(frame_row())
    assert len(r)==1 and r[0]['sequence']==9 and r[0]['session']==123
    assert r[0]['tsf_low_us']==0xfffffff0 and not r[0]['radio_retry']


@pytest.mark.parametrize('field,value',[('event','S'),('endpoint',8),('status',-71),('transfer',2)])
def test_not_usb_in_success_completion_is_not_native_rx(field,value):
    row=frame_row();row[field]=value
    assert not a.incoming_mc(row)


@pytest.mark.parametrize('mask',[1<<14,1<<15])
def test_descriptor_fcs_or_icv_error_never_proves_a_valid_early_arrival(mask):
    row=frame_row();data=bytearray(row['payload']);w0=struct.unpack_from('<I',data)[0]
    struct.pack_into('<I',data,0,w0|mask);row['payload']=bytes(data)
    assert not a.incoming_mc(row)


def test_corrupt_protocol_crc_is_not_correlated():
    row=frame_row();data=bytearray(row['payload']);data[-1]^=1;row['payload']=bytes(data)
    assert not a.incoming_mc(row)


def test_wrong_radio_receiver_is_not_correlated():
    row=frame_row();data=bytearray(row['payload']);data[60]^=1;row['payload']=bytes(data)
    assert not a.incoming_mc(row)


def test_tsf_wrap_is_relative_and_reset_does_not_become_huge_delay():
    assert a.delta32(0x20,0xfffffff0)==48
    with pytest.raises(ValueError):a.delta32(10,20)


def inputs_for_classification():
    late=[dict(sequence=7,kind=k,usb_relative_to_trip_ms=22.,usb_to_socket_ms=.04)
        for k in ('WHEEL_TELEMETRY','LINK_STATUS')]
    gaps=[dict(sequence=7,kind=k,source_step_ms=100,radio_rx_gap_ms=194.269,
        radio_to_usb_gap_difference_ms=.044) for k in ('WHEEL_TELEMETRY','LINK_STATUS')]
    return late,gaps


def test_radio_late_and_usb_fast_classifies_only_observed_boundary():
    late,gaps=inputs_for_classification()
    assert a.classify(late,gaps,[])=='DELAY_VISIBLE_AT_RADIO_RX_NOT_ADDED_BY_HOST_USB_DELIVERY'


@pytest.mark.parametrize('case',['early_usb','late_usb_after_radio','socket_slow','missing_frame','clock_inconsistent'])
def test_alternative_causes_never_get_the_same_positive_boundary_conclusion(case):
    late,gaps=inputs_for_classification();missing=[]
    if case=='early_usb':late[0]['usb_relative_to_trip_ms']=-100.
    elif case=='late_usb_after_radio':gaps[0]['radio_to_usb_gap_difference_ms']=94.
    elif case=='socket_slow':late[0]['usb_to_socket_ms']=100.
    elif case=='missing_frame':missing=[dict(sequence=7)]
    else:gaps[0]['source_step_ms']=200
    assert a.classify(late,gaps,missing)!='DELAY_VISIBLE_AT_RADIO_RX_NOT_ADDED_BY_HOST_USB_DELIVERY'


def test_all_real_window_packets_have_one_exact_usb_match(tmp_path):
    capture=a.ROOT/'docs/tenda_usb_capture_2026-10-27/capture_Fmpa7Bar/tenda_usb.pcap'
    diagnostic=a.ROOT/'docs/wand_return_path_trace_2026-10-27/return_path_7abqclg8'
    r=a.analyse(capture,diagnostic,tmp_path/'correlation')
    assert r['correlation_complete'] and not r['diagnostic_window_passed']
    assert r['actual_linux_packets']==r['uniquely_matched_usb_packets']==693
    assert r['usb_error_completions']==0 and not r['missing_or_ambiguous']
    assert r['usb_to_socket_max_ms']<2 and r['pending_rx_urbs_at_trip']==4
    assert r['classification']=='DELAY_VISIBLE_AT_RADIO_RX_NOT_ADDED_BY_HOST_USB_DELIVERY'
    assert {r['sequence'] for r in r['first_late_packets']}=={119923,119924}
    assert r['preserved']['feedback_protection_ms']==200
