#!/usr/bin/env python3
"""Offline libpcap filter check; never opens USB or any network interface."""
import argparse
import ctypes as C
import ctypes.util
import json
import os
import struct
import subprocess
import tempfile
from pathlib import Path


def expression(bus,device):
    if type(bus) is not int or not 1<=bus<=65535 or type(device) is not int or not 1<=device<=127:
        raise ValueError('USB bus/device range')
    # Installed libpcap USB_LINUX_MMAPPED: dev_addr at 11, LE bus_id at 12.
    return f'link[11] = {device} and link[12] = {bus&255} and link[13] = {bus>>8}'


def packet(bus,device):
    # 64-byte usbmon mmap header. A bulk-IN completion with no data.
    return struct.pack('<QBBBBHBBqiiII8siiII',1,ord('C'),3,0x84,device,bus,
        0,0,1,0,0,0,0,b'\0'*8,0,0,0,0)


def verify(bus,device):
    p=C.CDLL(ctypes.util.find_library('pcap'))
    class Program(C.Structure):_fields_=[('length',C.c_uint),('instructions',C.c_void_p)]
    class Timeval(C.Structure):_fields_=[('sec',C.c_long),('usec',C.c_long)]
    class Header(C.Structure):_fields_=[('ts',Timeval),('caplen',C.c_uint),('length',C.c_uint)]
    p.pcap_datalink_name_to_val.argtypes=[C.c_char_p];p.pcap_datalink_name_to_val.restype=C.c_int
    dlt=p.pcap_datalink_name_to_val(b'USB_LINUX_MMAPPED')
    if dlt!=220:raise ValueError('unexpected installed USB mmap DLT')
    p.pcap_open_dead.argtypes=[C.c_int,C.c_int];p.pcap_open_dead.restype=C.c_void_p
    p.pcap_compile.argtypes=[C.c_void_p,C.POINTER(Program),C.c_char_p,C.c_int,C.c_uint]
    p.pcap_geterr.argtypes=[C.c_void_p];p.pcap_geterr.restype=C.c_char_p
    p.pcap_offline_filter.argtypes=[C.POINTER(Program),C.POINTER(Header),C.c_void_p]
    p.pcap_offline_filter.restype=C.c_uint
    p.pcap_freecode.argtypes=[C.POINTER(Program)];p.pcap_close.argtypes=[C.c_void_p]
    handle=p.pcap_open_dead(dlt,262144);program=Program();expr=expression(bus,device)
    try:
        if p.pcap_compile(handle,C.byref(program),expr.encode(),1,0xffffffff):
            raise ValueError(p.pcap_geterr(handle).decode())
        cases=0
        for test_bus in {bus,1 if bus!=1 else 2,256 if bus!=256 else 257}:
            for test_device in range(1,128):
                raw=packet(test_bus,test_device);buf=C.create_string_buffer(raw)
                header=Header(Timeval(1,0),len(raw),len(raw))
                accepted=bool(p.pcap_offline_filter(C.byref(program),C.byref(header),buf))
                if accepted!=((test_bus,test_device)==(bus,device)):
                    raise ValueError('USB isolation filter mismatch')
                cases+=1
    finally:
        p.pcap_freecode(C.byref(program));p.pcap_close(handle)
    # Independent check of offsets via the actual installed tcpdump decoder.
    with tempfile.TemporaryDirectory(prefix='usb_filter_offline_') as folder:
        capture=Path(folder)/'fixture.pcap';raw=packet(bus,device)
        capture.write_bytes(struct.pack('<IHHIIII',0xa1b2c3d4,2,4,0,0,262144,dlt)+
            struct.pack('<IIII',1,0,len(raw),len(raw))+raw)
        # This root-only option is for the SELF-GENERATED OFFLINE fixture, not
        # the real capture (which explicitly drops to czy). A root-created
        # private temporary directory must remain readable by its decoder.
        privilege=['-Z','root'] if os.geteuid()==0 else []
        r=subprocess.run(['/usr/bin/tcpdump',*privilege,'-nn','-t','-r',str(capture)],
            capture_output=True,text=True,check=True)
        if f'{bus}:{device}:' not in r.stdout:
            raise ValueError('tcpdump decoder disagrees with USB header offsets: '+r.stdout)
    return dict(passed=True,offline_only=True,filter=expr,synthetic_bus_device_cases=cases)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--bus',type=int,required=True)
    ap.add_argument('--device',type=int,required=True);a=ap.parse_args()
    print(json.dumps(verify(a.bus,a.device)))
