"""Byte-level old-tier regression of two native C controllers, never devices."""
import ctypes
import json
import math
from pathlib import Path
import struct
import zlib
import sys
sys.path.insert(0,'/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_firmware_4wsc_2026-10-30/prepared_v4')
from stages import STAGES

HEADER=struct.Struct('<2sBBHIIIH')


class Oracle:
    def __init__(self, library, magic):
        self.lib=ctypes.CDLL(str(library));self.magic=magic;self.sequence=0
        self.lib.oracle_request.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint32,ctypes.c_void_p]
        self.lib.oracle_request.restype=ctypes.c_size_t
        self.lib.oracle_tick.argtypes=[ctypes.c_uint32]
        self.lib.oracle_packet.argtypes=[ctypes.c_uint,ctypes.c_void_p]
        self.lib.oracle_packet.restype=ctypes.c_size_t

    def send(self, kind, now, payload=b'', flags=0):
        self.sequence+=1
        raw=HEADER.pack(b'MC',1,kind,flags,self.sequence,19,now,len(payload))+payload
        raw+=struct.pack('<I',zlib.crc32(raw))
        src=ctypes.create_string_buffer(raw);dst=ctypes.create_string_buffer(512)
        self.lib.oracle_request(src,len(raw),now,dst)

    def velocity(self, now, values, enabled):
        self.send(0x10,now,struct.pack('<hhhHBB',*values,600,0,0),8 if enabled else 0)

    def init(self, stage):
        self.lib.oracle_init();self.sequence=0
        self.send(1,0)
        for now in range(10,601,10):
            self.velocity(now,(0,0,0),False);self.lib.oracle_tick(now)
        for enabled in (0,1):
            self.send(0x11,600,struct.pack('<BBH',enabled,0,self.magic+stage))

    def packet(self, kind):
        result=ctypes.create_string_buffer(512)
        n=self.lib.oracle_packet(kind,result)
        if not n: raise AssertionError('native oracle did not publish')
        return result.raw[:n]


def compare(old_library,new_library):
    old,new=Oracle(old_library,0x7C00),Oracle(new_library,0x7C00)
    frames=0
    for stage,s in enumerate(STAGES[:10]):
        old.init(stage);new.init(stage)
        for index in range(1500):
            t=index*.01;now=610+index*10
            values=(int(.65*s.speed*1000*math.cos(1.3*t)),
                    int(.65*s.speed*1000*math.sin(1.3*t)),
                    int(.65*s.yaw_rate*1000*math.sin(.8*t)))
            for oracle in (old,new):
                oracle.velocity(now,values,True);oracle.lib.oracle_tick(now)
            a,b=old.packet(0x30),new.packet(0x30)
            assert a[20:-4]==b[20:-4], ('wheel/control difference',stage,index)
            assert not struct.unpack_from('<I',a,20+32)[0], ('baseline fault',stage,index)
            a,b=old.packet(0x31)[20:-4],new.packet(0x31)[20:-4]
            assert b==a, ('diagnostic/output difference',stage,index)
            frames+=1
    return dict(passed=True,unchanged_stages=list(range(10)),
        compared_control_ticks=frames,per_tick_wheel_and_status_payload_equal=True,
        excluded_fields=[],
        synthetic_encoders_only=True,physical_motion=False)


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--old',type=Path,required=True);p.add_argument('--new',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=compare(a.old,a.new)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(result)
