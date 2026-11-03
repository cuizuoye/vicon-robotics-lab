"""Portable FILE-ONLY verification. Never import a driver, ROS or programmer."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def check_hex(path, binary, base):
    memory={};upper=0;eof=False
    for line in path.read_text().splitlines():
        if eof or not line.startswith(':'): raise ValueError('invalid Intel HEX record/order')
        record=bytes.fromhex(line[1:])
        if len(record)<5 or len(record)!=record[0]+5 or sum(record)%256:
            raise ValueError('Intel HEX length/checksum')
        n=record[0];address=int.from_bytes(record[1:3],'big');kind=record[3];data=record[4:-1]
        if kind==0:
            for index,value in enumerate(data):
                at=upper+address+index
                if at in memory or not base<=at<base+len(binary): raise ValueError('HEX overlap/range')
                memory[at]=value
        elif kind==4 and n==2 and address==0: upper=int.from_bytes(data,'big')<<16
        elif kind==5 and n==4 and address==0: pass  # ARM start address record.
        elif kind==1 and n==0 and address==0: eof=True
        else: raise ValueError('unsupported Intel HEX record')
    if not eof or not memory or min(memory)!=base or max(memory)!=base+len(binary)-1:
        raise ValueError('HEX image span mismatch')
    # objcopy binary fills inter-section holes with zero by default.
    rebuilt=bytearray(len(binary))
    for at,value in memory.items(): rebuilt[at-base]=value
    if rebuilt!=binary: raise ValueError('HEX contents differ from BIN')
    return dict(bytes=len(binary),hex_bin_equal=True,zero_filled_holes=len(binary)-len(memory))


def verify(folder):
    folder=folder.resolve()
    for line in (folder/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1)
        path=(folder/name).resolve()
        if not re.fullmatch('[0-9a-f]{64}',digest) or not path.is_relative_to(folder):
            raise ValueError('invalid checksum manifest path')
        if not path.is_file() or sha(path)!=digest: raise ValueError('file checksum mismatch: '+name)
    m=json.loads((folder/'BUNDLE.json').read_text())
    if m['identity']!='4WSB' or m['authority_version']!=7 or m['diagnostic_payload_bytes']!=208:
        raise ValueError('unexpected firmware contract')
    binary=(folder/m['bin']).read_bytes();base=int(m['flash_address'],16)
    if len(binary)!=m['image_bytes'] or len(binary)>65536: raise ValueError('image span/sector plan')
    stack,reset=struct.unpack_from('<II',binary)
    if not 0x20000000<stack<=0x20020000 or stack%8 or not reset&1 or not base<=(reset&~1)<base+len(binary):
        raise ValueError('STM32F407 vector table mismatch')
    result=check_hex(folder/m['hex'],binary,base)
    result.update(passed=True,identity='4WSB',image_sha256=sha(folder/m['bin']),
                  initial_stack=hex(stack),reset_handler=hex(reset),
                  firmware_flashed=False,hardware_opened=False,
                  physical_qualified=False,host_migration_complete=m['host_migration_complete'])
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--check',action='store_true')
    p.add_argument('--folder',type=Path,default=Path(__file__).resolve().parent)
    a=p.parse_args()
    print(json.dumps(verify(a.folder),ensure_ascii=False,indent=2))
