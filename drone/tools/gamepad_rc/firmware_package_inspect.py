#!/usr/bin/env python3
"""离线解包厂商PX4文件，核对源码包头；不访问设备、不构造可烧录修复包。"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import struct
import zlib


def inspect(package, prototype, source_commit):
    if package.get('magic') != 'PX4FWv1':
        raise ValueError('Not a PX4 firmware package')
    data=zlib.decompress(base64.b64decode(package['image'],validate=True))
    if len(data)!=package['image_size'] or len(data)>package['image_maxsize'] or len(data)<8:
        raise ValueError('Firmware image size mismatch')
    sp,reset=struct.unpack_from('<II',data)
    if not reset&1 or not 0x08000000<=reset&~1<0x08200000:
        raise ValueError('Invalid Cortex-M firmware reset vector')
    report={key:package.get(key) for key in ('board_id','git_hash','git_identity','build_time','image_size','description')}
    report.update(image_sha256=hashlib.sha256(data).hexdigest(),initial_sp=hex(sp),reset_vector=hex(reset),
                  source_commit=source_commit,source_prototype_board_id=prototype.get('board_id'),
                  source_commit_matches=package.get('git_hash')==source_commit,
                  source_package_board_id_matches=package['board_id']==prototype.get('board_id'),
                  exact_vendor_build_reproduced=False,flash_allowed=False,
                  reason='Matching Git hash alone does not prove vendor worktree, toolchain or binary equivalence')
    return data,report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--package',type=Path,required=True)
    ap.add_argument('--prototype',type=Path,required=True)
    ap.add_argument('--source-commit',required=True)
    ap.add_argument('--output-dir',type=Path,required=True)
    args=ap.parse_args()
    if not args.output_dir.is_dir() or any((args.output_dir/n).exists() for n in ('vendor_original.bin','package_comparison.json')):
        ap.error('Existing directory and new output files required')
    data,report=inspect(json.loads(args.package.read_text()),json.loads(args.prototype.read_text()),args.source_commit)
    report['package_path']=str(args.package);report['prototype_path']=str(args.prototype)
    with (args.output_dir/'vendor_original.bin').open('xb') as f:f.write(data)
    with (args.output_dir/'package_comparison.json').open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False)
    print(json.dumps(report,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
