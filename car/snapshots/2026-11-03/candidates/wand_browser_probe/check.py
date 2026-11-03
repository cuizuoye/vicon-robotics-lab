#!/usr/bin/env python3
import argparse
import hashlib
import json
from pathlib import Path

def check(out,draft=False):
    m=json.loads((out/'build_manifest.json').read_text())
    if (m['readonly_only'] is not True or m['hardware_entry'] is not False or m['ros_entry'] is not False
            or m['authority_can_ever_exist'] is not False or m['strict_limit_ms']!=100 or m['strict_failure_reset'] is not False):
        raise ValueError('Observer boundary invalid')
    if not draft and m['prepared'] is not True:raise ValueError('Observer tests pending')
    for group in ('protected_hashes','artifact_hashes','source_hashes'):
        for p,h in m[group].items():
            if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h:raise ValueError('Hash mismatch '+p)
    if hashlib.sha256((out/'channel.py').read_bytes()).hexdigest()!=m['strict_channel_sha']:
        raise ValueError('Strict validator changed')
    return dict(passed=True,readonly_only=True,hardware_entry=False,ros_entry=False,strict_limit_ms=100)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--draft',action='store_true');a=ap.parse_args()
    print(json.dumps(check(Path(__file__).resolve().parent,a.draft)))
