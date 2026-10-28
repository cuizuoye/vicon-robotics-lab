#!/usr/bin/env python3
import argparse
import hashlib
import json
from pathlib import Path

def check(out,draft=False):
    m=json.loads((out/'build_manifest.json').read_text())
    if (m['hardware_entry'] is not False or m['controller_changes'] or m['proof_limit_ms']!=100
            or m['control_gap_ms']!=100 or m['allowed_modes']!=['readonly'] or m['software_entry_enabled'] is not False):
        raise ValueError('Invalid offline fast candidate')
    if not draft and not m['browser_fast_passed']:raise ValueError('Offline fast validation pending')
    for group in ('protected_hashes','source_hashes','artifact_hashes'):
        for name,h in m[group].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=h:raise ValueError('Hash mismatch: '+name)
    return dict(passed=True,hardware_entry=False,controller_changes=[],proof_limit_ms=100,
        checked_files=sum(len(m[g]) for g in ('protected_hashes','source_hashes','artifact_hashes')))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--draft',action='store_true');a=ap.parse_args()
    print(json.dumps(check(Path(__file__).resolve().parent,a.draft)))
