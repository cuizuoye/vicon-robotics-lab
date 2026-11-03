#!/usr/bin/env python3
"""Hash-only inspection: no ROS initialization, listeners or car sockets."""
import argparse
import hashlib
import json
from pathlib import Path

def check(folder,require_pass=True):
    m=json.loads((folder/'build_manifest.json').read_text())
    if m['hardware_entry'] is not False or m['controller_changes'] or m['allowed_modes']!=['readonly','software']:
        raise ValueError('Invalid offline-only candidate')
    if require_pass and not m.get('browser_ssh_passed'):raise ValueError('Offline validation not completed')
    for group in ('protected_hashes','source_hashes','artifact_hashes'):
        for name,expected in m[group].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=expected:raise ValueError('Hash mismatch: '+name)
    if hashlib.sha256(Path(m['dependencies']['ws_package']).read_bytes()).hexdigest()!=m['dependencies']['ws_sha256']:
        raise ValueError('WebSocket dependency changed')
    return dict(passed=True,hardware_entry=False,controller_changes=[],
        checked_files=sum(len(m[g]) for g in ('protected_hashes','source_hashes','artifact_hashes')))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--draft',action='store_true');args=ap.parse_args()
    print(json.dumps(check(Path(__file__).resolve().parent,not args.draft)))
