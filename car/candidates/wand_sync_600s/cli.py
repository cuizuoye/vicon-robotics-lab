#!/usr/bin/env python3
"""Offline-only gate for a candidate that has never been physically qualified."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
DEFAULT = ROOT/'docs/wand_sync_600s_2026-10-28/prepared'


def check(folder):
    m = json.loads((folder/'build_manifest.json').read_text())
    if m.get('offline_passed') is not True or m.get('identity') != '4WSA':
        raise ValueError('600s offline preparation incomplete')
    for group in ('runtime_hashes', 'source_hashes', 'firmware_source_hashes', 'protected_input_hashes'):
        for name, digest in m[group].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
                raise ValueError('Source drift: '+name)
    f = m['firmware']
    if (f.get('authority_version') != 6 or f.get('diagnostic_payload_bytes') != 204
            or f.get('independent_limit_ms') != 630000
            or hashlib.sha256(Path(f['path']).read_bytes()).hexdigest() != f['sha256']):
        raise ValueError('New long-session image identity/hash mismatch')
    return m


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--prepared', type=Path, default=DEFAULT)
    p.add_argument('--check', action='store_true')
    p.add_argument('--launch', action='store_true')
    p.add_argument('--stage', type=int, choices=range(7), default=0)
    a = p.parse_args()
    if a.launch:
        raise ValueError('NOT FLASHED: new-image backup/readback and staged hardware admission required; no ROS or socket opened')
    m = check(a.prepared)
    values = json.loads((a.prepared/f'runtime/src/{m["hardware_package"]}/config/stage{a.stage}.NOT_APPROVED.json').read_text())['safety']
    print(f'PASS: 4WSA candidate,600s host/630s independent MCU; stage{a.stage}, '
          f'yaw {values["max_yaw_rate_radps"]}rad/s / {values["max_yaw_accel_radps2"]}rad/s2; '
          'OFFLINE ONLY, NOT FLASHED, no physical motion')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError) as error:
        print('BLOCKED: '+str(error), file=sys.stderr)
        raise SystemExit(2)
