#!/usr/bin/env python3
"""Fail closed on common accidental public uploads; reports paths, never matches.

This is defense in depth, not a mathematical guarantee that a file has no secret.
No remote calls and no credential-store access.
"""
import json
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
PATTERNS=[rb'gh[pousr]_[A-Za-z0-9_]{24,}',rb'github_pat_[A-Za-z0-9_]{24,}',
          rb'sk-(?:proj-)?[A-Za-z0-9_-]{24,}',rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
          rb'https?://[^\s/@:]+:[^\s/@]+@',rb'/home/czy(?:/|\b)']
DENY_PARTS={'.ssh','.codex','backups','param_backups','third_party','__pycache__','.venv'}
DENY_SUFFIXES={'.bin','.elf','.hex','.px4','.so','.pem','.key','.mcap','.db3','.AppImage'}

def main():
    data=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT)
    files=sorted(set(p for p in data.decode().split('\0') if p));errors=[];total=0
    for name in files:
        path=ROOT/name
        if not path.is_file():continue
        if path.is_symlink():errors.append(dict(path=name,reason='symlink must be reviewed'));continue
        if any(p in DENY_PARTS for p in path.relative_to(ROOT).parts) or path.suffix in DENY_SUFFIXES:
            errors.append(dict(path=name,reason='private/generated/binary filename'));continue
        if path.name in {'sdkconfig','sdkconfig.old','private.defaults','.env'}:
            errors.append(dict(path=name,reason='private config'));continue
        raw=path.read_bytes();total+=len(raw)
        if len(raw)>10*1024*1024:errors.append(dict(path=name,reason='unexpected large file'))
        if b'\x00' in raw:errors.append(dict(path=name,reason='binary content'));continue
        # The normalizer and scanner contain literal detection rules, not secrets.
        if name in ('scripts/public_scan.py','scripts/prepare_handoff.py'):continue
        if any(re.search(pattern,raw) for pattern in PATTERNS):
            errors.append(dict(path=name,reason='token/private-key/credential-URL/original-home marker'))
        if '/test/' not in name and '/tests/' not in name:
            if re.search(rb'CONFIG_MC_WIFI_(?:SOFTAP_)?PASSWORD\s*=\s*"[^"\n]+"',raw):
                errors.append(dict(path=name,reason='literal Wi-Fi credential'))
    result=dict(passed=not errors,files=len(files),bytes=total,errors=errors,
                notes='Selected files only; scan does not authorize disclosure of raw private material.')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
