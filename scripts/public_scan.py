#!/usr/bin/env python3
"""Fail closed on common accidental public uploads; reports paths, never matches.

This is defense in depth, not a mathematical guarantee that a file has no secret.
No remote calls and no credential-store access.
"""
import json
import hashlib
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET
from zipfile import ZipFile, BadZipFile

ROOT=Path(__file__).resolve().parents[1]
PATTERNS=[rb'gh[pousr]_[A-Za-z0-9_]{24,}',rb'github_pat_[A-Za-z0-9_]{24,}',
          rb'sk-(?:proj-)?[A-Za-z0-9_-]{24,}',rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
          rb'https?://[^\s/@:]+:[^\s/@]+@',rb'/home/czy(?:/|\b)']
DENY_PARTS={'.ssh','.codex','backups','param_backups','third_party','__pycache__','.venv'}
DENY_SUFFIXES={'.bin','.elf','.hex','.px4','.so','.pem','.key','.mcap','.db3','.AppImage'}

def reviewed_docx(path, expected):
    """Narrow DOCX exception: reviewed hash AND every ZIP member inspected."""
    if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        raise ValueError('reviewed DOCX hash mismatch')
    with ZipFile(path) as z:
        total=0
        for item in z.infolist():
            name=item.filename
            if name.startswith('/') or '..' in Path(name).parts:
                raise ValueError('unsafe DOCX member')
            total+=item.file_size
            if total>20*1024*1024 or item.file_size>10*1024*1024:
                raise ValueError('oversized DOCX archive')
            raw=z.read(item)
            if name.endswith(('.xml','.rels')):
                if b'<!DOCTYPE' in raw or any(re.search(p,raw) for p in PATTERNS):
                    raise ValueError('private marker in DOCX XML')
                tree=ET.fromstring(raw)
                if name.endswith('.rels'):
                    for rel in tree:
                        if rel.get('TargetMode')=='External':
                            if not rel.get('Target','').startswith('https://github.com/cuizuoye/vicon-robotics-lab/blob/main/car/'):
                                raise ValueError('unreviewed external DOCX relationship')
            elif name.startswith('word/media/') and name.endswith('.png') and raw.startswith(b'\x89PNG\r\n\x1a\n'):
                pass  # Its bytes are covered by the explicitly reviewed document hash.
            elif name=='docProps/thumbnail.jpeg' and raw.startswith(b'\xff\xd8\xff'):
                pass
            else:
                raise ValueError('unreviewed DOCX embedded member')

def main():
    data=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT)
    files=sorted(set(p for p in data.decode().split('\0') if p));errors=[];total=0
    review=ROOT/'handoff/REVIEWED_DOCUMENTS.json'
    documents={r['path']:r['sha256'] for r in json.loads(review.read_text())} if review.exists() else {}
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
        if path.suffix=='.docx' and name in documents:
            try:reviewed_docx(path,documents[name])
            except (ValueError,BadZipFile,ET.ParseError) as error:
                errors.append(dict(path=name,reason=str(error)))
            continue
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
