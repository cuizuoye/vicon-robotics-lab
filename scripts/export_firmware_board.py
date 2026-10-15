#!/usr/bin/env python3
"""Mechanical source-only board export; no hardware or original-tree writes."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    args = parser.parse_args()
    def git(*arguments):
        return subprocess.check_output(['git', '-C', str(args.source), *arguments])
    destination = ROOT / 'drone/firmware/micoair-v1.14.3'
    public_base = git('rev-parse', 'v1.14.3^{commit}').decode().strip()
    patch = git('diff', '--no-ext-diff', 'v1.14.3', 'HEAD', '--',
                'boards/micoair/h743-v2', ':!boards/micoair/h743-v2/extras/*.bin')
    assert b'GIT binary patch' not in patch and b'Binary files' not in patch
    output = destination / 'vendor-board.patch'
    with output.open('xb') as stream:
        stream.write(patch)
    changed = git('diff', '--name-only', 'HEAD').decode().splitlines()
    core = [p for p in changed if not p.startswith('boards/micoair/h743-v2/')]
    # Reconstruct just the changed source paths in an isolated temporary tree.
    # This checks both patch contexts and byte identity without touching PX4.
    archive = git('archive', 'v1.14.3', *core)
    with tempfile.TemporaryDirectory(prefix='micoair-source-reconstruct-') as tmp:
        with tarfile.open(fileobj=io.BytesIO(archive)) as stream:
            stream.extractall(tmp, filter='data')
        for filename in ('vendor-board.patch', 'changes.patch'):
            subprocess.run(['git', 'apply', '--check', str(destination / filename)], cwd=tmp, check=True)
            subprocess.run(['git', 'apply', str(destination / filename)], cwd=tmp, check=True)
        for name in changed:
            assert (Path(tmp) / name).read_bytes() == (args.source / name).read_bytes(), name
    provenance_path = destination / 'upstream.json'
    provenance = json.loads(provenance_path.read_text())
    provenance.update(public_base_commit=public_base, public_base_tag='v1.14.3',
                      vendor_board_patch_sha256=hashlib.sha256(patch).hexdigest(),
                      source_reconstruction_verified_files=len(changed),
                      bootloader_binary_excluded=True,
                      full_firmware_rebuild_in_handoff_verified=False)
    provenance_path.write_text(json.dumps(provenance, indent=2) + '\n')
    print(json.dumps({'source_reconstruction_verified_files': len(changed),
                      'public_base_commit': public_base, 'original_written': False}))

if __name__ == '__main__':
    main()
