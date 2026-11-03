#!/usr/bin/env python3
"""Create an additive, source-only car archive. Never writes the live workspace.

This is NOT an admission or deployment generator. Old releases are untouched.
Only explicit source trees and three reviewed frozen packages are selected.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re

from export_car_snapshot import normalize

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'car/snapshots/2026-11-03'
SKIP = {'.git', 'build', 'install', 'log', 'logs', 'sessions', 'backups',
        'hardware_backups', '__pycache__', '.pytest_cache', 'node_modules',
        'third_party', 'ThirdParty', 'private_ros_logs', 'CMakeFiles', 'arm', 'native'}
EXT = {'.py', '.c', '.cpp', '.h', '.hpp', '.s', '.ld', '.cmake', '.txt', '.md', '.yaml',
       '.yml', '.xml', '.cfg', '.sh', '.rviz', '.xacro', '.urdf', '.html',
       '.css', '.js', '.cjs', '.defaults', '.msg', '.projbuild'}
FROZEN = (
    'docs/wand_browser_direct_2026-10-30/prepared_v1',
    'docs/wand_background_session_display_2026-11-02/prepared_v1/host',
    'docs/wand_static_source_2026-11-02/prepared_v1/host',
    'docs/wand_stopfix_2026-11-02/prepared_v1/firmware',
)
GEOMETRY = dict(bar=['Sign1', 'Sign4', 'Sign3'], junction='Sign4',
    stem=['Sign2', 'Sign5'], forward_local=[0., 1., 0.], width_m=.24,
    line_residual_m=0., local_points={
        'Sign1': [-.12, .08, 0.], 'Sign2': [0., -.06, 0.],
        'Sign3': [.12, .08, 0.], 'Sign4': [0., .08, 0.],
        'Sign5': [0., -.18, 0.]})


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def json_text(value):
    return json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'


def canonical(text):
    """Mechanical public-source formatting, with original hashes retained."""
    return '\n'.join(line.rstrip(' \t\r') for line in text.splitlines()).rstrip('\n') + '\n'


def revoke(value):
    if isinstance(value, list):
        return [revoke(v) for v in value]
    if not isinstance(value, dict):
        return value
    result = {}
    for key, item in value.items():
        if 'confirmation' in key and isinstance(item, dict):
            result[key] = {k: False for k in item}
        elif key in {'staged_runtime_approval', 'trial_review', 'commissioning_review',
                     'long_duration_review', 'firmware_deployment'}:
            result[key] = {}
        elif key in {'human_authorized', 'hardware_launch_enabled', 'motion_authorized',
                     'physical_qualified', 'long_session_qualified'}:
            result[key] = False
        else:
            result[key] = revoke(item)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--complete-selection', action='store_true',
        help='Add missing source types; existing public files must already match exactly')
    args = parser.parse_args()
    source = args.source.resolve()
    if source == ROOT or source.is_relative_to(ROOT) or (DEST.exists() and not args.complete_selection):
        raise ValueError('Separate read-only source and a new destination are required')
    selected = set()

    def tree(rel, configs=False):
        for path in (source / rel).rglob('*'):
            relative = path.relative_to(source)
            if not path.is_file() or path.is_symlink() or any(p in SKIP for p in relative.parts):
                continue
            if path.name.startswith(('private', 'sdkconfig')):
                continue
            allowed = path.suffix.lower() in EXT or path.name in {'LICENSE', '.gitignore', 'COLCON_IGNORE'} or 'resource' in relative.parts
            if path.suffix == '.json':
                allowed = ((configs and ('config' in relative.parts or 'profiles' in relative.parts))
                    or path.name in {'sign_geometry.json', 'CMakePresets.json', 'package.json', 'package-lock.json'})
            if allowed:
                selected.add(relative)

    for name in ('src', 'protocol', 'tools', 'candidates', 'stm32/mecanum_f407', 'esp32/mecanum_wifi_bridge'):
        tree(name, configs=name == 'src')
    for name in FROZEN:
        tree(name, configs=True)
    # Design notes only, not raw captures, receipts, sessions or authority files.
    for path in (source / 'docs').glob('*/*.md'):
        selected.add(path.relative_to(source))
    for path in (source / 'docs').glob('*.md'):
        selected.add(path.relative_to(source))
    # The generated entry is source, not the compiled ARM image or raw flash.
    main_c = Path('docs/wand_stopfix_2026-11-02/prepared_v1/arm/four_link_bench_main_4wsc.c')
    selected.add(main_c)

    private_record = json.loads((source / 'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json').read_text())
    identifiers = [private_record['stlink_sn'], *private_record['chip_uid_words']]
    original_geometry = source / FROZEN[2] / 'sign_geometry.json'
    geometry_sha = digest(original_geometry.read_bytes())
    synthetic = json_text(GEOMETRY)
    synthetic_sha = digest(synthetic.encode())
    rows = []
    for relative in sorted(selected):
        path = source / relative
        before = path.read_bytes()
        if b'\0' in before:
            raise ValueError('Unexpected binary: ' + str(relative))
        after = normalize(before.decode('utf-8'), source)
        after = re.sub(r'\b[A-F0-9]{24}\b', '0' * 24, after)
        for identifier in identifiers:
            after = after.replace(identifier, '0' * len(identifier))
        after = re.sub(r'\b[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}\b',
                       '00000000-0000-4000-8000-000000000000', after)
        after = after.replace(geometry_sha, synthetic_sha)
        # Only a C include is relocated; archived launch/admission paths stay
        # intentionally non-deployable placeholders.
        after = re.sub(r'#include "/home/PRIMARY_USER/vicon_mecanum_ws/([^"\n]+)"',
            lambda m: '#include "' + os.path.relpath(DEST / m[1], (DEST / relative).parent) + '"', after)
        if path.name == 'sign_geometry.json':
            after = synthetic
        elif path.suffix == '.json':
            after = json_text(revoke(json.loads(after)))
        after = canonical(after)
        dest = DEST / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        if dest.exists():
            if canonical(dest.read_text()) != after:
                raise ValueError('Refuse to overwrite modified public source: ' + str(relative))
            if dest.read_bytes() != after.encode():
                dest.write_text(after)
        else:
            dest.write_text(after)
        rows.append(dict(path=str(dest.relative_to(ROOT)), source_relative=str(relative),
                         source_sha256=digest(before), exported_sha256=digest(after.encode())))
    receipt = dict(date='2026-11-03', scope='Additive public car source archive, not a deployment',
        source_tree_written=False, legacy_export_overwritten=False,
        source_files=len(rows), files=rows, synthetic_sign_geometry=True,
        hardware_authority_transferred=False, original_generators_portable=False,
        exclusions=['raw sessions and machine telemetry', 'Flash/NVS and build binaries',
                    'site permits and device identities', 'valid measured calibration',
                    'private deployment receipts and original hash/admission manifests'])
    (ROOT / 'handoff/CAR_EXPORT_2026-11-03.json').write_text(json_text(receipt))
    print(json_text(dict(files=len(rows), bytes=sum((ROOT/r['path']).stat().st_size for r in rows))))


if __name__ == '__main__':
    main()
