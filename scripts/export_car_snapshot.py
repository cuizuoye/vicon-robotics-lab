#!/usr/bin/env python3
"""Explicit, car-only public export. Source is read-only; never opens devices.

Historical generators remain source archives, not portable live launchers. The
frozen source snapshots and the separate offline test entry are reproducible.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SKIP = {'build', 'install', 'log', 'logs', 'backups', 'hardware_backups',
        '__pycache__', '.pytest_cache', '.git', '.vscode', 'node_modules',
        'third_party', 'ThirdParty', 'private_ros_logs'}
SUFFIXES = {'.py', '.c', '.h', '.s', '.ld', '.cmake', '.txt', '.md', '.json',
            '.yaml', '.yml', '.xml', '.cfg', '.sh', '.rviz', '.xacro', '.urdf',
            '.html', '.css', '.js', '.cjs', '.defaults', '.msg', '.projbuild'}
FROZEN = [
    'wand_rotation_hardware_2026-10-26/prepared',
    'wand_feedback_300ms_startup_2026-10-27/prepared',
    'wand_ros_bound_history_2026-10-27/prepared',
    'wand_browser_fast_2026-10-28/prepared',
    'wand_sync_4ws9_2026-10-27/prepared',
    'wand_sync_600s_2026-10-28/prepared',
    'wand_servo_4ws8_2026-10-26/demo_3x3_20s/ready',
]
EVIDENCE = [
    'wand_servo_4ws8_2026-10-26/demo_3x3_20s/accepted_20261026/ACCEPTED_BASELINE.json',
    'CONTINUOUS_3X2_VICON_RECOVERY_2026-10-21.md',
    'wand_rotation_preview_2026-10-26/DIRECTION_ACCEPTANCE.json',
    'wand_servo_matching_sim_2026-10-23/REPORT.md',
    'wand_rotation_accel_sim_2026-10-26/A2_FOLLOW_ACCEPTANCE.json',
    'wand_rotation_hardware_2026-10-26/prepared/PREPARATION_RESULT.json',
    'wand_tracking_workspace_2026-10-27/prepared/PREPARATION_RESULT.json',
    'wand_feedback_300ms_startup_2026-10-27/GROUND_60S_RESULT.json',
    'wand_ros_bound_history_2026-10-27/prepared/PREPARATION_RESULT.json',
    'wand_browser_fast_2026-10-28/prepared/PREPARATION_RESULT.json',
    'wand_tracking_autostart_2026-10-27/prepared/PREPARATION_RESULT.json',
    'wand_servo_matching_sim_2026-10-23/LIVE_ACCEPTANCE.md',
    'wand_sync_600s_2026-10-28/SUMMARY.md',
]


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def normalize(text, source):
    text = text.replace(str(source), '/home/PRIMARY_USER/vicon_mecanum_ws')
    text = re.sub(r'/home/[^/\s\"\'<>]+', '/home/PRIMARY_USER', text)
    text = re.sub(r'wlx[0-9a-fA-F]{12}', 'wlx000000000000', text)
    text = re.sub(r'(?<=sn=)[A-F0-9]{20,32}', '0' * 24, text)
    text = re.sub(r'(usb-1a86_USB_Single_Serial_)[A-F0-9]{8,16}', r'\g<1>00000000', text)
    text = re.sub(r"UUID='[0-9a-f-]{36}'", "UUID='00000000-0000-4000-8000-000000000000'", text)
    # Network identifiers are synthetic; private values are never published.
    text = re.sub(r'(?im)^(\s*CONFIG_MC_WIFI_(?:SOFTAP_)?PASSWORD\s*=).*$', r'\1""', text)
    text = re.sub(r'(?i)(?:[0-9a-f]{2}:){5}[0-9a-f]{2}', '02:00:00:00:00:00', text)
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--refresh-unreviewed', action='store_true', help='Refresh this uncommitted export before portable adaptations')
    args = ap.parse_args()
    source = args.source.resolve()
    if source == ROOT or source == ROOT/'car':
        raise ValueError('Source and export must be separate')
    receipt = ROOT/'handoff/CAR_EXPORT_2026-10-28.json'
    if receipt.exists() and not args.refresh_unreviewed:
        raise ValueError('One-time selected export; do not overwrite its receipt')
    selected = set()

    def tree(rel):
        folder = source/rel
        for p in folder.rglob('*'):
            if (not p.is_file() or p.is_symlink()
                    or any(part in SKIP for part in p.relative_to(source).parts)):
                continue
            if p.suffix.lower() in SUFFIXES or p.name in {'COLCON_IGNORE', '.gitignore', 'LICENSE'} or 'resource' in p.parts:
                selected.add(p.relative_to(source))

    for rel in ('src', 'protocol', 'tools', 'candidates', 'stm32/mecanum_f407', 'esp32/mecanum_wifi_bridge'):
        tree(rel)
    for rel in FROZEN:
        tree('docs/'+rel+'/runtime/src')
        tree('docs/'+rel+'/firmware')
        folder = source/'docs'/rel
        for p in folder.glob('*'):
            if p.is_file() and p.suffix in {'.py', '.js', '.cjs', '.html', '.css', '.c', '.md'}:
                selected.add(p.relative_to(source))
        for name in ('PREPARATION_RESULT.json', 'REAL_ROS_REHEARSAL.json', 'SAME_TIME_COMPARISON.json'):
            if (folder/name).is_file():
                selected.add((folder/name).relative_to(source))
    # ESP32 event-driven accepted transport source, without NVS or sdkconfig.
    tree('docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready/esp_project/main')
    for name in EVIDENCE:
        selected.add(Path('docs')/name)
    # Top-level design/measurement notes, not raw sessions or private approvals.
    for p in (source/'docs').glob('*.md'):
        selected.add(p.relative_to(source))
    rows = []
    for rel in sorted(selected):
        if ((rel.suffix == '.json' and any(word in rel.name.lower() for word in ('permit', 'user_authorization', 'flash_record')))
                or 'private.defaults' in rel.name
                or rel.name in {'sdkconfig', 'sdkconfig.old', 'build_manifest.json', 'protected_inputs.json'}):
            continue
        src = source/rel
        before = src.read_bytes()
        if b'\0' in before:
            raise ValueError('Unexpected binary: '+str(rel))
        after = normalize(before.decode('utf-8'), source)
        if rel.suffix == '.json' and ('config' in rel.parts or rel.name.endswith('.draft.json')):
            v = json.loads(after)
            # Public data is a software fixture, never a transferable site permit.
            for key in list(v):
                if key.endswith('confirmations') and isinstance(v[key], dict):
                    v[key] = {k: False for k in v[key]}
            if isinstance(v.get('trial_review'), dict):
                v['trial_review']['human_authorized'] = False
            if 'staged_runtime_approval' in v:
                v['staged_runtime_approval'] = {}
            after = json.dumps(v, ensure_ascii=False, indent=2)+'\n'
        dest = ROOT/'car'/rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(after)
        rows.append(dict(path=str(dest.relative_to(ROOT)), source_sha256=sha(before),
                         exported_sha256=sha(after.encode()), normalized=before != after.encode()))
    receipt.write_text(json.dumps(dict(date='2026-10-28', source_tree_written=False,
        scope='Car source and frozen runtime snapshots; drone and shared sources unchanged',
        files=rows, excluded=['Flash/NVS, SDK binaries, builds, raw sessions and bags',
            'private credentials, device identities, site permits and historical hash manifests'],
        historical_generators_portable=False, live_authority_transferred=False), indent=2)+'\n')
    print(json.dumps(dict(exported_files=len(rows), bytes=sum((ROOT/r['path']).stat().st_size for r in rows))))


if __name__ == '__main__':
    main()
