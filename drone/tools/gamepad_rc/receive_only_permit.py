"""Single-use, fusion-OFF USB reception exception; never enables active benches."""
import hashlib
import json
import math
import os
from pathlib import Path
import time

TOOLS = Path(__file__).resolve().parent
SCOPE = 'battery-off-usb-only-props-installed-fusion-off-static-reception-once'
CLOCK_SCOPE = 'battery-off-usb-only-props-installed-fusion-off-single-clock-sample-once'
UID = '000000000000000000000000000000000000'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_request(args, *, clock_sample=False):
    forbidden = ('clock_loss_check', 'clock_status_only', 'clock_characterize',
                 'fusion_preflight', 'horizontal_fusion_bench',
                 'height_fusion_bench', 'yaw_fusion_bench', 'combined_fusion_bench',
                 'combined_source_loss_bench', 'restore_horizontal_only',
                 'restore_height_only', 'restore_yaw_only', 'restore_combined_only',
                 'manual_translation_bench', 'manual_vertical_bench',
                 'fusion_physical_confirmed', 'fusion_backup', 'manual_control_file', 'vision_reference_permit')
    if (bool(getattr(args, 'clock_probe_only', False)) != clock_sample
            or (clock_sample and getattr(args, 'clock_settle_seconds', None) != 180)
            or not args.bench_send or args.seconds != 10 or args.uid2 != UID
            or any(getattr(args, name, False) for name in forbidden)):
        raise ValueError('Receive-only exception forbids fusion/restore/clock/manual modes; exactly 10s required')


def consume(path, output, heading, board, *, clock_sample=False, now=None):
    path, output = Path(path).resolve(), Path(output).resolve()
    record = json.loads(path.read_text())
    now = time.time() if now is None else now
    issued, expires = record.get('issued_unix'), record.get('expires_unix')
    if (type(issued) not in (int, float) or type(expires) not in (int, float)
            or not all(math.isfinite(x) for x in (issued, expires, now))
            or not issued <= now < expires or not 0 < expires-issued <= 900):
        raise ValueError('Expired/invalid receive-only permit')
    expected = {'schema': 1, 'scope': CLOCK_SCOPE if clock_sample else SCOPE,
                'uid2': UID, 'user_authorized': True,
                'output_path': str(output), 'stream_seconds': 0 if clock_sample else 10,
                'rate_hz': 0 if clock_sample else 30,
                'max_packets': 1 if clock_sample else 300, 'parameter_writes': 0, 'fusion_enabled': False,
                'physical_conditions': ['battery_disconnected', 'usb_only', 'props_installed', 'stationary', 'output_locked']}
    if clock_sample:
        expected['sample_hold_ms'] = 50
        expected['max_clock_stability_wait_seconds'] = 180
    if any(record.get(k) != v for k, v in expected.items()) or output.exists():
        raise ValueError('Receive-only permit scope/output mismatch')
    checks = {'runner_sha256': TOOLS/'usb_vicon_receive_bench.py',
              'permit_policy_sha256': Path(__file__),
              'message_policy_sha256': TOOLS/'usb_vision_policy.py',
              'incident_sha256': TOOLS/'known_incidents/micoair743_v1143_shell_crash.json',
              'physical_hold_sha256': TOOLS/'known_incidents/physical_safety_hold.json',
              'heading_sha256': Path(heading), 'board_sha256': Path(board)}
    if any(record.get(k) != digest(p) for k, p in checks.items()):
        raise ValueError('Receive-only evidence/code hash mismatch')
    if (json.loads(checks['incident_sha256'].read_text()).get('block_active_benches') is not True
            or json.loads(checks['physical_hold_sha256'].read_text()).get('block_props_off_benches') is not True):
        raise ValueError('General incident/physical holds must remain intact')
    fd = os.open(str(path)+'.consumed.json', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump({'scope': expected['scope'], 'permit_sha256': digest(path), 'consumed_unix': now,
                   'general_guards_modified': False, 'output_path': str(output)}, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    return record


def locked_state(state, boot_ms):
    return bool(state and boot_ms is not None and state['lockdown']
                and not any(state[k] for k in ('armed', 'manual_lockdown', 'force_failsafe'))
                and abs(state['timestamp_us']-boot_ms*1000) < 2_000_000)
