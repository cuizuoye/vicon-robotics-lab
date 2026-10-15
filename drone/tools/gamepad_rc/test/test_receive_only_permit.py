import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import receive_only_permit as p


def request(**kwargs):
    return SimpleNamespace(bench_send=True, seconds=10, uid2=p.UID, clock_settle_seconds=180, **kwargs)


@pytest.mark.parametrize('mode', ['combined_fusion_bench', 'horizontal_fusion_bench',
    'fusion_preflight', 'restore_combined_only', 'clock_loss_check', 'manual_translation_bench',
    'fusion_physical_confirmed', 'fusion_backup', 'clock_probe_only'])
def test_receive_exception_cannot_enable_any_other_mode(mode):
    with pytest.raises(ValueError):
        p.validate_request(request(**{mode: True}))


def test_exact_request_only():
    p.validate_request(request())
    for name, value in [('seconds', 11), ('uid2', 'other'), ('bench_send', False)]:
        r = request()
        setattr(r, name, value)
        with pytest.raises(ValueError):
            p.validate_request(r)


def make_permit(tmp_path, monkeypatch):
    monkeypatch.setattr(p, 'TOOLS', tmp_path)
    (tmp_path/'known_incidents').mkdir()
    for name, content in [('micoair743_v1143_shell_crash.json', {'block_active_benches': True}),
                          ('physical_safety_hold.json', {'block_props_off_benches': True})]:
        (tmp_path/'known_incidents'/name).write_text(json.dumps(content))
    for name in ('usb_vicon_receive_bench.py', 'usb_vision_policy.py', 'heading', 'board'):
        (tmp_path/name).touch()
    output = tmp_path/'result.json'
    r = dict(schema=1, scope=p.SCOPE, uid2=p.UID, user_authorized=True,
             output_path=str(output), stream_seconds=10, rate_hz=30, max_packets=300,
             parameter_writes=0, fusion_enabled=False, issued_unix=1000, expires_unix=1600,
             physical_conditions=['battery_disconnected','usb_only','props_installed','stationary','output_locked'])
    for key, path in {'runner_sha256':tmp_path/'usb_vicon_receive_bench.py',
        'permit_policy_sha256':Path(p.__file__), 'message_policy_sha256':tmp_path/'usb_vision_policy.py',
        'incident_sha256':tmp_path/'known_incidents/micoair743_v1143_shell_crash.json',
        'physical_hold_sha256':tmp_path/'known_incidents/physical_safety_hold.json',
        'heading_sha256':tmp_path/'heading','board_sha256':tmp_path/'board'}.items():
        r[key] = p.digest(path)
    permit = tmp_path/'permit.json'
    permit.write_text(json.dumps(r))
    return permit, output, r


def test_one_use_and_holds_not_modified(tmp_path, monkeypatch):
    permit, output, _ = make_permit(tmp_path, monkeypatch)
    holds = {x:x.read_bytes() for x in (tmp_path/'known_incidents').iterdir()}
    p.consume(permit, output, tmp_path/'heading', tmp_path/'board', now=1100)
    with pytest.raises(FileExistsError):
        p.consume(permit, output, tmp_path/'heading', tmp_path/'board', now=1101)
    assert all(x.read_bytes() == data for x,data in holds.items())


@pytest.mark.parametrize('field,value', [('scope','flight'),('fusion_enabled',True),
    ('parameter_writes',1),('expires_unix',500),('runner_sha256','modified'),('user_authorized',False)])
def test_invalid_permit_does_not_consume(tmp_path, monkeypatch, field, value):
    permit, output, r = make_permit(tmp_path, monkeypatch)
    r[field] = value
    permit.write_text(json.dumps(r))
    with pytest.raises(ValueError):
        p.consume(permit, output, tmp_path/'heading', tmp_path/'board', now=1100)
    assert not Path(str(permit)+'.consumed.json').exists()


def test_stale_unlocked_or_armed_state_rejected():
    state = dict(timestamp_us=1_000_000, armed=False, lockdown=True,
                 manual_lockdown=False, force_failsafe=False)
    assert p.locked_state(state, 1000)
    assert not p.locked_state(state, 4000)
    assert not p.locked_state(state, None)
    assert not p.locked_state({**state, 'lockdown':False}, 1000)
    assert not p.locked_state({**state, 'armed':True}, 1000)


def test_old_usb_telemetry_cannot_be_used_as_current_boot_clock():
    # Measured readonly regression: backlog was 22.6s behind current listener.
    state = dict(timestamp_us=5_254_790_346, armed=False, lockdown=True,
                 manual_lockdown=False, force_failsafe=False)
    assert not p.locked_state(state, 5_232_156)
    settled = {**state, 'timestamp_us':5_288_171_783}
    assert p.locked_state(settled, 5_288_535)


def test_clock_sample_request_is_not_a_stream_or_fusion_permission():
    p.validate_request(request(clock_probe_only=True), clock_sample=True)
    with pytest.raises(ValueError):
        p.validate_request(request(), clock_sample=True)
    with pytest.raises(ValueError):
        p.validate_request(request(clock_probe_only=True, combined_fusion_bench=True), clock_sample=True)


def test_clock_sample_requires_distinct_scope_and_exact_one_packet(tmp_path, monkeypatch):
    permit, output, r = make_permit(tmp_path, monkeypatch)
    with pytest.raises(ValueError):
        p.consume(permit, output, tmp_path/'heading', tmp_path/'board', clock_sample=True, now=1100)
    r.update(scope=p.CLOCK_SCOPE, stream_seconds=0, rate_hz=0, max_packets=1, sample_hold_ms=50,
             max_clock_stability_wait_seconds=180)
    permit.write_text(json.dumps(r))
    with pytest.raises(ValueError):
        p.consume(permit, output, tmp_path/'heading', tmp_path/'board', now=1100)
    assert not Path(str(permit)+'.consumed.json').exists()
    assert p.consume(permit, output, tmp_path/'heading', tmp_path/'board', clock_sample=True, now=1100)['max_packets'] == 1
