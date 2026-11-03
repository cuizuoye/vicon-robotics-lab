"""Static packet/configuration fixtures; no ROS graph, dynamic plant or socket."""
import ast
import hashlib
import inspect
import json
import os
from pathlib import Path
import textwrap

import pytest

from mecanum_vicon_control.bench_link_status import STATUS
from mecanum_vicon_control.motion_link_status import (
    MOTION_TAGS as LEGACY_TAGS, decode_motion_status as legacy_decode)
from mecanum_vicon_control.transport_protocol import ProtocolError
from mecanum_wand_hardware.commissioning import CONSENT, Runtime
from mecanum_wand_hardware.continuous_status import (
    AUTHORITY, SNAPSHOT, TAG, decode_motion_status, disabled_motion_status_ready)

PROFILE = Path(__file__).resolve().parents[1]/'config/hardware_acceptance.corner_2x2_4wgc_2026-10-23.json'


def packet(active=True, change=None):
    v = list(STATUS.unpack(bytes(100)))
    v[:8] = [TAG, 1, 1, 10, 10, 146, 0, 500]
    v[34] = int(active)
    authority = [0.]*4+[0]*4+[0, 1, 0]
    if active:
        v[16:20] = [80, -80, 80, -80]
        v[8:16] = [100, 92, 92, 100]*2
        v[20:24], v[24:28] = [-100]*4, [50]*4
        authority[:4], authority[4:8], authority[8] = [80.]*4, [1]*4, 15
    if change:
        change(v, authority)
    return STATUS.pack(*v)+SNAPSHOT.pack(*([0]*10))+AUTHORITY.pack(*authority)


def test_legal_reversal_uses_effective_and_granted_direction_not_future_request():
    s = decode_motion_status(packet())
    assert s['tag'] == '4WGC' and s['reversing_mask'] == 15
    assert s['target_cps'] == [-100]*4 and s['effective_cps'] == [80.]*4
    assert s['raw_duty'] == [80, -80, 80, -80]


@pytest.mark.parametrize('bad', ['tag', 'sample_gap', 'irq', 'watchdog', 'permit', 'version',
    'reserved', 'reversal_mask', 'grant', 'effective_nan', 'effective_limit', 'effective_zero',
    'effective_wrong_sign', 'missing_reversal', 'ccr', 'duty', 'target', 'body', 'zero_request', 'startup'])
def test_corrupt_or_unsafe_static_packets_are_rejected(bad):
    def mutate(v, a):
        if bad == 'tag': v[0] = 0x42475734
        if bad == 'sample_gap': v[4] = 31
        if bad == 'irq': v[5] = 1001
        if bad == 'watchdog': v[33] = 1
        if bad == 'permit': v[34] = 0
        if bad == 'version': a[9] = 0
        if bad == 'reserved': a[10] = 1
        if bad == 'reversal_mask': a[8] = 16
        if bad == 'grant': a[4] = 0
        if bad == 'effective_nan': a[0] = float('nan')
        if bad == 'effective_limit': a[0] = 1201.
        if bad == 'effective_zero': a[0] = 0.
        if bad == 'effective_wrong_sign': a[0], a[4] = -80., -1
        if bad == 'missing_reversal': a[8] = 0
        if bad == 'ccr': v[8] = 99
        if bad == 'duty': v[16] = 360
        if bad == 'target': v[20] = -1201
        if bad == 'body': v[28] = 151
        if bad == 'zero_request': v[20] = 0
        if bad == 'startup': v[31] = 1
    with pytest.raises(ProtocolError):
        decode_motion_status(packet(change=mutate))


@pytest.mark.parametrize('size', [0, 99, 101, 132, 155, 157, 256])
def test_truncated_and_old_offline_diagnostics_are_not_silently_accepted(size):
    with pytest.raises(ProtocolError):
        decode_motion_status((packet()+bytes(256))[:size])


def test_disabled_status_and_legacy_decoder_remain_separate():
    assert disabled_motion_status_ready(decode_motion_status(packet(False)))
    assert not disabled_motion_status_ready(decode_motion_status(packet()))
    assert '4WGC' not in LEGACY_TAGS
    with pytest.raises(ProtocolError):
        legacy_decode(packet(False))
    old = bytearray(packet(False)[:100])
    old[:4] = b'4WGB'
    assert decode_motion_status(old) == legacy_decode(old)


def test_profile_requires_exact_dialect_before_any_node_or_socket(tmp_path):
    raw = PROFILE.read_bytes()
    r = Runtime('hardware', str(PROFILE), hashlib.sha256(raw).hexdigest(), CONSENT)
    assert (r.motion_tag, r.motion_status_size) == ('4WGC', 156)
    assert (r.config.max_speed_mps, r.config.max_session_s, r.config.max_goal_radius_m) == (.05, 120., 3.)
    p = json.loads(raw)
    p['hardware']['diagnostic_payload_bytes'] = 132
    bad = json.dumps(p).encode()
    path = tmp_path/'bad.json'
    path.write_bytes(bad)
    with pytest.raises(ValueError):
        Runtime('hardware', str(path), hashlib.sha256(bad).hexdigest(), CONSENT)


def test_extension_preserves_inherited_safety_methods_and_copied_method_structure():
    from mecanum_vicon_control.cmd_vel_udp_bridge import CmdVelUdpBridge as Legacy
    from mecanum_wand_hardware.continuous_bridge import ContinuousCmdVelUdpBridge as Extended
    for name in ('send_command', 'send_pending_control', 'tick', 'latch_local_stop', 'on_estop',
                 'on_control_enable', 'on_clear_faults', 'send_hello_if_due'):
        if hasattr(Legacy, name):
            assert getattr(Extended, name) is getattr(Legacy, name)
    for name in ('__init__', 'drain_receive'):
        original = textwrap.dedent(inspect.getsource(getattr(Legacy, name)))
        if name == '__init__':
            original = original.replace("super().__init__('cmd_vel_udp_bridge')",
                                        "Node.__init__(self, 'cmd_vel_udp_bridge')")
        copied = textwrap.dedent(inspect.getsource(getattr(Extended, name)))
        assert ast.dump(ast.parse(original)) == ast.dump(ast.parse(copied))


def test_actual_c_serializer_fixtures_when_explicitly_supplied():
    directory = os.environ.get('WAND_STATIC_WIRE_FIXTURES')
    if not directory:
        pytest.skip('provide the static C serializer fixtures; no dynamic simulation is needed')
    idle = decode_motion_status((Path(directory)/'idle_status.bin').read_bytes())
    reversal = decode_motion_status((Path(directory)/'reversal_status.bin').read_bytes())
    assert disabled_motion_status_ready(idle)
    assert reversal['target_cps'][2] == -100 and reversal['effective_cps'][2] == 80
    assert reversal['granted_direction'][2] == 1 and reversal['reversing_mask'] == 15
