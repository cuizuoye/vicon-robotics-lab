from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from mecanum_wand_control.core import Velocity
from mecanum_wand_control.safety_core import ZERO
from mecanum_wand_hardware.commissioning import (Runtime, AuthoritySigner, AuthorityVerifier,
                                                TransportPolicy, McuSampleClock, CONSENT, REQUIRED_CONFIRMATIONS)

KEY = 'a1'*32


def approved(tmp_path, change=None):
    draft = Path(__file__).resolve().parents[1]/'config/hardware_acceptance.draft.json'
    value = json.loads(draft.read_text())
    value['confirmations'] = dict.fromkeys(REQUIRED_CONFIRMATIONS, True)
    value['safety'].update(arena_x_min=-1.5, arena_x_max=1.5, arena_y_min=-1., arena_y_max=1.,
                           ground_z_m=.1, half_length_m=.18, half_width_m=.15)
    if change: change(value)
    raw = json.dumps(value).encode()
    file = tmp_path/'test-only.json'
    file.write_bytes(raw)
    return str(file), hashlib.sha256(raw).hexdigest()


def test_default_shadow_has_no_transport_and_emulator_is_fixed_loopback():
    shadow, emulator = Runtime(), Runtime('emulator')
    assert shadow.ip == '' and shadow.source_domain == 0
    assert emulator.ip == '127.0.0.1' and emulator.source_domain == 93
    assert (emulator.command_port, emulator.telemetry_port) == (54600, 54601)


def test_approved_profile_is_exact_and_does_not_change_protocol(tmp_path):
    path, digest = approved(tmp_path)
    r = Runtime('hardware', path, digest, CONSENT)
    assert r.config.max_speed_mps == .05 and r.profile_hash == digest
    assert r.ip == '192.168.50.1' and r.command_port == 46000


def test_site_shadow_uses_same_fixed_arena_and_never_opens_real_endpoint():
    site = Path(__file__).resolve().parents[1]/'config/hardware_acceptance.site_2026-10-22.json'
    digest = hashlib.sha256(site.read_bytes()).hexdigest()
    shadow = Runtime('shadow', str(site))
    hardware = Runtime('hardware', str(site), digest, CONSENT)
    assert shadow.config == hardware.config and shadow.ip == ''
    assert shadow.config.inside(shadow.initial_pose)
    p = shadow.initial_pose
    from mecanum_wand_control.core import Pose
    assert shadow.config.inside(Pose(p.x+.2, p.y, p.z, p.yaw))
    assert shadow.config.inside(Pose(p.x, p.y+.2, p.z, p.yaw))
    assert shadow.config.half_length_m == .19 and shadow.config.half_width_m == .21


def test_wide_wand_profile_separates_input_reference_from_unchanged_car_fence():
    root = Path(__file__).resolve().parents[1]/'config'
    path = root/'hardware_acceptance.wide_wand_2026-10-22.json'
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    shadow = Runtime('shadow', str(path))
    emulator = Runtime('emulator', str(path))
    hardware = Runtime('hardware', str(path), digest, CONSENT)
    old = Runtime('shadow', str(root/'hardware_acceptance.site_2026-10-22.json'))
    assert shadow.config == hardware.config == emulator.config
    assert shadow.config.goal_boundary_policy == 'clamp'
    for k, v in asdict(old.config).items():
        if k != 'goal_boundary_policy': assert getattr(shadow.config, k) == v
    assert shadow.wand_reference_margin_m >= .5 and shadow.ip == ''
    assert emulator.ip == '127.0.0.1' and emulator.command_port == 54600
    assert hardware.ip == old.profile['hardware']['esp_ip']


@pytest.mark.parametrize('which', ['approval', 'consent', 'draft', 'confirmation', 'measurement',
                                  'timing_relaxation', 'speed', 'margin', 'identity', 'timeout', 'loopback'])
def test_unapproved_or_unsafe_hardware_profiles_rejected_before_node_or_socket(tmp_path, which):
    def change(v):
        if which == 'confirmation': v['confirmations']['power_cut_ready'] = False
        if which == 'measurement': v['safety']['ground_z_m'] = None
        if which == 'timing_relaxation': v['safety']['lease_timeout_s'] = .3
        if which == 'speed': v['safety']['max_speed_mps'] = .15
        if which == 'margin': v['safety']['boundary_margin_m'] = .001
        if which == 'identity': v['hardware']['required_motion_tag'] = '4WG1'
        if which == 'timeout': v['hardware']['wire_timeout_ms'] = 150
        if which == 'loopback': v['hardware']['esp_ip'] = '127.0.0.1'
    path, digest = approved(tmp_path, change)
    if which == 'draft':
        path = str(Path(__file__).resolve().parents[1]/'config/hardware_acceptance.draft.json')
        digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    with pytest.raises((ValueError, TypeError)):
        Runtime('hardware', path, 'bad' if which == 'approval' else digest,
                '' if which == 'consent' else CONSENT)


def test_authority_first_packet_must_be_disabled_and_owner_cannot_change():
    a, b, v = AuthoritySigner(KEY), AuthoritySigner(KEY), AuthorityVerifier(KEY)
    with pytest.raises(ValueError):
        v.decode(a.encode(ZERO, 10., 100., state='ARMED', epoch='e', permit=True), 10., 100.)
    v.decode(a.encode(ZERO, 10., 100.), 10., 100.)
    with pytest.raises(ValueError): v.decode(b.encode(ZERO, 10., 100.), 10., 100.)


@pytest.mark.parametrize('bad', ['signature', 'replay', 'expired', 'future', 'overspeed',
                                'nonzero_without_permit', 'active_without_epoch', 'bool_type'])
def test_signed_authority_validates_permission_velocity_freshness_and_replay(bad):
    s, v = AuthoritySigner(KEY), AuthorityVerifier(KEY)
    initial = s.encode(ZERO, 10., 100.)
    v.decode(initial, 10., 100.)
    kwargs = dict(state='ACTIVE', permit=True, epoch='e')
    command = Velocity(.02)
    if bad == 'overspeed': command = Velocity(.05, .05)
    if bad == 'nonzero_without_permit': kwargs['permit'] = False
    if bad == 'active_without_epoch': kwargs['epoch'] = ''
    if bad == 'bool_type': kwargs['permit'] = 1
    text = s.encode(command, 10., 100., **kwargs)
    if bad == 'signature':
        data = json.loads(text)
        data['payload']['permit'] = False
        text = json.dumps(data)
    if bad == 'replay': text = initial
    with pytest.raises((ValueError, TypeError)):
        v.decode(text, 10.2 if bad == 'expired' else 9.99 if bad == 'future' else 10., 100.)


def test_timeout_blocks_epoch_and_disabled_messages_do_not_restore_old_motion():
    s, p = AuthoritySigner(KEY), TransportPolicy(KEY)
    def send(t, state, permit, epoch='', cmd=ZERO):
        return p.receive(s.encode(cmd, t, t+90., state=state, permit=permit, epoch=epoch), t, t+90.)
    send(10., 'STANDBY', False)
    send(10.01, 'ARMED', True, 'one')
    send(10.02, 'ACTIVE', True, 'one', Velocity(.02))
    assert p.active(10.03)
    assert not p.active(10.13) and p.command == ZERO
    send(10.14, 'ACTIVE', True, 'one', Velocity(.02))
    assert not p.active(10.14)
    send(10.15, 'PAUSED', False)
    send(10.16, 'ARMED', True, 'one')
    send(10.17, 'ACTIVE', True, 'one', Velocity(.02))
    assert not p.active(10.17)
    send(10.18, 'ARMED', True, 'two')
    send(10.19, 'ACTIVE', True, 'two', Velocity(.02))
    assert p.active(10.19)


def test_corruption_and_estop_cannot_be_cleared_by_new_arm():
    s, p = AuthoritySigner(KEY), TransportPolicy(KEY)
    p.receive(s.encode(ZERO, 10., 100.), 10., 100.)
    assert not p.receive('{}', 10., 100.) and p.hard_fault
    p.receive(s.encode(ZERO, 10.01, 100.01, state='ARMED', permit=True, epoch='new'), 10.01, 100.01)
    assert not p.active(10.01)


def test_all_raw_legacy_motion_callbacks_are_inert():
    from mecanum_wand_hardware.nodes import HardwareBridge
    sentinel = SimpleNamespace(enabled=False, latest='unchanged')
    for name in ('on_cmd', 'on_enable', 'on_control_enable', 'on_clear_faults', 'on_estop'):
        getattr(HardwareBridge, name)(sentinel, SimpleNamespace(data=True))
    assert vars(sentinel) == dict(enabled=False, latest='unchanged')


def test_shadow_package_does_not_import_hardware_package():
    import ast
    import mecanum_wand_control
    root = Path(mecanum_wand_control.__file__).parent
    for path in root.glob('*.py'):
        tree = ast.parse(path.read_text())
        assert not any(isinstance(n, ast.ImportFrom) and n.module
                       and n.module.startswith('mecanum_wand_hardware') for n in ast.walk(tree))


def test_mcu_sample_clock_never_renews_cached_or_reordered_feedback_and_handles_wrap():
    c = McuSampleClock()
    assert c.observe('wheel', 0xfffffff0, 10.) == 10.
    assert c.observe('diagnostic', 0xfffffff0, 10.01) == 10.
    assert c.observe('wheel', 0xfffffff0, 10.02) is None
    assert c.observe('wheel', 0x18, 10.04) == pytest.approx(10.04)
    assert c.observe('wheel', 0x10, 10.05) is None
    assert c.observe('wheel', 0x20, 10.5) is None  # Advancing but stale backlog.
    assert c.observe('wheel', 0x28, 10.056) == pytest.approx(10.056)


def test_advancing_mcu_sample_arriving_earlier_than_initial_mapping_is_not_future_or_renewable():
    c = McuSampleClock()
    assert c.observe('wheel', 1000, 10.) == 10.
    assert c.observe('wheel', 1100, 10.09) == pytest.approx(10.09)
    assert c.observe('wheel', 1100, 10.095) is None
    assert c.accepted_at['wheel'] == pytest.approx(10.09)
    # Same MCU instant has the same mapped time, despite later diagnostic RX.
    assert c.observe('diagnostic', 1100, 10.091) == pytest.approx(10.09)
    assert c.observe('wheel', 1200, 10.45) is None  # Stale advancing backlog still rejected.


def test_foreign_session_packets_cannot_refresh_even_the_socket_sample_clock(monkeypatch):
    from mecanum_wand_hardware.nodes import ProgressSocket
    from mecanum_vicon_control.transport_protocol import encode_frame, MessageType
    class QueueSocket:
        def __init__(self, packets): self.packets = packets
        def recvfrom(self, size):
            if not self.packets: raise BlockingIOError
            return self.packets.pop(0), ('127.0.0.1', 54600)
    bad = encode_frame(MessageType.LINK_STATUS, 1, 999, 1000)
    sock = ProgressSocket(QueueSocket([bad]), '127.0.0.1', lambda: 123)
    with pytest.raises(BlockingIOError): sock.recvfrom(1024)
    assert sock.clock.accepted_at == {}


def test_active_session_cannot_reset_used_mcu_budget_or_restore_lost_permit(monkeypatch):
    from mecanum_wand_hardware.nodes import HardwareBridge, CmdVelUdpBridge
    import mecanum_wand_hardware.nodes as nodes
    monkeypatch.setattr(nodes.time, 'monotonic', lambda: 10.02)
    calls = []
    for name in ('on_clear_faults', 'on_control_enable', 'on_cmd', 'on_enable'):
        monkeypatch.setattr(CmdVelUdpBridge, name, lambda *a, name=name: calls.append(name))
    signer, policy = AuthoritySigner(KEY), TransportPolicy(KEY)
    policy.receive(signer.encode(ZERO, 10., 100.), 10., 100.)
    policy.receive(signer.encode(ZERO, 10.01, 100.01, state='ARMED', permit=True, epoch='e'), 10.01, 100.01)
    policy.receive(signer.encode(Velocity(.02), 10.02, 100.02, state='ACTIVE', permit=True, epoch='e'), 10.02, 100.02)
    diagnostic = dict(tag='4WGB', started_ms=1000, stationary_ms=500, permitted=0,
                      compares=[0]*8, raw_duty=[0]*4, target_cps=[0]*4, body_milli=[0]*3)
    fake = SimpleNamespace(policy=policy, session=SimpleNamespace(connected=True),
        last_motion_status=diagnostic, last_motion_status_monotonic=10.02,
        last_wheel_telemetry_monotonic=10.02, last_mcu_fault_bits=0, last_wheel_rpm=(0,)*4,
        last_armed_epoch='e', enabled_epoch='e', rearm_epoch='', rearm_started=None,
        permit_observed_epoch='e', local_stop_latched=False, pending_control_action=None,
        last_feedback_identity=(10.02, 10.02), runtime=SimpleNamespace(mode='emulator'),
        stop_wire=lambda *a, **k: calls.append('stop'), force_local_disabled=lambda: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100020000000)),
        transport_info=SimpleNamespace(publish=lambda m: None))
    bind_preparation(fake, HardwareBridge)
    fake.prepared_epoch, fake.prepared_session = 'e', fake.session.session_id
    HardwareBridge.safety_tick(fake)
    assert policy.fault == 'MCU_PERMIT_LOST' and not policy.active(10.02)
    assert 'stop' in calls and 'on_clear_faults' not in calls and 'on_control_enable' not in calls


def test_late_feedback_in_armed_retries_local_stop_reset_before_ready(monkeypatch):
    from mecanum_wand_hardware.nodes import HardwareBridge, CmdVelUdpBridge
    import mecanum_wand_hardware.nodes as nodes
    monkeypatch.setattr(nodes.time, 'monotonic', lambda: 10.02)
    calls = []
    def control(fake, msg):
        calls.append(msg.data)
        fake.control_enable_reset_seen = not msg.data
    monkeypatch.setattr(CmdVelUdpBridge, 'on_control_enable', control)
    signer, policy = AuthoritySigner(KEY), TransportPolicy(KEY)
    policy.receive(signer.encode(ZERO, 10., 100.), 10., 100.)
    policy.receive(signer.encode(ZERO, 10.01, 100.01, state='ARMED', permit=True, epoch='e'), 10.01, 100.01)
    diagnostic = dict(tag='4WGB', started_ms=0, stationary_ms=500, permitted=0,
                      compares=[0]*8, raw_duty=[0]*4, target_cps=[0]*4, body_milli=[0]*3)
    fake = SimpleNamespace(policy=policy, session=SimpleNamespace(connected=True),
        last_motion_status=diagnostic, last_motion_status_monotonic=10.02,
        last_wheel_telemetry_monotonic=10.02, last_mcu_fault_bits=0, last_wheel_rpm=(0,)*4,
        last_armed_epoch='e', enabled_epoch='', rearm_epoch='', rearm_started=None,
        permit_observed_epoch='', local_stop_latched=True, control_enable_reset_seen=False,
        pending_control_action=None, last_feedback_identity=(10.02, 10.02),
        runtime=SimpleNamespace(mode='emulator'), stop_wire=lambda *a, **k: None,
        force_local_disabled=lambda: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100020000000)),
        transport_info=SimpleNamespace(publish=lambda m: None))
    bind_preparation(fake, HardwareBridge)
    HardwareBridge.safety_tick(fake)
    HardwareBridge.safety_tick(fake)
    assert calls == [False] and fake.control_enable_reset_seen and not policy.active(10.02)


def bind_preparation(fake, bridge):
    """Bind real adapter guards to the small, ROS-free test transport."""
    import types
    fake.session.session_id = 123
    fake.motion_gate_requested = fake.safety_enabled = False
    fake.last_command_progress_monotonic = fake.last_wheel_telemetry_monotonic
    for name in ('reset_preparation', 'feedback_is_fresh', 'baseline_is_current', 'baseline_is_prepared'):
        setattr(fake, name, types.MethodType(getattr(bridge, name), fake))
    fake.reset_preparation()


def test_ordinary_disable_retains_only_original_stamped_feedback(monkeypatch):
    from mecanum_wand_hardware.nodes import HardwareBridge, CmdVelUdpBridge
    snapshot = dict(permitted=1)
    # A real subtype without Node.__init__: no ROS node, socket or executor.
    fake = object.__new__(HardwareBridge)
    fake.session = SimpleNamespace(connected=True, session_id=123)
    fake.last_motion_status, fake.last_motion_status_monotonic = snapshot, 10.
    fake.reset_preparation = lambda: None
    def parent(self, action, repeats):
        self.last_motion_status = self.last_motion_status_monotonic = None
        self.motion_gate_requested = False
    monkeypatch.setattr(CmdVelUdpBridge, 'queue_control_action', parent)
    HardwareBridge.queue_control_action(fake, 'disable')
    assert fake.last_motion_status is snapshot and fake.last_motion_status['permitted'] == 1
    assert fake.last_motion_status_monotonic == 10. and not fake.motion_gate_requested
    # Do not transfer a cached observation to a different MC-Link session.
    def replaced(self, action, repeats):
        parent(self, action, repeats)
        self.session.session_id = 456
    monkeypatch.setattr(CmdVelUdpBridge, 'queue_control_action', replaced)
    HardwareBridge.queue_control_action(fake, 'disable')
    assert fake.last_motion_status is None and fake.last_motion_status_monotonic is None


def test_active_feedback_timeout_revokes_old_epoch_without_waiting_for_supervisor(monkeypatch):
    from mecanum_wand_hardware.nodes import HardwareBridge, CmdVelUdpBridge
    import mecanum_wand_hardware.nodes as nodes
    monkeypatch.setattr(nodes.time, 'monotonic', lambda: 10.22)
    calls = []
    for name in ('on_clear_faults', 'on_control_enable', 'on_cmd', 'on_enable'):
        monkeypatch.setattr(CmdVelUdpBridge, name, lambda *a, name=name: calls.append(name))
    signer, policy = AuthoritySigner(KEY), TransportPolicy(KEY)
    policy.receive(signer.encode(ZERO, 10., 100.), 10., 100.)
    policy.receive(signer.encode(ZERO, 10.01, 100.01, state='ARMED', permit=True, epoch='e'), 10.01, 100.01)
    policy.receive(signer.encode(Velocity(.02), 10.22, 100.22, state='ACTIVE', permit=True, epoch='e'), 10.22, 100.22)
    diagnostic = dict(tag='4WGC', started_ms=1000, stationary_ms=0, permitted=1,
        compares=[1]*8, raw_duty=[1]*4, target_cps=[1]*4, body_milli=[1]*3)
    fake = SimpleNamespace(policy=policy, session=SimpleNamespace(connected=True),
        last_motion_status=diagnostic, last_motion_status_monotonic=10.,
        last_wheel_telemetry_monotonic=10., last_mcu_fault_bits=0, last_wheel_rpm=(0,)*4,
        last_armed_epoch='e', enabled_epoch='e', rearm_epoch='', rearm_started=None,
        permit_observed_epoch='e', local_stop_latched=False, pending_control_action=None,
        last_feedback_identity=(10., 10.), runtime=SimpleNamespace(mode='emulator'),
        stop_wire=lambda *a, **k: calls.append('stop'), force_local_disabled=lambda: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100220000000)),
        transport_info=SimpleNamespace(publish=lambda m: None))
    bind_preparation(fake, HardwareBridge)
    HardwareBridge.safety_tick(fake)
    assert policy.fault == 'MCU_FEEDBACK_TIMEOUT' and 'e' in policy.blocked
    assert calls == ['stop', 'stop'] and not policy.active(10.22)
    policy.receive(signer.encode(Velocity(.02), 10.23, 100.23, state='ACTIVE', permit=True, epoch='e'), 10.23, 100.23)
    assert not policy.active(10.23) and policy.command == ZERO


def test_governed_profile_changes_only_pc_reference_policy_not_existing_hardware_bounds():
    config = Path(__file__).resolve().parents[1]/'config'
    old_path = config/'hardware_acceptance.corner_2x2_4wgc_2026-10-23.json'
    path = config/'hardware_acceptance.corner_2x2_4wgc_governed_2026-10-23.json'
    old = Runtime('shadow', str(old_path))
    new = Runtime('hardware', str(path), hashlib.sha256(path.read_bytes()).hexdigest(), CONSENT)
    assert new.motion_tag == old.motion_tag == '4WGC'
    assert new.profile['hardware'] == old.profile['hardware']
    assert new.config.goal_reference_policy == 'governed'
    assert old.config.goal_reference_policy == 'direct'
    for k, value in asdict(old.config).items():
        if k != 'goal_reference_policy': assert getattr(new.config, k) == value
    assert hashlib.sha256(old_path.read_bytes()).hexdigest() == '513914a3c5877e8809421b987603956a081e15454081c8d343d0597b32ab1843'


def test_clock_refines_only_from_already_acceptable_advancing_packets_never_renews_cache():
    c = McuSampleClock()
    assert c.observe('wheel', 1000, 10.02) == pytest.approx(10.02)
    anchor = c.local_baseline
    assert c.observe('wheel', 1100, 10.101) == pytest.approx(10.101)
    assert c.local_baseline == pytest.approx(anchor-.019)
    old_wheel = c.accepted_at['wheel']
    assert c.observe('diagnostic', 1100, 10.104) == pytest.approx(10.101)
    assert c.accepted_at['wheel'] == old_wheel
    assert c.observe('wheel', 1100, 10.105) is None
    assert c.accepted_at['wheel'] == old_wheel
    assert c.observe('wheel', 1200, 10.201) == pytest.approx(10.201)
    assert c.observe('wheel', 1300, 10.601) is None  # Real stale backlog remains stale.
    assert c.observe('wheel', 200000, 10.602) is None  # No arbitrary clock rebase.


def test_clock_first_packet_uncertainty_is_not_repaired_by_accepting_too_future_sample():
    c = McuSampleClock()
    assert c.observe('wheel', 1000, 10.04) == 10.04
    baseline = c.local_baseline
    assert c.observe('wheel', 1100, 10.10) is None  # -40ms: outside ORIGINAL -20ms guard.
    assert c.local_baseline == baseline and c.accepted_at['wheel'] == 10.04
    assert c.observe('diagnostic', 1100, 10.125) == pytest.approx(10.125)
    assert c.observe('wheel', 1200, 10.21) == pytest.approx(10.21)
    assert c.local_baseline <= baseline


def test_recorded_reopen_packets_have_fewer_false_future_rejections_without_relaxed_guards():
    root = Path(__file__).resolve().parents[3]
    record = root/'docs/wand_pc_governed_recovery_2026-10-23/reopen_filter_trace/raw_ros_before_filter.jsonl'
    if not record.exists(): pytest.skip('site raw UDP recording is not packaged')
    c, old_rejections, new_rejections = McuSampleClock(), 0, 0
    previous_baseline = None
    for line in record.open():
        v = json.loads(line)
        old_rejections += v['accepted_sample_steady'] is None
        new_rejections += c.observe(v['kind'], v['tick'], v['received_steady']) is None
        if previous_baseline is not None: assert c.local_baseline <= previous_baseline
        previous_baseline = c.local_baseline
    assert old_rejections == 9 and new_rejections == 1


def test_optional_receive_trace_records_both_acceptance_and_rejection_without_granting_authority(tmp_path):
    from mecanum_wand_hardware.nodes import ProgressSocket
    from mecanum_vicon_control.transport_protocol import encode_frame, MessageType
    class QueueSocket:
        def __init__(self, packets): self.packets, self.closed = packets, False
        def recvfrom(self, size):
            if not self.packets: raise BlockingIOError
            return self.packets.pop(0), ('127.0.0.1', 54600)
        def close(self): self.closed = True
    frame = encode_frame(MessageType.LINK_STATUS, 1, 123, 1000)
    raw = QueueSocket([frame,frame]); path = tmp_path/'capture.jsonl'
    wrapper = ProgressSocket(raw, '127.0.0.1', lambda:123, str(path))
    wrapper.recvfrom(512)
    with pytest.raises(BlockingIOError): wrapper.recvfrom(512)
    wrapper.close()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert len(rows) == 2 and rows[0]['accepted_sample_steady'] is not None
    assert rows[1]['accepted_sample_steady'] is None and raw.closed
