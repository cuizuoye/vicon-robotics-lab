"""Host wand adapter only: no ROS executor, socket or real MCU is used."""
from types import MethodType, SimpleNamespace

import pytest

from mecanum_wand_control.safety_core import ZERO
from mecanum_wand_hardware.commissioning import AuthoritySigner, TransportPolicy
from mecanum_wand_hardware.nodes import HardwareBridge, CmdVelUdpBridge

KEY = '23'*32


def transport(monkeypatch, *, started=0):
    import mecanum_wand_hardware.nodes as nodes
    clock = [10.]
    monkeypatch.setattr(nodes.time, 'monotonic', lambda: clock[0])
    signer, policy = AuthoritySigner(KEY), TransportPolicy(KEY)
    policy.receive(signer.encode(ZERO, 10., 100.), 10., 100.)
    status = dict(tag='4WGB', started_ms=started, stationary_ms=500, permitted=0,
        compares=[0]*8, raw_duty=[0]*4, target_cps=[0]*4, body_milli=[0]*3)
    reports, calls = [], []
    f = SimpleNamespace(policy=policy, session=SimpleNamespace(connected=True, session_id=123),
        sock=object(), last_motion_status=status, last_motion_status_monotonic=10.,
        last_wheel_telemetry_monotonic=10., last_command_progress_monotonic=10.,
        last_receive_monotonic=10., last_telemetry_command_sequence=55,
        last_mcu_fault_bits=0, last_wheel_rpm=(0,)*4, last_armed_epoch='',
        enabled_epoch='', rearm_epoch='', rearm_started=None, permit_observed_epoch='',
        local_stop_latched=False, control_enable_reset_seen=False, readonly_mode=False,
        motion_gate_requested=False, safety_enabled=False, pending_control_action=None,
        last_feedback_identity=None, feedback_sequence=0, runtime=SimpleNamespace(mode='emulator'),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int((clock[0]+90)*1e9))),
        get_logger=lambda: SimpleNamespace(error=lambda text: calls.append(('error', text)),
                                          info=lambda text: None, warn=lambda text: None),
        transport_info=SimpleNamespace(publish=lambda m: reports.append(m.data)),
        feedback=SimpleNamespace(publish=lambda m: None),
        force_local_disabled=lambda: None, send_hello_if_due=lambda: calls.append('HELLO'))
    monkeypatch.setattr(CmdVelUdpBridge, 'on_cmd', lambda *args: None)
    monkeypatch.setattr(CmdVelUdpBridge, 'on_enable', lambda *args: None)
    for name in ('reset_preparation', 'feedback_is_fresh', 'baseline_is_current',
                 'baseline_is_prepared', 'queue_control_action', 'stop_wire', 'safety_tick'):
        setattr(f, name, MethodType(getattr(HardwareBridge, name), f))
    f.reset_preparation()
    def update(state='ARMED', step=.1, fresh=True):
        clock[0] += step
        if fresh:
            f.last_motion_status_monotonic = f.last_wheel_telemetry_monotonic = clock[0]
            f.last_command_progress_monotonic = clock[0]
        policy.receive(signer.encode(ZERO, clock[0], clock[0]+90, state=state,
            epoch='e', permit=True), clock[0], clock[0]+90)
        f.safety_tick()
    return f, clock, update, calls, reports


def prepare(f, update):
    for _ in range(8): update()
    assert f.prepared_epoch == 'e' and f.prepared_session == 123


def test_armed_primes_disabled_session_for_600ms_before_ready(monkeypatch):
    import json
    f, clock, update, calls, reports = transport(monkeypatch)
    for _ in range(6):
        update()
        assert not json.loads(reports[-1])['ready']
        assert not f.motion_gate_requested and f.pending_control_action is None
    update(step=.11)
    assert json.loads(reports[-1])['ready'] and f.prepared_epoch == 'e'
    assert 'HELLO' not in calls


def test_enable_reuses_prepared_session_and_keeps_genuine_feedback_and_progress(monkeypatch):
    f, clock, update, calls, reports = transport(monkeypatch)
    prepare(f, update)
    identity = f.last_motion_status
    update('ACTIVE', step=.02)
    assert f.enabled_epoch == 'e' and f.pending_control_action == 'enable'
    assert f.motion_gate_requested and f.enable_prime_complete
    assert f.session.connected and f.session.session_id == 123
    assert f.last_motion_status is identity
    assert f.last_motion_status_monotonic == f.last_wheel_telemetry_monotonic == clock[0]
    assert f.last_receive_monotonic == 10. and f.last_telemetry_command_sequence == 55
    assert 'HELLO' not in calls and not f.policy.fault


@pytest.mark.parametrize('defect', ['not_primed', 'stale', 'fault', 'new_session', 'wrong_epoch', 'nonzero', 'progress_stale'])
def test_unprepared_same_session_enable_is_rejected(monkeypatch, defect):
    f, clock, update, calls, reports = transport(monkeypatch)
    prepare(f, update)
    if defect == 'not_primed': f.baseline_since = clock[0]
    if defect == 'stale': f.last_motion_status_monotonic -= .21
    if defect == 'fault': f.last_mcu_fault_bits = 32
    if defect == 'new_session': f.session.session_id = 456
    if defect == 'wrong_epoch': f.prepared_epoch = 'old'
    if defect == 'nonzero': f.last_motion_status['body_milli'][0] = 1
    if defect == 'progress_stale': f.last_command_progress_monotonic -= .21
    # Directly exercise the hook even if its caller attempted to bypass ready.
    p = f.policy.envelope.copy()
    p['state'] = 'ACTIVE'
    f.policy.envelope = p
    f.queue_control_action('enable')
    assert f.policy.fault == 'MCU_NOT_PREPARED'
    assert not f.motion_gate_requested and f.pending_control_action is None
    assert 'HELLO' not in calls


def test_used_budget_reset_occurs_only_armed_after_baseline_and_same_session(monkeypatch):
    f, clock, update, calls, reports = transport(monkeypatch, started=5000)
    for _ in range(6):
        update()
        assert f.pending_control_action is None and not f.prepared_epoch
    update(step=.11)
    assert f.pending_control_action == 'clear_faults' and f.pending_control_repeats == 1
    assert f.session.connected and f.last_motion_status is not None
    assert not f.motion_gate_requested and not f.prepared_epoch and 'HELLO' not in calls
    assert f.rearm_started == clock[0]


def test_budget_reset_cannot_run_in_active(monkeypatch):
    f, clock, update, calls, reports = transport(monkeypatch, started=5000)
    # An otherwise valid baseline cannot authorize an ACTIVE budget reset.
    f.baseline_since, f.baseline_epoch, f.baseline_session = 9., 'e', 123
    f.policy.epoch = 'e'
    f.policy.envelope = dict(state='ACTIVE')
    f.queue_control_action('clear_faults', repeats=1)
    assert f.pending_control_action is None and f.policy.fault == 'MCU_NOT_PREPARED'


def test_armed_feedback_gap_discards_preparation_not_renewed_by_cached_status(monkeypatch):
    f, clock, update, calls, reports = transport(monkeypatch)
    prepare(f, update)
    # Signed authority is live; only the MCU diagnostic is stale.
    update(step=.21, fresh=False)
    assert f.baseline_since is None and not f.prepared_epoch
    update()
    assert f.baseline_since == clock[0] and not f.prepared_epoch


def test_active_session_change_stops_without_hello_or_reenable(monkeypatch):
    f, clock, update, calls, reports = transport(monkeypatch)
    prepare(f, update)
    f.session.session_id = 456
    update('ACTIVE', step=.02)
    assert f.policy.fault == 'MCU_PREPARED_SESSION_LOST'
    assert f.pending_control_action is None and not f.motion_gate_requested
    assert 'HELLO' not in calls


def test_enable_requires_permit_confirmation_with_bounded_ack_deadline(monkeypatch):
    f, clock, update, calls, reports = transport(monkeypatch)
    prepare(f, update)
    update('ACTIVE', step=.02)
    # Simulate the inherited sender having sent the repeated controls, but no
    # MCU permit arriving despite fresh healthy telemetry. Never re-enable.
    f.pending_control_action = None
    f.motion_gate_requested = False
    for _ in range(11): update('ACTIVE')
    assert f.policy.fault == 'MCU_ENABLE_TIMEOUT' and not f.prepared_epoch
