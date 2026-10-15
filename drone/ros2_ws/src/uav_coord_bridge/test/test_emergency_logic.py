"""Tests for emergency confirmation logic."""

from uav_coord_bridge.emergency_logic import TerminationGate, ack_result_name


def test_termination_is_disabled_by_default():
    """Termination preparation must fail when disabled."""
    gate = TerminationGate()
    accepted, reason = gate.prepare(10.0)
    assert not accepted
    assert '未启用' in reason


def test_prepare_and_confirm_inside_window_is_one_shot():
    """A valid confirmation window can only be consumed once."""
    gate = TerminationGate(enabled=True, window_s=5.0)
    assert gate.prepare(10.0)[0]
    assert gate.confirm(14.9)[0]
    accepted, reason = gate.confirm(14.9)
    assert not accepted
    assert '没有待确认' in reason


def test_expired_confirmation_is_consumed():
    """An expired window must fail and be cleared."""
    gate = TerminationGate(enabled=True, window_s=5.0)
    assert gate.prepare(10.0)[0]
    accepted, reason = gate.confirm(15.1)
    assert not accepted
    assert '已过期' in reason
    assert not gate.confirm(15.1)[0]


def test_cancel_clears_pending_confirmation():
    """Cancel must clear a pending window."""
    gate = TerminationGate(enabled=True, window_s=5.0)
    assert gate.prepare(10.0)[0]
    assert gate.cancel()[0]
    assert not gate.confirm(11.0)[0]


def test_invalid_confirmation_window_is_rejected():
    """Non-positive confirmation windows are invalid."""
    gate = TerminationGate(enabled=True, window_s=0.0)
    accepted, reason = gate.prepare(10.0)
    assert not accepted
    assert '大于 0' in reason


def test_ack_result_names():
    """Known and unknown PX4 ACK codes have stable labels."""
    assert ack_result_name(0) == 'ACCEPTED'
    assert ack_result_name(6) == 'CANCELLED'
    assert ack_result_name(255) == 'UNKNOWN(255)'
