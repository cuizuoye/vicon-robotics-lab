from types import SimpleNamespace

from mecanum_vicon_control.rectangle_run_guard import (
    RectangleRunGuard, disabled_motion_status_ready, motion_status_needs_rearm,
    rearmed_motion_status_ready, vicon_valid_with_grace,
)


def status(**overrides):
    value = {
        'tag': '4WM1',
        'permitted': 0,
        'stationary_ms': 500,
        'started_ms': 0,
        'compares': [0] * 8,
        'raw_duty': [0] * 4,
        'target_cps': [0] * 4,
        'body_milli': [0] * 3,
    }
    value.update(overrides)
    return value


def test_disabled_motion_status_accepts_only_safe_preflight_image():
    assert disabled_motion_status_ready(status())
    assert disabled_motion_status_ready(status(tag='4WG1'))
    assert disabled_motion_status_ready(status(tag='4WG2'))
    assert disabled_motion_status_ready(status(tag='4WG3'))
    assert disabled_motion_status_ready(status(tag='4WG4'))
    assert disabled_motion_status_ready(status(tag='4WG7'))
    assert disabled_motion_status_ready(status(tag='4WG8'))
    assert disabled_motion_status_ready(status(tag='4WG9'))
    assert disabled_motion_status_ready(status(tag='4WGA'))
    assert disabled_motion_status_ready(status(tag='4WGB'))
    assert not disabled_motion_status_ready(status(tag='4RO1'))
    assert not disabled_motion_status_ready(status(permitted=1))
    assert not disabled_motion_status_ready(status(stationary_ms=499))
    assert not disabled_motion_status_ready(status(raw_duty=[0, 0, 10, 0]))
    assert not disabled_motion_status_ready({'tag': '4WM1'})


def test_rearm_confirmation_requires_zero_started_time_and_safe_output():
    assert rearmed_motion_status_ready(status(tag='4WG9', started_ms=0))
    assert rearmed_motion_status_ready(status(tag='4WGA', started_ms=0))
    assert rearmed_motion_status_ready(status(tag='4WGB', started_ms=0))
    assert not rearmed_motion_status_ready(status(tag='4WG9', started_ms=1234))
    assert not rearmed_motion_status_ready(status(tag='4WG9', permitted=1))


def test_rearm_is_only_requested_after_a_run_consumed_the_budget():
    assert not motion_status_needs_rearm(status(tag='4WG9', started_ms=0))
    assert motion_status_needs_rearm(status(tag='4WG9', started_ms=1234))
    assert motion_status_needs_rearm(status(tag='4WGA', started_ms=1234))
    assert motion_status_needs_rearm(status(tag='4WGB', started_ms=1234))
    assert motion_status_needs_rearm({})


class QuietLogger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


def test_preflight_first_publishes_explicit_disable_boundary():
    calls = []
    node = SimpleNamespace(
        state='preflight', state_started=0.0, control_pub='control',
        preflight_timeout_s=15.0,
        preflight_ready=lambda: True,
        publish=lambda publisher, value: calls.append((publisher, value)),
        set_state=lambda value: setattr(node, 'state', value),
        get_logger=lambda: QuietLogger(),
    )
    RectangleRunGuard.tick(node)
    assert calls == [('control', False)]
    assert node.state == 'wait_disabled'


def test_disabled_boundary_precedes_4wg9_rearm(monkeypatch):
    calls = []
    node = SimpleNamespace(
        state='wait_disabled', state_started=10.0, fault=0,
        enable_timeout_s=5.0, safe_rearm_supported=True,
        diagnostics=status(tag='4WG9', started_ms=1234),
        rearm_pub='rearm', control_pub='control',
        vicon_healthy=lambda: True, preflight_ready=lambda: True,
        publish=lambda publisher, value: calls.append((publisher, value)),
        set_state=lambda value: setattr(node, 'state', value),
        begin_stop=lambda _reason: None,
        get_logger=lambda: QuietLogger(),
    )
    monkeypatch.setattr(
        'mecanum_vicon_control.rectangle_run_guard.time.monotonic',
        lambda: 10.31)
    RectangleRunGuard.tick(node)
    assert calls == [('rearm', True)]
    assert node.state == 'wait_rearm'


def test_vicon_grace_accepts_only_recent_last_true_sample():
    assert vicon_valid_with_grace(True, True, None, 10.0, 0.15)
    assert vicon_valid_with_grace(False, True, 9.90, 10.0, 0.15)
    assert not vicon_valid_with_grace(False, True, 9.84, 10.0, 0.15)
    assert not vicon_valid_with_grace(True, False, None, 10.0, 0.15)
