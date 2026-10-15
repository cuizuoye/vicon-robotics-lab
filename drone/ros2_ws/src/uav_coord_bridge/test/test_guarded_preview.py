from dataclasses import replace
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uav_coord_bridge.guarded_preview import GuardedPreview
from uav_coord_bridge.mission_preview import Observation


def obs(t=1.):
    return Observation(t, (2., 3., .1))


def ready():
    g = GuardedPreview()
    g.observe_device(0., 'receiver-session-1', connected=True)
    g.home_event(.1, 1)
    g.home_event(.3, 0)
    return g


def test_receiver_presence_alone_is_not_response():
    g = GuardedPreview()
    g.observe_device(0., 'receiver', connected=True)
    assert not g.start(obs(), confirmed=True)


def test_release_alone_or_held_home_cannot_pass():
    g = GuardedPreview()
    g.observe_device(0., 'receiver', connected=True)
    g.home_event(.1, 0)
    assert not g.start(obs(.2), confirmed=True)
    g.home_event(.3, 1)
    assert not g.start(obs(.4), confirmed=True)


def test_response_does_not_start_or_clear_any_real_lock():
    g = ready()
    assert g.tick(obs())['state'] == 'WAIT_START'
    assert not g.start(obs())
    assert g.start(obs(), confirmed=True)
    result = g.tick(obs(2.))
    assert result['actual_sent_to_fc'] == 0
    assert not result['hardware_adapter_present']
    assert not result['live_input_adapter_present']
    assert not result['flight_ready']


def test_stale_prestart_response_is_rejected():
    assert not ready().start(obs(31.), confirmed=True)


def test_normal_idle_does_not_invent_sleep_failure():
    g = ready()
    assert g.start(obs(), confirmed=True)
    # Stay in the same synthetic ascending state; no evdev events required.
    g.response_max_age_s = .1
    assert g.tick(obs(2.))['state'] == 'ASCEND'
    assert not g.tick(obs(3.))['sleep_detection_present']


@pytest.mark.parametrize('cause', ['disconnect', 'replacement', 'overflow'])
def test_input_fault_lands_advisory_and_reconnect_never_resumes(cause):
    g = ready()
    assert g.start(obs(), confirmed=True)
    if cause == 'disconnect':
        g.observe_device(2., 'receiver-session-1', connected=False)
    elif cause == 'replacement':
        g.observe_device(2., 'receiver-session-2', connected=True)
    else:
        g.input_fault('input_queue_overflow')
    airborne = replace(obs(3.), armed=True, landed=False)
    result = g.tick(airborne)
    assert result['state'] == 'ABORT_LAND'
    assert result['intent'] == 'preview_land_and_alert'
    assert result['actual_sent_to_fc'] == 0
    g.observe_device(4., 'receiver-session-1', connected=True)
    assert g.tick(replace(airborne, t=5.))['state'] == 'ABORT_LAND'
    assert g.fault and not g.start(obs(6.), confirmed=True)


def test_home_during_session_is_sticky_not_a_response_challenge():
    g = ready()
    assert g.start(obs(), confirmed=True)
    g.home_event(2., 1)
    g.home_event(2.2, 0)
    assert g.tick(obs(3.))['state'] == 'STOP_LATCHED'
    assert not g.start(obs(4.), confirmed=True)


def test_existing_external_emergency_gate_cannot_be_overridden():
    assert not ready().start(replace(obs(), emergency_available=False), confirmed=True)


def test_clock_failure_is_sticky():
    g = ready()
    assert not g.start(obs(.2), confirmed=True)
    assert g.fault
