from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import gamepad_estop_bench as bench


def args(port, guarded=False, props_off=False):
    return SimpleNamespace(usb_port=str(port), bench_lockdown=props_off,
                           usb_only_props_installed_stop_only_confirmed=guarded)


def test_installed_props_are_truthfully_recorded_without_clearing_hold(tmp_path, monkeypatch):
    usb = tmp_path / 'fc'
    usb.touch()
    monkeypatch.setattr(bench, 'FC_USB', usb)
    result = bench.physical_mode(args(usb, guarded=True))
    assert result['propeller_state'] == 'installed'
    assert result['propulsion_battery_disconnected_operator_confirmed']
    assert 'operator report' in result['physical_conditions_source']


@pytest.mark.parametrize('reason', ['absent', 'wrong_port', 'conflicting'])
def test_guarded_stop_confirmation_fails_closed(tmp_path, monkeypatch, reason):
    usb = tmp_path / 'fc'
    if reason != 'absent':
        usb.touch()
    monkeypatch.setattr(bench, 'FC_USB', usb)
    port = tmp_path / 'another' if reason == 'wrong_port' else usb
    with pytest.raises(ValueError):
        bench.physical_mode(args(port, guarded=True, props_off=reason == 'conflicting'))


def test_diagnostic_mode_does_not_invent_confirmation(tmp_path):
    result = bench.physical_mode(args(tmp_path / 'fc'))
    assert result['propeller_state'] == 'not_confirmed'
    assert not result['propulsion_battery_disconnected_operator_confirmed']


def test_original_props_off_mode_still_records_removed(tmp_path):
    assert bench.physical_mode(args(tmp_path / 'fc', props_off=True))['propeller_state'] == 'removed'
