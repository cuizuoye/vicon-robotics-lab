"""离线报文预算和失联参数解读，不连接串口。"""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from link_budget import budget, frequency_options, packet_sizes, safety_findings  # noqa: E402


def test_actual_encoded_packet_sizes_and_full_odometry_overload():
    sizes = packet_sizes()
    assert sizes['ODOMETRY'] == 244
    assert sizes['SET_POSITION_TARGET_LOCAL_NED'] == 65
    assert 120 < sizes['VISION_POSITION_ESTIMATE_covariance'] < 140
    profile = budget(sizes)
    assert profile['parts_bytes_per_second']['ODOMETRY'] == 7320
    assert not profile['capacity_comparison']['standard_57600']['fits_even_without_headroom']
    assert not profile['capacity_comparison']['high-speed_115200']['fits_even_without_headroom']


def test_compact_candidate_fits_only_high_speed_with_headroom():
    candidate = budget(packet_sizes(), 'VISION_POSITION_ESTIMATE_covariance')
    assert not candidate['capacity_comparison']['standard_57600']['fits_planning_limit']
    assert candidate['capacity_comparison']['high-speed_115200']['fits_planning_limit']
    assert not candidate['execution_verified']


def test_signing_and_emergency_burst_reserve_are_not_forgotten():
    sizes = packet_sizes()
    unsigned, signed = budget(sizes), budget(sizes, signing=True)
    assert signed['planned_peak_uplink_bytes_per_second'] - unsigned['planned_peak_uplink_bytes_per_second'] == 13 * 66
    assert unsigned['parts_bytes_per_second']['STOP_COMMAND_LONG'] == sizes['STOP_COMMAND_LONG'] * 5


def test_rc_exception_does_not_masquerade_as_offboard_loss_protection():
    findings = safety_findings({'COM_RCL_EXCEPT': 4, 'COM_OBL_RC_ACT': 0,
                                'NAV_DLL_ACT': 3, 'NAV_RCL_ACT': 3})
    assert len(findings) == 4
    assert any('ignored while in Offboard' in item for item in findings)
    assert any('NOT an executed' in item for item in findings)


@pytest.mark.parametrize('hz', [0, 2, 4, 51, float('nan')])
def test_control_rate_not_lowered_to_offboard_survival_threshold(hz):
    with pytest.raises(ValueError):
        budget(packet_sizes(), control_rate_hz=hz)


@pytest.mark.parametrize('hz', [5, 10, 29, 51, float('nan')])
def test_external_vision_not_arbitrarily_throttled_below_documented_planning_rate(hz):
    with pytest.raises(ValueError):
        budget(packet_sizes(), vision_rate_hz=hz)


def test_lower_control_rate_saves_bytes_but_not_enough_for_standard_trs():
    sizes = packet_sizes()
    slow = budget(sizes, control_rate_hz=10)
    fast = budget(sizes)
    assert fast['planned_peak_uplink_bytes_per_second'] - slow['planned_peak_uplink_bytes_per_second'] == 650
    assert not slow['capacity_comparison']['standard_57600']['fits_even_without_headroom']


def test_downlink_candidate_is_budget_only_and_compact_requires_fast_mode():
    result = frequency_options(packet_sizes())
    assert 600 <= result['planned_periodic_downlink_bytes_per_second'] <= 1000
    assert result['flight_controller_rate_changes'] == result['radio_mode_changes'] == []
    compact = result['profiles']['compact_covariance_control_10hz_candidate']
    assert compact['if_radio_capacity_is_shared_both_directions']['fits_high_speed_20_percent_headroom']
    assert not compact['capacity_comparison']['standard_57600']['fits_planning_limit']
