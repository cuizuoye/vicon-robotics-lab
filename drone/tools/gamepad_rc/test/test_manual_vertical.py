import copy
import math
from pathlib import Path
import subprocess
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from manual_vertical import endpoint, assess_vertical


def rows():
    return [{'elapsed_s': i * .25, 'local': {'timestamp_sample': 100+i, 'xy_valid': True, 'z_valid': True,
                                          'x': 1, 'y': 2, 'z': -20},
             'sent': {'source_stamp_ns': 100+i, 'position': [0,0,0], 'q': [1,0,0,0]},
             'fc_q': [1,0,0,0], 'attitude_age_ms': 50} for i in range(5)]


def test_vertical_endpoint_adds_valid_height_and_full_3d_stationarity():
    assert endpoint(rows())['fc_3d_stationary_spread_m'] == 0
    r = rows(); r[-1]['local']['z_valid'] = False
    assert endpoint(r) is None
    r = rows(); r[-1]['local']['z'] -= .05
    assert endpoint(r) is None
    r = rows(); r[-1]['local']['z'] = math.nan
    assert endpoint(r) is None
    assert endpoint(rows()[:4]) is None


def endpoints():
    start = endpoint(rows())
    end = copy.deepcopy(start)
    end['ev_position_frd_m'] = [0,0,-.2]
    end['fc_position_m'] = [1,2,-20.2]
    return start, end


def test_signed_delta_not_absolute_height_matches_different_origins():
    start, end = endpoints()
    r = assess_vertical(start, end)
    assert r['verified'] and r['observed_ekf_net_lift_m'] == pytest.approx(.2)
    assert not r['ground_zero_verified'] and not r['flight_ready']
    assert not assess_vertical(start, None)['verified']


def test_pure_yaw_is_warning_not_vertical_rejection():
    start, end = endpoints()
    end['ev_q_wxyz'] = [math.cos(math.radians(10)),0,0,math.sin(math.radians(10))]
    r = assess_vertical(start, end)
    assert r['verified'] and r['body_rotation_warning_only']
    assert r['tilt_axis_change_deg'] == pytest.approx(0)


@pytest.mark.parametrize('change', ['wrong_sign', 'wrong_scale', 'net_zero', 'sideways', 'tilt'])
def test_bad_vertical_observation_not_a_pass(change):
    start, end = endpoints()
    if change == 'wrong_sign': end['fc_position_m'][2] = -19.8
    if change == 'wrong_scale': end['fc_position_m'][2] = -20.4
    if change == 'net_zero': end['ev_position_frd_m'][2] = 0
    if change == 'sideways': end['ev_position_frd_m'][0] = .2
    if change == 'tilt': end['ev_q_wxyz'] = [math.cos(math.radians(6)),math.sin(math.radians(6)),0,0]
    assert not assess_vertical(start, end)['verified']


def test_world_alignment_includes_vertical_tilt_projection():
    start, end = endpoints()
    start['fc_q_wxyz'] = [math.cos(math.radians(2)),0,math.sin(math.radians(2)),0]
    end['fc_position_m'][2] = -20 - .2 * math.cos(math.radians(4))
    assert assess_vertical(start, end)['vertical_increment_difference_m'] == pytest.approx(0)


@pytest.mark.parametrize('extra', [[], ['--manual-translation-bench'], ['--manual-window-seconds', '181']])
def test_cli_requires_height_mode_exclusive_manual_profile_and_bounded_window(tmp_path, extra):
    script = Path(__file__).resolve().parents[1] / 'usb_vicon_receive_bench.py'
    cmd = [sys.executable, str(script), '--port', '/dev/nonexistent_vertical_test', '--uid2', 'test',
           '--heading-candidate', '/nonexistent_heading', '--board-center-candidate', '/nonexistent_board',
           '--output', str(tmp_path / 'new.json'), '--manual-vertical-bench', *extra]
    r = subprocess.run(cmd, capture_output=True, text=True)
    assert r.returncode == 2 and not (tmp_path / 'new.json').exists()
    assert 'manual' in r.stderr.lower()
