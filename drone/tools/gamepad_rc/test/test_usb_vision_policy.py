import math
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'ros2_ws/src/uav_coord_bridge'))
from usb_vision_policy import (PARAMS, fusion_disabled, send_allowed, shell_message,
                              parse_visual_snapshot, match_snapshot, parse_timesync_snapshot,
                              check_clock_probe, SourceLossLatch, ClockReadiness, assess_clock_series)
from uav_coord_bridge.mavlink_vision import odometry_message


def encoder():
    return m.MAVLink(None, srcSystem=250, srcComponent=190)


def test_default_blocks_actual_position():
    msg = odometry_message(encoder(), 123, [1, 2, 3], [1, 0, 0, 0])
    assert not send_allowed(msg, safe=True)
    assert not send_allowed(msg, bench_authorized=True, safe=False)
    assert send_allowed(msg, bench_authorized=True, safe=True)


@pytest.mark.parametrize('field,value', [('frame_id', m.MAV_FRAME_LOCAL_NED), ('estimator_type', m.MAV_ESTIMATOR_TYPE_AUTOPILOT),
                                       ('quality', 100), ('vx', 0), ('q', [0, 0, 0, 0]), ('time_usec', 0)])
def test_wrong_frame_estimator_or_fabricated_velocity_blocked(field, value):
    msg = odometry_message(encoder(), 123, [1, 2, 3], [1, 0, 0, 0])
    setattr(msg, field, value)
    assert not send_allowed(msg, bench_authorized=True, safe=True)


@pytest.mark.parametrize('command', [400, 176, 21, 22, 185, 241, 246, 511])
def test_control_and_frequency_commands_blocked(command):
    msg = encoder().command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0)
    assert not send_allowed(msg, bench_authorized=True, safe=True)


def test_parameter_set_and_arbitrary_shell_blocked():
    enc = encoder()
    assert not send_allowed(enc.param_set_encode(1, 1, b'EKF2_EV_CTRL', 15, 6), bench_authorized=True, safe=True)
    payload = b'commander arm\n'
    assert not send_allowed(enc.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0')), safe=True)
    assert send_allowed(shell_message(enc, 'vehicle_visual_odometry'), safe=True)
    assert not send_allowed(shell_message(enc, 'vehicle_visual_odometry'), safe=False)


def test_missing_or_enabled_fusion_rejected():
    assert not fusion_disabled({})
    params = dict.fromkeys(PARAMS, 0)
    assert fusion_disabled(params)
    for name in PARAMS:
        assert not fusion_disabled({**params, name: 1})


TEXT = ('TOPIC: vehicle_visual_odometry\n timestamp: 1000\n timestamp_sample: 999\n'
        ' pose_frame: 2\n position: [1, 2, 3]\n q: [1, 0, 0, 0]\n'
        ' velocity_frame: 0\n reset_counter: 0\n quality: 0\n')


def test_parse_full_snapshot_and_match_only_exact_frame_position():
    snapshot = parse_visual_snapshot(TEXT)
    sent = [{'position': [1, 2, 3], 'q': [1, 0, 0, 0], 'source_stamp_ns': 100}]
    assert match_snapshot(snapshot, sent)['source_stamp_ns'] == 100
    assert not match_snapshot({**snapshot, 'position': [2, 2, 3]}, sent)
    assert not match_snapshot({**snapshot, 'pose_frame': 1}, sent)


@pytest.mark.parametrize('text', [TEXT.replace(' quality: 0\n', ''), TEXT.replace('[1, 2, 3]', '[nan, 2, 3]'), 'never published'])
def test_incomplete_or_nonfinite_output_not_evidence(text):
    assert parse_visual_snapshot(text) is None


def test_timesync_does_not_allow_system_time_clock_set():
    enc = encoder()
    assert send_allowed(enc.timesync_encode(123, 456), safe=True)
    assert not send_allowed(enc.timesync_encode(123, 456), safe=False)
    assert not send_allowed(enc.system_time_encode(123, 456), safe=True)


SYNC_TEXT = ('TOPIC: timesync_status\n timestamp: 1000000\n source_protocol: 1\n'
             ' remote_timestamp: 9000000\n observed_offset: -8000000\n'
             ' estimated_offset: -8000000\n round_trip_time: 1200\n')


def test_fixed_clock_listener_and_signed_offset_parser():
    assert send_allowed(shell_message(encoder(), 'timesync_status'), safe=True)
    assert not send_allowed(shell_message(encoder(), 'timesync_status'), safe=False)
    assert parse_timesync_snapshot(SYNC_TEXT)['estimated_offset'] == -8000000
    assert parse_timesync_snapshot(SYNC_TEXT.replace(' source_protocol: 1', ' source_protocol: 0'))['source_protocol'] == 0
    assert parse_timesync_snapshot(SYNC_TEXT.replace(' source_protocol: 1', ' source_protocol: 2')) is None
    assert parse_timesync_snapshot(SYNC_TEXT.replace(' round_trip_time: 1200\n', '')) is None


def test_held_real_sample_distinguishes_clock_mapping_from_fallback():
    status = parse_timesync_snapshot(SYNC_TEXT)
    row = {'wire_time_usec': 8950000, 'source_stamp_ns': 123, 'position': [1, 2, 3], 'q': [1, 0, 0, 0]}
    snapshot = {**parse_visual_snapshot(TEXT), 'timestamp': 1000000, 'timestamp_sample': 950000}
    assert check_clock_probe(status, snapshot, row, 9100000)['verified']
    assert not check_clock_probe(status, {**snapshot, 'timestamp_sample': 999999}, row, 9100000)['verified']
    assert not check_clock_probe({**status, 'round_trip_time': 15000}, snapshot, row, 9100000)['verified']
    assert not check_clock_probe(status, snapshot, row, 15000000)['verified']
    assert not check_clock_probe(status, {**snapshot, 'position': [2, 2, 3]}, row, 9100000)['verified']
    lagged = check_clock_probe({**status, 'observed_offset': -7980000}, snapshot, row, 9100000)
    assert lagged['verified']
    assert not lagged['clock_accuracy_candidate_pass']


def test_input_loss_latches_even_when_fresh_input_returns():
    latch = SourceLossLatch()
    assert latch.permits(True)
    assert not latch.permits(False)
    assert not latch.permits(True)


def test_clock_requires_three_new_fresh_small_residual_snapshots():
    gate = ClockReadiness()
    status = parse_timesync_snapshot(SYNC_TEXT)
    for i in range(3):
        assert gate.update({**status, 'timestamp': i+1}, 9100000) is (i == 2)
    assert not gate.update({**status, 'timestamp': 4, 'observed_offset': -7980000}, 9100000)
    assert gate.count == 0
    assert not gate.update({**status, 'timestamp': 5}, 11000000)
    assert not gate.update({**status, 'timestamp': 5}, 9100000)
    assert not gate.update({**status, 'timestamp': 6, 'round_trip_time': 10000}, 9100000)


def test_clock_drift_summary_does_not_claim_independent_accuracy():
    status = parse_timesync_snapshot(SYNC_TEXT)
    rows = [{'status': {**status, 'observed_offset': -8000000 + r}} for r in (19000, 1000, 500, 200)]
    requests = [{'pc_receive_ns': 1000000000+i*1000000000,
                 'fc_originate_ns': 2000000000+i*1000100000} for i in range(20)]
    result = assess_clock_series(rows, requests)
    assert result['last_three_within_2ms']
    assert result['fc_vs_pc_arrival_clock_rate_ppm'] == pytest.approx(100)
    assert not result['absolute_clock_accuracy_independently_verified']
    assert assess_clock_series([], [])['last_residual_us'] is None
