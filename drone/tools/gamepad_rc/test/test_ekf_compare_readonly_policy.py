import copy
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ekf_compare_readonly_policy as p
from usb_vision_policy import shell_message as empty_shell


SELECTOR = ('TOPIC: estimator_selector_status\n timestamp: 1000\n primary_instance: 1\n'
            ' instances_available: 2\n instance_changed_count: 2\n last_instance_change: 950\n'
            ' accel_device_id: 1\n gyro_device_id: 1\n baro_device_id: 3\n mag_device_id: 0\n'
            ' gyro_fault_detected: False\n accel_fault_detected: False\n'
            ' combined_test_ratio: [1, 1, nan, nan, nan, nan, nan, nan, nan]\n'
            ' relative_test_ratio: [1, 0, nan, nan, nan, nan, nan, nan, nan]\n'
            ' healthy: [True, True, False, False, False, False, False, False, False]\n')


def enc():
    return m.MAVLink(None, srcSystem=250, srcComponent=190)


def test_full_selector_preserves_inactive_nan_without_fabricating_error_zero():
    r = p.parse_selector(SELECTOR)
    assert r['healthy'][:2] == [True, True]
    assert r['combined_test_ratio'][2] is None
    assert r['primary_instance'] == 1


@pytest.mark.parametrize('bad', [SELECTOR.replace('[1, 1, nan', '[nan, 1, nan'),
                                 SELECTOR.replace('[1, 1, nan', '[1, inf, nan'),
                                 SELECTOR.replace('primary_instance: 1', 'primary_instance: 2'),
                                 SELECTOR.replace('last_instance_change: 950', 'other: 950')])
def test_selector_rejects_incomplete_invalid_active_fields(bad):
    assert p.parse_selector(bad) is None


def test_only_reads_and_fixed_queries_no_instance_switch_or_write():
    assert p.allowed(p.shell_message(enc(), 'xy_1'), safe=True)
    assert not p.allowed(p.shell_message(enc(), 'xy_1'))
    assert p.allowed(enc().param_request_read_encode(1, 1, b'EKF2_MULTI_IMU', -1))
    assert not p.allowed(enc().param_set_encode(1, 1, b'EKF2_MULTI_IMU', 1, m.MAV_PARAM_TYPE_INT32), safe=True)
    for command in (400, 176, 185, 241, 246, 511, 42007):
        assert not p.allowed(enc().command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0), safe=True)
    text = b'ekf2 select_instance 0\n'
    assert not p.allowed(enc().serial_control_encode(10, 2, 0, 0, len(text), text.ljust(70, b'\0')), safe=True)
    assert not p.allowed(empty_shell(enc(), close=True))
    with pytest.raises(KeyError):
        p.shell_message(enc(), 'xy_9')


def row():
    def instance(i):
        return {'flags': {'cs_tilt_align': True, 'cs_yaw_align': False, 'cs_ev_pos': False,
                          'cs_ev_hgt': False, 'fs_bad_pos_d': False},
                'baro': {'estimator_instance': i, 'fusion_enabled': True, 'fused': True,
                         'innovation_rejected': False, 'time_last_fuse': 990,
                         'timestamp': 1000, 'timestamp_sample': 900},
                'ev': {'timestamp_sample': 500, 'time_last_fuse': 450}}
    return {'selector': p.parse_selector(SELECTOR), 'local': {'timestamp': 1100},
            'instances': {'0': instance(0), '1': instance(1)}}


def test_cached_ev_height_is_history_not_present_success_and_cause_not_overclaimed():
    first, later = row(), row()
    for i in ('0', '1'):
        later['instances'][i]['baro']['time_last_fuse'] += 100
        later['instances'][i]['baro']['timestamp_sample'] += 100
    r = p.assess_comparisons([first, later])
    assert r['valid'] and r['instances']['1']['baro_successful_fusion_time_advances']
    assert r['instances']['1']['last_ev_height_sample_is_unchanged_cache']
    assert not r['historical_switch_cause_independently_verified']
    later['instances']['1']['baro'] = copy.deepcopy(first['instances']['1']['baro'])
    assert not p.assess_comparisons([first, later])['instances']['1']['baro_successful_fusion_time_advances']
    assert not p.assess_comparisons([first])['valid']
