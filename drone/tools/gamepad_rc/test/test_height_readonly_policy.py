from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from height_readonly_policy import allowed, shell_message, parse_aid, assess_sources, absent_reply
from usb_vision_policy import shell_message as empty_shell


def encoder():
    return m.MAVLink(None, srcSystem=250, srcComponent=190)


@pytest.mark.parametrize('command', [400, 176, 185, 241, 246, 511])
def test_control_commands_always_denied(command):
    assert not allowed(encoder().command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0), safe=True)


def test_writes_and_positions_always_denied():
    assert not allowed(encoder().param_set_encode(1, 1, b'EKF2_EV_CTRL', 0, m.MAV_PARAM_TYPE_INT32), safe=True)
    assert allowed(encoder().param_request_read_encode(1, 1, b'EKF2_HGT_REF', -1))
    assert not allowed(encoder().param_request_read_encode(1, 1, b'EKF2_HGT_REF', 0))
    from uav_coord_bridge.mavlink_vision import odometry_message
    assert not allowed(odometry_message(encoder(), 123, [0, 0, 0], [1, 0, 0, 0]), safe=True)


def test_fixed_shell_and_unconditional_close_only():
    assert allowed(shell_message(encoder(), 'ev_2'), safe=True)
    assert not allowed(shell_message(encoder(), 'ev_2'))
    assert not allowed(empty_shell(encoder(), close=True))
    assert not allowed(empty_shell(encoder()))
    with pytest.raises(KeyError):
        shell_message(encoder(), 'ev_9')
    text = b'param set EKF2_HGT_REF 3\n'
    assert not allowed(encoder().serial_control_encode(10, 2, 0, 0, len(text), text.ljust(70, b'\0')), safe=True)


@pytest.mark.parametrize('key', ['distance', 'flow'])
def test_raw_sensor_reads_require_safe_identity(key):
    assert allowed(shell_message(encoder(), key), safe=True)
    assert not allowed(shell_message(encoder(), key), safe=False)


AID = ('TOPIC: estimator_aid_src_ev_hgt instance 0\n timestamp: 1000\n timestamp_sample: 950\n'
       ' estimator_instance: 0\n time_last_fuse: 980\n observation: 1\n observation_variance: 0.01\n'
       ' innovation: 0.1\n innovation_variance: 0.1\n test_ratio: 0.1\n fusion_enabled: True\n'
       ' innovation_rejected: False\n fused: True\n')


def test_aid_requires_complete_finite_and_correct_topic():
    assert parse_aid(AID, 'ev')['fused']
    assert parse_aid(AID, 'baro') is None
    assert parse_aid(AID.replace('observation: 1', 'observation: nan'), 'ev') is None
    assert parse_aid(AID.replace(' fused: True\n', ''), 'ev') is None


def test_enable_bit_and_stale_samples_are_not_fusion_proof():
    first = parse_aid(AID, 'ev')
    second = {**first, 'timestamp_sample': 1950, 'time_last_fuse': 1980}
    def row(s):
        return {'aids': {'ev': s}, 'flags': {'cs_ev_hgt': True},
                'selector': {'primary_instance': 0}, 'local': {'z_valid': True, 'timestamp': s['timestamp']}}
    rows = [row(first), row(second)]
    assert assess_sources(rows, 0)['ev']['continuous_internal_fusion_observed']
    assert not assess_sources(rows, 1)['ev']['continuous_internal_fusion_observed']
    second['fused'] = False
    assert not assess_sources(rows, 0)['ev']['continuous_internal_fusion_observed']
    assert not assess_sources([rows[0], rows[0]], 0)['ev']['continuous_internal_fusion_observed']


def test_expected_no_data_is_not_fabricated_sample_or_incomplete_reply():
    assert absent_reply('never published') == 'never_published'
    assert absent_reply('Waited for 2.0 seconds without a message. Giving up.') == 'no_message_within_listener_wait'
    assert absent_reply('arbitrary incomplete output') is None
    assert not assess_sources([{'aids': {}}], 0)['ev']['continuous_internal_fusion_observed']
