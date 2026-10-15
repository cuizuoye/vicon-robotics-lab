import copy
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'ros2_ws/src/uav_coord_bridge'))
from usb_fusion_policy import (allowed, control_message, fusion_shell_message, fusion_scope_ok,
                               READ_PARAMS, parse_aid, parse_selector, fusion_evidence_ok,
                               aid_topic_for_estimator)
from mavlink_readonly_probe import decode_parameter
from uav_coord_bridge.mavlink_vision import odometry_message


def enc():
    return m.MAVLink(None, srcSystem=250, srcComponent=190)


def test_integer_byte_encoding_and_closed_write_scope():
    msg = control_message(enc(), 1)
    assert decode_parameter(msg) == 1
    assert not allowed(msg, safe=True, backup_verified=True, clock_verified=True)
    assert not allowed(msg, authorized=True, safe=True, backup_verified=False, clock_verified=True)
    assert not allowed(msg, authorized=True, safe=True, backup_verified=True, clock_verified=False)
    assert allowed(msg, authorized=True, safe=True, backup_verified=True, clock_verified=True)
    msg.param_value = 1.0
    assert not allowed(msg, authorized=True, safe=True, backup_verified=True, clock_verified=True)
    for value in (2, 3, 15, True):
        with pytest.raises(ValueError):
            control_message(enc(), value)


def test_restore_does_not_depend_on_vicon_clock_but_requires_disarmed_identity_backup():
    restore = control_message(enc(), 0)
    assert allowed(restore, authorized=True, restore_safe=True, backup_verified=True)
    assert not allowed(restore, authorized=True, restore_safe=False, backup_verified=True)
    restore.param_id = 'EKF2_HGT_REF'
    assert not allowed(restore, authorized=True, restore_safe=True, backup_verified=True)
    pose = odometry_message(enc(), 123, [0, 0, 0], [1, 0, 0, 0])
    assert not allowed(pose, authorized=True, safe=True, restore_safe=True, backup_verified=True)
    assert allowed(pose, authorized=True, safe=True, position_authorized=True)


@pytest.mark.parametrize('cmd', [400, 176, 185, 241, 246, 511])
def test_no_arm_mode_lockdown_reboot_or_frequency_change(cmd):
    assert not allowed(enc().command_long_encode(1, 1, cmd, 0, 1, 0, 0, 0, 0, 0, 0),
                       authorized=True, safe=True, restore_safe=True, backup_verified=True, clock_verified=True)


def test_only_fixed_instance_listeners_and_read_parameters():
    assert allowed(fusion_shell_message(enc(), 'flags_2'), safe=True)
    assert not allowed(fusion_shell_message(enc(), 'flags_2'), safe=False)
    with pytest.raises(KeyError):
        fusion_shell_message(enc(), 'flags_9')
    assert allowed(enc().param_request_read_encode(1, 1, b'EKF2_EV_QMIN', -1))
    p = dict.fromkeys(READ_PARAMS, 0)
    assert fusion_scope_ok(p) and fusion_scope_ok({**p, 'EKF2_EV_CTRL': 1})
    assert not fusion_scope_ok({**p, 'EKF2_EV_CTRL': 15})
    assert not fusion_scope_ok({**p, 'EKF2_EV_QMIN': 50})


AID = ('TOPIC: estimator_aid_src_ev_pos instance 2 #1\n timestamp: 1000\n timestamp_sample: 900\n'
       ' estimator_instance: 2\n time_last_fuse: 950\n observation: [1, 2]\n observation_variance: [0.1, 0.1]\n'
       ' innovation: [0.01, 0.02]\n innovation_variance: [0.2, 0.2]\n test_ratio: [0.1, 0.2]\n'
       ' fusion_enabled: True\n innovation_rejected: False\n fused: True\n')


def test_aid_parser_requires_complete_finite_internal_evidence():
    assert parse_aid(AID)['estimator_instance'] == 2
    assert parse_aid(AID.replace(' fused: True\n', '')) is None
    assert parse_aid(AID.replace('[1, 2]', '[nan, 2]')) is None
    assert parse_aid('never published') is None
    assert parse_selector('TOPIC: estimator_selector_status\n timestamp: 1\n primary_instance: 2') is None


def rows():
    f = dict.fromkeys(('cs_ev_yaw','cs_ev_vel','cs_ev_hgt','cs_in_air','fs_bad_pos_n','fs_bad_pos_e','reject_hor_pos'), False)
    f.update(cs_tilt_align=True, cs_ev_pos=True)
    row = {'flags': f, 'aid': parse_aid(AID), 'local': {'xy_valid': True}}
    later = copy.deepcopy(row)
    later['aid']['time_last_fuse'] += 100
    later['aid']['timestamp_sample'] += 100
    return [row, later]


def test_status_bit_alone_is_not_actual_fusion_or_primary_instance_proof():
    r = rows()
    assert fusion_evidence_ok(r, 2)
    assert not fusion_evidence_ok(r, 1)
    for key in ('fusion_enabled', 'fused'):
        changed = rows()
        changed[0]['aid'][key] = False
        assert not fusion_evidence_ok(changed, 2)
    rejected = rows()
    rejected[0]['aid']['innovation_rejected'] = True
    assert not fusion_evidence_ok(rejected, 2)
    stale = rows()
    stale[1]['aid'] = copy.deepcopy(stale[0]['aid'])
    assert not fusion_evidence_ok(stale, 2)


def test_delayed_ev_publication_slots_must_not_be_confused_with_ekf_ids():
    topics = {0:{'estimator_instance':1}, 1:{'estimator_instance':0}}
    assert aid_topic_for_estimator(topics,0) == 1
    assert aid_topic_for_estimator(topics,1) == 0


@pytest.mark.parametrize('topics', [{0:None}, {0:{}}, {0:{'estimator_instance':1}},
    {0:{'estimator_instance':0},1:{'estimator_instance':0}}])
def test_missing_or_ambiguous_ekf_association_never_falls_back_to_slot_number(topics):
    with pytest.raises(ValueError):
        aid_topic_for_estimator(topics,0)
