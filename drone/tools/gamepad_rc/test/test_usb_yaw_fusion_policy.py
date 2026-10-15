import copy
import math
from pathlib import Path
import subprocess
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import usb_yaw_fusion_policy as y
import usb_height_fusion_policy as h
import usb_fusion_policy as original
from mavlink_readonly_probe import decode_parameter
from uav_coord_bridge.mavlink_vision import odometry_message


def enc():
    return m.MAVLink(None, srcSystem=250, srcComponent=190)


def test_only_separate_explicit_yaw_value_and_byte_encoding():
    msg = y.control_message(enc(), 8)
    assert decode_parameter(msg) == 8
    gate = dict(authorized=True, safe=True, backup_verified=True, clock_verified=True)
    assert y.allowed(msg, **gate)
    assert not original.allowed(msg, **gate) and not h.allowed(msg, **gate)
    assert not y.allowed(msg, safe=True, backup_verified=True, clock_verified=True)
    assert not y.allowed(msg, authorized=True, safe=True, backup_verified=True)
    msg.param_value = 8.0
    assert not y.allowed(msg, **gate)
    for value in (1, 3, 11, 15, True):
        with pytest.raises(ValueError): y.control_message(enc(), value)


@pytest.mark.parametrize('name', ['EKF2_HGT_REF', 'EKF2_MAG_TYPE', 'EKF2_GPS_CTRL', 'EKF2_EV_DELAY', 'EKF2_EVA_NOISE'])
def test_no_reference_magnetometer_gps_or_noise_writes(name):
    msg = y.control_message(enc(), 8); msg.param_id = name
    assert not y.allowed(msg, authorized=True, safe=True, backup_verified=True, clock_verified=True)


def test_restore_without_source_clock_and_no_positions_in_restore_mode():
    msg = y.control_message(enc(), 0)
    assert y.allowed(msg, authorized=True, restore_safe=True, backup_verified=True)
    assert not y.allowed(msg, authorized=True, restore_safe=False, backup_verified=True)
    pose = odometry_message(enc(), 123, [0,0,0], [1,0,0,0])
    assert not y.allowed(pose, authorized=True, safe=True)
    assert y.allowed(pose, authorized=True, safe=True, position_authorized=True)


def test_fixed_listener_and_no_flight_commands():
    assert y.allowed(y.fusion_shell_message(enc(), 'yaw_aid_1'), safe=True)
    assert not y.allowed(y.fusion_shell_message(enc(), 'yaw_aid_1'))
    with pytest.raises(KeyError): y.fusion_shell_message(enc(), 'yaw_aid_9')
    for command in (400, 176, 185, 241, 246, 511):
        assert not y.allowed(enc().command_long_encode(1,1,command,0,1,0,0,0,0,0,0),
                             authorized=True,safe=True,backup_verified=True,clock_verified=True)


def test_wrap_quaternion_sign_and_freshness():
    def q(angle): return [math.cos(math.radians(angle)/2),0,0,math.sin(math.radians(angle)/2)]
    r = y.compare_heading(q(179),q(-179),attitude_age_ms=30,sent_age_ms=10)
    assert r['fc_minus_vicon_wrapped_deg'] == pytest.approx(2)
    assert r['fresh'] and not r['earth_north_alignment_verified']
    assert y.compare_heading(q(45),[-v for v in q(45)],attitude_age_ms=30,sent_age_ms=10)['fc_minus_vicon_wrapped_deg'] == pytest.approx(0)
    assert not y.compare_heading(q(0),q(0),attitude_age_ms=500,sent_age_ms=10)['fresh']


def rows():
    flags = dict.fromkeys(('cs_gps','cs_mag_hdg','cs_mag_3d','cs_gps_yaw','cs_ev_pos','cs_ev_hgt',
                           'cs_ev_vel','cs_in_air','fs_bad_hdg','reject_yaw','cs_yaw_align'),False)
    flags.update(cs_tilt_align=True,cs_ev_yaw=True,cs_baro_hgt=True)
    row = {'flags':flags,'yaw_aid':{'estimator_instance':1,'fusion_enabled':True,'fused':True,
                                  'innovation_rejected':False,'timestamp':1000,'timestamp_sample':900,
                                  'time_last_fuse':950,'observation_variance':.1,'innovation_variance':.2,'test_ratio':.01},
           'local':{'timestamp':1100,'heading_good_for_control':True},
           'yaw_comparison':{'fresh':True,'fc_minus_vicon_wrapped_deg':.1}}
    later=copy.deepcopy(row)
    later['yaw_aid']['timestamp_sample'] += 100
    later['yaw_aid']['time_last_fuse'] += 100
    return [row,later]


def test_frd_yaw_success_does_not_require_false_geographic_north_flag_to_be_forced_true():
    assert y.fusion_evidence_ok(rows(),1)
    assert not y.fusion_evidence_ok(rows(),0)
    for key in ('cs_gps','cs_ev_pos','cs_ev_hgt','cs_ev_vel','cs_mag_hdg'):
        r=rows();r[0]['flags'][key]=True
        assert not y.fusion_evidence_ok(r,1)
    r=rows();r[0]['yaw_aid']['fused']=False
    assert not y.fusion_evidence_ok(r,1)
    r=rows();r[1]['yaw_aid']=r[0]['yaw_aid']
    assert not y.fusion_evidence_ok(r,1)
    r=rows();r[0]['yaw_comparison']['fc_minus_vicon_wrapped_deg']=10
    assert not y.fusion_evidence_ok(r,1)


@pytest.mark.parametrize('extra', [[], ['--height-fusion-bench'], ['--manual-vertical-bench']])
def test_yaw_cli_closed_scope_and_physical_gate(tmp_path,extra):
    script=Path(__file__).resolve().parents[1]/'usb_vicon_receive_bench.py'
    cmd=[sys.executable,str(script),'--port','/dev/nonexistent_yaw_test','--uid2','test',
         '--heading-candidate','/nonexistent_heading','--board-center-candidate','/nonexistent_board',
         '--output',str(tmp_path/'new.json'),'--yaw-fusion-bench','--bench-send',*extra]
    if '--manual-vertical-bench' in extra: cmd.append('--fusion-physical-confirmed')
    r=subprocess.run(cmd,capture_output=True,text=True)
    assert r.returncode==2 and not (tmp_path/'new.json').exists()
    assert any(text in r.stderr for text in ('explicit physical bench confirmation','Select one fusion mode',
                                           'Manual bench requires matching explicit fusion mode'))
