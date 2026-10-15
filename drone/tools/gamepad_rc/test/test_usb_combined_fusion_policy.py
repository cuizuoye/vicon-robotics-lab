import copy
from pathlib import Path
import subprocess
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import usb_combined_fusion_policy as c
import usb_height_fusion_policy as h
import usb_yaw_fusion_policy as y
import usb_fusion_policy as xy
from mavlink_readonly_probe import decode_parameter
from uav_coord_bridge.mavlink_vision import odometry_message


def enc(): return m.MAVLink(None,srcSystem=250,srcComponent=190)


def test_independent_eleven_only_gate_and_unchanged_old_modes():
    msg=c.control_message(enc(),11)
    gate=dict(authorized=True,safe=True,backup_verified=True,clock_verified=True)
    assert decode_parameter(msg)==11 and c.allowed(msg,**gate)
    assert not h.allowed(msg,**gate) and not y.allowed(msg,**gate) and not xy.allowed(msg,**gate)
    for key in gate:
        changed={**gate,key:False}
        assert not c.allowed(msg,**changed)
    msg.param_value=11.0
    assert not c.allowed(msg,**gate)
    for v in (1,3,8,15,True):
        with pytest.raises(ValueError): c.control_message(enc(),v)


def test_restore_without_clock_source_but_not_without_identity_or_backup():
    restore=c.control_message(enc(),0)
    assert c.allowed(restore,authorized=True,restore_safe=True,backup_verified=True)
    assert not c.allowed(restore,authorized=True,restore_safe=False,backup_verified=True)
    assert not c.allowed(restore,authorized=True,restore_safe=True)
    pose=odometry_message(enc(),123,[1,2,3],[1,0,0,0])
    assert not c.allowed(pose,authorized=True,safe=True)
    assert c.allowed(pose,authorized=True,safe=True,position_authorized=True)
    pose.frame_id=m.MAV_FRAME_LOCAL_NED
    assert not c.allowed(pose,authorized=True,safe=True,position_authorized=True)


@pytest.mark.parametrize('name',['EKF2_HGT_REF','EKF2_MAG_TYPE','EKF2_GPS_CTRL','EKF2_EV_DELAY','EKF2_EVA_NOISE'])
def test_no_additional_parameter_writes(name):
    msg=c.control_message(enc(),11); msg.param_id=name
    assert not c.allowed(msg,authorized=True,safe=True,backup_verified=True,clock_verified=True)


def rows():
    output=[]
    for stamp in (1_000_000,2_000_000,3_000_000):
        flags=dict.fromkeys(('cs_gps','cs_mag_hdg','cs_mag_3d','cs_gps_yaw','cs_ev_vel','cs_in_air',
                             'fs_bad_pos_n','fs_bad_pos_e','fs_bad_pos_d','fs_bad_hdg','reject_hor_pos',
                             'reject_ver_pos','reject_yaw','cs_yaw_align'),False)
        flags.update(timestamp=stamp,cs_tilt_align=True,cs_baro_hgt=True,cs_ev_pos=True,cs_ev_hgt=True,cs_ev_yaw=True)
        aid={'estimator_instance':1,'timestamp':stamp,'timestamp_sample':stamp-100,'time_last_fuse':stamp-10,
             'fusion_enabled':True,'fused':True,'innovation_rejected':False,'test_ratio':.01,'observation_variance':.1}
        output.append({'selector':{'primary_instance':1},'flags':flags,
                       'local':{'timestamp':stamp,'xy_valid':True,'z_valid':True,'v_xy_valid':True,
                                'v_z_valid':True,'heading_good_for_control':False},
                       'aid':{**aid,'test_ratio':[.01,.02],'observation_variance':[.1,.1]},
                       'height_aid':dict(aid),'yaw_aid':dict(aid),
                       'yaw_comparison':{'fresh':True,'fc_minus_vicon_wrapped_deg':.1}})
    return output


def test_simultaneous_xyz_yaw_no_faked_heading_flag():
    assert c.fusion_evidence_ok(rows(),1)
    assert not c.fusion_evidence_ok(rows(),0)
    # 成功融合不等于Commander/独立安全链路已验收。
    assert not c.control.assess(rows(),1)['flight_ready']


@pytest.mark.parametrize('case',['gps','mag','ev_vel','bad_pos','bad_heading','stale_yaw','failed_xyz','failed_yaw','cached','missing'])
def test_joint_requires_every_aid_and_no_conflicting_sources(case):
    data=copy.deepcopy(rows())
    if case=='gps': data[0]['flags']['cs_gps']=True
    if case=='mag': data[0]['flags']['cs_mag_hdg']=True
    if case=='ev_vel': data[0]['flags']['cs_ev_vel']=True
    if case=='bad_pos': data[0]['flags']['fs_bad_pos_n']=True
    if case=='bad_heading': data[0]['flags']['fs_bad_hdg']=True
    if case=='stale_yaw': data[0]['yaw_comparison']['fresh']=False
    if case=='failed_xyz': data[0]['height_aid']['fused']=False
    if case=='failed_yaw': data[0]['yaw_aid']['innovation_rejected']=True
    if case=='cached': data[1]['yaw_aid']=data[0]['yaw_aid']
    if case=='missing': data[0]['aid']=None
    assert not c.fusion_evidence_ok(data,1)


@pytest.mark.parametrize('extra',[[],['--height-fusion-bench'],['--yaw-fusion-bench'],['--manual-vertical-bench']])
def test_cli_physical_and_single_mode_guards_before_serial(tmp_path,extra):
    tool=Path(__file__).resolve().parents[1]/'usb_vicon_receive_bench.py'
    command=[sys.executable,str(tool),'--port','/dev/doesnotexist','--uid2','test',
             '--heading-candidate','/missing','--board-center-candidate','/missing',
             '--output',str(tmp_path/'new.json'),'--bench-send','--combined-fusion-bench',*extra]
    if '--manual-vertical-bench' in extra: command.append('--fusion-physical-confirmed')
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode==2 and not (tmp_path/'new.json').exists()
    assert any(s in result.stderr for s in ('explicit physical bench confirmation','Select one fusion mode',
                                          'Manual bench requires matching explicit fusion mode'))


def test_fixed_readonly_listeners_no_flight_command_or_velocity_enable():
    for key in ('aid_1','height_aid_1','yaw_aid_1','status_1','local_position','selector'):
        assert c.allowed(c.fusion_shell_message(enc(),key),safe=True)
    assert c.allowed(c.fusion_shell_message(enc(),'failsafe'),safe=True)
    assert not c.allowed(c.fusion_shell_message(enc(),'failsafe'))
    with pytest.raises(KeyError): c.fusion_shell_message(enc(),'status_9')
    for command in (400,176,185,241,246,511):
        assert not c.allowed(enc().command_long_encode(1,1,command,0,1,0,0,0,0,0,0),
                             authorized=True,safe=True,backup_verified=True,clock_verified=True)
