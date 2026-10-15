import copy
import os
from pathlib import Path
import subprocess
import sys

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from preview_target_mapping import preview
from uav_coord_bridge.world_reference import freeze_reference


def bench():
    rows=[];sent=[]
    for stamp in (1_000_000,2_000_000,3_000_000):
        flags=dict.fromkeys(('cs_gps','cs_mag_hdg','cs_mag_3d','cs_gps_yaw','cs_ev_vel','cs_in_air',
                             'fs_bad_pos_n','fs_bad_pos_e','fs_bad_pos_d','fs_bad_hdg',
                             'reject_hor_pos','reject_ver_pos','reject_yaw'),False)
        flags.update(cs_tilt_align=True,cs_baro_hgt=True,cs_ev_pos=True,cs_ev_hgt=True,cs_ev_yaw=True)
        aid={'timestamp':stamp,'timestamp_sample':stamp,'time_last_fuse':stamp,
             'estimator_instance':1,'fusion_enabled':True,'fused':True,'innovation_rejected':False,
             'test_ratio':.01,'observation_variance':.1}
        rows.append({'flags':flags,'local':{'timestamp':stamp,'timestamp_sample':stamp,
                     'xy_valid':True,'z_valid':True,'x':1,'y':2,'z':-20,'heading':0},
                     'aid':dict(aid),'height_aid':dict(aid),'yaw_aid':dict(aid),
                     'yaw_comparison':{'fresh':True,'fc_minus_vicon_wrapped_deg':.1},
                     'clock':{'estimated_offset':100}})
        sent.append({'wire_time_usec':stamp-100,'position':[1,-2,-.05],
                     'q':[1,0,0,0],'source_stamp_ns':stamp*1000})
    return {'combined_aiding_verified':True,'error':None,'primary_estimator':1,
            'parameters_after':{'EKF2_HGT_REF':0},'fusion_snapshots':rows,'sent':sent,
            'captured_at_utc':'test'}


def reference(): return freeze_reference([0,0,0,1],[8,9,.05])


def test_historical_mapping_is_not_live_dispatch_or_task_origin():
    result=preview(bench(),reference())
    assert result['arithmetic_examples_not_mission']['height_half_meter_controller_m']==pytest.approx([1,2,-20.5])
    assert not result['live_dispatch_permitted'] and not result['flight_ready']
    assert not result['task_origin_selected'] and result['actual_sent_to_fc']==0
    assert all(p['timestamp_difference_us']==0 for p in result['paired_samples'])


@pytest.mark.parametrize('case',['not_verified','error','wrong_instance','stale_pair','invalid_q','changed_height'])
def test_invalid_joint_data_rejected(case):
    data=copy.deepcopy(bench())
    if case=='not_verified': data['combined_aiding_verified']=False
    if case=='error': data['error']='failed'
    if case=='wrong_instance': data['primary_estimator']=0
    if case=='stale_pair': data['sent']=data['sent'][:1]
    if case=='invalid_q': data['sent'][0]['q']=[0,0,0,0]
    if case=='changed_height': data['parameters_after']['EKF2_HGT_REF']=3
    with pytest.raises(ValueError): preview(data,reference())


def test_ros_smoke_domain_guard_before_publishing(tmp_path):
    tool=Path(__file__).resolve().parents[1]/'combined_loss_ros_smoke.py'
    result=subprocess.run([sys.executable,str(tool),'--output',str(tmp_path/'new.json')],
                          env={**os.environ,'ROS_DOMAIN_ID':'0'},capture_output=True,text=True)
    assert result.returncode==2 and 'ROS_DOMAIN_ID=91' in result.stderr
    assert not (tmp_path/'new.json').exists()
