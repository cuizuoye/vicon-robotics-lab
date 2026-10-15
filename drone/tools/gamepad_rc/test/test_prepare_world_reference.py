from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from prepare_world_reference import prepare


def documents():
    return dict(capture={'source_checks_passed':True, 'frame_ids':['map'],
                         'source_timing':{'maximum_arrival_gap_ms':5},
                         'ros_header_age_ms':{'maximum':1},'position_std_mm':[.01,.02,.03],
                         'last_pose':{'position_m':[1,2,3],'quaternion_xyzw':[0,0,0,1],'stamp_ns':1},
                         'captured_at_utc':'test'},
                ground={'ev_position_frd_m':[8,-9,-.05]},
                heading={'body_orientation_in_rigid_xyzw_candidate':[0,0,0,1]},
                board={'board_center_in_rigid_m_candidate':[.1,0,0], 'target_reference':'candidate_board_center'})


def test_map_preserved_and_no_automatic_task_origin_or_fc_target():
    ref=prepare(**documents())
    assert ref['current_observation']['body_reference_position_map_m']==[1.1,2,3]
    assert ref['current_observation']['height_above_initial_ground_reference_m']==pytest.approx(2.95)
    assert ref['ground_reference_map_m']==[8,9,.05]
    assert ref['task_origin_map_m'] is None
    assert not ref['flight_waypoints_generated'] and ref['actual_sent_to_fc']==0


@pytest.mark.parametrize('case',['wrong_frame','failed','gap','age','motion','nan'])
def test_bad_capture_rejected(case):
    docs=documents(); capture=docs['capture']
    if case=='wrong_frame': capture['frame_ids']=['NED']
    if case=='failed': capture['source_checks_passed']=False
    if case=='gap': capture['source_timing']['maximum_arrival_gap_ms']=151
    if case=='age': capture['ros_header_age_ms']['maximum']=151
    if case=='motion': capture['position_std_mm']=[11,0,0]
    if case=='nan': capture['position_std_mm']=[float('nan'),0,0]
    with pytest.raises(ValueError): prepare(**docs)
