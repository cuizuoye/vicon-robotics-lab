import importlib.util
import json
from pathlib import Path

import pytest

spec=importlib.util.spec_from_file_location('live_analysis_under_test',Path(__file__).with_name('analyze_live_sign_sim.py'))
analysis=importlib.util.module_from_spec(spec);spec.loader.exec_module(analysis)


def test_first_application_timing_does_not_count_repeated_gate_ticks(tmp_path):
    status=dict(issued_steady=10.,issued_ros=100.,state='ACTIVE',reason='',epoch='e',
        anchors=dict(car_origin=dict(x=0.,y=0.),wand_origin=dict(x=0.,y=0.)),
        raw_goal=dict(x=.1,y=0.),mapped_goal=dict(x=.1,y=0.),
        pose_source_ages_s=dict(car=.01,wand=.02),goal_limited=False,reference_limited=False)
    rows=[]
    def add(topic,value,t):rows.append(dict(topic=topic,value=value,received_steady=t))
    add('supervisor_status',status,10.001)
    add('wand_pose',dict(source_ros=99.98,x=.1,y=0.),10.)
    add('car_pose',dict(source_ros=99.99,x=.05,y=0.),10.)
    add('operator_lease',dict(issued_steady=10.,issued_ros=100.,held=True),10.001)
    add('operator_lease',dict(issued_steady=10.01,issued_ros=100.01,held=False),10.011)
    for t,speed in ((10.006,.1),(10.016,0.)):
        add('gate_timing',dict(issued_steady=t,issued_ros=100.+t-10.,
            candidate_sequence=1,candidate_issued_steady=10.,
            command=dict(vx=speed,vy=0.,wz=0.)),t+.001)
    (tmp_path/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (tmp_path/'snapshot.json').write_text(json.dumps(dict(state_transitions=[],latest={})))
    (tmp_path/'console.log').write_text('[INFO] [100.010] [sim]: UI_STATE '+json.dumps(
        dict(held=False,key_down=False,focused=True))+'\n')
    result=analysis.analyze(tmp_path)
    assert result['candidate_to_gate_application_s']['count']==1
    assert result['candidate_to_gate_application_s']['max']==pytest.approx(.006)
    assert result['receiver_header_to_gate_application_s']['max']==pytest.approx(.026)
    assert result['raw_1to1_mapping_error_m']['max']==pytest.approx(0.)
    assert result['release_events'][0]['release_to_gate_zero_s']==pytest.approx(.006)
    assert result['release_events'][0]['release_cause']=='key_release_observed_by_ui'
    assert result['release_events'][0]['nonzero_command_before_release']
    assert result['occlusion_acceptance']=='NOT EXERCISED'
    assert result['physical_camera_to_wheel_latency_measured'] is False
