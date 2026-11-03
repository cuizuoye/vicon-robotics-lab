import math
import json
import pytest
from random_rotation import Gesture,LargeGesture,wrap,analyze


def test_seeded_random_rotation_is_reproducible_but_not_fixed_sine():
    a,b,c=Gesture(20261030),Gesture(20261030),Gesture(20261031)
    assert a.segments==b.segments and a.segments!=c.segments
    assert any(s.b>0 for s in a.segments) and any(s.b<0 for s in a.segments)


def test_reference_is_continuous_bounded_and_crosses_quaternion_wrap():
    g=Gesture(20261030)
    assert any(s.b+.3>math.pi for s in g.segments)
    previous=g.sample(0)['yaw'];last_rate=0.
    for i in range(1,math.ceil((g.end+1.)*1000)):
        value=g.sample(i/1000.)
        assert abs(value['yaw']-previous)<.00121
        assert abs(value['yaw_rate'])<=1.200001
        assert abs(value['yaw_rate']-last_rate)/.001<=3.501
        previous,last_rate=value['yaw'],value['yaw_rate']
    assert g.sample(g.end+1.)['yaw']==0.


def test_wrapped_orientation_difference_is_not_a_360_degree_error():
    assert wrap(math.radians(179)-math.radians(-179))==pytest.approx(math.radians(-2))


def test_pure_rotation_has_zero_translation_from_rest():
    g=Gesture(20261031,combined=False)
    assert all(g.xy(i*.01)==(0.,0.) for i in range(int(g.end*100)))


def test_real_scale_route_reaches_6_by_5_metres_and_returns_to_origin():
    g=LargeGesture(20261030,.55)
    assert g.xy(0.)==(0.,0.) and g.xy(g.duration)==(0.,0.)
    for segment in g.route:
        assert g.xy(segment['start']+segment['duration'])==pytest.approx(segment['b'])
    assert max(r['b'][0] for r in g.route)==6.
    assert max(r['b'][1] for r in g.route)==5.
    previous=g.xy(0)
    for i in range(1,int(g.duration*100)):
        position=g.xy(i*.01)
        assert math.dist(previous,position)/.01<=.550001
        previous=position
    # Rotation must still be present on the later route legs, not just start.
    assert abs(g.sample(90.)['yaw_rate'])>0 or g.sample(90.)['kind']=='hold'


@pytest.mark.parametrize('lag,passes',[(0.,True),(.30,False)])
def test_analysis_distinguishes_good_tracking_from_real_follow_lag(tmp_path,lag,passes):
    g=Gesture(20261030);path=tmp_path/'generated_trace.jsonl'
    with path.open('w') as file:
        for i in range(int((g.end+2)*100)):
            t=i*.01;reference=dict(g.sample(t),t=t)
            actual=g.sample(max(0,t-lag))['yaw']
            file.write(json.dumps(dict(state='ACTIVE',reference=reference,car=[0.,0.,actual]))+'\n')
    metrics=analyze(path,g)
    assert metrics['synchronization_passed'] is passes
    assert metrics['estimated_follow_lag_s']==pytest.approx(lag,abs=.011)
