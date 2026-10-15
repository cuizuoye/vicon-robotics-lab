import copy
import math
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from manual_translation import endpoint, assess_translation, validate_control


def rows():
    return [{'elapsed_s': i*.25, 'local': {'timestamp_sample': 100+i, 'xy_valid': True, 'x': 1, 'y': 2, 'z': -5},
             'sent': {'source_stamp_ns': 100+i, 'position': [0,0,0], 'q': [1,0,0,0]},
             'fc_q': [1,0,0,0], 'attitude_age_ms': 50} for i in range(5)]


def test_stationary_endpoint_requires_unique_fresh_finite_observations():
    assert endpoint(rows())['rows'] == 5
    assert endpoint(rows()[:4]) is None
    for field, value in [('attitude_age_ms', 500), ('attitude_age_ms', -1)]:
        r = rows(); r[-1][field] = value
        assert endpoint(r) is None
    r = rows(); r[-1]['local']['xy_valid'] = False
    assert endpoint(r) is None
    r = rows(); r[-1]['local']['timestamp_sample'] = r[-2]['local']['timestamp_sample']
    assert endpoint(r) is None
    r = rows(); r[-1]['sent']['position'][0] = .05
    assert endpoint(r) is None
    r = rows(); r[-1]['sent']['position'][0] = math.nan
    assert endpoint(r) is None


def test_quaternion_sign_flip_is_not_physical_rotation():
    r = rows(); r[-1]['sent']['q'] = [-1,0,0,0]
    assert endpoint(r)['ev_rotation_spread_deg'] == pytest.approx(0)


def test_displacement_alignment_does_not_compare_unaligned_axes():
    start = endpoint(rows())
    start['fc_q_wxyz'] = [math.sqrt(.5),0,0,math.sqrt(.5)]
    end = copy.deepcopy(start)
    end['ev_position_frd_m'] = [.25,0,0]
    end['fc_position_m'] = [1,2.25,-5]
    report = assess_translation(start,end)
    assert report['verified']
    assert report['horizontal_direction_difference_deg'] == pytest.approx(0)
    assert not report['independent_metrology_accuracy_verified']
    end['fc_position_m'] = [1.25,2,-5]
    assert not assess_translation(start,end)['verified']


@pytest.mark.parametrize('position', [[.01,0,0], [.25,0,.1], [.25,0,0]])
def test_wrong_scale_or_missing_motion_not_a_pass(position):
    start = endpoint(rows()); end = copy.deepcopy(start)
    end['ev_position_frd_m'] = position
    end['fc_position_m'] = [1.5,2,-5]
    assert not assess_translation(start,end)['verified']
    assert not assess_translation(start,None)['verified']


@pytest.mark.parametrize('value', [{'sequence':1,'command':'arm'}, {'sequence':True,'command':'finish'},
                                  {'sequence':1,'command':'finish','shell':'x'}, {'command':'finish'}])
def test_manual_handshake_is_not_a_flight_or_shell_command(value):
    with pytest.raises(ValueError): validate_control(value)
    assert validate_control({'sequence':1,'command':'finish'})['command'] == 'finish'
