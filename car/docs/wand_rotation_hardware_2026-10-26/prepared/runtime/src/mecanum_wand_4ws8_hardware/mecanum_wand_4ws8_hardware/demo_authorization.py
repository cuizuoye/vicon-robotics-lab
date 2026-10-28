"""New input semantics require their own exact request and approval scope."""
import hashlib
import json
from pathlib import Path

REQUEST_PATH=Path('/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_demo_120s_toggle/USER_REQUEST.json')
RANGE_PATH=Path('/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_demo_120s/USER_REQUEST.json')
RANGE_SHA='2bd13bbb25600655979108ec96204a20e2c64d2803b826d536c25130efe801d9'
INPUT_MODE='SPACE_TOGGLE_120S_V1'
KIND='BOUNDED_SPACE_TOGGLE_120S_NOT_QUALIFICATION'


def verify_range_request(review):
    path=Path(review.get('range_request',''))
    if path.resolve()!=REQUEST_PATH or review.get('operator_input_mode')!=INPUT_MODE:
        raise ValueError('exact one-click human request and input mode required')
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=review.get('range_request_sha256'):
        raise ValueError('one-click request drift')
    r=json.loads(raw)
    expected=dict(operator_input_mode=INPUT_MODE,goal_forward_m=[-1.5,.5],goal_left_m=[-.4,.5],
        buffer_m=.5,mapping_scale=1.,max_active_s=120.,stage=4,
        original_range_request=str(RANGE_PATH),original_range_request_sha256=RANGE_SHA,
        key_release_during_session_stops=False,synthetic_key_hold=False,
        firmware_change=False,automatic_hardware_launch=False,full_vicon_coverage_measured=False,
        stopping_distance_qualified=False,electrical_thermal_qualified=False)
    if any(r.get(k)!=v for k,v in expected.items()):raise ValueError('one-click scope mismatch')
    if hashlib.sha256(RANGE_PATH.read_bytes()).hexdigest()!=RANGE_SHA:
        raise ValueError('preserved original signed range request drift')
    for key,value in expected.items():
        if value is False and r.get(key) is not False:raise ValueError('explicit false safety claims required')
    return True
