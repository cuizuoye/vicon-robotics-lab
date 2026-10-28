"""Exact new duration/offset-arena request; old scope cannot authorize this one."""
import hashlib
import json
from pathlib import Path

REQUEST_PATH = Path('/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_demo_120s/USER_REQUEST.json')
KIND = 'BOUNDED_OFFSET_ARENA_120S_NOT_QUALIFICATION'


def verify_range_request(review):
    path = Path(review.get('range_request', ''))
    if path.resolve() != REQUEST_PATH:
        raise ValueError('120s offset-arena operation requires its own exact human request')
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != review.get('range_request_sha256'):
        raise ValueError('120s demonstration request drift')
    r = json.loads(data)
    expected = dict(goal_forward_m=[-1.5,.5], goal_left_m=[-.4,.5], buffer_m=.5,
        mapping_scale=1., max_active_s=120., stage=4, firmware_change=False,
        automatic_hardware_launch=False, full_vicon_coverage_measured=False,
        stopping_distance_qualified=False, electrical_thermal_qualified=False,
        clarification_answer='对的', reference='unchanged saved original start and heading')
    if any(r.get(k) != value for k,value in expected.items()):
        raise ValueError('exact confirmed120s offset-arena scope required')
    for key in ('firmware_change','automatic_hardware_launch','full_vicon_coverage_measured',
                'stopping_distance_qualified','electrical_thermal_qualified'):
        if r.get(key) is not False:raise ValueError('unmeasured acceptance cannot be invented')
    return True
