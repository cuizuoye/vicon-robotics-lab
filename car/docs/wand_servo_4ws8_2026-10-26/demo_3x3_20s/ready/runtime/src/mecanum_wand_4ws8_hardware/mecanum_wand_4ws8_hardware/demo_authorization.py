"""Bind the expanded spatial scope to the current human request, not old consent."""
import hashlib
import json
from pathlib import Path

REQUEST_PATH = Path('/home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407/candidates/servo_4ws8_demo_3x3_20s/USER_REQUEST.json')
KIND = 'BOUNDED_3X3_DEMO_20S_NOT_QUALIFICATION'


def verify_range_request(review):
    path = Path(review.get('range_request', ''))
    if path.resolve() != REQUEST_PATH:
        raise ValueError('the new demonstration scope requires its own human request')
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != review.get('range_request_sha256'):
        raise ValueError('demonstration request drift')
    request = json.loads(data)
    if (request.get('request') != '超过本入口15厘米上限的意思是：sign只要移动距离超过15cm就会触发停机？将这个距离延长到可以正常演示的范围，15cm太短了'
            or request.get('goal_forward_m') != [0., 3.]
            or request.get('goal_left_m') != [0., 3.]
            or request.get('buffer_m') != .50 or request.get('mapping_scale') != 1.
            or request.get('max_active_s') != 20. or request.get('stage') != 4
            or request.get('firmware_change') is not False
            or request.get('automatic_hardware_launch') is not False):
        raise ValueError('exact fixed-arena demonstration request required')
    return True
