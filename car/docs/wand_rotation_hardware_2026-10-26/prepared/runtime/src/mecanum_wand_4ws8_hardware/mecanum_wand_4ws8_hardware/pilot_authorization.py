"""A distinct, finite first-ground observation; not high-stage qualification.

The normal ground review is retained verbatim. Only this private runtime,
which adds independent fixed 20s/arena gates, recognizes this review kind.
No invented braking, electrical, previous-ground-stage or thermal evidence.
"""
import hashlib
import json
from pathlib import Path
from mecanum_wand_4ws8_control.pilot_core import POLICY

from .demo_authorization import KIND, verify_range_request


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_pilot_review(review, stage, purpose, firmware_sha):
    if (type(stage) is not int or stage != 4 or purpose != 'sign'
            or review.get('kind') != KIND or review.get('identity') != '4WS8'
            or review.get('image_sha256') != firmware_sha
            or review.get('authorized_stage') != 4 or review.get('purpose') != 'sign'
            or review.get('human_authorized') is not True
            or review.get('pilot_policy') != POLICY
            or review.get('continuous_ground_qualified') is not False
            or review.get('electrical_thermal_trial_reviewed') is not False):
        raise ValueError('finite pilot requires exact policy, stage and explicit scope')
    verify_range_request(review)
    evidence = Path(review['suspended_evidence'])
    prior = json.loads(evidence.read_text())
    if (sha(evidence) != review.get('suspended_evidence_sha256')
            or prior.get('identity') != '4WS8' or prior.get('image_sha256') != firmware_sha
            or prior.get('stage') != 4 or prior.get('purpose') != 'suspended'
            or prior.get('physical_trial') is not True
            or prior.get('action_and_stop_passed') is not True
            or prior.get('reviewed_by_human') is not True
            or sha(evidence.parent/'raw_link.jsonl') != prior.get('raw_log_sha256')
            or sha(evidence.parent/'events.json') != prior.get('events_sha256')):
        raise ValueError('actual matching suspended stage4 action/stop evidence required')
    user = Path(review['user_authorization'])
    if sha(user) != review.get('user_authorization_sha256'):
        raise ValueError('human authorization drift')
    data = json.loads(user.read_text())
    if (data.get('accepted') != '接受，先0.70／0.80小位移'
            or data.get('physical_confirmation') != '已落地回原起点，Sign可见，可随时断电'
            or data.get('heading_correction') != '刚才方向放反了，现在恢复正常了'):
        raise ValueError('exact ground-pilot human scope required')
    return True


# New scope cannot be approved by the historical translation-only review.
KIND='BOUNDED_XY_YAW_4WS8_EXISTING_ENVELOPE_NOT_QUALIFICATION'
_verify_existing_pilot = verify_pilot_review
ROTATION_REQUEST=Path('/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_rotation_hardware/USER_REQUEST.json')
ROTATION_REQUEST_SHA='23345ec603f399512b8cde8defbd7689300075d89d3ce6773400a6262b16cbed'
GEOMETRY_SHA='b73b607303df5fea583dcfdbc53387e820b8b80fcd3f63aa6f7b78dd2469812f'
def verify_pilot_review(review, stage, purpose, firmware_sha):
    if (review.get('rotation_request')!=str(ROTATION_REQUEST)
            or review.get('rotation_request_sha256')!=ROTATION_REQUEST_SHA
            or sha(ROTATION_REQUEST)!=ROTATION_REQUEST_SHA
            or review.get('rotation_geometry_sha256')!=GEOMETRY_SHA
            or review.get('rotation_mapping_scale')!=1.
            or review.get('rotation_physical_qualified') is not False):
        raise ValueError('Independent XY+yaw request, geometry and unqualified pilot scope required')
    return _verify_existing_pilot(review,stage,purpose,firmware_sha)
