"""Explicit operator-requested UNQUALIFIED trial, not measured long-duration approval."""
import hashlib
import json
import math
import time
from pathlib import Path
from .authorization import endpoint,verify_flash_record,require,IMAGE_SHA
from .ground_prior import verify as verify_prior,PATH as PRIOR_PATH,DIGEST as PRIOR_DIGEST
from mecanum_wand_4wsc_control.tracking_config import TrackingSafetyConfig

KIND='4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'
INPUT='SPACE_TOGGLE_STAGE6_LONG600_V1'
CONSENT='STOPFIX_STAGE6_600S_UNQUALIFIED_SITE_READY_V1'
REVIEW_KIND='4WSC_STAGE6_600S_MEASURED_REVIEW_V1'
UNKNOWN=('stop_envelope_m','battery_current_a','driver_temperature_c','motor_temperature_c')

def common(p):
    require(isinstance(p,dict) and p.get('identity')=='4WSC' and p.get('purpose')=='sign'
        and p.get('trial_kind')==KIND,'fixed stage6 long600 purpose required')
    require(type(p.get('stage')) is int and p['stage']==6 and p.get('max_trial_s')==600.,
        'stage6 and maximum 600 seconds only')
    require(p.get('physical_qualified') is False and p.get('long_session_qualified') is False
        and p.get('trial_review')=={},'do not manufacture physical qualification')
    require(isinstance(p.get('measurements'),dict) and set(p['measurements'])==set(UNKNOWN)
        and all(p['measurements'][k] is None for k in UNKNOWN),'prior timer trial is not a measurement approval')
    require(type(p.get('mapping_scale')) in (int,float) and p['mapping_scale']==1.
        and p.get('operator_input_mode')==INPUT and p.get('arena_revision')=='tracking_only_stage6_long600_4wsc_v8',
        'direct 1:1 long600 policy required')
    require(p.get('baseline_observation')==dict(path=str(PRIOR_PATH),sha256=PRIOR_DIGEST,stage=6,physical_observation_only=True),
        'pinned same-stage 60s physical observation required')
    verify_prior()
    c=TrackingSafetyConfig(**p['safety']);h=p['hardware']
    require(c.stage==6 and (c.max_speed_mps,c.max_accel_mps2,c.max_yaw_rate_radps,c.max_yaw_accel_radps2)==(.7,2.,2.,12.)
        and c.max_session_s==600. and c.feedback_timeout_s==.3,'same stage6 limits and original feedback bound required')
    require(h.get('required_motion_tag')=='4WSC' and h.get('output_authority_version')==8
        and h.get('diagnostic_payload_bytes')==208 and h.get('wire_timeout_ms')==600
        and h.get('pwm_cap_permille')==800 and h.get('motor_target_rpm_cap')==400.
        and h.get('wheel_slew_guard_cps_s')==26000.,'exact existing stage6 hardware contract required')
    return c

def finite(v):return type(v) in (int,float) and math.isfinite(v)
def record(ref):
    require(isinstance(ref,dict) and isinstance(ref.get('path'),str) and ref['path']
        and isinstance(ref.get('sha256'),str),'600秒实机尚未开放：缺少有来源的实测记录')
    raw=Path(ref['path']).read_bytes()
    require(len(raw)<=65536 and hashlib.sha256(raw).hexdigest()==ref['sha256'],'measurement evidence changed')
    r=json.loads(raw)
    require(r.get('physical_trial') is True and r.get('identity')=='4WSC'
        and r.get('image_sha256')==IMAGE_SHA and r.get('stage')==6
        and r.get('human_reviewed') is True,'current-image stage6 physical measurement required, not simulation')
    return r

def verify_long_review(p):
    r=p.get('long_duration_review',{})
    require(isinstance(r,dict) and r.get('kind')==REVIEW_KIND and r.get('human_reviewed') is True
        and r.get('identity')=='4WSC' and r.get('image_sha256')==IMAGE_SHA and r.get('stage')==6
        and r.get('baseline_review_sha256')==PRIOR_DIGEST
        and isinstance(r.get('reviewer'),str) and bool(r['reviewer'].strip()),
        '600秒实机尚未开放：缺运动中停车、失联停车余量和电气/温升审核；软件回归不代替测量')
    b=record(r.get('braking_evidence'))
    require(b.get('schema')=='4wsc-stage6-braking-v1' and b.get('manual_stop_during_motion_verified') is True
        and b.get('includes_600ms_fallback') is True and b.get('footprint_included') is True
        and finite(b.get('tested_speed_mps')) and b['tested_speed_mps']>=.7
        and finite(b.get('tested_yaw_rate_radps')) and b['tested_yaw_rate_radps']>=2.,
        'full current-stage moving-stop/fallback/footprint evidence required')
    d=b.get('measured_stop_envelope_m');radius=math.hypot(.095,.105);clearance=r.get('minimum_clearance_m')
    require(finite(d) and d>=.7*.6+radius and finite(clearance) and clearance>=d+.2,
        'measured full envelope and explicit clearance with original extra margin required')
    e=record(r.get('electrical_thermal_evidence'))
    require(e.get('schema')=='4wsc-stage6-electrical-thermal-v1' and e.get('accepted_for_controlled_600s_trial') is True
        and e.get('same_configuration_verified') is True and finite(e.get('observed_run_s')) and e['observed_run_s']>=60
        and all(finite(e.get(k)) and e[k]>=0 for k in ('battery_current_a','driver_temperature_c','motor_temperature_c')),
        'measured current and driver/motor temperature review required')
    return True

def verify_runtime_profile(p,purpose,now=None):
    common(p);require(purpose=='sign','real Sign input only');endpoint(p)
    r=p.get('commissioning_review',{});issued=r.get('issued_utc_s');now=time.time() if now is None else now
    require(finite(issued) and 0<=now-issued<=600 and r.get('kind')==KIND
        and r.get('operator_declared_site_ready_now') is True and r.get('unqualified_trial_acknowledged_now') is True
        and type(r.get('human_authorized_stage')) is int and r['human_authorized_stage']==6
        and r.get('max_active_s')==600.,'fresh explicit site declaration required')
    verify_flash_record(r.get('flash_record'))
    if 'operator_duration_trial' in p:
        verify_operator_trial(p)
    else:
        verify_long_review(p)
    return True


AUTHORIZATION_PATH=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_600s_trial_2026-11-02/OPERATOR_REQUEST.json')
AUTHORIZATION_SHA='4ce5a358454bcb1f2e828c7a9e0a0a3dfdc1121c55832e461837e2f55b930452'
# Appended into the private admission module by build.py. No hardware IO.
def verify_operator_trial(p):
    expected=dict(kind='EXPLICIT_UNQUALIFIED_600S_OPERATOR_REQUEST_V1',
        authorization_path=str(AUTHORIZATION_PATH),authorization_sha256=AUTHORIZATION_SHA,
        maximum_active_s=600.,stage=6,measurements_still_unknown=True,
        no_physical_qualification_claim=True,onsite_power_cut_operator_present=True)
    require(p.get('operator_duration_trial')==expected,'explicit unqualified duration trial declaration required')
    require(p.get('long_duration_review')=={},'trial must not masquerade as measured approval')
    raw=AUTHORIZATION_PATH.read_bytes()
    require(hashlib.sha256(raw).hexdigest()==AUTHORIZATION_SHA,'operator request record changed')
    r=json.loads(raw)
    require(r.get('operator_requested_unqualified_600s_trial') is True
        and r.get('site_confirmation_received') is True
        and r.get('physical_qualified') is False
        and r.get('electrical_thermal_verified') is False
        and r.get('physical_stop_envelope_verified') is False,
        'explicit request is not a physical qualification')
