"""Explicit, unqualified stage4 ground commissioning, NOT a release bypass.

This purpose exists to collect the first measurements. No measurement is
invented and this profile is never accepted as a qualified long demonstration.
"""
import math
import time
from .authorization import endpoint,verify_flash_record,require
from mecanum_wand_4wsc_control.tracking_config import TrackingSafetyConfig
from .ground_prior import verify as verify_prior,PATH as PRIOR_PATH,DIGEST as PRIOR_DIGEST

KIND='4WSC_STAGE4_GROUND_COMMISSIONING_SINGLE60S_V1'
INPUT='SPACE_TOGGLE_STAGE4_GROUND60_V1'
CONSENT='4WSC_GROUND_STAGE4_SINGLE60S_UNQUALIFIED_POWER_CUT_READY'
UNKNOWN=('stop_envelope_m','battery_current_a','driver_temperature_c','motor_temperature_c')


def common(p):
    require(isinstance(p,dict),'ground profile object required')
    require(p.get('trial_kind')==KIND and p.get('identity')=='4WSC' and p.get('purpose')=='sign',
        'explicit stage4 ground-commissioning purpose required')
    require(type(p.get('stage')) is int and p['stage']==4 and p.get('max_trial_s')==60.,'stage4 and sixty seconds only')
    require(p.get('physical_qualified') is False and p.get('long_session_qualified') is False,
        'first trial must remain unqualified')
    require(p.get('trial_review')=={} and isinstance(p.get('measurements'),dict) and set(p['measurements'])==set(UNKNOWN)
        and all(p['measurements'][k] is None for k in UNKNOWN),'do not forge prerequisite measurements')
    require(type(p.get('mapping_scale')) in (int,float) and p['mapping_scale']==1. and p.get('operator_input_mode')==INPUT
        and p.get('arena_revision')=='tracking_only_stage4_ground60_4wsc_v8','fixed direct 1:1 ground60 policy required')
    baseline=p.get('baseline_observation',{})
    require(baseline==dict(path=str(PRIOR_PATH),sha256=PRIOR_DIGEST,stage=0,physical_observation_only=True),
        'pinned current-image stage0 physical observation required; not high-tier qualification')
    verify_prior()
    c=TrackingSafetyConfig(**p['safety']);h=p['hardware']
    require(c.stage==4 and (c.max_speed_mps,c.max_accel_mps2,c.max_yaw_rate_radps,c.max_yaw_accel_radps2)==(.7,.8,1.5,6.)
        and c.max_session_s==60. and c.feedback_timeout_s==.3,'exact stage4 limits and original feedback bound required')
    require(h.get('required_motion_tag')=='4WSC' and h.get('output_authority_version')==8
        and h.get('diagnostic_payload_bytes')==208 and h.get('wire_timeout_ms')==600
        and h.get('pwm_cap_permille')==800 and h.get('motor_target_rpm_cap')==400.
        and h.get('wheel_slew_guard_cps_s')==26000.,'exact existing stage4 hardware contract required')
    return c


def verify_runtime_profile(p,purpose,now=None):
    common(p);require(purpose=='sign','ground Sign input only');endpoint(p)
    review=p.get('commissioning_review',{});issued=review.get('issued_utc_s');now=time.time() if now is None else now
    require(type(issued) in (int,float) and math.isfinite(issued) and 0<=now-issued<=600,
        'fresh explicit ground-site declaration required')
    require(review.get('kind')==KIND and review.get('operator_declared_site_ready_now') is True
        and review.get('unqualified_trial_acknowledged_now') is True
        and review.get('human_authorized_stage')==4 and type(review.get('human_authorized_stage')) is int
        and review.get('max_active_s')==60.,'explicit one-shot stage4 ground trial only')
    verify_flash_record(review.get('flash_record'))
    return True
