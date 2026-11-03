"""Explicit, unqualified stage0 ground commissioning, NOT a release bypass.

This purpose exists to collect the first measurements. No measurement is
invented and this profile is never accepted as a qualified long demonstration.
"""
import math
import time
from .authorization import endpoint,verify_flash_record,require
from mecanum_wand_4wsc_control.tracking_config import TrackingSafetyConfig

KIND='4WSC_STAGE0_GROUND_COMMISSIONING_SINGLE60S_V1'
INPUT='SPACE_TOGGLE_GROUND60_V1'
CONSENT='4WSC_GROUND_STAGE0_SINGLE60S_UNQUALIFIED_POWER_CUT_READY'
UNKNOWN=('stop_envelope_m','battery_current_a','driver_temperature_c','motor_temperature_c')


def common(p):
    require(isinstance(p,dict),'ground profile object required')
    require(p.get('trial_kind')==KIND and p.get('identity')=='4WSC' and p.get('purpose')=='sign',
        'explicit stage0 ground-commissioning purpose required')
    require(type(p.get('stage')) is int and p['stage']==0 and p.get('max_trial_s')==60.,'stage0 and sixty seconds only')
    require(p.get('physical_qualified') is False and p.get('long_session_qualified') is False,
        'first trial must remain unqualified')
    require(p.get('trial_review')=={} and isinstance(p.get('measurements'),dict) and set(p['measurements'])==set(UNKNOWN)
        and all(p['measurements'][k] is None for k in UNKNOWN),'do not forge prerequisite measurements')
    require(type(p.get('mapping_scale')) in (int,float) and p['mapping_scale']==1. and p.get('operator_input_mode')==INPUT
        and p.get('arena_revision')=='tracking_only_ground60_4wsc_v8','fixed direct 1:1 ground60 policy required')
    c=TrackingSafetyConfig(**p['safety']);h=p['hardware']
    require(c.stage==0 and (c.max_speed_mps,c.max_accel_mps2,c.max_yaw_rate_radps,c.max_yaw_accel_radps2)==(.2,.08,.2,.8)
        and c.max_session_s==60. and c.feedback_timeout_s==.3,'exact low-tier limits and original feedback bound required')
    require(h.get('required_motion_tag')=='4WSC' and h.get('output_authority_version')==8
        and h.get('diagnostic_payload_bytes')==208 and h.get('wire_timeout_ms')==600
        and h.get('pwm_cap_permille')==350 and h.get('motor_target_rpm_cap')==100.
        and h.get('wheel_slew_guard_cps_s')==26000.,'exact existing stage0 hardware contract required')
    return c


def verify_runtime_profile(p,purpose,now=None):
    common(p);require(purpose=='sign','ground Sign input only');endpoint(p)
    review=p.get('commissioning_review',{});issued=review.get('issued_utc_s');now=time.time() if now is None else now
    require(type(issued) in (int,float) and math.isfinite(issued) and 0<=now-issued<=600,
        'fresh explicit ground-site declaration required')
    require(review.get('kind')==KIND and review.get('operator_declared_site_ready_now') is True
        and review.get('unqualified_trial_acknowledged_now') is True
        and review.get('human_authorized_stage')==0 and type(review.get('human_authorized_stage')) is int
        and review.get('max_active_s')==60.,'explicit one-shot stage0 ground trial only')
    verify_flash_record(review.get('flash_record'))
    return True
