"""File evidence first. No socket, ROS, firmware writer or implicit approval."""
import hashlib
import json
import math
import ipaddress
from pathlib import Path
import time
from .stages import get_stage

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def verify_flash_record(record_path,build_manifest):
    if not record_path:raise ValueError('POST_FLASH_RECORD_REQUIRED')
    m=json.loads(Path(build_manifest).read_text());r=json.loads(Path(record_path).read_text())
    fw=m['firmware']
    if fw.get('identity')!='4WS8' or fw.get('authority_version')!=4:
        raise ValueError('4WS8 v4 build required; legacy readback cannot approve this image')
    image=Path(fw['path']);readback=Path(r['full_readback_path'])
    if (fw['identity']!='4WS8' or fw['authority_version']!=4 or
        sha(image)!=fw['sha256'] or r.get('image_sha256')!=fw['sha256'] or
        r.get('identity')!='4WS8' or r.get('flashed') is not True or
        r.get('independent_readback_verified') is not True or readback.stat().st_size!=1048576 or
        sha(readback)!=r.get('full_readback_sha256') or readback.read_bytes()[:image.stat().st_size]!=image.read_bytes()):
        raise ValueError('4WS8 v4 independent full readback evidence mismatch')
    return r

def verify_trial_review(review,stage,purpose,firmware_sha):
    get_stage(stage)
    if isinstance(review, dict) and review.get('kind') == 'TRACKING_ONLY_XY_YAW_4WS8_NOT_FULL_COVERAGE_QUALIFICATION':
        from .pilot_authorization import verify_pilot_review
        return verify_pilot_review(review,stage,purpose,firmware_sha)
    if (not isinstance(review,dict) or review.get('kind')!='BOUNDED_TRIAL_NOT_QUALIFICATION' or
        review.get('identity')!='4WS8' or review.get('image_sha256')!=firmware_sha or
        type(review.get('authorized_stage')) is not int or review.get('authorized_stage')!=stage or review.get('purpose')!=purpose or
        review.get('human_authorized') is not True):
        raise ValueError('stage-specific human trial review required')
    if stage:
        if (type(review.get('previous_stage')) is not int or review.get('previous_stage')!=stage-1 or
            review.get('previous_stage_action_and_stop_passed') is not True or
            not isinstance(review.get('previous_stage_evidence'),str)):
            raise ValueError('previous stage evidence required; no stage skipping')
        path=Path(review['previous_stage_evidence']);previous=json.loads(path.read_text())
        if (sha(path)!=review.get('previous_stage_evidence_sha256') or previous.get('identity')!='4WS8' or
            previous.get('image_sha256')!=firmware_sha or type(previous.get('stage')) is not int or
            previous['stage']!=stage-1 or previous.get('purpose')!=purpose or
            previous.get('physical_trial') is not True or previous.get('action_and_stop_passed') is not True or
            previous.get('reviewed_by_human') is not True):
            raise ValueError('previous physical trial evidence mismatch')
    if purpose=='sign' and stage:
        # No assumed coast deceleration or 50cm automatic approval. Numeric
        # envelope must include the existing 600ms fallback and body margin.
        stop=review.get('measured_trial_stop_distance_m');speed=review.get('stop_test_speed_mps')
        if (type(stop) not in (int,float) or not math.isfinite(stop) or stop<0 or
            type(speed) not in (int,float) or not math.isfinite(speed) or speed<get_stage(stage).speed or
            review.get('stop_envelope_includes_fallback_and_footprint') is not True or
            review.get('stop_envelope_fits_original_buffer') is not True or stop>.50 or
            review.get('electrical_thermal_trial_reviewed') is not True):
            raise ValueError('higher-stage ground stopping/electrical review incomplete')

def verify_runtime_profile(profile,purpose,now=None):
    if purpose not in ('sign','suspended'):raise ValueError('fixed purpose required')
    if profile.get('operator_input_mode')!='SPACE_TOGGLE_120S_V1':raise ValueError('explicit single-click profile required')
    stage=profile.get('stage');limits=get_stage(stage);hw=profile.get('hardware',{});s=profile.get('safety',{})
    from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig
    config=StagedSafetyConfig(**s)
    if (config.stage!=stage or profile.get('arena_revision')!='fixed_reference_signed_120s_buffer050_v1' or
        config.arena_forward_min_m!=-1.5 or config.arena_forward_max_m!=.5 or
        config.arena_left_min_m!=-.4 or config.arena_left_max_m!=.5 or config.arena_buffer_m!=.5 or
        config.half_length_m!=.095 or config.half_width_m!=.105 or config.boundary_margin_m<.20 or
        config.max_session_s>120 or config.goal_boundary_policy!='clamp'):
        raise ValueError('fixed1:1 / confirmed signed arena /50cm safety geometry required')
    ip=ipaddress.IPv4Address(hw.get('esp_ip',''))
    interface=hw.get('network_interface')
    if (not ip.is_private or ip.is_loopback or ip.is_unspecified or
        not isinstance(interface,str) or not interface or len(interface)>15 or
        not all(c.isalnum() or c in '_-.' for c in interface)):
        raise ValueError('explicit private endpoint and network interface required')
    a=profile.get('staged_runtime_approval',{});now=time.time() if now is None else now
    if (a.get('purpose')!=purpose or type(a.get('issued_utc_s')) not in (int,float) or
        not 0<=now-a['issued_utc_s']<=600 or a.get('clear_and_power_cut_ready_now') is not True or
        a.get('single_click_mode_acknowledged_now') is not True or
        (purpose=='suspended' and a.get('four_wheels_secured_clear_now') is not True)):
        raise ValueError('fresh stage-specific site approval required')
    record=verify_flash_record(a.get('flash_record'),a.get('build_manifest'))
    verify_trial_review(profile.get('trial_review'),stage,purpose,record['image_sha256'])
    if (profile.get('mapping_scale')!=1 or hw.get('required_motion_tag')!='4WS8' or
        hw.get('output_authority_version')!=4 or hw.get('diagnostic_payload_bytes')!=184 or
        hw.get('pwm_cap_permille')!=limits.duty or hw.get('motor_target_rpm_cap')!=limits.target_rpm or
        hw.get('wire_timeout_ms')!=600 or s.get('max_speed_mps')!=limits.speed or s.get('max_accel_mps2')!=limits.accel):
        raise ValueError('runtime differs from exact fixed stage')
    return True
