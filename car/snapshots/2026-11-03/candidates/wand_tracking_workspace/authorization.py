"""Independent tracking-space approval; does not claim surveyed coverage."""
import ipaddress
import time
from .tracking_parent_authorization import verify_flash_record, verify_trial_review
from .stages import get_stage
from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig, SPATIAL_POLICY

SITE_KEYS=('all_intended_paths_and_stopping_clearance_checked_now',
           'no_fixed_fence_or_obstacle_detection_acknowledged_now',
           'immediate_power_cut_ready_now')


def verify_runtime_profile(profile,purpose,now=None):
    if purpose!='sign' or profile.get('stage')!=4:
        raise ValueError('tracking-only entry is fixed to the existing stage4 Sign trial')
    if profile.get('operator_input_mode')!='SPACE_TOGGLE_120S_V1':
        raise ValueError('single-click input semantics required')
    c=TrackingSafetyConfig(**profile['safety']);s=get_stage(4);hw=profile['hardware']
    if (profile.get('arena_revision')!=SPATIAL_POLICY or profile.get('mapping_scale')!=1.
            or profile.get('input_workspace')!=dict(spatial_policy=SPATIAL_POLICY,
                fixed_fence_enabled=False,target_radius_limit_enabled=False,
                full_vicon_coverage_measured=False,obstacle_detection_available=False)):
        raise ValueError('exact tracking-only policy required; no claimed coverage polygon')
    ip=ipaddress.IPv4Address(hw.get('esp_ip',''));interface=hw.get('network_interface')
    if (not ip.is_private or ip.is_loopback or ip.is_unspecified or
            not isinstance(interface,str) or not 0<len(interface)<=15 or
            not all(k.isalnum() or k in '_-.' for k in interface)):
        raise ValueError('explicit private endpoint and interface required')
    a=profile.get('staged_runtime_approval',{});now=time.time() if now is None else now
    if (a.get('purpose')!='sign' or type(a.get('issued_utc_s')) not in (int,float)
            or not 0<=now-a['issued_utc_s']<=600
            or a.get('clear_and_power_cut_ready_now') is not True
            or a.get('single_click_mode_acknowledged_now') is not True
            or any(profile.get('tracking_confirmations',{}).get(k) is not True for k in SITE_KEYS)):
        raise ValueError('fresh tracking-space site confirmations required')
    r=verify_flash_record(a.get('flash_record'),a.get('build_manifest'))
    verify_trial_review(profile.get('trial_review'),4,'sign',r['image_sha256'])
    if (hw.get('required_motion_tag')!='4WS8' or hw.get('output_authority_version')!=4
            or hw.get('diagnostic_payload_bytes')!=184 or hw.get('wire_timeout_ms')!=600
            or hw.get('pwm_cap_permille')!=s.duty or hw.get('motor_target_rpm_cap')!=s.target_rpm
            or (c.stage,c.max_speed_mps,c.max_accel_mps2)!=(4,.7,.8)):
        raise ValueError('unchanged exact 4WS8 stage4 limits required')
    return True
