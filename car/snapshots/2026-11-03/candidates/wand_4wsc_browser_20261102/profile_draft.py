"""Unapproved, non-activating profiles derived from recorded measurements."""
import importlib.util
import json
from pathlib import Path

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
SPEC=importlib.util.spec_from_file_location('fourwsc_stage_data',ROOT/'candidates/wand_sync_4wsc_20261030/stages.py')
ST=importlib.util.module_from_spec(SPEC)
import sys
sys.modules[SPEC.name]=ST;SPEC.loader.exec_module(ST)

def draft(stage=0,purpose='sign'):
    s=ST.get_stage(stage)
    base=ROOT/'docs/wand_4wsb_host_2026-10-30/prepared_v9/runtime/src/mecanum_wand_4wsb_hardware/config/stage0.NOT_APPROVED.json'
    p=json.loads(base.read_text())
    p.update(identity='4WSC',purpose=purpose,stage=stage,arena_revision='tracking_only_600s_4wsc_v8',
        trial_review={},staged_runtime_approval={},physical_qualified=False)
    p['hardware'].update(required_motion_tag='4WSC',output_authority_version=8,
        motor_target_rpm_cap=s.target_rpm,pwm_cap_permille=s.duty,wheel_slew_guard_cps_s=s.wheel_slew_guard)
    p['safety'].update(max_speed_mps=s.speed,max_accel_mps2=s.accel,
        max_yaw_rate_radps=s.yaw_rate,max_yaw_accel_radps2=s.yaw_accel)
    p['evidence']['change_scope']='4WSC independent host draft; measurements are historical, not fresh site approval'
    p['tracking_confirmations']={k:False for k in ('all_intended_paths_and_stopping_clearance_checked_now',
        'no_fixed_fence_or_obstacle_detection_acknowledged_now','immediate_power_cut_ready_now')}
    return p
