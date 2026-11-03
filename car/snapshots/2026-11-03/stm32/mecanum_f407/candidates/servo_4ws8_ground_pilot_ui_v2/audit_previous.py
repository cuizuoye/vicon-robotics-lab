#!/usr/bin/env python3
"""Read-only analysis of the ended v1 observation, NOT an accepted trial."""
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-23/ground_pilot_070_080/ready'
for name in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
    sys.path.insert(0,str(OLD/'runtime/src'/name))
from mecanum_wand_4ws8_hardware.stage0_contract import decode_motion_status


def main():
    p=OLD/'ground_stage4_single_ndlzafos'
    rows=[json.loads(s) for s in (p/'observation.jsonl').read_text().splitlines()]
    raw=[json.loads(s) for s in (p/'mcu_receive.jsonl').read_text().splitlines()]
    last=next(r for r in reversed(raw) if r['kind']=='LINK_STATUS' and r['accepted_sample_steady'] is not None)
    final=decode_motion_status(bytes.fromhex(last['payload_hex']))
    active=[r for r in rows if r['topic']=='supervisor_status' and r['data']['state']=='ACTIVE']
    a,b=active[0]['received_steady'],active[-1]['received_steady']
    subset=lambda topic:[r for r in rows if r['topic']==topic and a-.03<=r['received_steady']<=b+.15]
    excursion=lambda poses:max(math.hypot(r['data']['x']-poses[0]['data']['x'],r['data']['y']-poses[0]['data']['y']) for r in poses)
    commands=[r for r in rows if r['topic']=='cmd_vel' and any(abs(v)>1e-9 for v in r['data'].values())]
    epochs={r['data']['epoch'] for r in active}
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    result=dict(identity='4WS8',purpose='sign',stage=4,physical_trial=True,
        action_and_stop_passed=False,reviewed_by_human=False,
        disposition='NOT_ACCEPTED_UI_REPREPARE_MISMATCH_AND_NEUTRAL_START_PROJECTION_DEFECT',
        actual_active_epochs=len(epochs),active_span_s=b-a,
        stop_reason='FINAL_GATE: PILOT_TIME_LIMIT_2S',
        observed_nonzero_command_count=len(commands),
        peak_gate_command_speed_mps=max(math.hypot(r['data']['vx'],r['data']['vy']) for r in commands),
        observed_sign_max_xy_excursion_m=excursion(subset('wand_pose')),
        observed_car_max_xy_excursion_m=excursion(subset('car_pose')),
        final_permitted=final['permitted'],final_pwm_permille=final['raw_duty'],final_compares=final['compares'],
        final_fault_wheel=final['fault_wheel'],final_watchdog_reset=final['watchdog_reset'],
        image_sha256=json.loads((p/'site_confirmed.json').read_text())['trial_review']['image_sha256'],
        user_report='操作台有问题，按顺序准备不了',
        measured_loaded_070_speed_passed=False,electrical_thermal_trial_reviewed=False,
        automatic_retry_performed=False,firmware_flashed_this_attempt=False,
        observation_sha256=sha(p/'observation.jsonl'),mcu_receive_sha256=sha(p/'mcu_receive.jsonl'),
        approved_profile_sha256=sha(p/'site_confirmed.json'))
    if result['final_permitted'] or any(result['final_pwm_permille']+result['final_compares']):
        raise ValueError('final zero not confirmed')
    target=p/'SESSION_RESULT.json'
    with target.open('x') as f:json.dump(result,f,indent=2,ensure_ascii=False);f.write('\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
