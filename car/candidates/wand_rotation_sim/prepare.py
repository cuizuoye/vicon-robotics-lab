#!/usr/bin/env python3
"""Generate only offline evidence. Does not flash, launch ROS, or move a car."""
import argparse
import csv
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

from bootstrap import ROOT,BASELINE,MANIFEST,RUNTIME,check
from experiments import sweep,run_case,cases
from rotation_core import SimRotationLimits

HERE=Path(__file__).resolve().parent
PREVIEW=ROOT/'candidates/wand_rotation_preview'
SESSION=ROOT/'docs/wand_rotation_preview_2026-10-26/readonly_uppz8hn2'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def own_hashes():
    files=[p for p in HERE.iterdir() if p.is_file() and p.suffix in ('.py','.json','.md')]
    files.append(ROOT/'tools/check_sign_rotation_sim.sh')
    return {str(p):sha(p) for p in files}
def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();out.mkdir(parents=True,exist_ok=False);out.chmod(0o700)
    preserved={}
    for key in ('runtime_hashes','source_hashes','protected_input_hashes','generated_firmware_hashes'):
        preserved.update(MANIFEST[key])
    preserved[str(BASELINE/'build_manifest.json')]=sha(BASELINE/'build_manifest.json')
    preserved[MANIFEST['firmware']['path']]=MANIFEST['firmware']['sha256']
    pm=ROOT/'docs/wand_rotation_preview_2026-10-26/prepared/manifest.json'
    for path,value in json.loads(pm.read_text())['source_hashes'].items():preserved[path]=value
    for path in (pm,SESSION/'geometry_confirmed.json',SESSION/'observation.jsonl',
            ROOT/'docs/wand_rotation_preview_2026-10-26/initial_geometry.json',
            ROOT/'docs/wand_rotation_preview_2026-10-26/DIRECTION_ACCEPTANCE.json',
            ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/tests/offline_harness.py',
            ROOT/'src/mecanum_vicon_control/mecanum_vicon_control/mecanum_kinematics.py'):
        preserved[str(path)]=sha(path)
    before=own_hashes()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(HERE)+os.pathsep+str(PREVIEW)+os.pathsep+os.environ.get('PYTHONPATH',''))
    def run(label,args,environment):
        result=subprocess.run(args,cwd=ROOT,env=environment,text=True,capture_output=True,timeout=90)
        (out/(label+'.log')).write_text(result.stdout+result.stderr)
        print(label+': '+('PASS' if result.returncode==0 else 'FAIL'),flush=True)
        if result.returncode:print((result.stdout+result.stderr)[-15000:]);raise ValueError(label+' failed')
        return result.stdout.strip().splitlines()[-1]
    new_tests=run('rotation_and_reference_tests',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
        str(HERE/'test_rotation_sim.py'),str(PREVIEW/'test_preview.py'),'--junitxml='+str(out/'new_tests.xml')],env)
    old_env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',STAGED_GENERATED=str(RUNTIME),
        ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
        EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
        AUTOPREP_OUTPUT=str(BASELINE),PYTHONPATH=str(BASELINE)+os.pathsep+str(ROOT/'stm32/mecanum_f407/candidates/servo_4ws8')+
            os.pathsep+str(ROOT/'src/mecanum_vicon_control')+os.pathsep+os.environ.get('PYTHONPATH',''))
    old_tests=run('accepted_runtime_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
        str(BASELINE/'tests'),'--junitxml='+str(out/'accepted_tests.xml')],old_env)
    result=sweep();write(out/'parameter_sweep.json',result)
    selected=result['recommended']
    if selected is None:raise ValueError('No software parameters meet declared normal-hand tracking criteria')
    c=SimRotationLimits(selected['rate_radps'],selected['accel_radps2'])
    all_results=[];series=[]
    for case in cases():
        r,rows=run_case(c,case,keep_rows=True);all_results.append(r)
        if not r['passed']:raise ValueError('selected model failed '+case.name)
        if case.name=='normal_90deg_1p5s':series=rows
    no_ff,_=run_case(c,cases()[1],feedforward=False)
    cap_baseline,_=run_case(SimRotationLimits(.1,.4),cases()[1])
    sensitivity=[run_case(c,replace(cases()[1],lag_s=lag))[0] for lag in (.06,.12,.20)]
    if not no_ff['passed'] or not cap_baseline['passed'] or not all(r['passed'] for r in sensitivity):
        raise ValueError('comparison/sensitivity model failed; do not hide failed runs')
    summary=dict(kind='OFFLINE_XY_YAW_ROTATION_SERVO',passed=True,software_model_only=True,
        tests=dict(new=new_tests,accepted_runtime=old_tests),recommended_sim_only=selected,
        scenarios=all_results,comparison=dict(same_candidate_with_current_yaw_caps=cap_baseline,
            same_limits_position_only=no_ff,feedforward=next(r for r in all_results if r['case']==cases()[1].name)),
        sensitivity=sensitivity,actuator_outputs=False,hardware_qualified=False,
        real_sign_simulation_integrated=False,
        next_required='Real Sign -> SOFTWARE car adapter with selected-source identity binding in independent gate; no hardware launch')
    write(out/'summary.json',summary)
    write(out/'recommended_software_only.json',dict(not_a_hardware_profile=True,limits=dict(
        xy_speed_mps=.70,xy_accel_mps2=.80,yaw_rate_radps=c.max_yaw_rate_radps,
        yaw_accel_radps2=c.max_yaw_accel_radps2,xy_scale=1.,yaw_scale=1.,max_active_s=120.),
        base_point='Sign5',direction='OUTSIDE_CROSSBAR_AWAY_FROM_STEM',physical_hardware_qualified=False))
    fields=('t','hand_turn','target_yaw','goal_yaw','car_yaw','angular_error_deg','command_wz','plant_wz',
        'car_x','car_y','target_x','target_y','same_time_error_m','command_speed_mps','wheel_rpm')
    with (out/'normal_90deg_series.csv').open('x') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in series:writer.writerow({k:row[k] for k in fields})
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import math
    t=[r['t'] for r in series];start=series[0]['target_yaw']-series[0]['hand_turn']
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
    axes[0].plot(t,[math.degrees(r['hand_turn']) for r in series],label='Wand reference')
    axes[0].plot(t,[math.degrees(r['car_yaw']-start) for r in series],label='Software car (lag 0.12s)')
    axes[0].set_ylabel('Relative yaw (deg)');axes[0].legend();axes[0].grid(alpha=.3)
    axes[0].set_title(f'SIM ONLY: 1:1 XY+yaw, yaw {c.max_yaw_rate_radps:.2f} rad/s, {c.max_yaw_accel_radps2:.2f} rad/s^2')
    axes[1].plot(t,[r['angular_error_deg'] for r in series],label='Same-time angular error')
    axes[1].set_ylabel('Error (deg)');axes[1].set_xlabel('Virtual time (s)');axes[1].grid(alpha=.3)
    fig.tight_layout();fig.savefig(out/'normal_90deg_tracking.png',dpi=130);plt.close(fig)
    if own_hashes()!=before:raise ValueError('new source changed while tests ran')
    check(BASELINE/'build_manifest.json')
    for path,value in preserved.items():
        if sha(path)!=value:raise ValueError('preserved file changed: '+path)
    outputs={str(p):sha(p) for p in out.iterdir() if p.is_file()}
    write(out/'manifest.json',dict(offline_passed=True,hardware_qualified=False,tests=summary['tests'],
        source_hashes=before,protected_hashes=preserved,output_hashes=outputs))
    print(json.dumps(dict(passed=True,protected_files=len(preserved),tests=summary['tests'],
        software_only_recommendation=[c.max_yaw_rate_radps,c.max_yaw_accel_radps2],
        hardware_qualified=False,output=str(out)),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
