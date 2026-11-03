#!/usr/bin/env python3
"""Prepare fresh software-only A2 comparison and full regressions."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from accel_core import ROOT,LIVE,AccelerationLimits
from bootstrap import BASELINE,RUNTIME
from compare import sweep,run,cases

HERE=Path(__file__).resolve().parent


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def sources():
    paths=[p for p in HERE.iterdir() if p.is_file() and p.suffix in ('.py','.json','.md')]
    paths.append(ROOT/'tools/start_sign_rotation_accel_sim.sh')
    return {str(p):sha(p) for p in paths}


def previous_acceptance(out):
    folder=ROOT/'docs/wand_rotation_live_sim_2026-10-26/real_sign___g6kw_s'
    rows=[json.loads(line) for line in (folder/'observation.jsonl').read_text().splitlines()]
    active=[r for r in rows if r['report']['state']=='ACTIVE']
    angles=[abs(r['report']['relative_hand_yaw'] or 0.) for r in active]
    # User report and machine trace are deliberately separate. Do not fabricate
    # a logged +/-90deg or Esc acceptance if it was in another/unsaved session.
    write(out/'PREVIOUS_USER_REPORT.json',dict(
        user_report='都正常，接下来可以提高加速度，提高同步性',
        user_reported_translation_rotation_return_and_esc_normal=True,
        inspected_session=str(folder),logged_active_samples=len(active),
        logged_max_abs_relative_hand_yaw_deg=max(angles,default=0.)*180/3.141592653589793,
        logged_esc_event=any('Esc/' in e['event'] for e in json.loads((folder/'SESSION_RESULT.json').read_text())['events']),
        logged_full_requested_maneuver_proven=False,
        interpretation='Human report retained; this archived trace alone does not demonstrate all requested movements. No new physical authority inferred.',
        physical_hardware_accepted=False,occlusion_manual_recovery_acceptance_pending=True))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();out.mkdir(parents=True,exist_ok=False);out.chmod(0o700)
    prior=ROOT/'docs/wand_rotation_live_sim_2026-10-26/prepared/manifest.json'
    m=json.loads(prior.read_text());protected={str(prior):sha(prior)}
    for group in ('source_hashes','protected_hashes','output_hashes'):protected.update(m[group])
    for path,digest in protected.items():
        if sha(path)!=digest:raise ValueError('upstream drift: '+path)
    before=sources()
    report=sweep();write(out/'ACCELERATION_COMPARISON.json',report)
    best=report['recommended'];c=AccelerationLimits(best['xy_accel_mps2'],best['yaw_accel_radps2'])
    write(out/'RECOMMENDED_SIM_ONLY.json',dict(not_a_hardware_profile=True,
        xy_speed_mps=.7,xy_accel_mps2=c.max_accel_mps2,yaw_rate_radps=2.,yaw_accel_radps2=c.max_yaw_accel_radps2,
        xy_scale=1.,yaw_scale=1.,max_active_s=120.,actuator_outputs=False,physical_hardware_qualified=False,
        source_point='Sign5',region=dict(forward=[-1.5,.5],left=[-.4,.5],buffer_m=.5)))
    previous_acceptance(out)
    series={}
    for limits,label in ((AccelerationLimits(.8,6.),'baseline'),(c,'higher_acceleration')):
        for case in cases():
            result,rows=run(limits,case,keep=True)
            if not result['passed']:raise ValueError('Selected/reference case failed')
            fields=('t','car_x','car_y','car_yaw','target_x','target_y','hand_turn','yaw_error_deg',
                'same_time_error_m','command_vx','command_vy','command_wz','plant_wz','wheel_rpm')
            with (out/(case.name+'_'+label+'.csv')).open('x') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
                for row in rows:writer.writerow({k:row[k] for k in fields})
            series[(case.name,label)]=rows
    sensitivity=[]
    for lag in (.06,.20):
        for limits in (AccelerationLimits(.8,6.),c):
            for case in (cases()[0],cases()[-1]):
                r,_=run(limits,case,lag_s=lag)
                if not r['passed']:raise ValueError('Sensitivity safety failure')
                sensitivity.append(dict(limits=limits.name,**r))
    write(out/'PLANT_LAG_SENSITIVITY.json',sensitivity)
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(HERE)+os.pathsep+str(LIVE)+os.pathsep+
        str(ROOT/'candidates/wand_rotation_sim')+os.pathsep+str(ROOT/'candidates/wand_rotation_preview')+
        os.pathsep+os.environ.get('PYTHONPATH',''))
    def execute(label,args,environment):
        p=subprocess.run(args,cwd=ROOT,env=environment,capture_output=True,text=True,timeout=90)
        (out/(label+'.log')).write_text(p.stdout+p.stderr)
        print(label+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode:raise ValueError((p.stdout+p.stderr)[-14000:])
        return p.stdout.strip().splitlines()[-1]
    with tempfile.TemporaryDirectory(prefix='rotation_A2_x_') as task_dir:
        display_file=Path(task_dir)/'display'
        with display_file.open('w+') as display,(out/'private_x_server.log').open('w') as log:
            server=subprocess.Popen(['Xephyr','-displayfd',str(display.fileno()),'-screen','1280x720',
                '-nolisten','tcp','-ac'],env=dict(os.environ,DISPLAY=os.environ.get('DISPLAY',':1'),
                    XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority')),
                pass_fds=(display.fileno(),),stdout=log,stderr=log)
            try:
                deadline=time.monotonic()+5
                while not display_file.read_text().strip():
                    if server.poll() is not None or time.monotonic()>deadline:raise ValueError('private test display unavailable')
                    time.sleep(.02)
                private=':'+display_file.read_text().strip()
                env.update(DISPLAY=private,ROTATION_SIM_PRIVATE_TEST_DISPLAY=private)
                tests=execute('accel_and_existing_rotation_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    str(HERE/'test_accel_sim.py'),str(LIVE/'test_live_sim.py'),
                    str(ROOT/'candidates/wand_rotation_sim/test_rotation_sim.py'),
                    str(ROOT/'candidates/wand_rotation_preview/test_preview.py'),
                    '--junitxml='+str(out/'new_tests.xml')],env)
            finally:
                server.terminate()
                try:server.wait(timeout=3)
                except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
    old_env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',STAGED_GENERATED=str(RUNTIME),
        ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
        EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
        AUTOPREP_OUTPUT=str(BASELINE),PYTHONPATH=str(BASELINE)+os.pathsep+
            str(ROOT/'stm32/mecanum_f407/candidates/servo_4ws8')+os.pathsep+
            str(ROOT/'src/mecanum_vicon_control')+os.pathsep+os.environ.get('PYTHONPATH',''))
    old_tests=execute('accepted_runtime_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
        str(BASELINE/'tests'),'--junitxml='+str(out/'accepted_tests.xml')],old_env)
    for path,digest in protected.items():
        if sha(path)!=digest:raise ValueError('protected file changed: '+path)
    if sources()!=before:raise ValueError('A2 source drift during tests')
    base=report['rows'][0]
    summary=dict(offline_passed=True,tests=tests,accepted_runtime_tests=old_tests,protected_files=len(protected),
        baseline_accel=[.8,6.],recommended_accel=[c.max_accel_mps2,c.max_yaw_accel_radps2],
        selected_xy_mean_rms_improvement_pct=100*(1-best['xy_error_ratio']),
        selected_yaw_mean_rms_improvement_pct=100*(1-best['yaw_error_ratio']),
        metrics_scope='arithmetic mean of scenario RMS, same model/input and fixed speed; not measured real-user latency',
        actuator_outputs=False,hardware_changed=False,live_high_acceleration_acceptance_pending=True,
        caution='Faster-than-speed-cap gestures and unknown hardware dynamics remain unsynchronized; no hardware qualification')
    write(out/'SUMMARY.json',summary)
    outputs={str(p):sha(p) for p in out.iterdir() if p.is_file()}
    write(out/'manifest.json',dict(offline_passed=True,actuator_outputs=False,source_hashes=before,
        protected_hashes=protected,output_hashes=outputs))
    print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
