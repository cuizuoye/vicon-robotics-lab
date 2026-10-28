#!/usr/bin/env python3
"""Build a separate diagnostic candidate; never initialise ROS or open hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/oneclick_v1/ready'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def hashes(d):
    return {str(p):sha(p) for p in d.rglob('*') if p.is_file() and not any(
        k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def edit(p,a,b):
    s=p.read_text()
    if s.count(a)!=1:raise ValueError('input drift: '+str(p)+' '+a)
    p.write_text(s.replace(a,b))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);(out/'COLCON_IGNORE').write_text('')
    runtime=out/'runtime/src';runtime.parent.mkdir()
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','*.egg-info','.pytest_cache'))
    c=runtime/m['control_package']/m['control_package']
    shutil.copyfile(HERE/'pose_quality.py',c/'pose_quality.py')
    edit(c/'safety_nodes.py','        self.core.inputs.graph_ok = False',
        '        from .pose_quality import CarPoseQuality\n        self.car_pose_quality = CarPoseQuality(self.core.c)\n        self.first_stop_reason = ""\n        self.core.inputs.graph_ok = False')
    edit(c/'safety_nodes.py','            self.core.inputs.observe_pose(source, pose, stamp, steady, ros, roll, pitch)',
        '            self.core.inputs.observe_pose(source, pose, stamp, steady, ros, roll, pitch)\n            if source == "car":\n                self.car_pose_quality.observe(pose, stamp, steady, ros, roll, pitch)')
    edit(c/'safety_nodes.py',"            self.core.inputs.event('POSE_DECODE_INVALID')",
        "            self.core.inputs.event('POSE_DECODE_INVALID')\n            if source == 'car':self.car_pose_quality.reject('POSE_DECODE_INVALID', steady)")
    edit(c/'safety_nodes.py','        f = self.core.tick(steady, ros)',
        "        f = self.core.tick(steady, ros)\n        if self.core.state in ('STOPPING','STOP_UNCONFIRMED','FAULT') and not self.first_stop_reason:\n            self.first_stop_reason = self.core.reason")
    edit(c/'safety_nodes.py','            pilot=self.core.pilot_status(steady),',
        '            pilot=self.core.pilot_status(steady), first_stop_reason=self.first_stop_reason,\n            car_pose_quality=self.car_pose_quality.snapshot(),')
    edit(c/'safety_nodes.py','            speed_mps=math.hypot(output.vx, output.vy)))))',
        '            speed_mps=math.hypot(output.vx, output.vy),\n            car_pose_quality=self.car_pose_quality.snapshot()))))')
    edit(c/'novice_ui.py',"    if model.pilot_done:return ''",
        "    if model.pilot_done:\n        from .pose_quality import stop_hint\n        return stop_hint(s,model.gate)")
    tests=out/'tests'
    shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    # Only diagnostic adapter fields differ; the safety algorithms must still
    # be byte-identical. The new tests separately exercise the actual adapter.
    edit(tests/'test_oneclick.py',"'safety_core.py','safety_nodes.py','pilot_core.py'",
        "'safety_core.py','pilot_core.py'")
    edit(tests/'test_demo_range.py',"'safety_wire.py','safety_nodes.py','arena_config.py'",
        "'safety_wire.py','arena_config.py'")
    edit(tests/'test_origin_capture.py','        transport_report=None,operator_gate_ready=lambda *a:True)',
        '        transport_report=None,operator_gate_ready=lambda *a:True,first_stop_reason="",\n        car_pose_quality=SimpleNamespace(snapshot=lambda:{}))')
    shutil.copyfile(HERE/'test_pose_quality.py',tests/'test_pose_quality.py')
    env=dict(os.environ,STAGED_GENERATED=str(runtime),ONECLICK_SOURCE=str(HERE),
        PYTHONPATH=str(BASE)+os.pathsep+os.environ.get('PYTHONPATH',''),
        DISPLAY=os.environ.get('DISPLAY',':1'),
        XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    r=subprocess.run([sys.executable,'-m','pytest','-q',str(tests),
        '--junitxml='+str(out/'host_tests.xml')],cwd=ROOT,env=env,capture_output=True,text=True,timeout=60)
    log=r.stdout+r.stderr;(out/'host_tests.log').write_text(log);print(log,flush=True)
    if r.returncode:raise ValueError('diagnostic regression failed; no hardware opened')
    pins=dict(m['protected_input_hashes'])
    for key in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[key])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    evidence=OLD/'oneclick_session_87zu239_'
    for name in ('observation.jsonl','mcu_receive.jsonl','launch.log'):
        pins[str(evidence/name)]=sha(evidence/name)
    sources={**m['source_hashes'],**hashes(HERE)}
    entry=ROOT/'tools/start_car_oneclick_v2.sh';sources[str(entry)]=sha(entry)
    result=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        source_hashes=sources,protected_input_hashes=pins,
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        host_change='Diagnostic only: continuous pose preflight and persistent first-stop detail. No pose repair, filtering, guard relaxation, recovery or default-entry change.')
    (out/'build_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    check(out/'build_manifest.json')
    r=subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
        '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),
        '--symlink-install','--packages-select',m['control_package'],m['hardware_package']],
        cwd=ROOT,env=env,capture_output=True,text=True,timeout=60)
    (out/'colcon_build.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True)
    if r.returncode:raise ValueError('private diagnostic build failed')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,
        tests=log.strip().splitlines()[-1],protected_files_unchanged=len(pins),
        hardware_started=False,flashed=False,live_vicon_opened=False,
        motion_guards_and_algorithm_byte_identical=True,default_entry_changed=False),indent=2)+'\n')

if __name__=='__main__':main()
