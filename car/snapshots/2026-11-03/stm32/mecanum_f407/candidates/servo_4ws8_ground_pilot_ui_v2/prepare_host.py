#!/usr/bin/env python3
"""Separate UI/neutral-start repair; preserves v1 trial, firmware and rectangle."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
V1=HERE.parent/'servo_4ws8_ground_pilot'
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-23/ground_pilot_070_080/ready'
BASE=HERE.parent/'servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
        and not any(v in ('__pycache__','.pytest_cache') or v.endswith('.egg-info') for v in p.parts)}


def edit(path,old,new):
    text=path.read_text()
    if text.count(old)!=1:raise ValueError('input drift: '+str(path)+' '+old)
    path.write_text(text.replace(old,new))


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    out=p.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);(out/'COLCON_IGNORE').write_text('')
    runtime=out/'runtime/src';runtime.parent.mkdir()
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','*.egg-info','.pytest_cache'))
    os.symlink(OLD/'firmware',out/'firmware',target_is_directory=True)
    control=runtime/m['control_package']/m['control_package'];hw=runtime/m['hardware_package']/m['hardware_package']
    pc=control/'pilot_core.py'
    edit(pc,'max_active_s=2.0','max_active_s=5.0')
    edit(pc,"return 'PILOT_TIME_LIMIT_2S'","return 'PILOT_TIME_LIMIT_5S'")
    edit(pc,'class PilotMonitor:\n    def __init__(self):','class PilotMonitor:\n    def __init__(self, config):\n        self.config = config')
    edit(pc,"        if self.used:\n            return 'PILOT_CONSUMED_RELAUNCH_REQUIRES_REVIEW'\n        if self.epoch == epoch:",
         "        if self.used:\n            return 'PILOT_CONSUMED_RELAUNCH_REQUIRES_REVIEW'\n        issue = start_issue(self.config, inputs)\n        if issue:\n            return issue\n        if self.epoch == epoch:")
    text=pc.read_text()
    if text.count('self.pilot = PilotMonitor()')!=2:raise ValueError('monitor constructor drift')
    pc.write_text(text.replace('self.pilot = PilotMonitor()','self.pilot = PilotMonitor(config)'))
    edit(pc,'def validate_config(config):',
        "def start_issue(config, inputs):\n    value = inputs.poses.get('car')\n    if not value or not value['pose'].valid():\n        return 'PILOT_START_POSE_REQUIRED'\n    car = value['pose']\n    goal = config.project_goal(car, car, config.max_goal_radius_m)\n    if math.hypot(goal.x-car.x, goal.y-car.y) > .0005:\n        return 'PILOT_START_REQUIRES_INTERIOR: 起点在绿色目标区外，请保持松键，人工向区域内部挪几厘米；软件不会自动挪车'\n    return ''\n\n\ndef validate_config(config):")
    edit(pc,"        return super().request_start(steady, ros)\n",
        "        issue = start_issue(self.c, self.inputs)\n        if issue:\n            return False, issue\n        return super().request_start(steady, ros)\n")
    edit(pc,'    def tick(self, steady, ros):\n        frame = super().tick(steady, ros)',
        "    def pilot_status(self, steady):\n        return dict(used=self.pilot.used, max_active_s=POLICY['max_active_s'],\n            remaining_s=max(0., POLICY['max_active_s']-(steady-self.pilot.started)) if self.pilot.started is not None else POLICY['max_active_s'],\n            prepare_blocked=start_issue(self.c,self.inputs) if not self.pilot.used else 'PILOT_CONSUMED')\n\n    def tick(self, steady, ros):\n        frame = super().tick(steady, ros)")
    edit(control/'safety_nodes.py','            pid=os.getpid(), state=self.core.state, reason=self.core.reason, epoch=self.core.epoch,',
        '            pid=os.getpid(), state=self.core.state, reason=self.core.reason, epoch=self.core.epoch,\n            pilot=self.core.pilot_status(steady),')
    shutil.copyfile(HERE/'pilot_ui.py',control/'pilot_ui.py')
    ui=control/'operator_console.py'
    edit(ui,'from .operator_model import OperatorModel','from .pilot_ui import PilotOperatorModel as OperatorModel')
    edit(ui,'from .operator_guidance import center_bounds, reason_text, real_instruction, wand_preview',
        'from .operator_guidance import center_bounds, wand_preview\nfrom .pilot_ui import reason_text, real_instruction')
    edit(ui,'单次落地小位移 · 最长2秒','单次落地小位移 · 最长5秒')
    edit(ui,'单次≤2秒 / 杆≤15cm / 车≤20cm','单次≤5秒 / 杆≤15cm / 车≤20cm')
    edit(ui,"                +(f'杆可在Vicon有效跟踪区活动，无额外位移限位\\n'",
        "                +(f'首轮杆仅前移约5cm，水平总偏移≤15cm\\n'")
    edit(ui,'本轮时限{self.config.max_session_s:.0f}秒','本轮最多5秒、一次使能；车中心偏移≤20cm')
    edit(ui,"                   and not status.get('hard_fault') and not self.model.held)",
        "                   and not status.get('hard_fault') and not self.model.held and not self.model.pilot_done)")
    edit(ui,"        self.status_text.set(f'监督器：{state}     末级门：{gate_text}\\n'",
        "        display_state = '本轮结束 / 等待停稳' if self.model.pilot_done and not status.get('hard_fault') and not gate.get('hard_fault') else state\n        self.status_text.set(f'监督器：{display_state}     末级门：{gate_text}\\n'")
    edit(hw/'commissioning.py',"CONSENT = '4WS8_FINITE_GROUND_PILOT_CLEAR_POWER_CUT_READY'",
        "CONSENT = '4WS8_FINITE_GROUND_PILOT_5S_CLEAR_POWER_CUT_READY'")
    edit(hw/'stages.py',"SCOPE='FINITE_GROUND_PILOT_4WS8_V4_STAGE_'","SCOPE='FINITE_GROUND_PILOT_V2_5S_4WS8_V4_STAGE_'")
    # A private executable copy with fixed resolved source roots; the old
    # launch file and its v1 evidence remain byte-identical.
    launcher=(V1/'launch_pilot.py').read_text()
    launcher=launcher.replace('HERE=Path(__file__).resolve().parent','HERE=Path('+repr(str(V1))+')')
    launcher=launcher.replace('max2s','max5s').replace('at most2s','at most5s').replace('最长2秒','最长5秒').replace('；杆只前移','；杆只前移')
    launcher=launcher.replace("phrase='4WS8_FINITE_GROUND_PILOT_CLEAR_POWER_CUT_READY'","phrase='4WS8_FINITE_GROUND_PILOT_5S_CLEAR_POWER_CUT_READY'")
    (out/'launch_pilot.py').write_text(launcher)
    # Re-run the previous 54 injection tests with the explicit new 5s policy.
    tests=out/'tests';tests.mkdir()
    code=(V1/'test_pilot.py').read_text()
    code=code.replace("Path(__file__).with_name('USER_AUTHORIZATION.json')","Path("+repr(str(V1/'USER_AUTHORIZATION.json'))+")")
    code=code.replace('h.run(2.0)','h.run(5.0)').replace('m.started+2.001','m.started+5.001').replace('h.now-2.01','h.now-5.01').replace('PILOT_TIME_LIMIT_2S','PILOT_TIME_LIMIT_5S')
    (tests/'test_pilot_bounds.py').write_text(code)
    shutil.copyfile(HERE/'test_ui.py',tests/'test_pilot_ui.py')
    env=os.environ.copy();env.update(STAGED_GENERATED=str(runtime),PYTHONPATH=str(BASE)+os.pathsep+env.get('PYTHONPATH',''))
    # A withdrawn Tk root checks actual widgets without showing an actionable
    # hardware window. Fake node only: no rclpy.init, services or UDP.
    env.update(DISPLAY=env.get('DISPLAY',':1'),XAUTHORITY=env.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    r=subprocess.run([sys.executable,'-m','pytest','-q',str(tests),'--junitxml='+str(out/'pilot_v2_tests.xml')],
        cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
    (out/'pilot_v2_tests.log').write_text(r.stdout);print(r.stdout,flush=True)
    if r.returncode:raise ValueError('private UI/neutral-start regression failed; no IO')
    pins=dict(m['protected_input_hashes'])
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[group])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    manifest=dict(m);manifest.update(runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        source_hashes={**m['source_hashes'],**hashes(HERE)},protected_input_hashes=pins,
        host_change='Finite pilot v2: explicit used/remaining/start-block state in GUI, no repeated prepare, neutral start check, at most5s, unchanged short-path bounds',
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    manifest['source_hashes'][str(out/'launch_pilot.py')]=sha(out/'launch_pilot.py')
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');check(out/'build_manifest.json')
    r=subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
        '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),
        '--symlink-install','--packages-select',m['control_package'],m['hardware_package']],cwd=ROOT,env=env,
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
    (out/'colcon_build.log').write_text(r.stdout);print(r.stdout,flush=True)
    if r.returncode:raise ValueError('private build failed')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,flashed=False,
        hardware_started=False,ground_qualified=False,previous_trial_preserved=True),indent=2)+'\n')


if __name__=='__main__':main()
