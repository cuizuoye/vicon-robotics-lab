#!/usr/bin/env python3
"""Separate single-click runtime. Preparation/tests never start real ROS or I/O."""
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
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_offset/ready'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(d):return {str(p):sha(p) for p in d.rglob('*') if p.is_file() and not any(
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
    c=runtime/m['control_package']/m['control_package'];h=runtime/m['hardware_package']/m['hardware_package']
    for name in ('toggle_ui.py','toggle_keys.py','demo_policy.py'):shutil.copyfile(HERE/name,c/name)
    shutil.copyfile(HERE/'demo_authorization.py',h/'demo_authorization.py')
    ui=c/'operator_console.py'
    edit(ui,'from .pilot_ui import PilotOperatorModel as OperatorModel',
        'from .toggle_ui import ToggleOperatorModel as OperatorModel\nfrom .toggle_keys import ToggleKeysMixin')
    edit(ui,"root.bind_class('SafeWandKeys', '<Escape>', lambda _: (self.node.request('stop'), 'break')[1])",
        "root.bind_class('SafeWandKeys', '<Escape>', self.escape)")
    edit(ui,'def run_console(runtime=None):',
        'class ToggleConsole(ToggleKeysMixin, Console):\n    def __init__(self,root,node,model):\n        if not isinstance(model,OperatorModel):raise ValueError("dedicated single-click model required")\n        super().__init__(root,node,model)\n\n\nConsole = ToggleConsole\n\n\ndef run_console(runtime=None):')
    edit(ui,"held=diagnostic[4], key_down=diagnostic[5], focused=diagnostic[6], blocked=diagnostic[7],",
        "held=diagnostic[4], key_down=diagnostic[5], focused=diagnostic[6], blocked=diagnostic[7],\n                operator_input_mode=self.model.input_mode,session_requested=self.model.session_requested,\n                click_pending=self.model.click_pending,trial_ended=self.model.ended,")
    edit(ui,'2.0×0.9m全向跟随演示 · 最长120秒','单击启停 · 2.0×0.9m跟随 · 最长120秒')
    edit(ui,"('pause', '暂停 / 松开空格')","('pause', '暂停 / 结束本轮')")
    edit(ui,'操作：点准备 → 等就绪 → 按住空格移动Sign；松开即结束。正常结束不要点急停。',
        '单击模式：准备 → 就绪 → 轻按松开空格启动；松键不停车。再次空格/Esc/停止结束。')
    edit(ui,'先按提示松键解锁，再准备；空格要持续按住，点按后松开不会运行。切出窗口会停车。',
        '单击模式：准备就绪后轻按松开空格启动；再次空格/Esc停止；切出窗口也停车。')
    edit(h/'stages.py','BOUNDED_SIGNED_ARENA_120S_4WS8_V4_STAGE_','BOUNDED_SPACE_TOGGLE_120S_4WS8_V4_STAGE_')
    edit(h/'authorization.py','BOUNDED_OFFSET_ARENA_120S_NOT_QUALIFICATION','BOUNDED_SPACE_TOGGLE_120S_NOT_QUALIFICATION')
    edit(h/'authorization.py',"    stage=profile.get('stage');limits=get_stage(stage);hw=profile.get('hardware',{});s=profile.get('safety',{})",
        "    if profile.get('operator_input_mode')!='SPACE_TOGGLE_120S_V1':raise ValueError('explicit single-click profile required')\n    stage=profile.get('stage');limits=get_stage(stage);hw=profile.get('hardware',{});s=profile.get('safety',{})")
    edit(h/'authorization.py',"a.get('clear_and_power_cut_ready_now') is not True or",
        "a.get('clear_and_power_cut_ready_now') is not True or\n        a.get('single_click_mode_acknowledged_now') is not True or")
    edit(h/'commissioning.py','4WS8_SIGNED_ARENA_120S_CLEAR_POWER_CUT_READY','4WS8_SPACE_TOGGLE_120S_CLEAR_POWER_CUT_READY')
    for p in (runtime/m['hardware_package']/'config').glob('stage*.NOT_APPROVED.json'):
        profile=json.loads(p.read_text());profile['operator_input_mode']='SPACE_TOGGLE_120S_V1'
        profile['evidence']['operator_request']=str(HERE/'USER_REQUEST.json')
        profile['evidence']['input_semantics']='Explicit tap starts; release does not stop; second Space/Esc/focus/fault/120s stops. Not ground-accepted.'
        p.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
    tests=out/'tests';shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    edit(tests/'test_demo_120s.py','clear_and_power_cut_ready_now=True,',
        'clear_and_power_cut_ready_now=True,single_click_mode_acknowledged_now=True,')
    for name in ('test_toggle.py',):shutil.copyfile(HERE/name,tests/name)
    env=dict(os.environ,STAGED_GENERATED=str(runtime),ONECLICK_SOURCE=str(HERE),
        PYTHONPATH=str(BASE)+os.pathsep+os.environ.get('PYTHONPATH',''),
        DISPLAY=os.environ.get('DISPLAY',':1'),XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    r=subprocess.run([sys.executable,'-m','pytest','-q',str(tests),'--junitxml='+str(out/'host_tests.xml')],
        cwd=ROOT,env=env,capture_output=True,text=True,timeout=60)
    log=r.stdout+r.stderr;(out/'host_tests.log').write_text(log);print(log,flush=True)
    if r.returncode:raise ValueError('single-click regressions failed; hardware never opened')
    pins=dict(m['protected_input_hashes'])
    for key in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[key])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    sources={**m['source_hashes'],**hashes(HERE)}
    for p in (ROOT/'tools/start_car_demo_120s_toggle.sh',Path('/home/PRIMARY_USER/.local/bin/小车单击')):sources[str(p)]=sha(p)
    result=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=pins,draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        host_change='Separate approved single-tap intent, second Space/Esc stop; no synthetic hold, no core/firmware/range/timing changes; old held UI untouched.')
    (out/'build_manifest.json').write_text(json.dumps(result,indent=2)+'\n');check(out/'build_manifest.json')
    r=subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
        '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),
        '--symlink-install','--packages-select',m['control_package'],m['hardware_package']],
        cwd=ROOT,env=env,capture_output=True,text=True,timeout=60)
    (out/'colcon_build.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True)
    if r.returncode:raise ValueError('independent single-click build failed')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,
        tests=log.strip().splitlines()[-1],protected_files_unchanged=len(pins),
        operator_input_mode='SPACE_TOGGLE_120S_V1',max_active_s=120.,
        core_algorithms_and_firmware_unchanged=True,old_held_entry_unchanged=True,
        hardware_started=False,flashed=False,live_ros_or_vicon_started=False,
        single_click_ground_accepted=False),indent=2)+'\n')

if __name__=='__main__':main()
