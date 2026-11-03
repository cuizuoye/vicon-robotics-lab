#!/usr/bin/env python3
"""Generate/test separate automatic-prepare host only. No hardware launch."""
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
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
LAUNCHER=ROOT/'tools/start_car_demo_120s_toggle_eventio.sh'
sys.path.insert(0,str(BASE))
from launch_cli import check

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file() and not any(
        k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def edit(path,old,new):
    text=path.read_text()
    if text.count(old)!=1:raise ValueError('exact source fragment drift: '+str(path)+' '+old[:80])
    path.write_text(text.replace(old,new))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').write_text('')
    before=hashes(HERE)
    pins=dict(m['protected_input_hashes'])
    for key in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[key])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    for name in ('ESP_BUILD_RESULT.json','ESP_SOURCE_INPUTS.json'):pins[str(OLD/name)]=sha(OLD/name)
    flash=ROOT/'esp32/mecanum_wifi_bridge/backups/pre_eventio_20261026_prx6c2en/POST_FLASH_VERIFIED.json'
    pins[str(flash)]=sha(flash)
    backup=out/'previous_eventio_launcher.sh';shutil.copyfile(LAUNCHER,backup)
    proposal=out/'PROPOSED_LAUNCHER.sh'
    text=LAUNCHER.read_text()
    if text.count(str(OLD.relative_to(ROOT)))!=1:raise ValueError('launcher source drift')
    proposal.write_text(text.replace(str(OLD.relative_to(ROOT)),str(out.relative_to(ROOT))).replace(
        '# Separate event-receive candidate. No automatic preparation or enable.',
        '# Guided auto-open/prepare. Motion requires a NEW real Space click.'))
    pins[str(backup)]=sha(backup)
    plan=dict(path=str(LAUNCHER),old_sha256=sha(LAUNCHER),new_sha256=sha(proposal),
        backup=str(backup),proposal=str(proposal),explicit_user_request=str(HERE/'USER_REQUEST.json'),
        only_authorized_source_path=str(LAUNCHER))
    (out/'LAUNCHER_MIGRATION_PLAN.json').write_text(json.dumps(plan,indent=2)+'\n')
    runtime=out/'runtime/src'
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    control=runtime/m['control_package']/m['control_package']
    shutil.copyfile(HERE/'auto_prepare.py',control/'auto_prepare.py')
    ui=control/'operator_console.py'
    edit(ui,'Console = ToggleConsole','''from .auto_prepare import AutoPrepareConsoleMixin

class AutomaticPrepareConsole(AutoPrepareConsoleMixin, ToggleConsole):
    pass

Console = AutomaticPrepareConsole''')
    edit(ui,"root.geometry('1120x790')","root.geometry('1280x720')")
    edit(ui,'root.minsize(1000, 740)','root.minsize(1000, 640)')
    # Pixel fonts keep 1280x720 usable on this workstation's high-DPI display,
    # without touching X11/desktop or Tk's global scaling settings.
    ui_text=ui.read_text()
    for points,pixels in ((18,24),(11,16),(10,14),(9,13)):
        for space in (' ', ''):
            ui_text=ui_text.replace(f"'Noto Sans CJK SC',{space}{points}",f"'Noto Sans CJK SC',{space}-{pixels}")
    ui.write_text(ui_text)
    edit(ui,'width=690, height=430','width=690, height=260')
    edit(ui,"'单击模式：准备 → 就绪 → 轻按松开空格启动；松键不停车。再次空格/Esc/停止结束。'",
        "'自动检查与准备 → 看到就绪后单击空格开始；再次空格/Esc/停止结束；松键不停车。'")
    app=out/'oneclick_app.py';shutil.copyfile(OLD/'oneclick_app.py',app)
    shutil.copyfile(HERE/'auto_open.py',out/'auto_open.py')
    edit(app,'READY=Path('+repr(str(OLD))+')','READY=Path('+repr(str(out))+')')
    edit(app,"from tkinter import ttk,messagebox","from tkinter import ttk,messagebox\nfrom auto_open import AutomaticOpenMixin")
    edit(app,'class App:','class ManualApp:')
    # Append wrapper before main use while leaving all base methods intact.
    edit(app,'def main():','class App(AutomaticOpenMixin, ManualApp):\n    pass\n\n\ndef main():')
    edit(app,'root.geometry(\'900x700\');root.minsize(820,650)',
        "root.geometry('1280x720');root.minsize(1000,640)")
    app_text=app.read_text()
    for points,pixels in ((19,26),(12,16),(11,16),(10,14)):
        app_text=app_text.replace(f"'Noto Sans CJK SC',{points}",f"'Noto Sans CJK SC',-{pixels}")
    if app_text.count('wraplength=820')!=2:raise ValueError('launcher wrapping source drift')
    app.write_text(app_text.replace('wraplength=820','wraplength=1160'))
    edit(app,'        frame=ttk.Frame(root,padding=22);frame.pack',
        "        style.configure('TCheckbutton',font=('Noto Sans CJK SC',-16))\n        frame=ttk.Frame(root,padding=18);frame.pack")
    edit(app,'self.detail=tk.Text(frame,height=9','self.detail=tk.Text(frame,height=6')
    edit(app,'小车单击 · 事件接收修复候选 · 120秒','小车单击 · 自动准备 · 1280×720 · 120秒')
    edit(app,'小车操作台：检查 → 准备 → 单击空格跟随','小车操作台：安全确认 → 自动检查/准备 → 单击空格跟随')
    edit(app,'只有检查通过并由你逐项确认后，才能打开跟随窗口。已运行的旧程序不会被自动杀掉。',
        '逐项确认现场条件后，检查通过会自动打开跟随窗口并准备；不会自动运动，也不会自动杀掉旧程序。')
    edit(app,'跟随窗口中：点“准备” → 等“准备就绪” → 轻按并松开空格一次，然后移动Sign。',
        '勾选现场条件 → 检查通过后自动打开并准备 → 看到就绪后轻按并松开空格一次，然后移动Sign。')
    edit(app,'连接检查通过。请逐项确认现场条件，再打开跟随窗口；真实MCU反馈将由安全链路继续核验。',
        '连接检查通过；逐项确认现场条件后将自动打开并准备。真实MCU反馈仍需通过安全链路核验。')
    edit(app,'跟随窗口已打开：请在新窗口点准备，等待就绪后单击空格；这里只显示运行状态。',
        '跟随窗口已自动打开：无需点准备，等显示准备就绪后单击空格；打开和准备均不运动。')
    edit(app,"WAND_4WS8_RAW_RECEIVE_TRACE=str(folder/'mcu_receive.jsonl'))",
        "WAND_4WS8_RAW_RECEIVE_TRACE=str(folder/'mcu_receive.jsonl'),\n        WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1')")
    tests=out/'tests';shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copyfile(HERE/'test_autoprep.py',tests/'test_autoprep.py')
    # Explicitly authorized UI wrapper changes. All hardware/safety/controller
    # decisions remain under the inherited exact-identity checks and new AST test.
    edit(tests/'test_eventio.py',"        if str(relative)!=f'{hardware}/{hardware}/nodes.py':",
        "        if str(relative) not in (f'{hardware}/{hardware}/nodes.py',\n                'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/operator_console.py'):")
    edit(tests/'test_startup_clock.py',
        "    allowed={f'{hardware}/{hardware}/nodes.py',f'{control}/{control}/novice_ui.py'}",
        "    allowed={f'{hardware}/{hardware}/nodes.py',f'{control}/{control}/novice_ui.py',\n        f'{control}/{control}/operator_console.py'}")
    env=dict(os.environ,STAGED_GENERATED=str(runtime),ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
        EVENTIO_OUTPUT=str(OLD),AUTOPREP_OUTPUT=str(out),PYTHONPATH=str(out)+os.pathsep+str(BASE)+os.pathsep+os.environ.get('PYTHONPATH',''),
        DISPLAY=os.environ.get('DISPLAY',':1'),XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    def run(name,args):
        p=subprocess.run(list(map(str,args)),cwd=ROOT,env=env,capture_output=True,text=True,timeout=90)
        (out/(name+'.log')).write_text(p.stdout+p.stderr)
        print(name+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode:print((p.stdout+p.stderr)[-14000:],flush=True);raise ValueError(name+' failed')
        return p.stdout+p.stderr
    tests_log=run('host_tests',[sys.executable,'-m','pytest','-q',tests,'--junitxml='+str(out/'host_tests.xml')])
    run('host_build',['colcon','--log-base',out/'colcon_log','build','--base-paths',runtime,
        '--build-base',out/'runtime/build','--install-base',out/'runtime/install','--symlink-install',
        '--packages-select',m['control_package'],m['hardware_package']])
    if hashes(HERE)!=before:raise ValueError('candidate source changed during tests')
    manifest=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        source_hashes={**m['source_hashes'],**before,str(app):sha(app),str(out/'auto_open.py'):sha(out/'auto_open.py')},
        protected_input_hashes=pins,draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        host_change='Fresh human site confirmation; auto-open and one safe prepare; NEW real Space click only for motion; 1280x720.')
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,tests=tests_log.strip().splitlines()[-1],
        protected_files_unchanged=len(pins),hardware_started=False,flashed=False,
        motion_started=False,default_window_size=[1280,720],automatic_motion=False,
        automatic_prepare_attempts_max=1,human_site_confirmations_preserved=True,
        feedback_protection_ms=200,hello_cap_ms=40,launcher_promoted=False),indent=2)+'\n')

if __name__=='__main__':main()
