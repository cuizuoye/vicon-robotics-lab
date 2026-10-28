#!/usr/bin/env python3
"""Private automatic frontend; preserve every accepted controller/firmware file."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_tracking_workspace_2026-10-27/prepared'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
        and not any(k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def edit(p,old,new):
    s=p.read_text()
    if s.count(old)!=1:raise ValueError('Exact parent layout changed: '+str(p)+' / '+old)
    p.write_text(s.replace(old,new))
def section(p,start,end,new):
    s=p.read_text();a=s.index(start);b=s.index(end,a);p.write_text(s[:a]+new+s[b:])


def generate(out):
    m=check(OLD/'build_manifest.json')
    if not m.get('tracking_workspace_passed'):raise ValueError('accepted tracking-only parent required')
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    protected={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):
        protected.update(m[group])
    runtime=out/'runtime/src';shutil.copytree(m['runtime_src'],runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    c=runtime/m['control_package']/m['control_package'];h=runtime/m['hardware_package']/m['hardware_package']
    policy=h/'standing_policy.py';shutil.copyfile(HERE/'standing_policy.py',policy)
    edit(policy,'__REQUEST_PATH__',repr(str(HERE/'USER_REQUEST.json')))
    edit(policy,'__REQUEST_SHA__',repr(sha(HERE/'USER_REQUEST.json')))
    auth=h/'authorization.py'
    section(auth,'SITE_KEYS=','\n\ndef verify_runtime_profile',
        'from .standing_policy import verify_preconditions\n')
    edit(auth,'def verify_runtime_profile(profile,purpose,now=None):',
         'def verify_runtime_profile(profile,purpose,now=None,steady=None):\n    verify_preconditions(profile)')
    edit(auth,"a=profile.get('staged_runtime_approval',{});now=time.time() if now is None else now",
         "a=profile.get('staged_runtime_approval',{});now=time.time() if now is None else now\n    steady=time.monotonic() if steady is None else steady")
    edit(auth,"or a.get('clear_and_power_cut_ready_now') is not True\n            or a.get('single_click_mode_acknowledged_now') is not True\n            or any(profile.get('tracking_confirmations',{}).get(k) is not True for k in SITE_KEYS)",
         "or a.get('automatic_preflight_launch_requested') is not True\n            or type(a.get('preflight_completed_steady_s')) not in (int,float)\n            or not 0<=steady-a['preflight_completed_steady_s']<=120.")
    edit(auth,'fresh tracking-space site confirmations required','fresh automatic machine preflight required; site conditions are user-declared defaults')
    commissioning=h/'commissioning.py'
    edit(commissioning,"CONSENT = '4WS8_TRACKING_ONLY_NO_FIXED_FENCE_CLEAR_POWER_CUT_READY'",
         "CONSENT = '4WS8_TRACKING_USER_DEFAULTS_FRESH_SPACE_REQUIRED_V1'")
    section(commissioning,'REQUIRED_CONFIRMATIONS =','MEASURED_FIELDS =',
        "REQUIRED_CONFIRMATIONS = ('footprint_measured','ground_height_measured','wiring_secured',\n"
        "                          'battery_protection_checked','wire_watchdog_600ms_documented')\n")
    section(commissioning,"        if mode == 'hardware':\n            required =",'        if self.profile and self.profile.get(\'mapping_scale\')',
        "        from .standing_policy import verify_preconditions\n        verify_preconditions(self.profile)\n")
    # Do not claim a fresh person has ticked anything in generated profiles.
    for i in range(7):
        p=runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json';v=json.loads(p.read_text())
        v.pop('tracking_confirmations',None);v['site_preconditions']={}
        for key in ('tracking_only_no_obstacle_detection_acknowledged','heading_checked','power_cut_ready'):
            v['confirmations'].pop(key,None)
        v['evidence']['site_scope']='User-declared standing prerequisites, NOT machine verification or a fresh human confirmation'
        write(p,v)
    for name in ('oneclick_app.py','auto_open.py','pose_check.py','ros_rehearsal.py'):
        shutil.copyfile(OLD/name,out/name)
    shutil.copyfile(HERE/'launcher_ui.py',out/'launcher_ui.py')
    app=out/'oneclick_app.py';edit(app,'READY=Path('+repr(str(OLD))+')','READY=Path('+repr(str(out))+')')
    edit(app,'from large_type import LargeTypeLauncherMixin','from launcher_ui import build_launcher')
    edit(app,"if m.get('tracking_workspace_passed') is not True:","if m.get('autostart_passed') is not True:")
    edit(app,'无固定围栏版离线验证尚未完成；不得启动','免勾选自动启动版尚未验证；不得启动')
    edit(app,"    return m\n\ndef check_port():", 
         "    from mecanum_wand_4ws8_hardware.standing_policy import preconditions_record\n    preconditions_record()\n    return m\n\ndef check_port():")
    edit(app,'def create_session(confirmations,preflight_at,now=None):','def create_session(preflight_at,now=None):')
    edit(app,"    if len(confirmations)!=3 or not all(v is True for v in confirmations):raise ValueError('请逐项确认现场条件；不会自动替你确认')\n",'')
    edit(app,"    profile['staged_runtime_approval']=dict(purpose='sign',issued_utc_s=time.time(),\n        clear_and_power_cut_ready_now=True,single_click_mode_acknowledged_now=True,flash_record=str(FLASH),build_manifest=str(READY/'build_manifest.json'))\n    profile['confirmations']['tracking_only_no_obstacle_detection_acknowledged']=True\n    profile['tracking_confirmations']={k:True for k in profile['tracking_confirmations']}",
         "    from mecanum_wand_4ws8_hardware.standing_policy import preconditions_record\n    profile['site_preconditions']=preconditions_record()\n    profile['staged_runtime_approval']=dict(purpose='sign',issued_utc_s=time.time(),\n        automatic_preflight_launch_requested=True,preflight_completed_steady_s=preflight_at,\n        flash_record=str(FLASH),build_manifest=str(READY/'build_manifest.json'))")
    edit(app,"approved=folder/'site_confirmed.json'","approved=folder/'automatic_preflight_session.json'")
    section(app,'    def __init__(self,root,instance=None,preview=False):','    def append(self,text):',
        '    def __init__(self,root,instance=None,preview=False):\n        build_launcher(self,root,instance,preview)\n')
    edit(app,' and all(v.get() for v in self.checks)','')
    edit(app,'create_session([v.get() for v in self.checks],self.good_at)','create_session(self.good_at)')
    edit(app,'                for v in self.checks:v.set(False)\n','')
    edit(app,'连接检查通过；逐项确认现场条件后将自动打开并准备。真实MCU反馈仍需通过安全链路核验。',
         '自检通过，自动打开并准备；仍需真实MCU零输出及停稳握手，READY后单击空格才运动。')
    edit(app,'下一轮先检查结果，再重新检查连接、确认现场；不会自动启动。',
         '本轮不自动重启；检查结果后可点击重新检查连接，仍须新的空格启动。')
    edit(app,'重新确认本轮全部路径空旷后才能准备','使用用户默认前提；自检通过才自动准备，必须单击空格才运动')
    section(app,'class App(LargeTypeLauncherMixin, AutomaticOpenMixin, ManualApp):','\n\ndef main():',
        'class App(AutomaticOpenMixin, ManualApp):\n    pass\n')
    # Share the existing global exclusion lock, but never silently display an
    # old/manual console as though it were this newly requested frontend.
    edit(app,"s.settimeout(1.);s.connect(str(path));s.sendall(b'SHOW')",
         "s.settimeout(1.);s.connect(str(path));s.sendall(b'SHOW_AUTO_DEFAULT_V1')\n            if s.recv(64)!=b'AUTO_DEFAULT_V1':raise ValueError('旧版/其他操作台仍在运行：请先正常关闭旧窗口，再打开新版；不会抢占或杀进程')")
    edit(app,"if connection.recv(16)==b'SHOW':self.root.deiconify();self.root.lift();self.root.focus_force()",
         "if connection.recv(64)==b'SHOW_AUTO_DEFAULT_V1':\n                        self.root.deiconify();self.root.lift();self.root.focus_force();connection.sendall(b'AUTO_DEFAULT_V1')")
    auto=out/'auto_open.py'
    edit(auto,'"""Auto-open ONLY after fresh preflight and real human site confirmations."""',
         '"""Auto-open after fresh machine preflight; prerequisites are standing user defaults."""')
    edit(auto,'<=120\n            and all(v.get() is True for v in self.checks))','<=120)')
    pose=out/'pose_check.py';edit(pose,repr(str(OLD/'runtime/src/mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json')),
         repr(str(runtime/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json')))
    inventory=[]
    for p in runtime.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:
            rel=p.relative_to(runtime);old=Path(m['runtime_src'])/rel
            if not old.is_file() or sha(p)!=sha(old):inventory.append(str(rel))
    expected={str(Path(m['hardware_package'])/m['hardware_package']/name) for name in
        ('authorization.py','commissioning.py','standing_policy.py')}
    if any(p not in expected and not p.startswith(m['hardware_package']+'/config/') for p in inventory):
        raise ValueError('Unexpected change to controller/bridge/console sources: '+str(inventory))
    write(out/'PRIVATE_CHANGE_INVENTORY.json',inventory)
    sources=hashes(HERE)
    for p in (ROOT/'tools/start_car_tracking_auto.sh',Path('/home/PRIMARY_USER/.local/bin/小车全场演示'),
        *(out/name for name in ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py'))):
        sources[str(p)]=sha(p)
    result=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=protected,autostart_passed=False,
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        startup_policy='USER_STANDING_SITE_PRECONDITIONS_V1',manual_checkboxes_required=False,
        automatic_motion_allowed=False,fresh_space_click_required=True,
        controller_bridge_and_console_bytes_unchanged=True)
    write(out/'build_manifest.json',result);check(out/'build_manifest.json');return result


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    generate(a.output.resolve());print('OFFLINE frontend generated; validation pending')
