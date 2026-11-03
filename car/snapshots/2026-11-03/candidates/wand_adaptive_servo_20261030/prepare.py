"""Browser software-only adaptive fork; never edits accepted direct baseline."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_browser_direct_2026-10-30/prepared_v1'
OUT=ROOT/'docs/wand_adaptive_servo_2026-10-30/prepared_v1'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def change(s,old,new,count=1):
    if s.count(old)!=count:raise ValueError('pinned source layout: '+old[:80])
    return s.replace(old,new)

def prepare():
    subprocess.run(['bash',str(ROOT/'tools/start_sign_browser_direct.sh'),'--check'],check=True,capture_output=True)
    benchmark=json.loads((OUT.parent/'BENCHMARK_V3.json').read_text())
    if not benchmark['passed'] or not benchmark['selected']['eligible']:raise ValueError('offline comparison has unaccepted regressions')
    OUT.mkdir(exist_ok=False)
    for name in ('server.cjs','worker.py','channel.py','control_channel.py','mailbox.py','view_slot.cjs',
                 'lifecycle.py','software_runtime.py','browser_operator.py','profile_direct.py','plant_direct.py','node_direct.py','test_protocol.cjs'):
        shutil.copyfile(BASE/name,OUT/name)
    shutil.copytree(BASE/'runtime/src',OUT/'runtime/src',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    shutil.copytree(BASE/'web',OUT/'web')
    control=OUT/'runtime/src/mecanum_wand_4wsb_control/mecanum_wand_4wsb_control'
    shutil.copyfile(control/'direct_target.py',control/'direct_base.py')
    shutil.copyfile(HERE/'adaptive_servo.py',control/'adaptive_servo.py')
    with (control/'direct_target.py').open('a') as f:
        f.write('\n# Independent experiment; accepted baseline module remains untouched.\nfrom .adaptive_servo import AdaptiveSession\nDirectSession = AdaptiveSession\n')
    p=control/'safety_nodes.py';s=p.read_text()
    s=change(s,'reference_limited=self.core.follower.reference_limited,',
        "reference_limited=self.core.follower.reference_limited,\n            adaptive_servo=getattr(self.core.follower,'last_adaptive',{}),")
    p.write_text(s)
    for name in ('profile_direct.py','plant_direct.py','browser_operator.py'):
        p=OUT/name;s=p.read_text().replace('DIRECT_SIGN_NOMINAL_NO_C_FIRMWARE','ADAPTIVE_SIGN_NOMINAL_NO_C_FIRMWARE')
        s=s.replace('PRIVATE_BROWSER_DIRECT_SIGN_1TO1_V1_','PRIVATE_BROWSER_ADAPTIVE_SIGN_1TO1_V1_')
        s=s.replace("target_policy='DIRECT_SIGN_1TO1'","target_policy='ADAPTIVE_SIGN_1TO1'")
        if name=='profile_direct.py':
            s=change(s,"profile='BROWSER_DIRECT_SIGN_1TO1'","profile='BROWSER_ADAPTIVE_SIGN_1TO1'")
            s=change(s,'position_gain_per_s=1.4,',
                "adaptive=True,accel_range_mps2=[1.,4.],yaw_accel_range_radps2=[1.5,12.],position_gain_per_s='1.4..4.0',")
        p.write_text(s)
    p=OUT/'test_protocol.cjs';s=p.read_text().replace("'DIRECT_SIGN_1TO1'","'ADAPTIVE_SIGN_1TO1'");p.write_text(s)
    p=OUT/'web/app.js';s=p.read_text().replace('直接Sign · 1:1平移＋旋转','无级调速候选 · 1:1平移＋旋转')
    s=s.replace('4.00m/s² · 无实车输出','动态a 1～4m/s² / α 1.5～12rad/s² · 无实车输出');p.write_text(s)
    p=OUT/'web/index.html';s=p.read_text().replace('真实Sign → 1:1直接跟随','真实Sign → 无级调速候选').replace('Sign 1:1直接跟随 · 浏览器仿真','无级调速候选 · 浏览器仿真');p.write_text(s)
    manifest=dict(software_only=True,hardware_entry=False,domain=191,controller_modified=True,
        scope='BROWSER_ADAPTIVE_SIGN_1TO1',qualified=False,
        source_hashes={str(p):sha(p) for p in HERE.iterdir() if p.is_file()},
        artifact_hashes={str(p):sha(p) for p in OUT.rglob('*') if p.is_file()},
        fixed_baseline_manifest_sha256=sha(BASE/'MANIFEST.json'))
    save(OUT/'MANIFEST.json',manifest);print(OUT)

if __name__=='__main__':prepare()
