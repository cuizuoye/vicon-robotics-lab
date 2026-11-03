"""Reproducible independent fork; sealed SIM12 and rectangle remain immutable."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE = Path(__file__).resolve().parent
OUT = ROOT/'docs/wand_browser_direct_2026-10-30/prepared_v1'
BASE = ROOT/'docs/wand_browser_sim12_2026-10-30/prepared_v4'
HOST = ROOT/'docs/wand_4wsb_host_2026-10-30/prepared_v9'
SIM = ROOT/'candidates/wand_sim12_20261030'


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p, value): p.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
def change(s, old, new, count=1):
    if s.count(old) != count: raise ValueError('sealed layout mismatch: '+old[:100])
    return s.replace(old, new)


def generate():
    if (OUT/'MANIFEST.json').exists() and json.loads((OUT/'MANIFEST.json').read_text())['qualified']:
        raise ValueError('qualified release is immutable; prepare a new version')
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ('server.cjs','worker.py','channel.py','control_channel.py','mailbox.py','view_slot.cjs','lifecycle.py'):
        shutil.copyfile(BASE/name, OUT/name)
    p=OUT/'server.cjs';s=p.read_text()
    s=change(s,'car:v.car,goal:v.goal,reference:v.reference,',
        'car:v.car,goal:v.goal,target_policy:v.target_policy,')
    p.write_text(s)
    p=OUT/'worker.py';s=p.read_text()
    s=change(s,"'car','wand','goal','reference',", "'car','wand','goal','target_policy',")
    s=change(s,'goal=goal,reference=None,trace=', "goal=goal,target_policy='DIRECT_SIGN_1TO1',trace=")
    p.write_text(s)
    shutil.copytree(BASE/'web', OUT/'web', dirs_exist_ok=True)
    shutil.copytree(HOST/'runtime', OUT/'runtime', dirs_exist_ok=True, ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    control = OUT/'runtime/src/mecanum_wand_4wsb_control/mecanum_wand_4wsb_control'
    shutil.copyfile(HERE/'direct_target.py', control/'direct_target.py')
    # Distance-to-governed-proxy bounds are NOT repurposed as raw demand
    # bounds: a large valid hand target can lag while motor limits apply.
    p = control/'safety_core.py';s = p.read_text()
    s = change(s, "elif math.hypot(goal.x-self.inputs.poses['car']['pose'].x,\n                            goal.y-self.inputs.poses['car']['pose'].y) > self.c.max_tracking_error_m:",
        "elif (not getattr(self.c, 'direct_sign_target', False) and\n                    math.hypot(goal.x-self.inputs.poses['car']['pose'].x,\n                            goal.y-self.inputs.poses['car']['pose'].y) > self.c.max_tracking_error_m):")
    s = change(s, "elif math.hypot(f.goal.x-self.inputs.poses['car']['pose'].x,\n                        f.goal.y-self.inputs.poses['car']['pose'].y) > self.c.max_tracking_error_m:",
        "elif (not getattr(self.c, 'direct_sign_target', False) and\n                math.hypot(f.goal.x-self.inputs.poses['car']['pose'].x,\n                        f.goal.y-self.inputs.poses['car']['pose'].y) > self.c.max_tracking_error_m):")
    p.write_text(s)
    p = control/'rotation_core.py';s = p.read_text();cut=s.index('class RotationSupervisor(')
    p.write_text(s[:cut]+'# Independent direct-target cores; same physical output and source guards.\nfrom .direct_target import DirectSupervisor as RotationSupervisor, DirectFinalGate as RotationFinalGate\n')
    p = control/'demo_policy.py';s=p.read_text()
    s=change(s,"if abs(math.atan2(math.sin(goal.yaw-car.yaw), math.cos(goal.yaw-car.yaw)))>.65:",
        "if (not getattr(monitor.config, 'direct_sign_target', False) and\n            abs(math.atan2(math.sin(goal.yaw-car.yaw), math.cos(goal.yaw-car.yaw)))>.65):")
    s=s.replace("reference_point='Sign5'", "reference_point='Sign4'")
    p.write_text(s)
    shutil.copyfile(HERE/'profile_direct.py', OUT/'profile_direct.py')
    s=(SIM/'plant12.py').read_text().replace('from profile12 import Policy','from profile_direct import Policy').replace('SIM12_NOMINAL_NO_C_FIRMWARE','DIRECT_SIGN_NOMINAL_NO_C_FIRMWARE')
    (OUT/'plant_direct.py').write_text(s)
    s=(SIM/'node12.py').read_text().replace('from profile12 import runtime,install,save','from profile_direct import runtime,install,save')
    (OUT/'node_direct.py').write_text(s)
    s=(BASE/'software_runtime.py').read_text()
    s=change(s,'from profile12 import runtime,save,protected,contract','from profile_direct import runtime,save,protected,contract')
    s=change(s,'from plant12 import SoftwarePlant','from plant_direct import SoftwarePlant')
    s=change(s,"str(SIM/'node12.py')", "str(Path(__file__).parent/'node_direct.py')")
    s=s.replace("frontend='LOCAL_BROWSER_SIM12'", "frontend='LOCAL_BROWSER_DIRECT_SIGN'")
    s=s.replace('reference=self.gesture.last.copy()', 'synthetic_input=self.gesture.last.copy()')
    (OUT/'software_runtime.py').write_text(s)
    s=(BASE/'browser_operator.py').read_text().replace('SIM12_NOMINAL_NO_C_FIRMWARE','DIRECT_SIGN_NOMINAL_NO_C_FIRMWARE')
    s=change(s,"goal=s.get('mapped_goal') or s.get('goal'),reference=s.get('goal'),", "goal=s.get('goal'),target_policy='DIRECT_SIGN_1TO1',")
    (OUT/'browser_operator.py').write_text(s)
    p=OUT/'web/app.js';s=p.read_text()
    s=s.replace('SIM12 · 1:1平移＋旋转','直接Sign · 1:1平移＋旋转')
    s=change(s,'const c=latest.car,g=latest.goal,r=latest.reference,trace=latest.trace||[];', 'const c=latest.car,g=latest.goal,trace=latest.trace||[];')
    s=change(s,'for(const p of [c,g,r])','for(const p of [c,g])')
    s=change(s,"[[g,'#d38d1f'],[r,'#008b8b']]", "[[g,'#d38d1f']]")
    s=change(s,'goal:null,reference:null','goal:null')
    p.write_text(s)
    p=OUT/'web/index.html';s=p.read_text().replace('SIM12 · 本机浏览器仿真','Sign 1:1直接跟随 · 浏览器仿真').replace('真实Sign → SIM12浏览器仿真','真实Sign → 1:1直接跟随')
    s=change(s,'蓝：软件小车　橙：目标　青：参考点（自动缩放，非电子围栏）','蓝：软件小车　橙：Sign的1:1位置与朝向（无中间参考点、无电子围栏）')
    p.write_text(s)
    # Reuse the already-qualified protocol peer, adding a changed-contract
    # assertion on every live view frame. This peer is not browser automation.
    peer=(ROOT/'candidates/wand_browser_sim12_20261030/test_protocol.cjs').read_text()
    peer=change(peer,"viewCount++;view.send(JSON.stringify({type:'ack',sequence:x.view_sequence}));",
        "viewCount++;assert(!Object.hasOwn(x,'reference'),'intermediate target leaked into view');\n"
        "          if(x.goal)assert.equal(x.target_policy,'DIRECT_SIGN_1TO1');\n"
        "          view.send(JSON.stringify({type:'ack',sequence:x.view_sequence}));")
    peer=change(peer,"assert.equal(result.reason,expected);",
        "assert(name==='occlusion'?(result.reason===expected||result.reason.startsWith('car or wand pose timeout:')):result.reason===expected,JSON.stringify(result));")
    (OUT/'test_protocol.cjs').write_text(peer)
    manifest=dict(software_only=True, hardware_entry=False, domain=191, controller_modified=True,
        scope='BROWSER_DIRECT_SIGN_1TO1', qualified=False,
        source_hashes={str(p):sha(p) for p in HERE.iterdir() if p.is_file()},
        artifact_hashes={str(p):sha(p) for p in OUT.rglob('*') if p.is_file()
            and p.name not in ('MANIFEST.json','RELEASE.json') and 'sessions' not in p.parts and '__pycache__' not in p.parts},
        preserved_baseline_manifest_sha256=sha(BASE/'MANIFEST.json'))
    save(OUT/'MANIFEST.json',manifest)
    print(OUT)


if __name__ == '__main__': generate()
