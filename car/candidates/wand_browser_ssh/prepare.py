#!/usr/bin/env python3
"""Isolated browser transport; copy the reviewed controller byte for byte."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_remote_startup_v2_2026-10-27/prepared'
DEFAULT=ROOT/'docs/wand_browser_ssh_2026-10-27/prepared'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
        and not any(v in ('__pycache__','.pytest_cache') or v.endswith('.egg-info') for v in p.parts)}
def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')

def generate(out):
    spec=importlib.util.spec_from_file_location('browser_parent_helpers',ROOT/'candidates/wand_tracking_autostart/prepare.py')
    helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)
    m=helpers.check(OLD/'build_manifest.json')
    if m.get('remote_startup_v2_passed') is not True:raise ValueError('Verified parent required')
    protected={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):
        protected.update(m[group])
    # The accepted human-facing entries are also explicitly pinned.
    for p in (ROOT/'tools/start_car_tracking_auto.sh',Path('/home/PRIMARY_USER/.local/bin/小车全场演示')):
        protected[str(p)]=sha(p)
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    runtime=out/'runtime/src'
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    for name in ('channel.py','browser_operator.py','software_runtime.py','worker.py','server.cjs','check.py'):
        shutil.copyfile(HERE/name,out/name)
    shutil.copytree(HERE/'web',out/'web')
    inventory=[]
    for p in runtime.rglob('*'):
        if p.is_file() and sha(p)!=sha(Path(m['runtime_src'])/p.relative_to(runtime)):inventory.append(str(p.relative_to(runtime)))
    if inventory:raise ValueError('Controller/firmware changed: '+str(inventory))
    sources=hashes(HERE);sources[str(ROOT/'tools/start_car_tracking_browser.sh')]=sha(ROOT/'tools/start_car_tracking_browser.sh')
    artifacts={str(out/n):sha(out/n) for n in ('channel.py','browser_operator.py','software_runtime.py','worker.py','server.cjs','check.py')}
    artifacts.update(hashes(out/'web'));artifacts.update(hashes(runtime))
    write(out/'build_manifest.json',dict(schema=1,parent=str(OLD),parent_manifest_sha=sha(OLD/'build_manifest.json'),
        protected_hashes=protected,source_hashes=sources,artifact_hashes=artifacts,runtime_src=str(runtime),
        controller_changes=inventory,default_mode='readonly',allowed_modes=['readonly','software'],
        hardware_entry=False,browser_ssh_passed=False,physical_acceptance=False,
        software_domain=187,proof_limit_ms=100,operator_lease_ms=150,
        pose_limit_ms=200,mcu_feedback_limit_ms=300,max_active_s=120,
        dependencies={'ws_package':'/usr/share/nodejs/ws/package.json','ws_sha256':sha('/usr/share/nodejs/ws/package.json')}))
    return out

def refresh_draft(out):
    """Regenerate only our unsealed artifacts; retain diagnostic attempts."""
    m=json.loads((out/'build_manifest.json').read_text())
    if m.get('browser_ssh_passed'):raise ValueError('Sealed candidate cannot be refreshed')
    for name in ('channel.py','browser_operator.py','software_runtime.py','worker.py','server.cjs','check.py'):
        shutil.copyfile(HERE/name,out/name)
    for p in (HERE/'web').iterdir():shutil.copyfile(p,out/'web'/p.name)
    m['source_hashes']=hashes(HERE)
    tool=ROOT/'tools/start_car_tracking_browser.sh';m['source_hashes'][str(tool)]=sha(tool)
    for p in list(m['artifact_hashes']):m['artifact_hashes'][p]=sha(p)
    write(out/'build_manifest.json',m)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=DEFAULT)
    ap.add_argument('--refresh-draft',action='store_true');a=ap.parse_args();out=a.output.resolve()
    if a.refresh_draft:refresh_draft(out);print(out)
    else:print(generate(out))
