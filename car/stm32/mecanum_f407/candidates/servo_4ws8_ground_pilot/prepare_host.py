#!/usr/bin/env python3
"""Build a private, finite ground-pilot host; never touch firmware or hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OLD = ROOT/'docs/wand_servo_4ws8_2026-10-23/ready'
BASE = ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
EVIDENCE = ROOT/'docs/wand_servo_4ws8_2026-10-23/requested_suspended_050_070/host_ready/stage4_suspended_l79694pa/bench/SESSION_RESULT.json'
sys.path.insert(0, str(BASE))
from launch_cli import check


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
            and not any(v in ('__pycache__','.pytest_cache') or v.endswith('.egg-info') for v in p.parts)}


def edit(path, old, new):
    text = path.read_text()
    if text.count(old) != 1: raise ValueError('input drift: '+str(path)+' '+old)
    path.write_text(text.replace(old, new))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    out = parser.parse_args().output.resolve()
    m = check(OLD/'build_manifest.json')
    out.mkdir(parents=True, exist_ok=False)
    (out/'COLCON_IGNORE').write_text('')
    runtime = out/'runtime/src'
    runtime.parent.mkdir()
    shutil.copytree(m['runtime_src'], runtime, ignore=shutil.ignore_patterns('__pycache__','*.egg-info','.pytest_cache'))
    os.symlink(OLD/'firmware', out/'firmware', target_is_directory=True)
    control = runtime/m['control_package']/m['control_package']
    hw = runtime/m['hardware_package']/m['hardware_package']
    shutil.copyfile(HERE/'pilot_core.py', control/'pilot_core.py')
    shutil.copyfile(HERE/'pilot_authorization.py', hw/'pilot_authorization.py')
    shutil.copyfile(HERE/'observer.py', hw/'pilot_observer.py')
    edit(runtime/m['hardware_package']/'setup.py',
         "        'wand_acceptance_source = mecanum_wand_4ws8_hardware.nodes:source_main',",
         "        'wand_pilot_observer = mecanum_wand_4ws8_hardware.pilot_observer:main',\n        'wand_acceptance_source = mecanum_wand_4ws8_hardware.nodes:source_main',")
    edit(runtime/m['hardware_package']/'launch/wand_4ws8_staged.launch.py',
         "    if gui == 'true': nodes += [node('wand_acceptance_console', 'wand_real_operator')]",
         "    if gui == 'true': nodes += [node('wand_acceptance_console', 'wand_real_operator')]\n    nodes += [node('wand_pilot_observer', 'wand_pilot_observer')]")
    edit(control/'safety_nodes.py',
         'from .safety_core import FinalGate, Frame, Lease, SafetyConfig, Supervisor, ZERO',
         'from .safety_core import Frame, Lease, SafetyConfig, ZERO\nfrom .pilot_core import PilotFinalGate as FinalGate, PilotSupervisor as Supervisor')
    edit(hw/'authorization.py', '    get_stage(stage)\n',
         "    get_stage(stage)\n    if isinstance(review, dict) and review.get('kind') == 'FINITE_FIRST_GROUND_OBSERVATION_NOT_QUALIFICATION':\n        from .pilot_authorization import verify_pilot_review\n        return verify_pilot_review(review,stage,purpose,firmware_sha)\n")
    edit(hw/'commissioning.py', "DOMAIN, NAMESPACE, WORLD = 119, '/wand_4ws8_staged', 'map'",
         "DOMAIN, NAMESPACE, WORLD = 120, '/wand_4ws8_ground_pilot', 'map'")
    edit(hw/'commissioning.py', "CONSENT = '4WS8_STAGED_SIGN_CLEAR_POWER_CUT_READY'",
         "CONSENT = '4WS8_FINITE_GROUND_PILOT_CLEAR_POWER_CUT_READY'")
    edit(hw/'commissioning.py', '        self.mode = mode\n',
         "        self.mode = mode\n        candidate = json.loads(Path(profile).read_text())\n        from .pilot_authorization import KIND\n        if candidate.get('trial_review', {}).get('kind') != KIND or candidate.get('stage') != 4:\n            raise ValueError('this private runtime accepts only the finite stage4 pilot')\n")
    edit(hw/'stages.py', "SCOPE='HARDWARE_4WS8_V4_STAGE_'", "SCOPE='FINITE_GROUND_PILOT_4WS8_V4_STAGE_'")
    edit(control/'operator_console.py', "f'域119 · 固定档{self.config.stage} · 1:1 · ≤{self.config.max_speed_mps:.2f}m/s · '",
         "f'域120 · 单次≤2秒 / 杆≤15cm / 车≤20cm · 1:1 · ≤{self.config.max_speed_mps:.2f}m/s · '")
    edit(control/'operator_console.py', "'hardware': f'真实 Sign → 4WS8档{self.config.stage}",
         "'hardware': f'单次落地小位移 · 最长2秒 → 4WS8档{self.config.stage}")
    # Baseline algorithms and original normal review remain protected on disk.
    pins = dict(m['protected_input_hashes'])
    for group in ('source_hashes','runtime_hashes','generated_firmware_hashes'): pins.update(m[group])
    for p in (OLD/'build_manifest.json', EVIDENCE, EVIDENCE.parent/'raw_link.jsonl', EVIDENCE.parent/'events.json'):
        pins[str(p)] = sha(p)
    pins[m['firmware']['path']] = m['firmware']['sha256']
    manifest = dict(m)
    manifest.update(preparation_passed=False, runtime_src=str(runtime), runtime_hashes=hashes(runtime),
                    source_hashes={**m['source_hashes'], **hashes(HERE)}, protected_input_hashes=pins,
                    draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
                    host_change='Distinct finite first-ground observation: independent 2s, 15cm hand/goal, 20cm car, one engagement; not qualification',
                    firmware_reused_unchanged=True, firmware_flashed_this_preparation=False)
    env = os.environ.copy()
    env.update(STAGED_GENERATED=str(runtime), STAGED_NATIVE=str(OLD/'native'),
               PYTHONPATH=str(BASE)+os.pathsep+env.get('PYTHONPATH',''))
    # Original suite uses the original Runtime contract. Run it against the
    # unchanged base first; private-wrapper tests then exercise this release.
    original_env = dict(env, STAGED_GENERATED=m['runtime_src'])
    for name, args, test_env in (
        ('base', [str(BASE/'test_staged.py'),str(OLD/'tests')], original_env),
        ('pilot', [str(HERE/'test_pilot.py')], env)):
        r = subprocess.run([sys.executable,'-m','pytest','-q',*args,'--junitxml='+str(out/(name+'_tests.xml'))],
                           cwd=ROOT,env=test_env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
        (out/(name+'_tests.log')).write_text(r.stdout)
        print(r.stdout,flush=True)
        if r.returncode: raise RuntimeError(name+' regression failed; hardware not opened')
    for p,v in pins.items():
        if sha(p) != v: raise ValueError('protected source/evidence drift: '+p)
    manifest['preparation_passed'] = True
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    check(out/'build_manifest.json')
    # Build only two packages in a dedicated prefix. This never installs over
    # the existing rectangle or accepted 4WS8 workspace.
    r = subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
                        '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),
                        '--symlink-install','--packages-select',m['control_package'],m['hardware_package']],
                       cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
    (out/'colcon_build.log').write_text(r.stdout)
    print(r.stdout,flush=True)
    if r.returncode: raise RuntimeError('private install failed; no hardware opened')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,firmware_reused=True,
        flashed=False,hardware_started=False,ground_qualified=False,protected_files=len(pins)),indent=2)+'\n')


if __name__ == '__main__': main()
