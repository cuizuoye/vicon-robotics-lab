#!/usr/bin/env python3
"""Copy pinned arena sources to a NEW private runtime, never launch/flash.

This is a mechanical generator. Existing rectangle, 4W20, previous console and
install trees are inputs only. All candidate hardware modes/entrypoints are
disabled, even if someone supplies an old site-confirmed profile.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
BASE = WORKSPACE/'docs/wand_one_to_one_3x3_2026-10-23/verified_runtime/src'
OLD_CONTROL = 'mecanum_wand_one_to_one_control'
OLD_HARDWARE = 'mecanum_wand_one_to_one_hardware'
CONTROL = 'mecanum_wand_servo_control'
HARDWARE = 'mecanum_wand_servo_hardware'


def hashes(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}


def edit(path, old, new, count=1):
    value = path.read_text()
    if value.count(old) != count:
        raise RuntimeError('source drift: '+str(path)+' '+old)
    path.write_text(value.replace(old, new))


def generate(output):
    if output.exists():
        raise FileExistsError('fresh output required: '+str(output))
    before = hashes(BASE)
    output.mkdir(parents=True)
    for old,new in ((OLD_CONTROL, CONTROL), (OLD_HARDWARE, HARDWARE)):
        shutil.copytree(BASE/old, output/new, ignore=shutil.ignore_patterns('__pycache__'))
    pairs = ((OLD_CONTROL, CONTROL), (OLD_HARDWARE, HARDWARE),
             ('WAND_ONE_TO_ONE_', 'WAND_SERVO_'),
             ('wand_one_to_one_acceptance', 'wand_servo_offline'),
             ("97, '/wand_servo_offline'", "112, '/wand_servo_offline'"),
             ('domain97', 'domain112'), ('Domain97', 'Domain112'))
    for path in sorted(output.rglob('*')):
        if path.is_file():
            text = path.read_text()
            for old,new in pairs:
                text = text.replace(old,new)
            path.write_text(text)
    for path in sorted(output.rglob('*'), key=lambda p: len(p.parts), reverse=True):
        name = path.name
        for old,new in pairs:
            name = name.replace(old,new)
        if name != path.name:
            path.rename(path.with_name(name))
    control = output/CONTROL/CONTROL
    hardware = output/HARDWARE/HARDWARE
    for name in ('continuous_servo.py', 'servo_config.py', 'offline_validation.py'):
        (control/name).write_text((HERE/name).read_text())
    edit(control/'core.py',
         '            tx, ty = limit_vector(\n'
         '                feedforward[0]+self.config.kp_position * bx,\n'
         '                feedforward[1]+self.config.kp_position * by,\n'
         '                self.config.max_speed_mps)',
         '            if hasattr(self, "translation_target"):\n'
         '                tx, ty = self.translation_target(bx, by, feedforward)\n'
         '            else:\n'
         '                tx, ty = limit_vector(\n'
         '                    feedforward[0]+self.config.kp_position * bx,\n'
         '                    feedforward[1]+self.config.kp_position * by,\n'
         '                    self.config.max_speed_mps)')
    edit(control/'safety_core.py', 'from .core import Config, Pose, Velocity, WandSession',
         'from .core import Config, Pose, Velocity\nfrom .continuous_servo import ContinuousSession as WandSession')
    edit(control/'safety_core.py', "self.follower.observe(source, value['pose'], value['received'])",
         "self.follower.observe(source, value['pose'], value['received'], source_stamp=value['source_ros'])", 2)
    edit(hardware/'commissioning.py',
         'from mecanum_wand_servo_control.arena_config import ArenaSafetyConfig as SafetyConfig',
         'from mecanum_wand_servo_control.servo_config import ServoSafetyConfig as SafetyConfig\n'
         'from mecanum_wand_servo_control.continuous_servo import SERVO_POLICY')
    edit(hardware/'commissioning.py',
         "        if mode not in ('shadow', 'emulator', 'hardware'): raise ValueError('invalid acceptance mode')",
         "        if mode != 'shadow': raise ValueError('OFFLINE_ONLY: hardware/emulator transport is disabled')")
    edit(hardware/'commissioning.py', "        self.source_domain = 93 if mode == 'emulator' else 0",
         "        self.source_domain = 113  # synthetic-only candidate input domain, never live Vicon")
    edit(hardware/'commissioning.py', "        hw = self.profile.get('hardware', {})",
         "        if self.profile.get('servo_candidate_revision') != 'continuous_xy_offline_v1':\n"
         "            raise ValueError('explicit offline servo profile required')\n"
         "        if self.profile.get('offline_only') is not True:\n"
         "            raise ValueError('OFFLINE_ONLY profile required')\n"
         "        if self.profile.get('servo_policy') != SERVO_POLICY:\n"
         "            raise ValueError('offline servo policy mismatch')\n"
         "        hw = self.profile.get('hardware', {})")
    config = output/HARDWARE/'config'
    # All inherited profiles/planning artifacts are removed only in this new
    # copy, avoiding ambiguous launch/profile selection in the candidate.
    template = config/'one_to_one.3x3_BUFFER050.SITE_CONFIRMATION_REQUIRED.json'
    profile = json.loads(template.read_text())
    for path in config.iterdir():
        if path.is_file():
            path.unlink()
    profile.update(offline_only=True, servo_candidate_revision='continuous_xy_offline_v1')
    policy_ast=next(n.value for n in ast.parse((HERE/'continuous_servo.py').read_text()).body
                    if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='SERVO_POLICY' for t in n.targets))
    profile['servo_policy']=ast.literal_eval(policy_ast)
    profile['evidence']['change_scope'] = 'OFFLINE continuous XY servo; no position low-pass; one command acceleration limiter'
    target = config/'servo.3x3_1to1.OFFLINE_ONLY.json'
    target.write_text(json.dumps(profile, ensure_ascii=False, indent=2)+'\n')
    # Do not ship a launchable actuator bridge or inherited real Sign entry.
    for package in (CONTROL, HARDWARE):
        setup = output/package/'setup.py'
        text = setup.read_text()
        start = text.index("    entry_points=")
        entries = ([f'wand_servo_offline = {CONTROL}.offline_validation:main']
                   if package == CONTROL else [])
        setup.write_text(text[:start]+f"    entry_points={{'console_scripts': {entries!r}}},\n)\n")
        for path in (output/package/'launch').glob('*.py'):
            path.write_text("def generate_launch_description():\n    raise RuntimeError('OFFLINE_ONLY: use deterministic validation, no ROS or real Sign launch')\n")
        (output/package/'README.md').write_text(
            '# OFFLINE ONLY continuous XY servo\n\n'
            'Domain112; synthetic source113;1:1;fixed3x3m;0.50m buffer;0.20m/s;0.08m/s2.\n'
            'No firmware, hardware/real Sign launch, transport or automatic motion.\n')
    after = hashes(BASE)
    if before != after:
        raise RuntimeError('accepted source changed during copy')
    # Verify that only the follower wiring changed: independent gate must be
    # byte-identical to the arena baseline (it lives in the same source file).
    baseline = (BASE/OLD_CONTROL/OLD_CONTROL/'safety_core.py').read_text()
    candidate = (control/'safety_core.py').read_text()
    gate = lambda source: ast.get_source_segment(source, next(n for n in ast.parse(source).body
                                                             if isinstance(n, ast.ClassDef) and n.name=='FinalGate'))
    if gate(baseline) != gate(candidate):
        raise RuntimeError('independent FinalGate changed')
    report = dict(offline_only=True, domain=112, source_domain=113,
        control_package=CONTROL, hardware_package=HARDWARE,
        existing_source_sha256=before, independent_final_gate_unchanged=True,
        hardware_started=False, firmware_changed=False,
        profile=str(target.resolve()), profile_sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    (output/'generation_report.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    report = generate(args.output.resolve())
    print(json.dumps({k:v for k,v in report.items() if k!='existing_source_sha256'}, indent=2))
