#!/usr/bin/env python3
"""Generate a separate startup-clock fix; never start ROS, UDP or hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OLD = ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle/ready'
SOURCE = ROOT/'candidates/wand_demo_120s_toggle'
BASE = ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0, str(BASE))
from launch_cli import check


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def hashes(d):
    return {str(p): sha(p) for p in d.rglob('*') if p.is_file() and not any(
        k in ('__pycache__', '.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}


def edit(path, old, new):
    text = path.read_text()
    if text.count(old) != 1:
        raise ValueError('protected source fragment drift: '+str(path)+' '+old[:90])
    path.write_text(text.replace(old, new))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    out = parser.parse_args().output.resolve()
    m = check(OLD/'build_manifest.json')
    input_hashes = hashes(HERE)
    evidence = {str(p): sha(p) for p in (
        OLD/'oneclick_session_53vp_i3d/mcu_receive.jsonl',
        ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/native/libstage0_oracle.so')}
    out.mkdir(parents=True, exist_ok=False)
    (out/'COLCON_IGNORE').write_text('')
    runtime = out/'runtime/src'
    shutil.copytree(m['runtime_src'], runtime,
        ignore=shutil.ignore_patterns('__pycache__', '*.egg-info', '.pytest_cache'))
    c = runtime/m['control_package']/m['control_package']
    h = runtime/m['hardware_package']/m['hardware_package']
    for name in ('startup_clock.py', 'startup_bridge.py'):
        shutil.copyfile(HERE/name, h/name)
    nodes = h/'nodes.py'
    text = nodes.read_text()
    start, end = text.index('class ProgressSocket:'), text.index('\n\nclass RawSource')
    text = text[:start]+'from .startup_clock import StartupProgressSocket as ProgressSocket\nfrom .startup_bridge import StartupBridgeMixin\n'+text[end:]
    nodes.write_text(text)
    edit(nodes, 'class HardwareBridge(CmdVelUdpBridge):', 'class HardwareBridge(StartupBridgeMixin, CmdVelUdpBridge):')
    # Keep the original implementations reviewable; dispatch only these three
    # methods through the startup-specific mixin. No global monkey-patching.
    for name in ('send_hello_if_due', 'feedback_is_fresh', 'safety_tick', 'tick'):
        # tick also exists on SignSource: restrict to HardwareBridge's text.
        text = nodes.read_text(); offset = text.index('class HardwareBridge(')
        head, tail = text[:offset], text[offset:]
        old = '    def '+name+'('
        if tail.count(old) != 1: raise ValueError('hardware method drift: '+name)
        nodes.write_text(head+tail.replace(old, '    def _legacy_'+name+'(', 1))
    edit(nodes, '        return super().tick()', '        return CmdVelUdpBridge.tick(self)')
    edit(nodes, "            fresh=fresh, local_stop=self.local_stop_latched, fault=self.policy.fault,",
        "            fresh=fresh, local_stop=self.local_stop_latched, fault=self.policy.fault,\n"
        "            startup_clock=self.sock.clock.snapshot(now),\n"
        "            startup_hello_attempts=self.sock.hello_attempts,")
    novice = c/'novice_ui.py'
    edit(novice, "and not t.get('local_stop') and not t.get('fault')",
        "and t.get('startup_clock',{}).get('ready') is True\n        and not t.get('local_stop') and not t.get('fault')")
    edit(novice, "    health=s.get('input_health','')\n    if health:return HEALTH.get(health,'准备被阻止：'+health)",
        "    health=s.get('input_health','')\n"
        "    if hasattr(model,'transport_status') and health in ('','FEEDBACK_TIMEOUT'):\n"
        "        t=model.transport_status; clock=t.get('startup_clock',{})\n"
        "        if 0<=now-model.transport_at<=.25:\n"
        "            if t.get('local_stop') or t.get('fault') or clock.get('fault'):\n"
        "                return '启动通信保护已阻止准备；不自动清故障。详情：'+str(t.get('fault') or clock.get('fault') or '通信锁止')\n"
        "            if clock and not clock.get('ready'):\n"
        "                return '等待启动握手/连续零输出遥测建基准，尚未就绪：'+str(clock.get('phase'))\n"
        "    if health:return HEALTH.get(health,'准备被阻止：'+health)")
    app = out/'oneclick_app.py'
    shutil.copyfile(SOURCE/'oneclick_app.py', app)
    edit(app, 'HERE=Path(__file__).resolve().parent', 'HERE=Path('+repr(str(SOURCE))+')')
    edit(app, "READY=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle/ready'", 'READY=Path('+repr(str(out))+')')
    edit(app, "root.title('小车单击 · 120秒独立演示')", "root.title('小车单击 · 启动时钟修复候选 · 120秒')")
    tests = out/'tests'
    shutil.copytree(OLD/'tests', tests, ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    shutil.copyfile(HERE/'test_startup_clock.py', tests/'test_startup_clock.py')
    edit(tests/'test_oneclick.py', 'm.transport_status=dict(fresh=True,local_stop=False,',
        "m.transport_status=dict(startup_clock=dict(ready=True),fresh=True,local_stop=False,")
    edit(tests/'test_toggle.py', "('nodes.py','continuous_bridge.py','stage0_contract.py','stage0_session.py')",
        "('continuous_bridge.py','stage0_contract.py','stage0_session.py')")
    edit(tests/'test_demo_120s.py', "('nodes.py','stage0_contract.py','stage0_session.py','continuous_bridge.py','startup_idle.py')",
        "('stage0_contract.py','stage0_session.py','continuous_bridge.py','startup_idle.py')")
    edit(tests/'test_pose_quality.py', "('nodes.py','stage0_contract.py','stage0_session.py')",
        "('stage0_contract.py','stage0_session.py')")
    env = dict(os.environ, STAGED_GENERATED=str(runtime), ONECLICK_SOURCE=str(SOURCE),
        CLOCKFIX_OUTPUT=str(out), PYTHONPATH=str(BASE)+os.pathsep+os.environ.get('PYTHONPATH',''),
        DISPLAY=os.environ.get('DISPLAY',':1'), XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    result = dict(passed=False, hardware_started=False, flashed=False, real_udp_sent=False,
                  real_ros_or_vicon_started=False, old_entries_promoted_or_modified=False)
    try:
        r = subprocess.run([sys.executable, '-m', 'pytest', '-q', str(tests),
            '--junitxml='+str(out/'host_tests.xml')], cwd=ROOT, env=env,
            capture_output=True, text=True, timeout=60)
        log = r.stdout+r.stderr
        (out/'host_tests.log').write_text(log); print(log, flush=True)
        if r.returncode: raise ValueError('offline regression failed')
        if hashes(HERE) != input_hashes or any(sha(p) != value for p,value in evidence.items()):
            raise ValueError('candidate or offline evidence changed during testing')
        pins = dict(m['protected_input_hashes'])
        for group in ('runtime_hashes','source_hashes','generated_firmware_hashes'):
            pins.update(m[group])
        pins[str(OLD/'build_manifest.json')] = sha(OLD/'build_manifest.json')
        pins.update(evidence)
        source_hashes = dict(m['source_hashes'], **hashes(HERE))
        source_hashes[str(app)] = sha(app)
        launcher = ROOT/'tools/start_car_demo_120s_toggle_clockfix.sh'
        source_hashes[str(launcher)] = sha(launcher)
        manifest = dict(m, runtime_src=str(runtime), runtime_hashes=hashes(runtime),
            source_hashes=source_hashes, protected_input_hashes=pins,
            draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
            host_change='ACK-bound conservative clock; bounded qualified handshake; same-session startup/readiness proof; no motion/source-time rewrite/firmware change.')
        draft_manifest = out/'BUILD_MANIFEST.INCOMPLETE.json'
        draft_manifest.write_text(json.dumps(manifest, indent=2)+'\n')
        check(draft_manifest)
        r = subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
            '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),
            '--symlink-install','--packages-select',m['control_package'],m['hardware_package']],
            cwd=ROOT, env=env, capture_output=True, text=True, timeout=60)
        (out/'colcon_build.log').write_text(r.stdout+r.stderr); print(r.stdout+r.stderr, flush=True)
        if r.returncode: raise ValueError('isolated packages build failed')
        draft_manifest.rename(out/'build_manifest.json')
        check(out/'build_manifest.json'); check(OLD/'build_manifest.json')
        result.update(passed=True, tests=log.strip().splitlines()[-1], protected_files_unchanged=len(pins),
            operator_input_mode='SPACE_TOGGLE_120S_V1', max_active_s=120., firmware_unchanged=True,
            hardware_acceptance_passed=False, hello_round_trip_cap_ms=40)
    finally:
        (out/'PREPARATION_RESULT.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__': main()
