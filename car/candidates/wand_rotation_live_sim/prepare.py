#!/usr/bin/env python3
"""Build read-only real Sign SIM evidence; never initializes ROS or hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from engine import ROOT
from bootstrap import BASELINE, RUNTIME

HERE=Path(__file__).resolve().parent


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value): path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def sources():
    paths=[p for p in HERE.iterdir() if p.is_file() and p.suffix in ('.py','.md','.json')]
    paths.append(ROOT/'tools/start_sign_rotation_sim.sh')
    return {str(p):sha(p) for p in paths}


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve(); out.mkdir(parents=True,exist_ok=False); out.chmod(0o700)
    prior=ROOT/'docs/wand_rotation_sim_2026-10-26/offline_ready/manifest.json'
    m=json.loads(prior.read_text()); protected={str(prior):sha(prior)}
    for group in ('source_hashes','protected_hashes','output_hashes'): protected.update(m[group])
    for path,digest in protected.items():
        if sha(path)!=digest: raise ValueError('upstream drift: '+path)
    before=sources()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',
        PYTHONPATH=str(HERE)+os.pathsep+str(ROOT/'candidates/wand_rotation_sim')+
            os.pathsep+str(ROOT/'candidates/wand_rotation_preview')+os.pathsep+os.environ.get('PYTHONPATH',''))
    def run(label,args,environment,timeout=90):
        p=subprocess.run(args,cwd=ROOT,env=environment,capture_output=True,text=True,timeout=timeout)
        (out/(label+'.log')).write_text(p.stdout+p.stderr)
        print(label+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode: raise ValueError((p.stdout+p.stderr)[-14000:])
        return p.stdout.strip().splitlines()[-1]
    # Use a newly allocated PRIVATE nested display; never inject test input on
    # the user's desktop or change keyboard/autorepeat configuration.
    with tempfile.TemporaryDirectory(prefix='rotation_sim_x_') as task_dir:
        display_file=Path(task_dir)/'display'
        with display_file.open('w+') as display, (out/'private_x_server.log').open('w') as server_log:
            server=subprocess.Popen(['Xephyr','-displayfd',str(display.fileno()),'-screen','1280x720',
                '-nolisten','tcp','-ac'],env=dict(os.environ,DISPLAY=os.environ.get('DISPLAY',':1'),
                    XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority')),
                pass_fds=(display.fileno(),),stdout=server_log,stderr=server_log)
            try:
                deadline=time.monotonic()+5
                while not display_file.read_text().strip():
                    if server.poll() is not None or time.monotonic()>deadline:
                        raise ValueError('private X server unavailable')
                    time.sleep(.02)
                private=':'+display_file.read_text().strip()
                env.update(DISPLAY=private,ROTATION_SIM_PRIVATE_TEST_DISPLAY=private)
                tests=run('new_and_rotation_reference_tests',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
                    str(HERE/'test_live_sim.py'),str(ROOT/'candidates/wand_rotation_sim/test_rotation_sim.py'),
                    str(ROOT/'candidates/wand_rotation_preview/test_preview.py'),
                    '--junitxml='+str(out/'new_tests.xml')],env)
            finally:
                server.terminate()
                try: server.wait(timeout=3)
                except subprocess.TimeoutExpired: server.kill(); server.wait(timeout=3)
    old_env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',STAGED_GENERATED=str(RUNTIME),
        ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
        EVENTIO_OUTPUT=str(ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'),
        AUTOPREP_OUTPUT=str(BASELINE),PYTHONPATH=str(BASELINE)+os.pathsep+
            str(ROOT/'stm32/mecanum_f407/candidates/servo_4ws8')+os.pathsep+str(ROOT/'src/mecanum_vicon_control')+
            os.pathsep+os.environ.get('PYTHONPATH',''))
    old_tests=run('accepted_runtime_regression',[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
        str(BASELINE/'tests'),'--junitxml='+str(out/'accepted_tests.xml')],old_env)
    for path,digest in protected.items():
        if sha(path)!=digest: raise ValueError('protected file changed: '+path)
    if sources()!=before: raise ValueError('candidate source changed during tests')
    write(out/'SUMMARY.json',dict(passed=True,tests=tests,accepted_runtime_tests=old_tests,
        protected_files=len(protected),actuator_outputs=False,hardware_changed=False,
        source_binding='immutable same raw frame, digest+stamp, independent local gate reconstruction',
        live_acceptance_pending=True,multi_process_wire_protocol_qualified=False))
    outputs={str(p):sha(p) for p in out.iterdir() if p.is_file()}
    write(out/'manifest.json',dict(offline_passed=True,actuator_outputs=False,source_hashes=before,
        protected_hashes=protected,output_hashes=outputs))
    print(json.dumps(dict(passed=True,tests=tests,old_tests=old_tests,protected_files=len(protected),
        output=str(out)),ensure_ascii=False),flush=True)


if __name__=='__main__': main()
