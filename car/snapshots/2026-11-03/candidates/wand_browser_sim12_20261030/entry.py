"""One command: checked real Sign -> private software car -> local browser."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import webbrowser

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
DOCS=ROOT/'docs/wand_browser_sim12_2026-10-30'
PREPARED=DOCS/'prepared_v4'
OLD=ROOT/'docs/wand_browser_fast_2026-10-28/prepared'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def check():
    sys.path.insert(0,str(ROOT/'candidates/wand_sim12_20261030'))
    from entry12 import check as check_sim
    check_sim()
    spec=importlib.util.spec_from_file_location('checked_old_browser',OLD/'check.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.check(OLD)
    manifest=json.loads((PREPARED/'MANIFEST.json').read_text())
    if (manifest['hardware_entry'] is not False or not manifest['software_only'] or manifest['domain']!=191
            or manifest['controller_modified'] is not False):raise ValueError('invalid browser SIM12 scope')
    for group in ('source_hashes','artifact_hashes'):
        for name,digest in manifest[group].items():
            if sha(Path(name))!=digest:raise ValueError('candidate changed: '+name)
    release=json.loads((PREPARED/'RELEASE.json').read_text())
    if release.get('software_qualified') is not True or release.get('hardware_qualified') is not False:
        raise ValueError('software regressions pending')
    if release['manifest_sha256']!=sha(PREPARED/'MANIFEST.json'):raise ValueError('release manifest changed')
    for name,digest in release['evidence'].items():
        if sha(Path(name))!=digest:raise ValueError('release evidence changed')
    node=shutil.which('node')
    if not node:raise ValueError('Node runtime not found')
    return dict(passed=True,node=node,mode='real_sign_software_only',hardware_entry=False,
        max_session_s=600,mapping='1:1 XY+yaw',straight_mps=1.2,diagonal_mps=1.2/2**.5,accel_mps2=4.,
        ui_proof_ms=100,operator_lease_ms=150,hardware_output=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true');parser.add_argument('--no-open',action='store_true')
    args=parser.parse_args();verified=check()
    print('BROWSER_SIM12_CHECK '+json.dumps(verified,ensure_ascii=False),flush=True)
    if args.check:return
    ownership=(DOCS/'RUN.lock').open('a')
    try:fcntl.flock(ownership,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        ownership.close()
        raise SystemExit('已有浏览器SIM12入口运行；请先Esc结束并关闭旧入口，再重开。不会接管旧会话。')
    launch_dir=Path(tempfile.mkdtemp(prefix='launch_',dir=DOCS))
    log=(launch_dir/'server.log').open('w');errors=(launch_dir/'server_error.log').open('w')
    child=subprocess.Popen([verified['node'],str(PREPARED/'server.cjs'),'--real-sign'],stdout=log,stderr=errors)
    follow=(launch_dir/'server.log').open();stopped=False;info=None;started=time.monotonic()
    def stop(*_):
        nonlocal stopped
        stopped=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    try:
        while not stopped and time.monotonic()-started<840.:
            line=follow.readline()
            if not line:
                if child.poll() is not None:break
                time.sleep(.05);continue
            print(line,end='',flush=True)
            if line.startswith('BROWSER_READY '):
                info=json.loads(line.split(' ',1)[1])
                if info['mode']!='software' or info.get('synthetic') is not False or info['hardware_entry'] is not False:
                    raise RuntimeError('refuse non-real-Sign software entry')
                (DOCS/'LATEST.json').write_text(json.dumps(dict(info,launcher=str(launch_dir),hardware_output=False),ensure_ascii=False,indent=2)+'\n')
                if not args.no_open:webbrowser.open_new_tab(info['url'])
        if info is None:
            raise RuntimeError('Sign/ROS自检未通过；请查看 '+str(launch_dir/'server_error.log'))
    finally:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=7)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
        follow.close();log.close();errors.close();ownership.close()


if __name__=='__main__':main()
