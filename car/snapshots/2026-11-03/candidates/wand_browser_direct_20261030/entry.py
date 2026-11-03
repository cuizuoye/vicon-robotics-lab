"""One command, browser-only direct Sign software follow. No physical IO."""
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
DOCS=ROOT/'docs/wand_browser_direct_2026-10-30'
READY=DOCS/'prepared_v1'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def check():
    # Validate old release by its own independent checker; never rewrite pins.
    path=ROOT/'candidates/wand_browser_sim12_20261030/entry.py'
    spec=importlib.util.spec_from_file_location('preserved_browser_release',path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.check()
    m=json.loads((READY/'MANIFEST.json').read_text())
    if not (m['qualified'] and m['controller_modified'] and m['software_only']
            and not m['hardware_entry'] and m['domain']==191 and m['scope']=='BROWSER_DIRECT_SIGN_1TO1'):
        raise ValueError('direct-Sign software qualification pending')
    for group in ('source_hashes','artifact_hashes'):
        for name,digest in m[group].items():
            if sha(Path(name))!=digest:raise ValueError('candidate changed: '+name)
    r=json.loads((READY/'RELEASE.json').read_text())
    if not r['software_qualified'] or r['hardware_qualified'] or r['manifest_sha256']!=sha(READY/'MANIFEST.json'):
        raise ValueError('invalid direct-Sign release evidence')
    for name,digest in r['evidence'].items():
        if sha(Path(name))!=digest:raise ValueError('evidence changed: '+name)
    node=shutil.which('node')
    if not node:raise ValueError('Node runtime not found')
    return dict(passed=True,node=node,software_only=True,hardware_output=False,intermediate_reference=False,
        mapping='direct 1:1 XY+yaw',max_session_s=600.,speed_mps=1.2,diagonal_mps=1.2/2**.5,accel_mps2=4.,
        browser_timeout_ms=100,pose_timeout_ms=200,command_timeout_ms=100)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--check',action='store_true');p.add_argument('--no-open',action='store_true')
    args=p.parse_args();verified=check()
    print('BROWSER_DIRECT_CHECK '+json.dumps(verified,ensure_ascii=False),flush=True)
    if args.check:return
    locks=[]
    for path in (ROOT/'docs/wand_browser_sim12_2026-10-30/RUN.lock',DOCS/'RUN.lock'):
        handle=path.open('a');locks.append(handle)
        try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('已有浏览器跟随入口运行；请先Esc停止并关闭旧入口，不会接管会话。')
    folder=Path(tempfile.mkdtemp(prefix='launch_',dir=DOCS))
    log=(folder/'server.log').open('w');errors=(folder/'server_error.log').open('w')
    child=subprocess.Popen([verified['node'],str(READY/'server.cjs'),'--real-sign'],stdout=log,stderr=errors)
    follow=(folder/'server.log').open();stopped=False;info=None;started=time.monotonic()
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
                    raise ValueError('refuse non-real-Sign software entry')
                (DOCS/'LATEST.json').write_text(json.dumps(dict(info,launcher=str(folder),hardware_output=False),ensure_ascii=False,indent=2)+'\n')
                if not args.no_open:webbrowser.open_new_tab(info['url'])
        if info is None:raise ValueError('Sign/ROS自检未通过，请查看 '+str(folder/'server_error.log'))
    finally:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=7)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
        follow.close();log.close();errors.close()
        for handle in locks:handle.close()


if __name__=='__main__':main()
