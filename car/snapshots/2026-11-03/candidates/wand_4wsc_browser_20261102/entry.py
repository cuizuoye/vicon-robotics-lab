"""File check by default. Real socket/browser launch needs an explicit reviewed profile."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import webbrowser
from prepare import ROOT,DOCS,OUT,sha

FLASH_RECORD=ROOT/'docs/wand_firmware_4wsc_2026-10-30/DEPLOYMENT_2026-11-02.json'

def default_ready():
    pointer=DOCS/'LATEST_PREPARED.json'
    return Path(json.loads(pointer.read_text())['prepared']) if pointer.exists() else OUT

def check(ready,require_release=True):
    m=json.loads((ready/'HOST_MANIFEST.json').read_text())
    if m['identity']!='4WSC' or m['authority_version']!=8 or m['diagnostic_payload_bytes']!=208:raise ValueError('wrong host identity')
    for group in ('protected','source_hashes','artifact_hashes'):
        for p,digest in m[group].items():
            if sha(p)!=digest:raise ValueError('source/artifact drift: '+p)
    if require_release:
        if not m['host_offline_passed']:raise ValueError('4WSC host offline qualification pending')
        for p,digest in m['evidence'].items():
            if sha(p)!=digest:raise ValueError('evidence changed: '+p)
    return m

def free_port():
    # Keep test endpoints outside Linux ephemeral and DDS domain192 ports.
    # A free ephemeral port can be taken by our OWN DDS initialization before
    # the transport child binds it. This is test scaffolding, not network tuning.
    for _ in range(100):
        port=12000+secrets.randbelow(8000)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            try:s.bind(('127.0.0.1',port));return port
            except OSError:pass
    raise ValueError('no dedicated loopback test port available')

def setup_rehearsal(stage,folder,purpose='sign'):
    from profile_draft import draft
    p=draft(stage,purpose);ports=[free_port(),free_port()]
    while ports[0]==ports[1] or any(x in (46000,46001) for x in ports):ports=[free_port(),free_port()]
    p.update(software_rehearsal=True,namespace='/wand_4wsc_test_'+secrets.token_hex(5),base_yaw_offset_rad=0.)
    p['hardware'].update(esp_ip='127.0.0.1',network_interface='lo',command_port=ports[0],telemetry_port=ports[1])
    path=folder/'rehearsal_profile.json';path.write_text(json.dumps(p,indent=2)+'\n')
    return path,'LOOPBACK_ONLY_C_REHEARSAL'

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ready',type=Path,default=default_ready());ap.add_argument('--check',action='store_true')
    mode=ap.add_mutually_exclusive_group();mode.add_argument('--launch',action='store_true');mode.add_argument('--rehearsal',action='store_true');mode.add_argument('--idle-probe',action='store_true')
    ap.add_argument('--profile',type=Path);ap.add_argument('--profile-sha',default='');ap.add_argument('--consent',default='')
    ap.add_argument('--flash-record',type=Path,default=FLASH_RECORD)
    ap.add_argument('--stage',type=int,choices=range(10),default=0);ap.add_argument('--purpose',choices=('sign','suspended'),default='sign')
    ap.add_argument('--seconds',type=float);ap.add_argument('--no-open',action='store_true')
    ap.add_argument('--test-draft-rehearsal',action='store_true',help=argparse.SUPPRESS)
    a=ap.parse_args();a.ready=a.ready.resolve()
    if a.test_draft_rehearsal and not a.rehearsal:raise ValueError('test override is loopback-only')
    m=check(a.ready,require_release=not a.test_draft_rehearsal)
    if not (a.launch or a.rehearsal or a.idle_probe):
        print(json.dumps(dict(passed=True,identity='4WSC',host_offline_passed=m['host_offline_passed'],
            hardware_opened=False,motion_authorized=False,default='FILE_CHECK_ONLY',physical_qualified=False),ensure_ascii=False));return
    sys.path.insert(0,str(a.ready));import bootstrap
    if a.idle_probe:
        if a.profile is None or sha(a.profile)!=a.profile_sha:raise ValueError('explicit endpoint profile hash required before socket construction')
        from idle_probe import run
        folder=DOCS/('idle_probe_'+time.strftime('%Y%m%d_%H%M%S'))
        print(json.dumps(run(json.loads(a.profile.read_text()),a.flash_record,folder,a.consent,8. if a.seconds is None else a.seconds),ensure_ascii=False));return
    if a.seconds is None:a.seconds=600.
    if not 1<=a.seconds<=600:raise ValueError('1..600s limit')
    lock=(DOCS/'RUN.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Same physical port lock is additionally acquired by the sole transport.
    folder=Path(tempfile.mkdtemp(prefix='launch_',dir=DOCS))
    if a.rehearsal:profile,consent=setup_rehearsal(a.stage,folder,a.purpose)
    else:
        if a.profile is None or sha(a.profile)!=a.profile_sha:raise ValueError('exact approved real profile required before ROS/socket/browser construction')
        profile,consent=a.profile.resolve(),a.consent
    env=dict(os.environ,WAND_4WSC_MODE='rehearsal' if a.rehearsal else 'hardware',
        WAND_4WSC_PROFILE=str(profile),WAND_PROFILE_APPROVAL=sha(profile),WAND_HARDWARE_CONSENT=consent,
        WAND_4WSC_PURPOSE=a.purpose,PYTHONDONTWRITEBYTECODE='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k:v for k,v in env.items() if k.startswith('WAND_')})
    rt=bootstrap.runtime()
    if rt.config.stage!=a.stage or rt.purpose!=a.purpose:raise ValueError('CLI stage/purpose must match the reviewed profile')
    seconds=min(a.seconds,5.) if a.purpose=='suspended' else a.seconds
    node=shutil.which('node')
    if node is None:raise ValueError('Node runtime not found')
    args=[node,str(a.ready/'server.cjs'),'--rehearsal' if a.rehearsal else '--hardware','--seconds',str(seconds)]
    if a.rehearsal:args+=['--synthetic']
    log=(folder/'server.log').open('x');error=(folder/'server_error.log').open('x');tail=None;child=None;done=False
    def stop(*_):
        nonlocal done
        done=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    try:
        child=subprocess.Popen(args,stdout=log,stderr=error,env=env,cwd=a.ready)
        tail=(folder/'server.log').open();started=time.monotonic();opened=False
        while not done and time.monotonic()-started<840:
            line=tail.readline()
            if line:
                print(line,end='',flush=True)
                if line.startswith('BROWSER_READY '):
                    v=json.loads(line.split(' ',1)[1]);opened=True
                    if v['mode']!=rt.mode or v['hardware_entry']!=a.launch:raise ValueError('browser/runtime mode mismatch')
                    (DOCS/'LATEST.json').write_text(json.dumps(dict(v,launch_folder=str(folder),stage=a.stage,purpose=a.purpose),ensure_ascii=False,indent=2)+'\n')
                    if not a.no_open:webbrowser.open_new_tab(v['url'])
            else:
                if child.poll() is not None:break
                time.sleep(.03)
        if not opened:raise ValueError('4WSC自检未通过；请查看 '+str(folder/'server_error.log'))
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=7)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
        if tail is not None:tail.close()
        log.close();error.close();lock.close()

if __name__=='__main__':
    try:main()
    except (ValueError,OSError,KeyError) as e:raise SystemExit('BLOCKED: '+str(e))
