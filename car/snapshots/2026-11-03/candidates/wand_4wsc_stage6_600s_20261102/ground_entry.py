"""One command, stage6 long600 only; no automatic motor enable, flash or network changes."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import sys
import tempfile
import time
import webbrowser
from build import ROOT,HERE,BASE,OUT,sha,save

KIND='4WSC_STAGE6_LONG600_CONTROLLED_REVIEWED_V1'
CONSENT='4WSC_STAGE6_LONG600_MEASURED_REVIEW_CLEAR_POWER_CUT_READY'
RECORD=ROOT/'docs/wand_firmware_4wsc_2026-10-30/DEPLOYMENT_2026-11-02.json'


def current():
    p=OUT/'LATEST_PREPARED.json'
    return Path(json.loads(p.read_text())['prepared']) if p.exists() else OUT/'prepared_v1'


def check(ready,release=True):
    m=json.loads((ready/'HOST_MANIFEST.json').read_text())
    if (m.get('identity'),m.get('trial_kind'),m.get('stage'),m.get('max_trial_s'))!=('4WSC',KIND,6,600.):
        raise ValueError('exact independent stage6 long600 host required')
    for group in ('protected','source_hashes','artifact_hashes'):
        for path,digest in m[group].items():
            if sha(path)!=digest:raise ValueError('preserved source/artifact changed: '+path)
    if release:
        if m.get('host_offline_passed') is not True:raise ValueError('long600 offline qualification pending')
        for path,digest in m['evidence'].items():
            if sha(path)!=digest:raise ValueError('qualification evidence changed: '+path)
    return m


def free_port():
    for _ in range(100):
        p=12000+secrets.randbelow(8000)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            try:s.bind(('127.0.0.1',p));return p
            except OSError:pass
    raise ValueError('no dedicated loopback port')


def profile(ready,folder,software,site_ready=False):
    p=json.loads((ready/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
    if software:
        ports=[free_port(),free_port()]
        while ports[0]==ports[1]:ports[1]=free_port()
        p.update(software_rehearsal=True,namespace='/wand_4wsc_test_'+secrets.token_hex(5),base_yaw_offset_rad=0.)
        p['hardware'].update(esp_ip='127.0.0.1',network_interface='lo',command_port=ports[0],telemetry_port=ports[1])
        consent='LOOPBACK_ONLY_C_REHEARSAL'
    else:
        if site_ready is not True:raise ValueError('请先确认落地、接线和场地就绪且可立即断电；使用 --site-ready')
        p['commissioning_review']=dict(kind=KIND,issued_utc_s=time.time(),human_authorized_stage=6,max_active_s=600.,
            operator_declared_site_ready_now=True,unqualified_trial_acknowledged_now=True,flash_record=str(RECORD))
        review=OUT/'APPROVED_LONG_REVIEW.json'
        if review.exists():p['long_duration_review']=json.loads(review.read_text())
        consent=CONSENT
    path=folder/'PROFILE.json';save(path,p);return path,consent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ready',type=Path,default=current());p.add_argument('--check',action='store_true')
    mode=p.add_mutually_exclusive_group();mode.add_argument('--launch',action='store_true');mode.add_argument('--rehearsal',action='store_true')
    p.add_argument('--site-ready',action='store_true');p.add_argument('--seconds',type=float,default=600.)
    p.add_argument('--no-open',action='store_true');p.add_argument('--test-draft-rehearsal',action='store_true',help=argparse.SUPPRESS)
    a=p.parse_args();ready=a.ready.resolve()
    if a.test_draft_rehearsal and not a.rehearsal:raise ValueError('draft override can only run dedicated loopback')
    if not 1<=a.seconds<=600:raise ValueError('ground trial duration must be 1..600 seconds')
    m=check(ready,release=not a.test_draft_rehearsal)
    if not(a.launch or a.rehearsal):
        print(json.dumps(dict(passed=True,trial_kind=KIND,host_offline_passed=m['host_offline_passed'],
            stage=6,max_trial_s=600.,hardware_opened=False,motion_authorized=False,physical_qualified=False,requires_measured_long_review=True)));return
    if a.launch and not a.site_ready:raise ValueError('hardware launch requires the current --site-ready declaration')
    lock=(OUT/'RUN.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    folder=Path(tempfile.mkdtemp(prefix='launch_',dir=OUT))
    path,consent=profile(ready,folder,a.rehearsal,a.site_ready)
    env=dict(os.environ,WAND_4WSC_MODE='rehearsal' if a.rehearsal else 'hardware',WAND_4WSC_PROFILE=str(path),
        WAND_PROFILE_APPROVAL=sha(path),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign',
        PYTHONDONTWRITEBYTECODE='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k:v for k,v in env.items() if k.startswith('WAND_')})
    sys.path.insert(0,str(ready));import bootstrap
    rt=bootstrap.runtime()  # all admission checks, before hardware or ROS
    from receiver import Receiver
    receiver=Receiver();child=None;tail=None;done=False
    def end(*_):
        nonlocal done
        done=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,end)
    try:
        if a.launch:
            from idle_probe import run,CONSENT as ZERO_CONSENT
            idle=run(rt.profile,RECORD,folder/'DISABLED_ZERO',ZERO_CONSENT,8.)
            if not idle['passed']:raise ValueError('失能握手未就绪，不启动ROS/网页')
            receiver.start(folder)
        args=['node',str(ready/'server.cjs'),'--rehearsal' if a.rehearsal else '--hardware','--seconds',str(a.seconds)]
        if a.rehearsal:args+=['--synthetic']
        with (folder/'server.log').open('x') as log,(folder/'server_error.log').open('x') as err:
            child=subprocess.Popen(args,env=env,stdout=log,stderr=err,cwd=ready)
            tail=(folder/'server.log').open();start=time.monotonic();opened=False
            while not done and time.monotonic()-start<840:
                line=tail.readline()
                if line:
                    print(line,end='',flush=True)
                    if line.startswith('BROWSER_READY '):
                        v=json.loads(line.split(' ',1)[1]);opened=True
                        if v['hardware_entry']!=a.launch:raise ValueError('browser/hardware mode mismatch')
                        save(OUT/'LATEST.json',dict(v,folder=str(folder),stage=6,max_trial_s=a.seconds,physical_qualified=False))
                        if not a.no_open:webbrowser.open_new_tab(v['url'])
                else:
                    if child.poll() is not None:break
                    time.sleep(.03)
            if not opened:raise ValueError('自检未通过，未使能；日志 '+str(folder/'server_error.log'))
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=7)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
        if tail is not None:tail.close()
        receiver.close();lock.close()


if __name__=='__main__':
    try:main()
    except (ValueError,OSError,KeyError) as e:raise SystemExit('BLOCKED: '+str(e))
