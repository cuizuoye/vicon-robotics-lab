"""Independent STOPFIX entry: check, disabled-zero probe, or explicitly approved browser."""
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
from build import ROOT,HERE,BASE,OUT,sha,save,AUTH,KIND,CONSENT,RECORD

def current():return OUT/'host'

def check(ready,release=True):
    if Path(ready).resolve()!=current():raise ValueError('exact STOPFIX deployed snapshot required')
    m=json.loads((ready/'HOST_MANIFEST.json').read_text())
    if (m.get('identity'),m.get('firmware_revision'),m.get('trial_kind'),m.get('stage'),m.get('max_trial_s'))!=('4WSC','STOPFIX_V1',KIND,6,600.):
        raise ValueError('exact STOPFIX stage6/600s host required')
    for group in ('protected','source_hashes','artifact_hashes'):
        for p,digest in m[group].items():
            if sha(p)!=digest:raise ValueError('preserved source/artifact changed: '+p)
    if release:
        if not m.get('host_offline_passed') or not m.get('hardware_launch_enabled'):raise ValueError('STOPFIX deployed host qualification pending')
        if not m.get('evidence'):raise ValueError('qualification evidence required')
        for p,digest in m['evidence'].items():
            if sha(p)!=digest:raise ValueError('qualification evidence changed: '+p)
    from deployment_record import verify_flash_record
    verify_flash_record(RECORD)
    return m

def free_port():
    for _ in range(100):
        value=12000+secrets.randbelow(8000)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            try:s.bind(('127.0.0.1',value));return value
            except OSError:pass
    raise ValueError('no dedicated loopback port')

def profile(ready,folder,software,site_ready=False,operator_trial=False):
    p=json.loads((ready/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
    if software:
        ports=[free_port(),free_port()]
        while ports[0]==ports[1]:ports[1]=free_port()
        p.update(software_rehearsal=True,namespace='/wand_4wsc_test_'+secrets.token_hex(5),base_yaw_offset_rad=0.)
        p['hardware'].update(esp_ip='127.0.0.1',network_interface='lo',command_port=ports[0],telemetry_port=ports[1])
        consent='LOOPBACK_ONLY_C_REHEARSAL'
    else:
        if site_ready is not True:raise ValueError('fresh site-ready declaration required; old site confirmations do not authorize motion')
        p['commissioning_review']=dict(kind=KIND,issued_utc_s=time.time(),human_authorized_stage=6,max_active_s=600.,
            operator_declared_site_ready_now=True,unqualified_trial_acknowledged_now=True,flash_record=str(RECORD))
        if operator_trial:
            p['operator_duration_trial']=dict(kind='EXPLICIT_UNQUALIFIED_600S_OPERATOR_REQUEST_V1',
                authorization_path=str(AUTH),authorization_sha256=sha(AUTH),maximum_active_s=600.,stage=6,
                measurements_still_unknown=True,no_physical_qualification_claim=True,onsite_power_cut_operator_present=True)
        consent=CONSENT
    path=folder/'PROFILE.json';save(path,p);return path,consent

def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group();mode.add_argument('--check',action='store_true');mode.add_argument('--launch',action='store_true');mode.add_argument('--rehearsal',action='store_true');mode.add_argument('--idle-only',action='store_true')
    p.add_argument('--confirm-disabled-zero-now',action='store_true');p.add_argument('--site-ready',action='store_true');p.add_argument('--operator-trial',action='store_true')
    p.add_argument('--seconds',type=float,default=600.);p.add_argument('--no-open',action='store_true')
    p.add_argument('--test-draft-rehearsal',action='store_true',help=argparse.SUPPRESS)
    a=p.parse_args();ready=current()
    if a.test_draft_rehearsal and not a.rehearsal:raise ValueError('draft override is loopback rehearsal only')
    if not 1<=a.seconds<=600:raise ValueError('trial duration must be 1..600 seconds')
    m=check(ready,release=not a.test_draft_rehearsal)
    if not (a.launch or a.rehearsal or a.idle_only):
        print(json.dumps(dict(passed=True,firmware_revision='STOPFIX_V1',host_offline_passed=m['host_offline_passed'],
            deployment_record=str(RECORD),deployment_record_verified=True,stage=6,max_trial_s=600.,
            hardware_opened=False,motion_authorized=False,physical_qualified=False,live_handshake_verified=False)));return
    if a.idle_only and not a.confirm_disabled_zero_now:raise ValueError('explicit current disabled-zero-only confirmation required')
    if a.launch and (not a.site_ready or not a.operator_trial):raise ValueError('browser hardware launch requires fresh --site-ready and --operator-trial')
    if a.operator_trial and not a.launch:raise ValueError('operator trial declaration is hardware-browser only')
    lock=(OUT/'RUN.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    folder=Path(tempfile.mkdtemp(prefix='idle_' if a.idle_only else 'launch_',dir=OUT))
    sys.path.insert(0,str(ready));import bootstrap
    if a.idle_only:
        from idle_probe import run,CONSENT as ZERO_CONSENT
        try:
            draft=json.loads((ready/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
            result=run(draft,RECORD,folder/'DISABLED_ZERO',ZERO_CONSENT,8.)
            save(OUT/'LATEST_IDLE.json',dict(folder=str(folder),result=result));print(json.dumps(result,ensure_ascii=False))
            if not result['passed']:raise ValueError('disabled handshake not ready; do not clear/retry/enable automatically')
        finally:lock.close()
        return
    path,consent=profile(ready,folder,a.rehearsal,a.site_ready,a.operator_trial)
    env=dict(os.environ,WAND_4WSC_MODE='rehearsal' if a.rehearsal else 'hardware',WAND_4WSC_PROFILE=str(path),
        WAND_PROFILE_APPROVAL=sha(path),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign',
        PYTHONDONTWRITEBYTECODE='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.update({k:v for k,v in env.items() if k.startswith('WAND_')})
    rt=bootstrap.runtime()
    from receiver import Receiver
    receiver=Receiver();child=tail=None;done=False
    def end(*_):
        nonlocal done
        done=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,end)
    try:
        if a.launch:
            from idle_probe import run,CONSENT as ZERO_CONSENT
            idle=run(rt.profile,RECORD,folder/'DISABLED_ZERO',ZERO_CONSENT,8.)
            if not idle['passed']:raise ValueError('disabled zero/clock/fault check failed; browser remains closed')
            receiver.start(folder)
        args=['node',str(ready/'server.cjs'),'--rehearsal' if a.rehearsal else '--hardware','--seconds',str(a.seconds)]
        if a.rehearsal:args+=['--synthetic']
        with (folder/'server.log').open('x') as log,(folder/'server_error.log').open('x') as error:
            child=subprocess.Popen(args,env=env,stdout=log,stderr=error,cwd=ready)
            tail=(folder/'server.log').open();started=time.monotonic();opened=False
            while not done and time.monotonic()-started<840:
                line=tail.readline()
                if line:
                    print(line,end='',flush=True)
                    if line.startswith('BROWSER_READY '):
                        value=json.loads(line.split(' ',1)[1]);opened=True
                        if value['hardware_entry']!=a.launch:raise ValueError('browser/hardware mismatch')
                        save(OUT/'LATEST.json',dict(value,folder=str(folder),physical_qualified=False))
                        if not a.no_open:webbrowser.open_new_tab(value['url'])
                elif child.poll() is not None:break
                else:time.sleep(.03)
            if not opened:raise ValueError('preflight failed; browser not opened. Logs: '+str(folder))
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
