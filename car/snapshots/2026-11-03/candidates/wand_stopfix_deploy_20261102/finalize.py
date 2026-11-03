"""Qualify the independent migrated host from offline evidence only; no physical IO."""
import json
import hashlib
import os
from pathlib import Path
import subprocess
import sys
from build import HERE,OUT,hashes,save,sha
from ground_entry import check
from deployment_record import RECORD_SHA,IMAGE_SHA

def main(index_path):
    ready=OUT/'host';m=check(ready,release=False)
    if m['host_offline_passed']:raise ValueError('never overwrite a qualified snapshot')
    index=json.loads(index_path.read_text());assert index['complete'] and not index['physical_output'] and Path(index['prepared'])==ready
    evidence={str(index_path):sha(index_path)}
    for row in index['records']:
        assert row['exit_code']==0,row
        for key in ('stdout','stderr'):evidence[row[key]]=sha(row[key])
    admission=next(x for x in index['records'] if x['name']=='admission')
    unit=json.loads(Path(admission['stdout']).read_text().splitlines()[-1])
    assert unit['passed'] and unit['retained_tests']==47 and unit['new_tests']==18 and unit['total_tests']==65
    assert not unit['hardware_opened']
    focus=json.loads(Path(index['focus_client']).read_text());assert focus['passed'] and focus['checks']==24 and not focus['hardware_output']
    zero=json.loads(Path(index['disabled_zero']).read_text());assert zero['passed'] and zero['actual_native_C'] and not zero['physical_output']
    assert zero['never_selected'] and zero['never_permitted'] and not zero['enable_sent'] and not zero['clear_fault_sent'] and zero['fault_bits']==0
    assert len(index['native'])==6 and len(index['browser'])==11
    sessions=[]
    for path in index['native']:
        r=json.loads(Path(path).read_text());assert r['passed'] and not r['physical_output'] and r['stop']['gate_output_zero']
        assert r['stop']['stop_confirmed'] or r['case']=='feedback'
        if r['case']!='normal':assert r['native_revocation_seen'] and not r['resumed_after_revocation']
        sessions.append(Path(path).parent)
    normal=[]
    for path in index['browser']:
        r=json.loads(Path(path).read_text());assert r['passed'] and not r['physical_output'] and r['stop_confirmed']
        raw=json.loads((Path(r['session'])/'result.json').read_text())
        assert raw['ros_domain']==196 and not raw['hardware_socket_opened'] and not raw['physical_motion_started']
        if r['case']=='normal':
            t=raw['stop_evidence']['transitions'];a=next(x['t'] for x in t if x['state']=='ACTIVE');b=next(x['t'] for x in t if x['state']=='STOPPING' and x['t']>=a)
            assert 59.7<=b-a<=60.5;normal.append(b-a)
        sessions.append(Path(r['session']))
    audit=json.loads(Path(index['audit']).read_text());assert audit['passed'] and audit['audit_cannot_renew_authority'] and audit['reason']=='BROWSER_PROOF_TIMEOUT' and not audit['physical_output']
    sessions.append(Path(audit['session']))
    for folder in [index_path.parent,*sessions]:
        for f in folder.rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    change=OUT/'DRAFT_ENTRY_MODE_CHANGE.json';revision=json.loads(change.read_text())
    entry=(HERE/'ground_entry.py').read_text()
    new="    mode=p.add_mutually_exclusive_group();mode.add_argument('--check',action='store_true');mode.add_argument('--launch',action='store_true');mode.add_argument('--rehearsal',action='store_true');mode.add_argument('--idle-only',action='store_true')"
    old="    p.add_argument('--check',action='store_true')\n    mode=p.add_mutually_exclusive_group();mode.add_argument('--launch',action='store_true');mode.add_argument('--rehearsal',action='store_true');mode.add_argument('--idle-only',action='store_true')"
    assert entry.count(new)==1 and sha(HERE/'ground_entry.py')==revision['after_sha256']
    assert hashlib.sha256(entry.replace(new,old).encode()).hexdigest()==revision['before_sha256']
    evidence[str(change)]=sha(change)
    # Only the parser's mutual-exclusion definition changed after the full
    # integration suite. Re-run admission and prove conflicts never reach IO.
    for name,script,expected in (('current_admission','test_admission.py',65),('entry_modes','test_entry_modes.py',6)):
        p=subprocess.run([sys.executable,'-B',str(HERE/script)],capture_output=True,text=True,
            env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'),timeout=30)
        for suffix,value in (('stdout',p.stdout),('stderr',p.stderr)):
            target=OUT/(name+'.'+suffix+'.log');target.write_text(value);evidence[str(target)]=sha(target)
        assert p.returncode==0,p.stdout+p.stderr
        r=json.loads(p.stdout.splitlines()[-1]);assert r['passed'] and not r['hardware_opened']
        assert r.get('total_tests',r.get('conflicting_mode_cases'))==expected
    m.update(host_offline_passed=True,hardware_launch_enabled=True,evidence=evidence,source_hashes=hashes(HERE))
    wrapper=HERE.parents[1]/'tools/start_car_stopfix.sh';m['source_hashes'][str(wrapper)]=sha(wrapper)
    # Temporarily qualified from complete regression evidence, then exercise
    # the real constructor with sockets/process creation forbidden. On any
    # failure, revoke the qualification bits; never bypass a production guard.
    save(ready/'HOST_MANIFEST.json',m)
    try:
        p=subprocess.run([sys.executable,'-B',str(HERE/'test_admission.py'),'--post-qualified'],
            capture_output=True,text=True,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'),timeout=30)
        for suffix,value in (('stdout',p.stdout),('stderr',p.stderr)):
            target=OUT/('post_qualified.'+suffix+'.log');target.write_text(value);evidence[str(target)]=sha(target)
        assert p.returncode==0,p.stdout+p.stderr
        post=json.loads(p.stdout.splitlines()[-1]);assert post['passed'] and post['real_qualified_Runtime'] and not post['hardware_opened']
        report=dict(passed=True,firmware_revision='STOPFIX_V1',image_sha256=IMAGE_SHA,deployment_record_sha256=RECORD_SHA,
            host_hardware_admission_prepared=True,real_qualified_constructor_file_only_passed=True,
            hardware_opened_this_step=False,firmware_flashed_this_step=False,real_car_started=False,network_changed=False,
            physical_qualified=False,live_handshake_verified=False,new_revision_physical_trial_passed=False,
            retained_policy_tests=47,new_admission_tests=18,unit_tests=65,conflicting_entry_mode_cases=6,focus_DOM_VM_checks=24,
            disabled_zero_probe_exact_C_passed=True,native_ROS_cases=6,browser_protocol_cases=11,
            normal_active_s=normal,audit_cannot_renew_authority=True,original_snapshots_preserved=True,
            limits=dict(speed_mps=.7,accel_mps2=2.,yaw_radps=2.,yaw_accel_radps2=12.,max_active_s=600.),
            inherited_600s_timer_evidence=m['inherited_600s_software_qualification'],
            scope='Offline migrated-host admission, DOM/VM, actual ROS/UDP/exact-C loopback only. No live MC-Link/Vicon use. Old-image observations are retained as history, not promoted to physical acceptance of the new revision. Next stage is an explicitly authorized disabled-zero-only live check.')
        target=OUT/'QUALIFICATION.json';save(target,report);evidence[str(target)]=sha(target)
        m['evidence']=evidence;m['qualification']=str(target);save(ready/'HOST_MANIFEST.json',m);check(ready)
        save(OUT/'LATEST_PREPARED.json',dict(prepared=str(ready),manifest_sha256=sha(ready/'HOST_MANIFEST.json'),qualification=str(target),live_handshake_verified=False))
        print(json.dumps(report,ensure_ascii=False))
    except BaseException:
        m.update(host_offline_passed=False,hardware_launch_enabled=False);save(ready/'HOST_MANIFEST.json',m);raise
if __name__=='__main__':main(Path(sys.argv[1]).resolve())
