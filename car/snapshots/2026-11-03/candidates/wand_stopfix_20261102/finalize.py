"""Single offline-only qualification/pinning transaction. Never deployment authority."""
import argparse
import json
from pathlib import Path
from build import HERE,OUT,TARGET,hashes,sha,save
from ground_entry import check
from verify_build import main as verify_build

def finalize(index_path,late_path,audit_path):
    ready=OUT/'host';m=check(ready,release=False)
    if m['host_offline_passed']:raise ValueError('never overwrite a qualified snapshot')
    verify_build()
    index=json.loads(index_path.read_text());late=json.loads(late_path.read_text())
    assert Path(index['prepared']).resolve()==ready and index['complete'] and not index['physical_output']
    assert all(v['exit_code']==0 for v in index['records'])
    unit=next(v for v in index['records'] if v['name']=='unit')
    unit_text=Path(unit['stderr']).read_text();assert 'Ran 51 tests' in unit_text and unit_text.rstrip().endswith('OK')
    focus=next(v for v in index['records'] if v['name']=='focus_client')
    ui=json.loads((Path(focus['stdout']).parent/'focus_client/RESULT.json').read_text())
    assert ui['passed'] and not ui['hardware_output'] and ui['checks']>=24
    assert len(index['native'])==6 and len(index['browser'])==11
    evidence={str(index_path):sha(index_path),str(late_path):sha(late_path),str(audit_path):sha(audit_path)}
    for row in index['records']:
        for key in ('stdout','stderr'):evidence[row[key]]=sha(row[key])
    for path in index['native']:
        r=json.loads(Path(path).read_text())
        assert r['passed'] and not r['physical_output'] and r['stop']['gate_output_zero']
        assert r['stop']['stop_confirmed'] or r['case']=='feedback'
        if r['case']!='normal':assert r['native_revocation_seen'] and not r['resumed_after_revocation']
        for f in Path(path).parent.rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    normal=[]
    for path in index['browser']:
        r=json.loads(Path(path).read_text());assert r['passed'] and not r['physical_output'] and r['stop_confirmed']
        raw=json.loads((Path(r['session'])/'result.json').read_text())
        assert not raw['hardware_socket_opened'] and not raw['physical_motion_started'] and raw['ros_domain']==196
        if r['case']=='normal':
            t=raw['stop_evidence']['transitions'];start=next(x['t'] for x in t if x['state']=='ACTIVE')
            end=next(x['t'] for x in t if x['state']=='STOPPING' and x['t']>=start)
            assert 59.7<=end-start<=60.5;normal.append(end-start)
        for folder in (Path(path).parent,Path(r['session'])):
            for f in folder.rglob('*'):
                if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    assert late['passed'] and not late['physical_output'] and late['new_native_C']
    assert late['late_stop_confirmed'] and late['reopen_same_C_without_reset'] and len(late['results'])==3
    assert late['persistent_delayed_stop_stays_revoked']
    for path in late['results']:
        r=json.loads(Path(path).read_text());assert r['passed'] and not r['physical_output']
        assert r['native_fault_bits']==0 and r['stop']['gate_output_zero']
        if r['persistent_invalid_stop_stream']:
            assert r['observed_confirmation_unavailable']==(not r['stop']['stop_confirmed']) and r['no_automatic_motion_resume']
            assert r['stop']['state'] in ('PAUSED','STOPPING','STOP_UNCONFIRMED')
            assert not r['stop']['stop_confirmed'] or r['stop']['state']=='PAUSED'
        else:assert r['stop']['stop_confirmed']
        if r['late_stop_injected']:assert r['delayed_frames']>0 and r['max_actual_proxy_delay_s']>=.12
        for f in Path(path).parent.rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    audit=json.loads(audit_path.read_text())
    assert audit['passed'] and not audit['physical_output'] and audit['audit_cannot_renew_authority']
    assert audit['reason']=='BROWSER_PROOF_TIMEOUT' and audit['focus_audit']['rows']>0 and audit['focus_audit']['diagnostic_only']
    for folder in (audit_path.parent.parent,Path(audit['session'])):
        for f in folder.rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    motion=json.loads((OUT/'MOTION_EQUIVALENCE.json').read_text())
    assert motion['passed'] and motion['unchanged_stages']==list(range(10)) and motion['compared_control_ticks']==15000
    assert motion['excluded_fields']==[] and not motion['physical_motion']
    native=(OUT/'native_tests.log').read_text();assert '100% tests passed, 0 tests failed out of 7' in native
    assert 'PASS: 149 STOPFIX' in (OUT/'native/Testing/Temporary/LastTest.log').read_text()
    artifact=json.loads((OUT/'ARTIFACT_VERIFICATION.json').read_text());assert artifact['passed'] and artifact['hex_bin_equal'] and not artifact['flashed']
    # Pin the NEW oracle, ARM image, source, build evidence and focus evidence.
    # Additional diagnostic/test source files may be added before qualification;
    # none of the already-pinned generator/runtime/test sources may change.
    for path,digest in m['source_hashes'].items():assert sha(path)==digest
    extras={**hashes(OUT/'firmware'),**hashes(OUT/'flash_bundle')}
    for f in (OUT/'native/libstage0_oracle.so',OUT/'BUILD.json'):
        extras[str(f)]=sha(f)
    for name in ('native_configure.log','native_build.log','native_tests.log','native/Testing/Temporary/LastTest.log',
                 'arm_configure.log','arm_build.log','arm_sizes.log','hex_export.log','MOTION_EQUIVALENCE.json','ARTIFACT_VERIFICATION.json'):
        f=OUT/name;evidence[str(f)]=sha(f)
    for f in (Path(focus['stdout']).parent/'focus_client').rglob('*'):
        if f.is_file():evidence[str(f)]=sha(f)
    report=dict(passed=True,firmware_revision='STOPFIX_V1',flashed=False,hardware_output=False,hardware_launch_enabled=False,
        host=str(ready),unit_tests=51,focus_client_checks=ui['checks'],native_C_groups=7,new_stop_assertion_groups=149,
        old_motion_ticks_compared=15000,native_ROS_cases=6,browser_protocol_cases=11,normal_active_s=normal,
        late_stop_140ms_burst_reopen_and_persistent_delay_cases=3,audit_cannot_renew_proof_verified=True,physical_qualified=False,field_focus_cause_identified=False,
        field_stop_subcondition_identified=False,full_new_600s_physical_trial_completed=False,
        inherited_600s_timer_evidence=m['inherited_600s_software_qualification'],
        no_safety_timeout_relaxed=True,original_entries_unchanged=True,network_changed=False,
        image=str(OUT/'flash_bundle'/(TARGET+'.bin')),image_sha256=artifact['image_sha256'],
        caveat='Offline DOM/VM, ROS/UDP/native-C and ARM build verification only. Not actual browser/OS focus or real hardware validation. Same wire identity4WSC/v8, independent patch image and hash; original hardware entries unchanged and this candidate rejects hardware launch.')
    target=OUT/'QUALIFICATION.json';save(target,report);evidence[str(target)]=sha(target)
    m.update(host_offline_passed=True,hardware_launch_enabled=False,artifact_hashes={**m['artifact_hashes'],**extras},
        source_hashes=hashes(HERE),evidence=evidence,qualification=str(target))
    save(ready/'HOST_MANIFEST.json',m);check(ready)
    save(OUT/'LATEST_PREPARED.json',dict(prepared=str(ready),manifest_sha256=sha(ready/'HOST_MANIFEST.json'),
        flashed=False,hardware_launch_enabled=False,qualification=str(target)))
    print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--index',required=True,type=Path);p.add_argument('--late-result',required=True,type=Path);p.add_argument('--audit-result',required=True,type=Path)
    a=p.parse_args();finalize(a.index.resolve(),a.late_result.resolve(),a.audit_result.resolve())
