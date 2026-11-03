"""File-only offline release. This never approves a physical result or long demo."""
import argparse
import json
from pathlib import Path
from build import HERE,OUT,hashes,save,sha
from ground_entry import check


def release(index_path):
    index=json.loads(index_path.read_text());ready=Path(index['prepared']);m=check(ready,release=False)
    if m['host_offline_passed']:raise ValueError('never rewrite a released snapshot')
    assert index['complete'] and not index['physical_output'] and all(r['exit_code']==0 for r in index['records'])
    unit=next(r for r in index['records'] if r['name']=='unit')
    assert 'Ran 46 tests' in Path(unit['stderr']).read_text() and Path(unit['stderr']).read_text().rstrip().endswith('OK')
    assert len(index['native'])==6 and len(index['browser'])==11
    evidence={str(index_path.resolve()):sha(index_path)};observations=[]
    for row in index['records']:
        for k in ('stdout','stderr'):evidence[row[k]]=sha(row[k])
    for path in index['native']:
        p=Path(path);r=json.loads(p.read_text());assert r['passed'] and not r['physical_output']
        assert r['stop']['gate_output_zero'] and (r['stop']['stop_confirmed'] or r['case']=='feedback')
        if r['case']!='normal':assert r['native_revocation_seen'] and not r['resumed_after_revocation']
        for f in p.parent.rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
        measurements=json.loads((p.parent/'MEASUREMENTS.json').read_text())
        assert not measurements['physical_trial'] and not measurements['physical_stop_envelope_verified'] and not measurements['qualified_long_demo']
    for path in index['browser']:
        p=Path(path);r=json.loads(p.read_text());assert r['passed'] and not r['physical_output'] and r['stop_confirmed']
        session=Path(r['session']);raw=json.loads((session/'result.json').read_text())
        assert raw['ros_domain']==196 and not raw['hardware_socket_opened'] and not raw['physical_motion_started']
        if r['case']=='normal':
            t=raw['stop_evidence']['transitions'];start=next(v['t'] for v in t if v['state']=='ACTIVE')
            end=next(v['t'] for v in t if v['state']=='STOPPING' and v['t']>=start)
            assert 7.7<=end-start<=8.5;observations.append(dict(normal_active_s=end-start,reason=r['reason']))
        for folder in (p.parent,session):
            for f in folder.rglob('*'):
                if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    report=dict(passed=True,prepared=str(ready),unit_tests=46,inherited_600s_software_qualification=m['inherited_600s_software_qualification'],native_chain_cases=6,browser_protocol_cases=11,
        normal_run=observations,physical_qualified=False,physical_motion_started=False,
        physical_measurements_verified=False,stage=6,max_active_s=600.,
        base_controller_reused=True,firmware_changed=False,network_changed=False,
        qualification='Unqualified operator-requested 600s controlled trial. Frozen motion/ROS/gate/timer reuse plus ready-handshake state fix of parent full 600s software pass; local admission and short protocol/fault regression passed. Unknown physical measurements remain unknown; formal measured route unchanged.')
    target=index_path.parent/'QUALIFICATION.json';save(target,report);evidence[str(target.resolve())]=sha(target)
    m.update(host_offline_passed=True,evidence=evidence,source_hashes=hashes(HERE),qualification=str(target.resolve()))
    save(ready/'HOST_MANIFEST.json',m);check(ready)
    save(OUT/'LATEST_PREPARED.json',dict(prepared=str(ready),manifest_sha256=sha(ready/'HOST_MANIFEST.json'),
        physical_qualified=False,qualification=str(target.resolve())))
    print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--index',type=Path,required=True);release(p.parse_args().index.resolve())
