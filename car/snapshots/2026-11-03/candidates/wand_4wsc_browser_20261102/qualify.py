"""File-only qualification of an exact host snapshot and explicit offline evidence.

Does not authorize a physical trial or create an approved runtime profile.
Never discovers a device, initializes ROS, or opens a network socket.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from entry import check
from prepare import HERE, DOCS, hashes, save, sha


def read(path):
    return json.loads(path.read_text())


def qualify(a):
    ready=a.ready.resolve()
    manifest=check(ready,require_release=False)
    if manifest['host_offline_passed']:
        raise ValueError('do not overwrite a qualified host')
    output=a.output.resolve()
    if output.exists():
        raise ValueError('new qualification folder required')
    output.mkdir(parents=True)
    command=[sys.executable,'-B','-m','unittest','discover','-s',str(HERE),'-p','test_host.py','-v']
    unit=subprocess.run(command,env=dict(os.environ,WAND_4WSC_TEST_READY=str(ready)),
        text=True,capture_output=True,timeout=45)
    (output/'unit_stdout.log').write_text(unit.stdout)
    (output/'unit_stderr.log').write_text(unit.stderr)
    assert unit.returncode==0 and 'Ran 15 tests' in unit.stderr and unit.stderr.rstrip().endswith('OK'),unit.stderr
    results=[]
    for path in a.trial:
        path=path.resolve();r=read(path)
        assert r['passed'] and r['active_ever'] and r['physical_output'] is False
        assert r['actual_exclusive_bridge_used'] and r['exact_C_peer_used'] and r['ros_initialized']
        assert r['stop']['gate_output_zero']
        if r['case']=='feedback':
            assert r['native_revocation_seen'] and not r['resumed_after_revocation']
            assert r['stop']['stop_confirmed'] or (r['feedback_loss_stop_confirmation_unavailable_expected'] and r['stop']['state']=='STOP_UNCONFIRMED')
        else:
            assert r['stop']['stop_confirmed']
        if r['case']!='normal':
            assert r['native_revocation_seen'] and not r['resumed_after_revocation']
        contract=read(path.parent/'CONTRACT.json')
        assert contract['identity']=='4WSC' and contract['mode']=='rehearsal'
        assert contract['direct_target'] and not contract['intermediate_reference'] and not contract['fixed_fence']
        assert not contract['car01_feedback_used'] and contract['source_clock_not_renewed']
        results.append(dict(path=str(path),result=r))
    zero=[r for r in results if r['result']['case']=='normal' and r['result']['stage']==0]
    assert len(zero)==2 and all(r['result']['active_elapsed_s']>=6 for r in zero)
    high=[r for r in results if r['result']['case']=='normal' and r['result']['stage']==9]
    assert len(high)==1 and high[0]['result']['active_elapsed_s']>=8
    for case in ('occlusion','feedback','loop_gap'):
        assert len([r for r in results if r['result']['case']==case])==1
    probe=read(a.probe);assert probe['passed'] and probe['exact_C_never_permitted']
    for path,early in ((a.suspended,False),(a.early_release,True)):
        r=read(path)
        assert r['passed'] and r['release_early']==early and r['active_ever']
        assert not r['physical_output'] and r['stop']['stop_confirmed'] and r['stop']['state']=='COMPLETE'
    browser=read(a.browser)
    assert browser['passed'] and browser['complete'] and len(browser['records'])==9
    for r in browser['records']:
        assert r['passed'] and r['stop_confirmed'] and r['actual_ROS_UDP_C'] and not r['physical_output']
    normal=next(r for r in browser['records'] if r['case']=='normal')
    real_normal=read(Path(normal['session'])/'result.json')
    transitions=real_normal['stop_evidence']['transitions']
    active_at=next(r['t'] for r in transitions if r['state']=='ACTIVE')
    stopped_at=next(r['t'] for r in transitions if r['state']=='STOPPING' and r['t']>=active_at)
    browser_active_seconds=stopped_at-active_at
    assert real_normal['completed'] and real_normal['passed'] and browser_active_seconds>=60
    ui_note=read(a.ui_observation)
    assert ui_note['prepared']==str(ready) and ui_note['actual_browser_UI_observed'] is True
    assert ui_note['ready_seen'] and ui_note['active_after_single_space_seen'] and ui_note['STOPPED_CONFIRMED_after_Esc_seen']
    assert ui_note['correct_short_countdown_seen'] and ui_note['physical_output'] is False
    ui_session=Path(ui_note['session']);ui_result=read(ui_session/'result.json')
    assert ui_result['reason']=='CLIENT_ESC' and ui_result['authority_ever_acquired']
    assert ui_result['mode']=='rehearsal' and ui_result['ros_domain']==192 and not ui_result['physical_motion_started']
    assert ui_result['stop_evidence']['stop_confirmed'] and ui_result['stop_evidence']['gate_output_zero']
    assert ui_result['last_state']['remaining_s']<=ui_note['requested_timer_s']
    roots={p.resolve().parent for p in a.trial+[a.probe,a.suspended,a.early_release,a.browser]}
    roots.update(Path(r['session']) for r in browser['records'])
    roots.add(ui_session)
    evidence={}
    for folder in sorted(roots):
        for path in folder.rglob('*'):
            if path.is_file() and path.suffix in ('.json','.jsonl','.log'):
                evidence[str(path)]=sha(path)
    evidence[str(a.ui_observation.resolve())]=sha(a.ui_observation)
    evidence[ui_note['screenshot']]=sha(ui_note['screenshot'])
    # The suite index/command logs tie the accepted results to the exact ready
    # directory. Keep these alongside per-session native/ROS traces.
    if a.suite_index is not None:
        index=read(a.suite_index)
        assert index['complete'] and index['prepared']==str(ready) and all(r['exit_code']==0 for r in index['records'])
        evidence[str(a.suite_index.resolve())]=sha(a.suite_index)
        for row in index['records']:
            for kind in ('stdout','stderr'):evidence[row[kind]]=sha(row[kind])
    for path in output.iterdir():
        if path.is_file():evidence[str(path)]=sha(path)
    report=dict(identity='4WSC',prepared=str(ready),passed=True,unit_tests=15,
        real_ROS_exclusive_UDP_bridge_exact_C_loopback=True,browser_protocol_cases=9,
        browser_protocol_normal_seconds=browser_active_seconds,
        browser_actual_UI_test=dict(observation=str(a.ui_observation.resolve()),session=str(ui_session),passed=True,
            note='local in-app browser only; not a remote Windows/SSH or physical acceptance'),
        physical_qualified=False,real_disabled_handshake_verified=False,physical_motion_started=False,
        flashed_again=False,network_or_legacy_entry_modified=False,
        image_sha256=manifest['image_sha256'],created_utc_s=time.time(),trials=results,
        pending=['fresh real disabled zero handshake','single stage0 five-second secured wheels-up trial',
            'physical stage-by-stage electrical thermal stopping and sign-follow acceptance',
            '600-second new-host continuous physical acceptance'],
        caveat='Encoder plant is uncalibrated software; no physical speed/current/temperature/braking claim. Feedback loss may leave stop unconfirmed.')
    save(output/'QUALIFICATION.json',report)
    evidence[str(output/'QUALIFICATION.json')]=sha(output/'QUALIFICATION.json')
    # Inputs/artifacts were checked before consuming the evidence. Recheck before
    # changing the local release bit. This does not alter the frozen flash bundle.
    check(ready,require_release=False)
    manifest.update(host_offline_passed=True,evidence=evidence,source_hashes=hashes(HERE),
        qualification=str(output/'QUALIFICATION.json'))
    save(ready/'HOST_MANIFEST.json',manifest)
    check(ready)
    save(DOCS/'LATEST_PREPARED.json',dict(prepared=str(ready),manifest_sha256=sha(ready/'HOST_MANIFEST.json'),
        qualification=str(output/'QUALIFICATION.json'),physical_qualified=False))
    print(json.dumps(dict(passed=True,prepared=str(ready),qualification=str(output/'QUALIFICATION.json'),
        protected_inputs=len(manifest['protected']),motion_authorized=False),ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ready',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--trial',required=True,action='append',type=Path)
    for name in ('probe','suspended','early-release','browser'):p.add_argument('--'+name,required=True,type=Path)
    p.add_argument('--ui-observation',required=True,type=Path);p.add_argument('--suite-index',type=Path)
    qualify(p.parse_args())
