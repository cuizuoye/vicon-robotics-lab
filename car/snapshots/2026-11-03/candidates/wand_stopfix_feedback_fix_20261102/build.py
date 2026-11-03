"""Independent preflight-policy repair; no firmware or network operations."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_stopfix_deploy_2026-11-02/prepared_v1/host'
OUT=ROOT/'docs/wand_stopfix_feedback_2026-11-02/prepared_v1'
PARENT=ROOT/'candidates/wand_stopfix_deploy_20261102'
RECORD=ROOT/'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json'
AUTH=ROOT/'docs/wand_4wsc_stage6_600s_trial_2026-11-02/OPERATOR_REQUEST.json'
KIND='4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1';CONSENT='STOPFIX_STAGE6_600S_UNQUALIFIED_SITE_READY_V1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def hashes(folder):return {str(p):sha(p) for p in sorted(folder.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and 'sessions' not in p.parts}
def edit(path,old,new):
    text=path.read_text()
    if text.count(old)!=1:raise ValueError('unexpected layout '+str(path)+' '+old[:60])
    path.write_text(text.replace(old,new))
def main():
    resume=OUT.exists()
    if resume and ((OUT/'host/HOST_MANIFEST.json').exists() or (OUT/'BUILD.json').exists()):raise ValueError('never overwrite a prepared snapshot')
    m=json.loads((BASE/'HOST_MANIFEST.json').read_text());assert m['host_offline_passed']
    pins={str(BASE/'HOST_MANIFEST.json'):sha(BASE/'HOST_MANIFEST.json')}
    for group in ('protected','source_hashes','artifact_hashes','evidence'):pins.update(m[group])
    for p,value in pins.items():assert sha(p)==value,'parent drift '+p
    ready=OUT/'host'
    if not resume:shutil.copytree(BASE,ready,ignore=shutil.ignore_patterns('sessions','HOST_MANIFEST.json','__pycache__','*.pyc'))
    else:
        for path,value in m['artifact_hashes'].items():
            if Path(path).is_relative_to(BASE):assert sha(ready/Path(path).relative_to(BASE))==value,'incomplete copy changed'
    for name in ('ground_entry.py','deployment_record.py','test_admission.py'):
        target=HERE/name
        if target.exists():assert resume and sha(target)==sha(PARENT/name),'do not overwrite candidate source '+str(target)
        else:shutil.copyfile(PARENT/name,target)
    edit(HERE/'deployment_record.py',"HOST=ROOT/'docs/wand_stopfix_deploy_2026-11-02/prepared_v1/host'", "HOST=ROOT/'docs/wand_stopfix_feedback_2026-11-02/prepared_v1/host'")
    h=ready/'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware'
    shutil.copyfile(HERE/'deployment_record.py',h/'deployment_record.py')
    probe=ready/'idle_probe.py'
    edit(probe,"CONSENT='4WSC_DISABLED_ZERO_ONLY_NO_ENABLE_NO_CLEAR'", "COMMAND_PROGRESS_TIMEOUT_S=1.0  # Match the existing real bridge, not an extra200ms preflight guard.\nCONSENT='4WSC_DISABLED_ZERO_ONLY_NO_ENABLE_NO_CLEAR'")
    edit(probe,'0<=now-progress_at<=.20)','0<=now-progress_at<=COMMAND_PROGRESS_TIMEOUT_S)')
    edit(probe,"                raise ValueError('feedback exceeds existing 300ms bound')", "                raise ValueError('feedback exceeds existing 300ms bound')\n            if session.connected and wrapped.clock.calibrated and progress_at is not None and now-progress_at>COMMAND_PROGRESS_TIMEOUT_S:\n                raise ValueError('command progress exceeds existing bridge1s bound')")
    edit(probe,"            command_progress_age_s=None if progress_at is None else now-progress_at,", "            command_progress_age_s=None if progress_at is None else now-progress_at,\n            command_progress_timeout_s=COMMAND_PROGRESS_TIMEOUT_S,")
    edit(probe,"        result=dict(passed=False,reason=type(e).__name__+': '+str(e),enable_sent=False,clear_fault_sent=False,motion_started=False)", """        now=time.monotonic()
        result=dict(passed=False,reason=type(e).__name__+': '+str(e),mode='DISABLED_ZERO_ONLY',elapsed_s=now-start,
            clock=None if wrapped is None else wrapped.clock.snapshot(now),fault_bits=None if wheel is None else wheel.fault_bits,
            status=status,command_progress_age_s=None if progress_at is None else now-progress_at,
            command_progress_timeout_s=COMMAND_PROGRESS_TIMEOUT_S,hardware_socket_opened=not rehearsal,
            software_loopback_only=rehearsal,enable_sent=False,select_sent=False,clear_fault_sent=False,motion_started=False,physical_motion_authority=False)""")
    changed=[str(p.relative_to(ready)) for p in ready.rglob('*') if p.is_file() and sha(p)!=sha(BASE/p.relative_to(ready))]
    assert set(changed)=={'idle_probe.py','runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/deployment_record.py'}
    inherited={p:value for p,value in m['inherited_runtime_hashes'].items() if (ready/p).exists() and sha(ready/p)==value}
    direct={str(Path(p).relative_to(BASE)):value for p,value in m['artifact_hashes'].items() if Path(p).is_relative_to(BASE) and str(Path(p).relative_to(BASE)) not in changed}
    m.update(host_offline_passed=False,hardware_launch_enabled=False,protected=pins,
        source_hashes={str(HERE/name):sha(HERE/name) for name in ('build.py','ground_entry.py','deployment_record.py')},
        artifact_hashes=hashes(ready),evidence={},inherited_runtime_hashes=inherited,inherited_parent_runtime_hashes=direct,
        changed_runtime_files=changed,parent_qualification=str(BASE.parent/'QUALIFICATION.json'))
    m.pop('qualification',None);save(ready/'HOST_MANIFEST.json',m)
    save(OUT/'BUILD.json',dict(passed=True,resumed_identical_incomplete_copy=resume,changed_runtime_files=changed,feedback_guard_s=.3,bridge_progress_stall_guard_s=1.,
        old_extra_probe_progress_s=.2,new_probe_progress_s=1.,continuous_ready_window_s=.6,
        no_motion_algorithm_changed=True,firmware_changed=False,network_changed=False))
    print(str(ready))
if __name__=='__main__':main()
