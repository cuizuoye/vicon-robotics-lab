"""Small independent display-result fix over the frozen backend-session release."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_background_session_2026-11-02/prepared_v1/host'
OUT=ROOT/'docs/wand_background_session_display_2026-11-02/prepared_v1'
PARENT=ROOT/'candidates/wand_background_session_20261102'
RECORD=ROOT/'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json'
AUTH=ROOT/'docs/wand_4wsc_stage6_600s_trial_2026-11-02/OPERATOR_REQUEST.json'
KIND='4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'
CONSENT='STOPFIX_STAGE6_600S_UNQUALIFIED_SITE_READY_V1'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def hashes(folder):
    return {str(p):sha(p) for p in sorted(folder.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and 'sessions' not in p.parts}
def edit(v,old,new):
    assert v.count(old)==1,old[:80];return v.replace(old,new)

def main():
    if OUT.exists():raise ValueError('never overwrite prepared snapshots')
    m=json.loads((BASE/'HOST_MANIFEST.json').read_text());assert m['host_offline_passed'] and m['hardware_launch_enabled']
    pins={str(BASE/'HOST_MANIFEST.json'):sha(BASE/'HOST_MANIFEST.json')}
    for g in ('protected','source_hashes','artifact_hashes','evidence'):pins.update(m[g])
    for p,d in pins.items():assert sha(p)==d,p
    ready=OUT/'host';shutil.copytree(BASE,ready,ignore=shutil.ignore_patterns('sessions','HOST_MANIFEST.json','__pycache__','*.pyc'))
    for name in ('ground_entry.py','deployment_record.py','test_admission.py','startup_entry.py','stop_command.py','launcher.py','launcher.js','launcher.html','launcher.css'):
        assert not (HERE/name).exists()
        (HERE/name).write_text((PARENT/name).read_text().replace('docs/wand_background_session_2026-11-02/prepared_v1/host','docs/wand_background_session_display_2026-11-02/prepared_v1/host'))
    shutil.copyfile(HERE/'deployment_record.py',ready/'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/deployment_record.py')
    v=(ready/'server.cjs').read_text()
    v=edit(v,"if(owner)send(owner.control,{kind:'result',result:{...v.result,last_state:undefined},transport});", """// Keep full native-C/ROS diagnostics on disk, not on the bounded control socket.
      if(owner)send(owner.control,{kind:'result',result:{passed:v.result.passed,completed:v.result.completed,
        reason:v.result.reason,stop_evidence:{stop_confirmed:v.result.stop_evidence?.stop_confirmed===true,
          gate_output_zero:v.result.stop_evidence?.gate_output_zero===true}}});""")
    (ready/'server.cjs').write_text(v)
    v=(ready/'web/app.js').read_text()
    v=edit(v,'  lastViewSequence=v.view_sequence;lastView=performance.now();latest=v;', """  lastViewSequence=v.view_sequence;lastView=performance.now();
  if(resultSeen){viewWs.send(JSON.stringify({type:'ack',sequence:v.view_sequence}));return;}
  latest=v;""")
    (ready/'web/app.js').write_text(v)
    v=(PARENT/'test_browser.cjs').read_text()
    v=edit(v,'  await sleep(150);',"""  if(!page.isClosed()){
   const expected=name==='estop'?'STOP_UNCONFIRMED':'STOPPED_CONFIRMED';
   await until(()=>page.evaluate(expected=>resultSeen&&latest?.state===expected,expected),4000);
   await sleep(250);assert.equal(await page.evaluate(()=>latest.state),expected,'late view overwrote terminal result');
  }
  await sleep(150);""")
    (HERE/'test_browser.cjs').write_text(v)
    changed={str(p.relative_to(ready)) for p in ready.rglob('*') if p.is_file() and sha(p)!=sha(BASE/p.relative_to(ready))}
    m.update(host_offline_passed=False,hardware_launch_enabled=False,protected=pins,source_hashes=hashes(HERE),artifact_hashes=hashes(ready),evidence={},
      direct_changed_runtime_files=sorted(changed),changed_runtime_files=sorted(set(m['changed_runtime_files'])|changed),
      inherited_runtime_hashes={p:d for p,d in m['inherited_runtime_hashes'].items() if (ready/p).exists() and sha(ready/p)==d},
      inherited_parent_runtime_hashes={str(Path(p).relative_to(BASE)):d for p,d in m['artifact_hashes'].items() if Path(p).is_relative_to(BASE) and str(Path(p).relative_to(BASE)) not in changed},
      presentation_revision='BACKGROUND_SESSION_DISPLAY_V2')
    m.pop('qualification',None);save(ready/'HOST_MANIFEST.json',m);print(ready)

if __name__=='__main__':main()
