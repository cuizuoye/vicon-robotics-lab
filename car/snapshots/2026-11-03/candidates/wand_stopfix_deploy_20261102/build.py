"""Prepare a separate STOPFIX hardware-admissible host. File operations only."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_stopfix_2026-11-02/prepared_v1/host'
OUT=ROOT/'docs/wand_stopfix_deploy_2026-11-02/prepared_v1'
RECORD=ROOT/'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json'
AUTH=ROOT/'docs/wand_4wsc_stage6_600s_trial_2026-11-02/OPERATOR_REQUEST.json'
KIND='4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'
CONSENT='STOPFIX_STAGE6_600S_UNQUALIFIED_SITE_READY_V1'
LEGACY=ROOT/'candidates/wand_4wsc_stage6_600s_trial_v2_20261102'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,data):path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def hashes(folder):return {str(p):sha(p) for p in sorted(folder.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and 'sessions' not in p.parts}
def edit(path,old,new):
    raw=path.read_text()
    if raw.count(old)!=1:raise ValueError('source layout changed: '+str(path)+' '+old[:60])
    path.write_text(raw.replace(old,new))

def main():
    if OUT.exists():raise ValueError('never overwrite a prepared snapshot')
    m=json.loads((BASE/'HOST_MANIFEST.json').read_text());pins={str(BASE/'HOST_MANIFEST.json'):sha(BASE/'HOST_MANIFEST.json')}
    assert m['host_offline_passed'] and not m['hardware_launch_enabled']
    for group in ('protected','source_hashes','artifact_hashes','evidence'):pins.update(m[group])
    for p,digest in pins.items():assert sha(p)==digest,'parent drift '+p
    from deployment_record import verify_flash_record,MIGRATION,RECORD_SHA
    receipt=verify_flash_record(RECORD)
    for key in ('backup_pass1_path','backup_pass2_path','full_readback_path','archived_programmed_image'):
        pins[receipt[key]]=sha(receipt[key])
    folder=Path(receipt['full_readback_path']).parent
    for name in (*receipt['log_sha256'],'option_bytes_before_pass1.bin','option_bytes_before_pass2.bin','option_bytes_after.bin','POST_FLASH_RECORD.json','BACKUP_VERIFIED.json','verify_stopfix_deployment.py'):
        pins[str(folder/name)]=sha(folder/name)
    pins[str(RECORD)]=sha(RECORD);pins[str(LEGACY/'receiver.py')]=sha(LEGACY/'receiver.py')
    ready=OUT/'host';shutil.copytree(BASE,ready,ignore=shutil.ignore_patterns('HOST_MANIFEST.json','sessions','__pycache__','*.pyc'))
    h=ready/'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware';c=ready/'runtime/src/mecanum_wand_4wsc_control/mecanum_wand_4wsc_control'
    shutil.copyfile(HERE/'deployment_record.py',h/'deployment_record.py')
    shutil.copyfile(LEGACY/'receiver.py',ready/'receiver.py')
    shutil.copyfile(HERE/'admission_check.py',ready/'admission_check.py')
    profile=ready/'profiles/stage6.LONG600_NOT_APPROVED.json';p=json.loads(profile.read_text());p['firmware_deployment']=MIGRATION;save(profile,p)
    edit(h/'authorization.py',"IMAGE_SHA='b471a2a3fe5b24b7b23844a0452a5923bd3b9fc14a78df2a5ffaa9707a7e5769'","IMAGE_SHA='"+MIGRATION['image_sha256']+"'")
    path=h/'authorization.py';raw=path.read_text();a=raw.index('def verify_flash_record(');b=raw.index('\ndef verify_trial_review',a)
    path.write_text(raw[:a]+"def verify_flash_record(record_path,build_manifest=None):\n    from .deployment_record import verify_flash_record as verify\n    return verify(record_path,build_manifest)\n\n"+raw[b:])
    edit(h/'ground_admission.py',"CONSENT='4WSC_STAGE6_LONG600_OPERATOR_TRIAL_UNKNOWN_MEASUREMENTS_SITE_READY'","CONSENT='"+CONSENT+"'")
    edit(h/'host_profile.py',"        if mode=='hardware':raise ValueError('STOPFIX_OFFLINE_ONLY_NOT_FLASHED_OR_FIELD_ACCEPTED')",
        "        if mode=='hardware':\n            from .deployment_record import verify_host_snapshot\n            verify_host_snapshot()")
    edit(h/'host_profile.py',"            verify_runtime_profile(self.profile,self.purpose)","            from .deployment_record import verify_migration\n            verify_migration(self.profile)\n            verify_runtime_profile(self.profile,self.purpose)")
    edit(h/'host_profile.py',"DOMAIN,NAMESPACE,WORLD=129,'/wand_4wsc_stage6_long600','map'","DOMAIN,NAMESPACE,WORLD=129,'/wand_stopfix_stage6_long600','map'")
    for path in (h/'stages.py',c/'stages.py'):
        edit(path,"SCOPE='HARDWARE_4WSC_V8_STAGE6_LONG600_STAGE_'","SCOPE='HARDWARE_STOPFIX_V1_4WSC_V8_STAGE6_LONG600_STAGE_'")
    edit(ready/'server.cjs',"if(mode==='hardware')throw Error('STOPFIX_OFFLINE_ONLY_NOT_FLASHED_OR_FIELD_ACCEPTED');", """if(process.env.WAND_4WSC_MODE!==mode)throw Error('Explicit matching runtime mode required');
if(mode==='hardware'&&synthetic)throw Error('synthetic input forbidden on real hardware');
if(mode==='hardware'){
  const p=require('child_process').spawnSync('/usr/bin/python3',[path.join(root,'admission_check.py')],
    {env:process.env,encoding:'utf8',timeout:10000,maxBuffer:32768});
  if(p.status!==0)throw Error('STOPFIX hardware admission blocked: '+String(p.stderr||p.error||'check failed').slice(-2000));
}""")
    edit(ready/'web/index.html','4WSC · 档6 · 600秒受控试运行','STOPFIX_V1 · 档6 · 最长600秒受控试运行')
    changed=set()
    for p in ready.rglob('*'):
        if p.is_file() and (not (BASE/p.relative_to(ready)).exists() or sha(p)!=sha(BASE/p.relative_to(ready))):changed.add(str(p.relative_to(ready)))
    inherited={str(Path(p).relative_to(BASE)):value for p,value in m['artifact_hashes'].items() if Path(p).is_relative_to(BASE) and str(Path(p).relative_to(BASE)) not in changed}
    save(ready/'HOST_MANIFEST.json',dict(identity='4WSC',firmware_revision='STOPFIX_V1',authority_version=8,diagnostic_payload_bytes=208,
        trial_kind=KIND,stage=6,max_trial_s=600.,host_offline_passed=False,hardware_launch_enabled=False,
        physical_qualified=False,long_session_qualified=False,deployment_record_sha256=RECORD_SHA,
        protected=pins,source_hashes={p:v for p,v in hashes(HERE).items() if Path(p).name in ('build.py','ground_entry.py','deployment_record.py','admission_check.py')},
        artifact_hashes=hashes(ready),evidence={},changed_runtime_files=sorted(changed),
        inherited_runtime_hashes=inherited,inherited_600s_software_qualification=m['inherited_600s_timer_evidence'] if 'inherited_600s_timer_evidence' in m else m['inherited_600s_software_qualification']))
    save(OUT/'BUILD.json',dict(passed=True,hardware_opened=False,firmware_flashed_this_step=False,protected_files=len(pins),changed_runtime_files=sorted(changed)))
    print('DEPLOY_HOST_PREPARED '+str(ready))
if __name__=='__main__':main()
