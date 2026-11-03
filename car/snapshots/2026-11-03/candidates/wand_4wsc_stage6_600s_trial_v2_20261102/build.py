"""Separate explicitly requested, unqualified duration trial; reuse frozen motion code."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_4wsc_stage6_600s_2026-11-02/prepared_v4'
OUT=ROOT/'docs/wand_4wsc_stage6_600s_trial_2026-11-02'
KIND='4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'
CONSENT='4WSC_STAGE6_LONG600_OPERATOR_TRIAL_UNKNOWN_MEASUREMENTS_SITE_READY'
PARENT_SHA='3340a0285ee0e102e3ad8e95170bf3d1c24b5f863622988711fc8f35b25e5a2a'
AUTH=OUT/'OPERATOR_REQUEST.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def hashes(folder):return {str(p):sha(p) for p in sorted(folder.rglob('*')) if p.is_file() and not any(k in ('sessions','__pycache__') for k in p.parts)}
def validate_parent():
    assert sha(BASE/'HOST_MANIFEST.json')==PARENT_SHA,'frozen qualified parent changed'
    m=json.loads((BASE/'HOST_MANIFEST.json').read_text())
    assert m['host_offline_passed'] and m['stage']==6 and m['max_trial_s']==600.
    for group in ('protected','source_hashes','artifact_hashes','evidence'):
        for p,digest in m[group].items():assert sha(p)==digest,'preserved input changed: '+p
    return m
def generate(ready):
    parent=validate_parent()
    if ready.exists():raise ValueError('never overwrite prepared snapshots')
    shutil.copytree(BASE,ready,ignore=shutil.ignore_patterns('sessions','profiles','HOST_MANIFEST.json','__pycache__','*.pyc'))
    h=ready/'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware'
    admission=(h/'ground_admission.py').read_text()
    admission=admission.replace('"""Fixed stage6 long-duration software policy; no missing-measurement bypass."""',
        '"""Explicit operator-requested UNQUALIFIED trial, not measured long-duration approval."""')
    admission=admission.replace('4WSC_STAGE6_LONG600_CONTROLLED_REVIEWED_V1',KIND).replace('4WSC_STAGE6_LONG600_MEASURED_REVIEW_CLEAR_POWER_CUT_READY',CONSENT)
    old='    verify_long_review(p)\n    return True\n'
    assert admission.count(old)==1
    admission=admission.replace(old,
        "    if 'operator_duration_trial' in p:\n"
        "        verify_operator_trial(p)\n"
        "    else:\n"
        "        verify_long_review(p)\n"
        "    return True\n")
    admission+='\nAUTHORIZATION_PATH=Path('+repr(str(AUTH))+')\nAUTHORIZATION_SHA='+repr(sha(AUTH))+'\n'+(HERE/'operator_admission.py').read_text()
    (h/'ground_admission.py').write_text(admission)
    p=json.loads((BASE/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
    p['trial_kind']=KIND
    save(ready/'profiles/stage6.LONG600_NOT_APPROVED.json',p)
    # Only labels and admission differ: every motor/ROS/safety/timing file is byte-identical.
    for f in ('web/index.html','web/app.js'):
        path=ready/f;s=path.read_text().replace('600秒持续演示候选','600秒受控试运行').replace('未物理验收','停车/电流/温升未量化')
        path.write_text(s)
    auto=ready/'runtime/src/mecanum_wand_4wsc_control/mecanum_wand_4wsc_control/auto_prepare.py'
    s=auto.read_text()
    s=s.replace('self.qualified_zero_baseline_seen=False','self.qualified_zero_baseline_seen=False\n        self.ready_confirmed=False;self.reconfirm_since=None')
    old="                self.phase='READY'\n            elif self.requested_at is not None"
    new="                self.phase='READY';self.ready_confirmed=True;self.reconfirm_since=None\n            elif self.ready_confirmed:\n                if self.reconfirm_since is None:self.reconfirm_since=now\n                if now-self.reconfirm_since>5.:self.cancel('就绪条件连续5秒未恢复；保持停止，不自动重试')\n                else:self.phase='WAITING_READY'\n            elif self.requested_at is not None"
    assert s.count(old)==1
    s=s.replace(old,new)
    s=s.replace("            READY='准备就绪：", "            WAITING_READY='已完成准备：当前就绪条件暂不满足，保持失能；恢复后仍须新的人工单击',\n            READY='准备就绪：")
    auto.write_text(s)
    changed={'runtime/src/mecanum_wand_4wsc_control/mecanum_wand_4wsc_control/auto_prepare.py','runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/ground_admission.py','web/index.html','web/app.js'}
    inherited={}
    for original,digest in parent['artifact_hashes'].items():
        rel=Path(original).relative_to(BASE)
        if rel.parts[0]=='profiles' or str(rel) in changed:continue
        target=ready/rel
        assert target.exists() and sha(target)==digest,'unapproved runtime difference: '+str(rel)
        inherited[str(rel)]=digest
    previous=OUT/'prepared_v1/HOST_MANIFEST.json'
    prev=json.loads(previous.read_text())
    protected={str(BASE/'HOST_MANIFEST.json'):PARENT_SHA,str(AUTH):sha(AUTH),str(previous):sha(previous)}
    for group in ('protected','source_hashes','artifact_hashes','evidence'):protected.update(prev[group])
    failed=OUT/'prepared_v1/sessions/fast_hardware_U82rFf/result.json'
    protected[str(failed)]=sha(failed)
    for group in ('protected','source_hashes','artifact_hashes','evidence'):protected.update(parent[group])
    save(ready/'HOST_MANIFEST.json',dict(identity='4WSC',authority_version=8,diagnostic_payload_bytes=208,
        trial_kind=KIND,host_offline_passed=False,physical_qualified=False,long_session_qualified=False,
        protected=protected,source_hashes=hashes(HERE),artifact_hashes=hashes(ready),
        inherited_runtime_hashes=inherited,inherited_600s_software_qualification=parent['qualification'],
        default_motion_authority=False,max_trial_s=600.,stage=6))
    print(ready)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);generate(p.parse_args().output.resolve())
