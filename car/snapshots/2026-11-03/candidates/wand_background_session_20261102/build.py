"""Freeze an independent bounded backend-session fork, never physical IO."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_tracker_startup_fix_2026-11-02/prepared_v1/host'
OUT=ROOT/'docs/wand_background_session_2026-11-02/prepared_v1'
PARENT=ROOT/'candidates/wand_tracker_startup_fix_20261102'
RECORD=ROOT/'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json'
AUTH=ROOT/'docs/wand_4wsc_stage6_600s_trial_2026-11-02/OPERATOR_REQUEST.json'
KIND='4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'
CONSENT='STOPFIX_STAGE6_600S_UNQUALIFIED_SITE_READY_V1'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def hashes(folder):
    return {str(p):sha(p) for p in sorted(folder.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and 'sessions' not in p.parts}
def edit(v,old,new):
    assert v.count(old)==1,old[:120]
    return v.replace(old,new)

def client_source():
    renderer=(BASE/'web/app.js').read_text().split('var TrackerWorldView =',1)[1]
    renderer='var TrackerWorldView ='+renderer
    renderer=edit(renderer,"if(!resultSeen)text('instruction'", "if(!resultSeen&&!ended)text('instruction'")
    renderer=edit(renderer,"  text('timing',`UI周期最大 ${guard.maxGap.toFixed(1)}ms；回应调度最大 ${maxReplyDispatch.toFixed(1)}ms；严格100ms`);\n",'')
    return (HERE/'client.js').read_text()+'\n'+renderer

def session_prepare_source():
    p=BASE/'runtime/src/mecanum_wand_4wsc_control/mecanum_wand_4wsc_control/auto_prepare.py'
    v=p.read_text().split('\nclass AutoPrepareConsoleMixin:',1)[0]
    v=v.replace('from .novice_ui import','from mecanum_wand_4wsc_control.novice_ui import').replace('from .idle_admission import','from mecanum_wand_4wsc_control.idle_admission import')
    v=edit(v,'class AutoPrepareOnce:','class SessionPrepare:')
    v=edit(v,'released and model.focused','released and model.backend_available')
    return v

def worker_source():
    v=(BASE/'worker.py').read_text()
    v=edit(v,'from control_channel import Channel','from session_channel import Channel')
    v=edit(v,"                elif kind=='wire':", """                elif kind=='backend_tick':
                    channel.backend_tick(packet.get('issued_steady'),time.monotonic())
                elif kind=='local_stop':
                    channel.stop('OPERATOR_ESTOP' if packet.get('reason')=='ESTOP' else 'CLIENT_PAGE_CLOSED' if packet.get('reason')=='PAGE_CLOSED' else 'LOCAL_OPERATOR_STOP')
                elif kind=='wire':""")
    v=edit(v,"                    if accepted:channel.events.extend(data['events'])\n",'')
    v=edit(v,'单击跟随中，剩余≤','后台跟随中，剩余≤')
    v=edit(v,'秒；松键继续；再次空格、Esc或停止按钮结束','秒；切换窗口、最小化继续；返回网页空格/Esc或终端--stop结束')
    v=edit(v,"reply_dispatch='MAIN_THREAD_CHALLENGE_CALLBACK; independent periodic guard100ms'", "reply_dispatch='NO_BROWSER_HEARTBEAT; explicit bounded backend session; backend loop100ms'")
    return v

def server_source():
    v=(BASE/'server.cjs').read_text()
    v=edit(v,"const {createAudit}=require('./focus_audit.cjs');", "const {createAudit}=require('./focus_audit.cjs');\nconst {createStopAPI}=require('./stop_api.cjs');")
    v=edit(v,'let actualPort=port,owner=null,child=null,ending=false,lastState=null,lastLoop=steady(),maxLoop=0;',
           'const parentPid=process.ppid;\nlet actualPort=port,owner=null,child=null,ending=false,lastState=null,lastOffer=null,lastLoop=steady(),maxLoop=0,workerReady=false;')
    v=edit(v,"  if(owner?.opened&&!owner.revoked&&owner.viewSentAt!==null&&n-owner.viewSentAt>.100)revoke('VIEW_ACK_TIMEOUT_100MS');", "  if(process.ppid!==parentPid){shutdown('LAUNCHER_EXITED');return;}\n  if(workerReady&&!ending)toChild({kind:'backend_tick',connection:'local-backend',issued_steady:n});")
    v=edit(v,'},10).unref();','},20).unref();')
    v=edit(v,'const server=http.createServer((req,res)=>{',"""const stopAPI=createStopAPI(token,reason=>{
  if(owner)owner.revoked=true;
  toChild({kind:'local_stop',connection:'local-console',reason});
},originOK);
const server=http.createServer((req,res)=>{""")
    v=edit(v,"  if(req.url==='/diagnostic')", "  if(req.url==='/operator-stop'){stopAPI(req,res);return;}\n  if(req.url==='/diagnostic')")
    v=edit(v,'return {mode,state:v.state,reason:String(v.reason||\'\').slice(0,100),car:v.car,goal:v.goal,target_policy:v.target_policy,',
           "return {...status(v),intent_offer:lastOffer,mode,state:v.state,reason:String(v.reason||'').slice(0,100),car:v.car,goal:v.goal,target_policy:v.target_policy,")
    v=edit(v,"send(ws,{kind:'hello',mode,synthetic,launcher_url:", "send(ws,{kind:'hello',authority_mode:'WAND_BACKEND_SESSION_V1',mode,synthetic,launcher_url:")
    v=edit(v,"if(lastState){send(owner.control,status(lastState));owner.slot.offer(viewFrame(lastState));}", "if(lastState)owner.slot.offer(viewFrame(lastState));")
    v=edit(v,"      if(linkedOwner.viewSentAt!==null&&steady()-linkedOwner.viewSentAt>.100){revoke('VIEW_ACK_TIMEOUT_100MS');return;}\n",'')
    v=edit(v,"    // Timing samples are server-owned metadata, never accepted as browser authority.", "    if(linkedOwner.revoked)return;\n    // Explicit intents only; browser timing is not authority in this mode.")
    v=edit(v,"    if(v.kind==='ready'){", "    if(v.kind==='ready'){\n      workerReady=true;")
    v=edit(v,"    }else if(v.kind==='challenge'){\n      if(owner&&owner.opened&&!owner.revoked&&v.connection===owner.id)send(owner.control,v);", "    }else if(v.kind==='intent_offer'){\n      lastOffer=v;")
    v=edit(v,"lastState=v;if(owner&&owner.opened&&!owner.revoked){send(owner.control,status(v));owner.slot.offer(viewFrame(v));}",
           "lastState=v;if(owner&&owner.opened&&!owner.revoked)owner.slot.offer(viewFrame(v));")
    v=edit(v,"'\\n控制/画面分流；严格100ms；仅明确批准的硬件模式可输出；空格前保持失能。\\n'", "'\\n后台独立会话；最小化继续；后台100ms保护；空格前保持失能。\\n'")
    return v

def refresh_draft():
    ready=OUT/'host';m=json.loads((ready/'HOST_MANIFEST.json').read_text())
    assert not m['host_offline_passed'] and not m['hardware_launch_enabled'],'never mutate a qualified host'
    for name in ('session_channel.py','session_model.py','session_operator.py','stop_api.cjs'):
        shutil.copyfile(HERE/name,ready/name)
    (ready/'session_prepare.py').write_text(session_prepare_source())
    (ready/'worker.py').write_text(worker_source());(ready/'server.cjs').write_text(server_source())
    (ready/'web/app.js').write_text(client_source())
    value=(BASE/'hardware_runtime.py').read_text()
    value=edit(value,'from mecanum_wand_4wsc_control.toggle_ui import ToggleOperatorModel','from session_model import SessionOperatorModel as ToggleOperatorModel')
    value=edit(value,'from browser_operator import BrowserOperator','from session_operator import SessionOperator as BrowserOperator')
    value=edit(value,'browser_timeout_s=.1,command_timeout_s=.1,feedback_timeout_s=.3,pose_timeout_s=.2,',
               "browser_timeout_s=None,authority_mode='WAND_BACKEND_SESSION_V1',command_timeout_s=.1,feedback_timeout_s=.3,pose_timeout_s=.2,")
    # Rehearsal evidence only. Physical mode has no software peer and never uses
    # this sample to upgrade its normal stop-confirmation result.
    value=edit(value,'permit_ever=self.peer.permit_ever))',
               'permit_ever=self.peer.permit_ever,final_sample=self.peer.sample()))')
    (ready/'hardware_runtime.py').write_text(value)
    index=(BASE/'web/index.html').read_text().replace('STOPFIX_V1 · 档6','Sign遥操作 · 后台独立演示')
    (ready/'web/index.html').write_text(index)
    changed={str(p.relative_to(ready)) for p in ready.rglob('*') if p.is_file() and p.name!='HOST_MANIFEST.json' and (not (BASE/p.relative_to(ready)).exists() or sha(p)!=sha(BASE/p.relative_to(ready)))}
    m.update(changed_runtime_files=sorted(changed),source_hashes=hashes(HERE),artifact_hashes=hashes(ready),
             inherited_runtime_hashes={p:d for p,d in m['inherited_runtime_hashes'].items() if (ready/p).exists() and sha(ready/p)==d},
             inherited_parent_runtime_hashes={str(Path(p).relative_to(BASE)):d for p,d in json.loads((BASE/'HOST_MANIFEST.json').read_text())['artifact_hashes'].items() if Path(p).is_relative_to(BASE) and str(Path(p).relative_to(BASE)) not in changed})
    m['artifact_hashes'].pop(str(ready/'HOST_MANIFEST.json'),None);save(ready/'HOST_MANIFEST.json',m)

def main():
    if OUT.exists():raise ValueError('never overwrite a prepared snapshot')
    m=json.loads((BASE/'HOST_MANIFEST.json').read_text());assert m['host_offline_passed'] and m['hardware_launch_enabled']
    pins={str(BASE/'HOST_MANIFEST.json'):sha(BASE/'HOST_MANIFEST.json')}
    for group in ('protected','source_hashes','artifact_hashes','evidence'):pins.update(m[group])
    for p,d in pins.items():assert sha(p)==d,p
    ready=OUT/'host';shutil.copytree(BASE,ready,ignore=shutil.ignore_patterns('sessions','HOST_MANIFEST.json','__pycache__','*.pyc'))
    for name in ('ground_entry.py','deployment_record.py','test_admission.py','startup_entry.py','launcher.py','launcher.js','launcher.html','launcher.css','page_warmup.js','test_startup.py'):
        assert not (HERE/name).exists()
        v=(PARENT/name).read_text().replace('docs/wand_tracker_startup_fix_2026-11-02/prepared_v1/host','docs/wand_background_session_2026-11-02/prepared_v1/host')
        if name=='test_admission.py':
            v=edit(v,"('worker.py','ground_runtime.py','hardware_runtime.py','browser_operator.py','control_channel.py','web/input_guard.js','web/focus_evidence.js','focus_audit.cjs')",
                   "('ground_runtime.py','browser_operator.py','control_channel.py','web/input_guard.js','web/focus_evidence.js','focus_audit.cjs')")
        if name=='startup_entry.py':
            v=edit(v,"def main():\n    if '--launch'", "def main():\n    if sys.argv[1:]==['--stop']:\n        from stop_command import stop\n        return stop()\n    if '--launch'")
        (HERE/name).write_text(v)
    shutil.copyfile(HERE/'deployment_record.py',ready/'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/deployment_record.py')
    m.update(host_offline_passed=False,hardware_launch_enabled=False,protected=pins,source_hashes={},artifact_hashes={},evidence={},
             authority_mode='WAND_BACKEND_SESSION_V1',browser_foreground_required_during_motion=False,
             presentation_revision='BACKGROUND_SESSION_V1')
    m.pop('qualification',None);save(ready/'HOST_MANIFEST.json',m);refresh_draft()
    print(ready)

if __name__=='__main__':main()
