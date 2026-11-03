"""Independent STOPFIX candidate: file/build/loopback only. Never hardware IO."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_4wsc_stage6_600s_trial_2026-11-02/prepared_v2'
FWBASE=ROOT/'docs/wand_firmware_4wsc_2026-10-30/prepared_v4'
OUT=ROOT/'docs/wand_stopfix_2026-11-02/prepared_v1'
KIND='4WSC_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'
CONSENT='4WSC_STAGE6_LONG600_OPERATOR_TRIAL_UNKNOWN_MEASUREMENTS_SITE_READY'
AUTH=BASE.parent/'OPERATOR_REQUEST.json'
TARGET='mecanum_f407_4wsc_stopfix_v1_NOT_FLASHED'
OFFLINE='STOPFIX_OFFLINE_ONLY_NOT_FLASHED_OR_FIELD_ACCEPTED'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def hashes(folder):return {str(p):sha(p) for p in sorted(folder.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and 'sessions' not in p.parts}
def edit(p,old,new):
    s=p.read_text()
    if s.count(old)!=1:raise ValueError('pinned source layout changed: '+str(p)+' '+old[:70])
    p.write_text(s.replace(old,new))
def command(name,args):
    r=subprocess.run(list(map(str,args)),cwd=ROOT,capture_output=True,text=True,timeout=300,
        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
    (OUT/(name+'.log')).write_text(r.stdout+r.stderr)
    print(name+': '+str(r.returncode),flush=True)
    if r.returncode:raise ValueError((r.stdout+r.stderr)[-6000:])
def parent_pins():
    host=json.loads((BASE/'HOST_MANIFEST.json').read_text())
    if not host['host_offline_passed']:raise ValueError('qualified parent required')
    pins={str(BASE/'HOST_MANIFEST.json'):sha(BASE/'HOST_MANIFEST.json')}
    for group in ('protected','source_hashes','artifact_hashes','evidence'):pins.update(host[group])
    fw=json.loads((FWBASE/'DELIVERY.json').read_text())
    for group in ('source_hashes','firmware_source_hashes'):pins.update(fw[group])
    pins.update(json.loads((FWBASE/'protected_inputs.json').read_text()))
    pins[str(FWBASE/'DELIVERY.json')]=sha(FWBASE/'DELIVERY.json')
    pins[fw['bin']]=fw['bin_sha256']
    for p,digest in pins.items():
        if sha(p)!=digest:raise ValueError('preserved baseline drift: '+p)
    return host,pins
def firmware():
    fw=OUT/'firmware';shutil.copytree(FWBASE/'firmware',fw)
    p=fw/'Src/four_link_motion.c';s=p.read_text()
    start=s.index('    /* Relative source/receive skew guard,')
    end=s.index('    if (h->message_type == MC_MSG_CONTROL_ENABLE)',start)
    p.write_text(s[:start]+(HERE/'stop_timing.c.fragment').read_text()+s[end:])
    shutil.copyfile(HERE/'test_stop_safety.c',fw/'tests/test_stop_safety.c')
    p=fw/'CMakeLists.txt';s=p.read_text().replace('mecanum_f407_4wsc_staged_NOT_FLASHED',TARGET)
    s=s.replace('READY,4WSC_STAGED_V8_NOT_FLASHED','READY,4WSC_STOPFIX_V1_V8_NOT_FLASHED')
    s+='''
if(NOT SERVO_ARM)
  add_executable(test_stop_safety tests/test_stop_safety.c ${ALGORITHMS})
  foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(values test_stages ${property})
    if(values)
      set_property(TARGET test_stop_safety PROPERTY ${property} "${values}")
    endif()
  endforeach()
  target_link_libraries(test_stop_safety PRIVATE m)
  add_test(NAME stale_stop_revokes_without_false_latch COMMAND test_stop_safety)
endif()
''';p.write_text(s)
    command('native_configure',['cmake','-S',fw,'-B',OUT/'native','-G','Ninja','-DCMAKE_BUILD_TYPE=Debug'])
    command('native_build',['cmake','--build',OUT/'native','-j2'])
    command('native_tests',['ctest','--test-dir',OUT/'native','--output-on-failure'])
    command('arm_configure',['cmake','-S',fw,'-B',OUT/'arm','-G','Ninja','-DSERVO_ARM=ON','-DCMAKE_BUILD_TYPE=Debug'])
    command('arm_build',['cmake','--build',OUT/'arm','--target',TARGET,'-j2'])
    command('hex_export',['arm-none-eabi-objcopy','-O','ihex',OUT/'arm'/(TARGET+'.elf'),OUT/'arm'/(TARGET+'.hex')])
    command('arm_sizes',['arm-none-eabi-size',OUT/'arm'/(TARGET+'.elf')])
    bundle=OUT/'flash_bundle';bundle.mkdir()
    for ext in ('bin','hex','elf','map'):shutil.copyfile(OUT/'arm'/(TARGET+'.'+ext),bundle/(TARGET+'.'+ext))
    image=bundle/(TARGET+'.bin');assert image.stat().st_size<65536
    save(bundle/'BUNDLE.json',dict(identity='4WSC',protocol_authority_version=8,diagnostic_payload_bytes=208,
        firmware_revision='STOPFIX_V1',image=str(image),image_sha256=sha(image),image_bytes=image.stat().st_size,
        flash_address='0x08000000',application_sectors=[0,1,2,3],flashed=False,physical_qualified=False,
        independent_candidate=True,original_image_unchanged=True,host_hardware_launch_enabled=False,
        source_age_lag_ms=120,source_age_lead_ms=60,wire_timeout_ms=600,absolute_run_limit_ms=630000,
        note='Same wire format/identity, distinct revision and exact image hash. Requires new full backup/readback authorization before deployment. No original live entry is updated.'))
    save(bundle/'SHA256.json',hashes(bundle))
def host(parent,pins):
    ready=OUT/'host'
    shutil.copytree(BASE,ready,ignore=shutil.ignore_patterns('sessions','HOST_MANIFEST.json','__pycache__','*.pyc'))
    edit(ready/'native_peer.py',str(FWBASE/'native/libstage0_oracle.so'),str(OUT/'native/libstage0_oracle.so'))
    h=ready/'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware'
    edit(h/'host_profile.py',"    def __init__(self,mode='hardware',profile='',approval='',consent=''):\n",
        "    def __init__(self,mode='hardware',profile='',approval='',consent=''):\n        if mode=='hardware':raise ValueError('"+OFFLINE+"')\n")
    shutil.copyfile(HERE/'focus_evidence.js',ready/'web/focus_evidence.js')
    shutil.copyfile(HERE/'focus_audit.cjs',ready/'focus_audit.cjs')
    edit(ready/'web/index.html','<script src="/app.js" defer>','<script src="/focus_evidence.js" defer></script><script src="/app.js" defer>')
    app=ready/'web/app.js'
    edit(app,"const $=id=>document.getElementById(id),token=location.hash.slice(1),guard=new InputGuard();",
        "const $=id=>document.getElementById(id),token=location.hash.slice(1),guard=new InputGuard(),focusEvidence=new FocusEvidence();")
    old="window.addEventListener('focus',connect);\nwindow.addEventListener('blur',()=>{if(ws)stop('WINDOW_BLUR');});\ndocument.addEventListener('visibilitychange',()=>{if(document.hidden&&ws)stop('WINDOW_HIDDEN');else connect();});\nwindow.addEventListener('pagehide',()=>stop('PAGE_CLOSED'));"
    new="""function observeFocus(kind,event){
  const target=event?.target===window?'window':event?.target===document?'document':event?.target?'element':'unknown';
  const value=focusEvidence.observe(kind,event,target,{at_ms:performance.now(),focused:document.hasFocus(),
    hidden:document.hidden,visibility:document.visibilityState||'unknown',active_tag:(document.activeElement?.tagName||'').slice(0,32),
    width:Math.max(0,Math.min(32768,Math.round(window.innerWidth||0))),height:Math.max(0,Math.min(32768,Math.round(window.innerHeight||0))),fullscreen:!!document.fullscreenElement});
  // Revoke FIRST. Audit cannot grant authority or delay this stop.
  if(value.reason&&ws)stop(value.reason);
  if(value.record.sequence<=256&&token){
    try{fetch('/diagnostic',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({token,record:value.record}),keepalive:true}).catch(()=>{});}catch{}
  }
}
window.addEventListener('focus',e=>{observeFocus('focus',e);connect();});
window.addEventListener('blur',e=>observeFocus('blur',e));
document.addEventListener('visibilitychange',e=>{observeFocus('visibilitychange',e);if(!document.hidden)connect();});
window.addEventListener('pagehide',e=>observeFocus('pagehide',e));
window.addEventListener('resize',e=>observeFocus('resize',e));
document.addEventListener('fullscreenchange',e=>observeFocus('fullscreenchange',e));"""
    edit(app,old,new)
    server=ready/'server.cjs'
    edit(server,"const root=__dirname,args=process.argv.slice(2);let mode='hardware'", "const {createAudit}=require('./focus_audit.cjs');\nconst root=__dirname,args=process.argv.slice(2);let mode='rehearsal'")
    edit(server,"if(mode==='hardware'&&synthetic)","if(mode==='hardware')throw Error('"+OFFLINE+"');\nif(mode==='hardware'&&synthetic)")
    edit(server,"const token=crypto.randomBytes(24).toString('hex'),steady=()=>Number(process.hrtime.bigint())/1e9;",
        "const token=crypto.randomBytes(24).toString('hex'),steady=()=>Number(process.hrtime.bigint())/1e9;\nconst focusAudit=createAudit(path.join(session,'focus_events.jsonl'),token);")
    edit(server,"'/input_guard.js':['input_guard.js','text/javascript']","'/focus_evidence.js':['focus_evidence.js','text/javascript'],'/input_guard.js':['input_guard.js','text/javascript']")
    edit(server,"  const f=files[req.url];if(req.method!=='GET'||!f)","  if(req.url==='/diagnostic'){focusAudit.handle(req,res,originOK);return;}\n  const f=files[req.url];if(req.method!=='GET'||!f)")
    edit(server,"  if(ending)return;ending=true;revoke(reason);","  if(ending)return;ending=true;revoke(reason);focusAudit.close();")
    edit(server,"separate_control_and_view:true,view_ack_is_authority:false,ssh_outer_stream_shared:true","separate_control_and_view:true,view_ack_is_authority:false,ssh_outer_stream_shared:true,focus_audit:focusAudit.stats()")
    changed={str(p.relative_to(ready)) for p in (ready/'native_peer.py',h/'host_profile.py',app,server,ready/'web/index.html')}
    inherited={}
    for p,digest in parent['artifact_hashes'].items():
        relative=Path(p).relative_to(BASE)
        if str(relative) not in changed:
            assert sha(ready/relative)==digest,'unexpected runtime drift '+str(relative)
            inherited[str(relative)]=digest
    save(ready/'HOST_MANIFEST.json',dict(identity='4WSC',authority_version=8,diagnostic_payload_bytes=208,
        trial_kind=KIND,firmware_revision='STOPFIX_V1',stage=6,max_trial_s=600.,host_offline_passed=False,
        hardware_launch_enabled=False,physical_qualified=False,long_session_qualified=False,
        protected=pins,source_hashes=hashes(HERE),artifact_hashes=hashes(ready),
        inherited_runtime_hashes=inherited,inherited_600s_software_qualification=parent['inherited_600s_software_qualification']))
def main():
    if OUT.exists():raise ValueError('never overwrite a candidate/evidence folder')
    parent,pins=parent_pins();OUT.mkdir(parents=True);save(OUT/'protected_inputs.json',pins)
    firmware();host(parent,pins)
    save(OUT/'BUILD.json',dict(passed=True,firmware_prepared=True,flashed=False,hardware_output=False,host_hardware_launch_enabled=False,
        protected=pins,source_hashes=hashes(HERE),firmware_source_hashes=hashes(OUT/'firmware'),binary_sha256=sha(OUT/'flash_bundle'/(TARGET+'.bin'))))
    print('STOPFIX_BUILD '+str(OUT),flush=True)
if __name__=='__main__':main()
