"""Freeze a separate stage0 ground-commissioning host; no device IO."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_4wsc_browser_2026-11-02/prepared_v12'
OUT=ROOT/'docs/wand_4wsc_ground_2026-11-02'
C='mecanum_wand_4wsc_control';H='mecanum_wand_4wsc_hardware'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def hashes(folder):return {str(p):sha(p) for p in sorted(folder.rglob('*')) if p.is_file() and not any(k in ('sessions','__pycache__') for k in p.parts)}
def edit(p,old,new):
    s=p.read_text()
    if s.count(old)!=1:raise ValueError('pinned input layout changed: '+str(p)+' '+old[:90])
    p.write_text(s.replace(old,new))


def generate(ready):
    sys.path.insert(0,str(ROOT/'candidates/wand_4wsc_browser_20261102'))
    from entry import check
    m=check(BASE)
    if ready.exists():raise ValueError('never overwrite a prepared or qualified snapshot')
    shutil.copytree(BASE,ready,ignore=shutil.ignore_patterns('sessions','profiles','HOST_MANIFEST.json','__pycache__','*.pyc'))
    c,h=ready/'runtime/src'/C/C,ready/'runtime/src'/H/H
    protected={**m['protected'],**m['source_hashes'],**m['artifact_hashes'],str(BASE/'HOST_MANIFEST.json'):sha(BASE/'HOST_MANIFEST.json')}
    for p,digest in protected.items():
        if sha(p)!=digest:raise ValueError('preserved input changed '+p)
    for p in (c/'stages.py',h/'stages.py'):
        edit(p,"SCOPE='HARDWARE_4WSC_V8_600S_DIRECT_XY_YAW_STAGE_'","SCOPE='HARDWARE_4WSC_V8_GROUND_COMMISSION_60S_STAGE_'")
    edit(c/'staged_config.py','self.max_session_s != 600.','self.max_session_s != 60.')
    edit(c/'staged_config.py','Exact 600s session','Exact ground60 session')
    edit(c/'staged_config.py','if len(choices)!=1:','if choices!=[0]:')
    p=c/'demo_policy.py';s=p.read_text().replace('SPACE_TOGGLE_600S_V1','SPACE_TOGGLE_GROUND60_V1').replace('max_active_s=600.0','max_active_s=60.0').replace('NO_FIXED_FENCE_600S','NO_FIXED_FENCE_GROUND60');p.write_text(s)
    edit(c/'pilot_core.py',"'PILOT_TIME_LIMIT_600S'","'PILOT_TIME_LIMIT_60S'")
    shutil.copyfile(HERE/'admission.py',h/'ground_admission.py')
    p=h/'host_profile.py';s=p.read_text()
    s=s.replace('from .authorization import verify_runtime_profile,endpoint','from .authorization import endpoint\nfrom .ground_admission import verify_runtime_profile,common,CONSENT')
    s=s.replace("125,'/wand_4wsc_sync'","126,'/wand_4wsc_ground60'").replace("CONSENT='4WSC_SPACE_TOGGLE_600S_CLEAR_POWER_CUT_READY'\n",'')
    s=s.replace("self.profile=json.loads(raw);","self.profile=json.loads(raw);common(self.profile);")
    s=s.replace('self.domain=192;','self.domain=193;')
    p.write_text(s)
    p=h/'nodes.py'
    edit(p,"            native_fault_bits=self.last_mcu_fault_bits,",
        "            native_fault_bits=self.last_mcu_fault_bits,\n"
        "            commissioning_observation=(dict(wheel_rpm=list(self.last_wheel_rpm),raw_duty=status['raw_duty'],target_cps=status['target_cps']) if fresh else None),")
    p=ready/'server.cjs';s=p.read_text().replace("?'192':'125'","?'193':'126'")
    s=s.replace('seconds>600','seconds>60').replace('seconds=600','seconds=60');p.write_text(s)
    p=ready/'worker.py';s=p.read_text().replace('default=60.','default=60.').replace('args.seconds<=600','args.seconds<=60')
    s=s.replace('from hardware_runtime import Rig','from ground_runtime import Rig')
    s=s.replace("'duration1..120 required'","'ground trial 1..60 seconds required'")
    # Both supervisor and final gate have their own 60s bound. A correctly
    # triggered independent deadline is an expected finish, not a hidden fault.
    s=s.replace("channel.fault or 'TIMER_COMPLETE'","channel.fault or 'TIMER_COMPLETE'")
    s=s.replace("passed=bool(complete and not channel.fault and stop_evidence and stop_evidence['stop_confirmed'])",
        "passed=bool((complete or channel.fault=='GROUND60_TIME_COMPLETE') and channel.fault in ('','GROUND60_TIME_COMPLETE') and stop_evidence and stop_evidence['stop_confirmed'])")
    s=s.replace('completed=complete,',"completed=bool(complete or channel.fault=='GROUND60_TIME_COMPLETE'),")
    s=s.replace('4WSC host; rehearsal is loopback/software only; real trials do not qualify higher tiers',
        'UNQUALIFIED STAGE0 GROUND COMMISSIONING ONLY; no automatic long-demo or higher-stage release')
    p.write_text(s)
    for name in ('ground_runtime.py','measurements.py'):shutil.copyfile(HERE/name,ready/name)
    p=ready/'web/index.html';p.write_text(p.read_text().replace('4WSC · 真实Sign → 小车','4WSC · 档0落地初验 · 未物理验收'))
    p=ready/'web/app.js';s=p.read_text().replace('4WSC · 固定审查档','4WSC · 档0落地初验 ≤60秒 · 未物理验收')
    s=s.replace('`4WSC · 档${v.stage}', '`档0落地初验（未物理验收） · 档${v.stage}')
    p.write_text(s)
    p=json.loads((BASE/'profiles/stage0.NOT_APPROVED.json').read_text())
    p.update(trial_kind='4WSC_STAGE0_GROUND_COMMISSIONING_SINGLE60S_V1',max_trial_s=60.,
        arena_revision='tracking_only_ground60_4wsc_v8',operator_input_mode='SPACE_TOGGLE_GROUND60_V1',
        physical_qualified=False,long_session_qualified=False,commissioning_review={},trial_review={},
        measurements={k:None for k in ('stop_envelope_m','battery_current_a','driver_temperature_c','motor_temperature_c')})
    p['safety']['max_session_s']=60.
    save(ready/'profiles/stage0.GROUND_NOT_AUTHORIZED.json',p)
    # Reuse the already-tested chain/fault drivers, adapting only the private
    # runtime fixture, domain and expected independent 60-second deadline.
    tests=ready/'tests';tests.mkdir(exist_ok=True)
    old=ROOT/'candidates/wand_4wsc_browser_20261102'
    s=(old/'trial.py').read_text()
    s=s.replace('from entry import setup_rehearsal','from ground_entry import profile as make_profile')
    s=s.replace('profile,consent=setup_rehearsal(stage,folder)','profile,consent=make_profile(ready,folder,True)')
    s=s.replace("ROS_DOMAIN_ID='192'","ROS_DOMAIN_ID='193'")
    s=s.replace('from hardware_runtime import FollowRig','from ground_runtime import GroundRig as FollowRig')
    s=s.replace("if case=='occlusion':rig.occlude=True","if case=='occlusion':rig.occlude=True\n                    elif case=='car_loss':rig.drop_car=True")
    s=s.replace('rig.occlude=False;rig.peer.feedback=True','rig.occlude=False;rig.drop_car=False;rig.peer.feedback=True')
    s=s.replace("'occlusion','feedback','loop_gap'","'occlusion','car_loss','feedback','loop_gap'")
    s=s.replace("type=int,default=0)","type=int,choices=(0,),default=0)")
    (tests/'native_trial.py').write_text(s)
    p=ready/'hardware_runtime.py'
    edit(p,"if now-sample['at']<=.20:","if now-sample['at']<=.20 and not getattr(rig,'drop_car',False):")
    edit(p,'        if self.closing:self.operator.pump()',
        "        if hasattr(self,'before_operator_step'):self.before_operator_step(time.monotonic())\n        if self.closing:self.operator.pump()")
    s=(old/'test_browser.cjs').read_text().replace('r.ros_domain===192','r.ros_domain===193')
    s=s.replace('views=0;','views=0,present=true;').replace('focused:true,visible:true,space,','focused:present,visible:present,space,')
    s=s.replace("if(name==='proof_timeout')", "if(name==='esc')setTimeout(()=>control.send(JSON.stringify({type:'stop',reason:'ESC'})),1500);\n     if(name==='blur')setTimeout(()=>present=false,1500);\n     if(name==='proof_timeout')")
    s=s.replace("normal:'TIMER_COMPLETE',", "normal:'TIMER_COMPLETE',esc:'CLIENT_ESC',blur:'WINDOW_NOT_FOREGROUND',")
    s=s.replace("else assert.equal(r.reason,expected,JSON.stringify(r));", "else if(name==='normal')assert(['TIMER_COMPLETE','GROUND60_TIME_COMPLETE'].includes(r.reason),JSON.stringify(r));\n  else assert.equal(r.reason,expected,JSON.stringify(r));")
    (tests/'browser_peer.cjs').write_text(s)
    save(ready/'HOST_MANIFEST.json',dict(identity='4WSC',authority_version=8,diagnostic_payload_bytes=208,
        trial_kind='4WSC_STAGE0_GROUND_COMMISSIONING_SINGLE60S_V1',host_offline_passed=False,physical_qualified=False,
        protected=protected,source_hashes=hashes(HERE),artifact_hashes=hashes(ready),
        default_motion_authority=False,max_trial_s=60.,stage=0))
    print(ready)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path)
    generate(p.parse_args().output.resolve())
