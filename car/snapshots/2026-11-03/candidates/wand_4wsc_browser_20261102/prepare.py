"""Generate an isolated 4WSC browser host. No ROS initialization/device access."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE = Path(__file__).resolve().parent
BASE = ROOT/'docs/wand_browser_direct_2026-10-30/prepared_v1'
HOST = ROOT/'docs/wand_4wsb_host_2026-10-30/prepared_v9'
FW = ROOT/'docs/wand_firmware_4wsc_2026-10-30/prepared_v4'
DOCS = ROOT/'docs/wand_4wsc_browser_2026-11-02'
OUT = DOCS/'prepared_v1'
C = 'mecanum_wand_4wsc_control'
H = 'mecanum_wand_4wsc_hardware'


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
def rename(s): return s.replace('4wsb','4wsc').replace('4WSB','4WSC').replace('4WSC_V7','4WSC_V8').replace('4wsc_v7','4wsc_v8')
def edit(p, old, new, count=1):
    s=p.read_text()
    if s.count(old)!=count: raise ValueError('sealed input layout changed: '+str(p)+' '+old[:80])
    p.write_text(s.replace(old,new))
def hashes(folder):
    return {str(p):sha(p) for p in sorted(folder.rglob('*')) if p.is_file()
        and not any(k in ('__pycache__','.pytest_cache','sessions') or k.endswith('.egg-info') for k in p.parts)}


def generate(out):
    if out.exists(): raise ValueError('new version required; do not overwrite a draft or qualified result')
    m=json.loads((BASE/'MANIFEST.json').read_text())
    if not m['qualified']: raise ValueError('qualified fixed direct-target baseline required')
    protected={}
    for group in ('source_hashes','artifact_hashes'): protected.update(m[group])
    protected.update(json.loads((FW/'protected_inputs.json').read_text()))
    protected[str(BASE/'MANIFEST.json')]=sha(BASE/'MANIFEST.json')
    protected[str(BASE/'RELEASE.json')]=sha(BASE/'RELEASE.json')
    protected[str(FW/'native/libstage0_oracle.so')]=sha(FW/'native/libstage0_oracle.so')
    for p,digest in protected.items():
        if sha(p)!=digest: raise ValueError('preserved input changed: '+p)
    out.mkdir(parents=True); (out/'COLCON_IGNORE').touch()
    rt=out/'runtime/src'
    shutil.copytree(BASE/'runtime/src', rt, ignore=shutil.ignore_patterns('__pycache__','*.pyc','*.egg-info'))
    for p in rt.rglob('*'):
        if p.is_file(): p.write_text(rename(p.read_text()))
    for p in sorted(rt.rglob('*'), key=lambda x:len(x.parts), reverse=True):
        if rename(p.name)!=p.name:p.rename(p.with_name(rename(p.name)))
    c,h=rt/C/C,rt/H/H
    stages=(ROOT/'candidates/wand_sync_4wsc_20261030/stages.py').read_text()+"\nSCOPE='HARDWARE_4WSC_V8_600S_DIRECT_XY_YAW_STAGE_'\n"
    for dest in (c/'stages.py',h/'stages.py'): dest.write_text(stages)
    contract=(FW/'flash_bundle/fourwsc_wire/contract.py').read_text()
    contract=contract.replace('MOTION_LINEAR_LIMITS={\'4WSC\':.70}',"MOTION_LINEAR_LIMITS={'4WSC':1.20}")
    contract=contract.replace('class ProtocolError(ValueError):\n    """Offline decoder only; no live transport is constructed."""\n    pass','from mecanum_vicon_control.transport_protocol import ProtocolError')
    (h/'stage0_contract.py').write_text(contract)
    for name in ('stage0_session.py',):
        p=h/name;p.write_text(p.read_text().replace('0x7B00','0x7C00').replace('0x7B80','0x7C80'))
    edit(h/'startup_idle.py',"s.get('output_authority_version')!=4","s.get('output_authority_version')!=8")
    edit(h/'startup_clock.py',"def __init__(self, sock, ip, session_id, trace_path=''):","def __init__(self, sock, ip, session_id, trace_path='', stage=0):")
    edit(h/'startup_clock.py','        self.socket, self.ip, self.session_id = sock, ip, session_id',
        '        from .stages import get_stage\n        get_stage(stage); self.stage=stage\n        self.socket, self.ip, self.session_id = sock, ip, session_id')
    edit(h/'startup_clock.py',"if enabled > 1 or clear > 1 or reserved not in (0, 0x7804):",
        "if (enabled > 1 or clear > 1 or reserved not in (0, 0x7C00+self.stage) or\n                    (enabled and (clear or reserved != 0x7C00+self.stage)) or (clear and reserved)):")
    edit(h/'nodes.py',"os.environ.get('WAND_4WSC_RAW_RECEIVE_TRACE', ''))", "os.environ.get('WAND_4WSC_RAW_RECEIVE_TRACE', ''), stage=self.runtime.config.stage)")
    shutil.copyfile(HERE/'feedback_pair.py',h/'feedback_pair.py')
    shutil.copyfile(HERE/'idle_admission.py',c/'idle_admission.py')
    edit(h/'nodes.py',"from .receive_event import ReceiveEventPump", "from .receive_event import ReceiveEventPump\nfrom .feedback_pair import paired_advance")
    edit(h/'nodes.py','if fresh and identity != self.last_feedback_identity:', 'if fresh and paired_advance(identity, self.last_feedback_identity):')
    edit(h/'nodes.py',"            ready=bool(fresh and self.last_mcu_fault_bits == 0 and disabled_motion_status_ready(status)",
        "            native_disabled_zero=bool(fresh and status and disabled_motion_status_ready(status)),\n"
        "            healthy_budget_used=bool(fresh and status and status['started_ms']),\n"
        "            native_fault_bits=self.last_mcu_fault_bits,\n"
        "            ready=bool(fresh and self.last_mcu_fault_bits == 0 and disabled_motion_status_ready(status)")
    edit(c/'auto_prepare.py',"        progress=t.get('command_progress_age_s')",
        "        from .idle_admission import zero_preparation_ready\n        progress=t.get('command_progress_age_s')")
    edit(c/'auto_prepare.py',"and t.get('fresh') is True and t.get('ready') is True", "and t.get('fresh') is True and zero_preparation_ready(t)")
    # Numeric policy is copied from the exact new firmware, not renamed v7.
    p=h/'commissioning.py';s=p.read_text();tail=s[s.index('def runtime_from_environment():'):]
    tail=tail.replace("os.environ.get('WAND_4WSC_MODE', 'shadow')","os.environ.get('WAND_4WSC_MODE', 'hardware')")
    imports=s[:s.index('DOMAIN, NAMESPACE, WORLD =')]
    p.write_text(imports+'from .host_profile import Runtime, DOMAIN, NAMESPACE, WORLD, CONSENT\n\n'+tail)
    for name in ('host_profile.py','authorization.py'):
        shutil.copyfile(HERE/name,h/name)
    # Fixed direct-target controller and source-bound gate are reused. No proxy.
    edit(c/'tracking_config.py','    def inside(self, pose):','    @property\n    def direct_sign_target(self): return True\n\n    def inside(self, pose):')
    edit(c/'tracking_config.py','workspace_radius_enabled=False)','workspace_radius_enabled=False, wand_deadband_m=0.)')
    # Geometry helper runs only against this private renamed package.
    sim=ROOT/'candidates/wand_sim12_20261030'
    (out/'geometry.py').write_text(rename((sim/'geometry12.py').read_text()))
    shutil.copyfile(sim/'sign_geometry.json',out/'sign_geometry.json')
    for name in ('channel.py','control_channel.py','mailbox.py','view_slot.cjs','lifecycle.py','worker.py','server.cjs','browser_operator.py'):
        (out/name).write_text(rename((BASE/name).read_text()))
    shutil.copytree(BASE/'web',out/'web')
    edit(out/'browser_operator.py',"        if not runtime.software_only or runtime.experiment!='DIRECT_SIGN_NOMINAL_NO_C_FIRMWARE':\n            raise ValueError('browser SIM12 has no hardware entry')", "        if runtime.motion_tag!='4WSC' or runtime.purpose!='sign':\n            raise ValueError('4WSC Sign runtime required')")
    edit(out/'browser_operator.py',"        if self.context.get_domain_id()!=191:raise ValueError('private DDS191 only')", "        runtime.guard(self)")
    for name in ('bootstrap.py','node.py','hardware_runtime.py','suspended_runtime.py','idle_probe.py','native_peer.py'):
        shutil.copyfile(HERE/name,out/name)
    p=out/'worker.py'
    edit(p,"choices=('software',)","choices=('hardware','rehearsal')")
    edit(p,'from software_runtime import Rig','from hardware_runtime import Rig')
    edit(p,"ros_initialized=rig is not None,ros_domain=191 if rig else None,", "ros_initialized=bool(rig and rig.ros_initialized_ever),ros_domain=rig.rt.domain if rig and rig.ros_initialized_ever else None,")
    edit(p,"hardware_bridge_constructed=False,hardware_socket_opened=False,physical_motion_started=False,", "hardware_bridge_constructed=bool(rig and rig.bridge_constructed),hardware_socket_opened=bool(rig and rig.physical),physical_motion_started=bool(rig and rig.physical and rig.activation_ever),")
    edit(p,"software_stop_evidence=stop_evidence,", "stop_evidence=stop_evidence,")
    edit(p,"qualification='SOFTWARE ONLY; not physical Sign/car acceptance'", "qualification='4WSC host; rehearsal is loopback/software only; real trials do not qualify higher tiers'")
    edit(p,"emit(dict(kind='ready',mode=args.mode,hardware_entry=False))", "emit(dict(kind='ready',mode=args.mode,hardware_entry=args.mode=='hardware'))")
    edit(p,"last_state=dict(kind='state',mode=args.mode,software_only=True,physical_output=False,", "last_state=dict(kind='state',mode=args.mode,software_only=args.mode=='rehearsal',physical_output=args.mode=='hardware',")
    edit(p,"'trace','gate','session_requested','trial_ended','remaining_s')", "'trace','gate','session_requested','trial_ended','remaining_s','stage','limits','purpose')")
    # The worker's optional shorter test timer must also be what the operator
    # sees. This changes display only, never the fixed 600s supervisor/MCU caps.
    edit(p,"                state['trace']=(state['trace'] or [])[-60:]",
        "                if rig.rt.purpose=='sign':\n"
        "                    active_elapsed=0. if rig.active_since is None else now-rig.active_since\n"
        "                    state['remaining_s']=max(0.,min(state.get('remaining_s') or args.seconds,args.seconds-active_elapsed))\n"
        "                    state['instruction']=(state.get('instruction') or '').replace('600秒',f'{args.seconds:g}秒')\n"
        "                    if state['state']=='ACTIVE':\n"
        "                        state['instruction']=f\"单击跟随中，剩余≤{state['remaining_s']:.1f}秒；松键继续；再次空格、Esc或停止按钮结束\"\n"
        "                state['trace']=(state['trace'] or [])[-60:]")
    # Do not emit untrue software-only badges on a physical interface.
    p=out/'server.cjs'
    edit(p,"let mode='software'", "let mode='hardware'")
    edit(p,"  if(args[i]==='--real-sign')mode='software';", "  if(args[i]==='--hardware')mode='hardware';\n  else if(args[i]==='--rehearsal')mode='rehearsal';")
    edit(p,"if(!Number.isInteger(port)", "if(mode==='hardware'&&synthetic)throw Error('synthetic input forbidden on real hardware');\nif(!Number.isInteger(port)")
    p.write_text(p.read_text().replace("software_only:true,physical_output:false", "software_only:mode==='rehearsal',physical_output:mode==='hardware'")
        .replace("physical_output:false", "physical_output:mode==='hardware'").replace("hardware_entry:false", "hardware_entry:mode==='hardware'")
        .replace("ROS_DOMAIN_ID:'191'", "ROS_DOMAIN_ID:mode==='rehearsal'?'192':'125'")
        .replace('本入口没有实车输出。','仅明确批准的硬件模式可输出；空格前保持失能。'))
    edit(p,"kind:'hello',mode,synthetic,owner:","kind:'hello',mode,synthetic,purpose:process.env.WAND_4WSC_PURPOSE,owner:")
    edit(p,"channel:v.channel,gate:v.gate,", "channel:v.channel,gate:v.gate,stage:v.stage,limits:v.limits,purpose:v.purpose,")
    p=out/'web/app.js';s=p.read_text()
    s=s.replace("v.result.software_stop_evidence", "v.result.stop_evidence")
    s=s.replace('直接Sign · 1:1平移＋旋转 · 直行1.20 / 斜向0.85m/s · 4.00m/s² · 无实车输出', '4WSC · 固定审查档 · 1:1平移＋旋转 · 参数以本轮合同为准')
    s=s.replace('自动测试手势 → ROS软件车 · 不是实时Sign · 无实车输出','本机回环C固件排练 · 合成Sign · 无实车输出')
    s=s.replace('软件模式：自动READY后','本轮合同模式：自动READY后').replace('本轮软件跟随','本轮跟随')
    s=s.replace("mode==='readonly'?'只读软件样例':'ROS软件仿真'", "mode==='hardware'?'真实硬件':'回环软件排练'")
    s=s.replace('软件车 X=', '小车 X=').replace('等待软件后端数据','等待4WSC后端数据')
    s=s.replace("controlStatus=v;latest={...latest,...v};", "controlStatus=v;latest={...latest,...v};\n      if(v.limits)text('mode',`4WSC · 档${v.stage} · ${v.purpose==='suspended'?'悬空按住使能 ≤5秒':'1:1平移＋旋转'} · ${v.limits.speed.toFixed(2)}m/s · ${v.limits.accel.toFixed(2)}m/s² · ${v.limits.yaw.toFixed(2)}rad/s · ${v.limits.yaw_accel.toFixed(2)}rad/s²`);")
    s=s.replace("text('connection','双通道连接中；挑战即时回应；严格上限100ms');", "if(v.purpose==='suspended'){text('mode','4WSC · 单次悬空测试 · 固定审查档 · 最长5秒');text('footer','轻按松开空格解锁→自动准备（不转轮）→READY后按住空格；松键立即停；本窗口不重跑。');}\n      text('connection','双通道连接中；挑战即时回应；严格上限100ms');")
    p.write_text(s)
    p=out/'web/index.html';s=p.read_text().replace('浏览器仿真','4WSC独立实机入口').replace('真实Sign → 1:1直接跟随','4WSC · 真实Sign → 小车')
    s=s.replace('等待连接 · 本入口不支持实车输出','等待自检 · 当前未使能 · 硬件模式须明确审批')
    s=s.replace('蓝：软件小车','蓝：小车反馈').replace('软件小车与目标轨迹','小车与Sign目标轨迹')
    s=s.replace('位置、朝向：等待软件数据','位置、朝向：等待反馈')
    s=s.replace('软件急停（锁存）','紧急停止（软件锁存）')
    s=s.replace('只读与软件仿真均无实车输出。','仅明确审批的硬件模式可输出；准备不转轮。')
    p.write_text(s)
    # Drop old launchers/configs rather than leave accidentally usable v7 paths.
    shutil.rmtree(rt/H/'launch')
    for folder in (rt/H/'config',):
        shutil.rmtree(folder);folder.mkdir()
    for stage in range(10):
        from profile_draft import draft
        save(out/'profiles'/f'stage{stage}.NOT_APPROVED.json',draft(stage))
    save(out/'HOST_MANIFEST.json',dict(identity='4WSC',authority_version=8,diagnostic_payload_bytes=208,
        host_offline_passed=False,physical_qualified=False,firmware_flashed_record=str(ROOT/'docs/wand_firmware_4wsc_2026-10-30/DEPLOYMENT_2026-11-02.json'),
        image_sha256=json.loads((FW/'DELIVERY.json').read_text())['bin_sha256'],
        protected=protected,source_hashes=hashes(HERE),artifact_hashes=hashes(out),
        default_motion_authority=False,hardware_output_during_preparation=False))
    print(out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=OUT)
    generate(p.parse_args().output.resolve())
