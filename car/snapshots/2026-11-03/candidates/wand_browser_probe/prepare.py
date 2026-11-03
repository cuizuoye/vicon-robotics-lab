#!/usr/bin/env python3
"""Generate a NEW observer, preserving the sealed browser/control candidate."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
PARENT=ROOT/'docs/wand_browser_ssh_2026-10-27/prepared'
OUT=ROOT/'docs/wand_browser_probe_2026-10-27/prepared'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(d):return {str(p):sha(p) for p in d.rglob('*') if p.is_file() and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts}
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def edit(p,old,new):
    s=p.read_text()
    if s.count(old)!=1:raise ValueError('Parent layout changed: '+str(p)+' / '+old)
    p.write_text(s.replace(old,new))

def generate(out):
    spec=importlib.util.spec_from_file_location('probe_parent_check',PARENT/'check.py')
    c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c);c.check(PARENT)
    m=json.loads((PARENT/'build_manifest.json').read_text())
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    protected={str(PARENT/'build_manifest.json'):sha(PARENT/'build_manifest.json')}
    for group in ('protected_hashes','source_hashes','artifact_hashes'):protected.update(m[group])
    for n in ('probe.py','worker.py','check.py'):shutil.copyfile(HERE/n,out/n)
    for n in ('server.cjs','channel.py'):shutil.copyfile(PARENT/n,out/n)
    shutil.copytree(PARENT/'web',out/'web')
    server=out/'server.cjs'
    edit(server,"else if(args[i]==='--software')mode='software';",
         "else if(args[i]==='--software')throw Error('Readonly observer has no software/hardware control entry');")
    edit(server,"const sessions=path.join(root,'sessions');",
         "const nodeSteady=()=>Number(process.hrtime.bigint())/1e9;\n"
         "let nodePrevious=nodeSteady(),nodeMaxGap=0;\n"
         "setInterval(()=>{const n=nodeSteady();nodeMaxGap=Math.max(nodeMaxGap,n-nodePrevious);nodePrevious=n;},10).unref();\n"
         "const sessions=path.join(root,'sessions');")
    edit(server,"}else sendToChild({kind:'wire',connection:id,data});",
         "}else sendToChild({kind:'wire',connection:id,data,node_received_steady:nodeSteady(),node_event_gap_max_s:nodeMaxGap});")
    app=out/'web/app.js'
    edit(app,"if(value.kind==='challenge')nonce=value.nonce;",
         "if(value.kind==='challenge'){nonce=value.nonce;ws.send(JSON.stringify({type:'diagnostic_echo',nonce:value.nonce}));} // OBSERVER sample only, never a lease")
    edit(app,"if(gap>100||gap<0){stop('CLIENT_LOOP_GAP');return;}",
         "if(gap<0){stop('CLIENT_CLOCK_INVALID');return;} // OBSERVER ONLY: late intervals are measured, never authorized.")
    edit(app,"  if(document.hidden||!document.hasFocus()){stop('WINDOW_NOT_FOREGROUND');return;}\n",'')
    edit(app,"sequence:++sequence,focused:true,visible:true,space,",
         "sequence:++sequence,focused:document.hasFocus(),visible:!document.hidden,space,")
    edit(app,"window.addEventListener('blur',()=>{if(ws)stop('WINDOW_BLUR');});",
         "// OBSERVER ONLY: focus/visibility are reported, original strict validator stays latched on failure.")
    edit(app,"document.addEventListener('visibilitychange',()=>{if(document.hidden&&ws)stop('WINDOW_HIDDEN');else connect();});",
         "document.addEventListener('visibilitychange',()=>{if(!document.hidden)connect();});")
    edit(app,"'只读验收 · 软件动画 · 不接ROS / Vicon / 小车'",
         "'只读链路诊断 · 软件动画 · 保护失败仍记录，不代表控制可用'")
    edit(app,"'只读：空格仅记录，不启动小车。请保持页面前台直到自动结束。'",
         "'只读诊断：没有运动权限。100ms严格校验失败仍保留；动画继续不代表实车可用。请观察60秒。'")
    edit(app,"text('instruction','结果已自动保存在Linux端，无需复制终端长报文。');",
         "text('instruction',value.result.diagnostic_completed?'只读记录已完成；严格校验：'+(value.result.passed?'通过':'未通过 '+value.result.reason):'只读记录提前结束：'+value.result.reason);")
    edit(app,"text('timing',`浏览器主线程最大间隔 ${(clientMaxGap).toFixed(1)}ms；挑战证据≤100ms`);",
         "const stats=latest.diagnostics?.timings?.echo_roundtrip_s;\n"
         "  text('timing',`浏览器主线程最大间隔 ${clientMaxGap.toFixed(1)}ms；完整回应耗时最大 ${stats?(stats.max*1000).toFixed(1):'--'}ms；严格上限仍100ms`);")
    edit(app,"'连接已断开：已撤能；不会自动恢复，请查看Linux端保存的结果。'",
         "'只读连接已断开：请查看Linux端保存的结果。此入口始终没有运动权限。'")
    edit(out/'web/index.html','Sign跟随 · Windows本地绘图','只读链路诊断 · Windows本地绘图')
    generated=hashes(out);sources=hashes(HERE)
    tool=ROOT/'tools/start_car_tracking_browser_probe.sh';sources[str(tool)]=sha(tool)
    write(out/'build_manifest.json',dict(schema=1,prepared=False,readonly_only=True,
        hardware_entry=False,ros_entry=False,strict_channel_sha=sha(PARENT/'channel.py'),
        authority_can_ever_exist=False,strict_limit_ms=100,strict_failure_reset=False,
        observation_after_failure=True,protected_hashes=protected,artifact_hashes=generated,source_hashes=sources))
    return out

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=OUT)
    print(generate(ap.parse_args().output.resolve()))
