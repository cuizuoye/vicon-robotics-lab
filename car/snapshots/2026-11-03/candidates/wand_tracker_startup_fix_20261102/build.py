"""Independent startup/renderer fix; copies and pins the previous frozen demo."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
BASE=ROOT/'docs/wand_stopfix_tracker_ui_2026-11-02/prepared_v1/host'
OUT=ROOT/'docs/wand_tracker_startup_fix_2026-11-02/prepared_v1'
PARENT=ROOT/'candidates/wand_stopfix_tracker_ui_20261102'
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
def replace(value,old,new):
    assert value.count(old)==1,old[:100]
    return value.replace(old,new)

def client_source():
    v=(BASE/'web/app.js').read_text()
    v=replace(v,"let resourcesReady=false;", "let resourcesReady=false,pageReady=false,launcherUrl=null;\nconst pageWarmup=new PageWarmup();")
    v=replace(v,"if(!resourcesReady||ws||ended||document.hidden||!document.hasFocus())return;", "if(!resourcesReady||!pageReady||ws||ended||document.hidden||!document.hasFocus())return;")
    v=replace(v,"mode=v.mode;connectView(v.owner);", """mode=v.mode;connectView(v.owner);
      if(v.launcher_url){
        try{const u=new URL(v.launcher_url);if(u.protocol==='http:'&&u.hostname==='127.0.0.1'&&u.port&&u.hash)launcherUrl=u.href;}catch{}
      }""")
    v=replace(v,"if(ended)return;ended=true;events=[];space=false;", "if(ended)return;ended=true;events=[];space=false;\n  if(launcherUrl)$('restart').hidden=false;")
    v=replace(v,"resultSeen=ended=true;", "resultSeen=ended=true;if(launcherUrl)$('restart').hidden=false;")
    v=replace(v,"if(v.channel?.fault){ended=true;", "if(v.channel?.fault){ended=true;if(launcherUrl)$('restart').hidden=false;")
    v=replace(v,"$('stop').onclick=()=>stop('STOP_BUTTON');$('estop').onclick=()=>stop('ESTOP');", """$('stop').onclick=()=>stop('STOP_BUTTON');$('estop').onclick=()=>stop('ESTOP');
$('restart').onclick=()=>{if(ended&&launcherUrl)location.replace(launcherUrl);};""")
    # Cache only the fixed grid, axes and labels. Moving poses/trace stay live.
    start=v.index("    ctx.fillStyle = '#242b32'")
    end=v.index('    ctx.save(); ctx.beginPath(); ctx.rect(0, 0, width, height); ctx.clip();',start)
    bg=v[start:end]
    v=v[:start]+"""    function background(ctx){
"""+bg+"""    }
    const key=[width,height,s,v.major].join(':');
    if(typeof document!=='undefined'&&document.createElement){
      if(camera.backgroundKey!==key){
        const layer=document.createElement('canvas');layer.width=width;layer.height=height;
        background(layer.getContext('2d'));camera.background=layer;camera.backgroundKey=key;
      }
      ctx.drawImage(camera.background,0,0);
    }else background(ctx);
"""+v[end:]
    v=replace(v,"const trackerCamera={halfY:3};", "const trackerCamera={halfY:3};let fontsPrimed=false,lastText=0;")
    v=replace(v,"requestAnimationFrame(render);if(now-lastDraw<33||!latest)return;lastDraw=now;", """requestAnimationFrame(render);if(now-lastDraw<33||(!latest&&pageReady))return;lastDraw=now;
  const drawStarted=performance.now();
  if(!pageReady){
    const canvas=$('map'),box=canvas.getBoundingClientRect();canvas.width=Math.round(box.width);canvas.height=Math.round(box.height);
    if(!resourcesReady||document.hidden||!document.hasFocus()){
      text('instruction','请点击页面；画布预热后连接，尚未使能。');return;
    }
    if(!fontsPrimed){
      // Prime the actual Chinese sprite fonts offscreen too, not just DOM fonts.
      const layer=document.createElement('canvas');layer.width=canvas.width;layer.height=canvas.height;
      TrackerWorldView.draw(layer.getContext('2d'),layer.width,layer.height,{x:0,y:0,yaw:0},{x:1,y:1,yaw:0},[],{});fontsPrimed=true;
    }
    TrackerWorldView.draw(canvas.getContext('2d'),canvas.width,canvas.height,null,null,[],trackerCamera);
    pageReady=pageWarmup.frame(performance.now(),performance.now()-drawStarted);
    text('instruction',pageReady?'页面已就绪，正在连接；尚未使能。':'正在预热画布与字体；尚未使能。');
    if(pageReady)connect();return;
  }
  if(now-lastText>=100){lastText=now;""")
    v=replace(v,"  const c=latest.car,g=latest.goal,trace=latest.trace||[];", "  const c=latest.car;")
    v=replace(v,"  const canvas=$('map'),box=canvas.getBoundingClientRect(),w=box.width,h=box.height;", "  }\n  const c=latest.car,g=latest.goal,trace=latest.trace||[];\n  const canvas=$('map'),box=canvas.getBoundingClientRect(),w=box.width,h=box.height;")
    return v

def prepare_ground_entry():
    v=(PARENT/'ground_entry.py').read_text()
    v=replace(v,"        if a.launch:\n            from idle_probe", "        if a.launch:\n            print('STARTUP_PHASE '+json.dumps(dict(phase='DISABLED_ZERO',message='正在检查小车时钟、故障位和失能零速通信')),flush=True)\n            from idle_probe")
    v=replace(v,"if not idle['passed']:raise ValueError('disabled zero/clock/fault check failed; browser remains closed')", "if not idle['passed']:raise ValueError('disabled zero/clock/fault check failed: '+json.dumps(idle,ensure_ascii=False))")
    v=replace(v,"            receiver.start(folder)", "            print('STARTUP_PHASE '+json.dumps(dict(phase='VICON',message='小车失能检查通过；正在连接Vicon接收器')),flush=True)\n            receiver.start(folder)")
    v=replace(v,"        args=['node',", "        print('STARTUP_PHASE '+json.dumps(dict(phase='WARMUP',message='正在检查新鲜定位、反馈与末级安全门；尚未使能')),flush=True)\n        args=['node',")
    v=replace(v,"if not opened:raise ValueError('preflight failed; browser not opened. Logs: '+str(folder))", "if not opened:raise ValueError('定位/时钟/安全门预检失败；'+(folder/'server_error.log').read_text()[-1600:]+'；日志：'+str(folder))")
    return v

def main():
    if OUT.exists():raise ValueError('never overwrite a prepared snapshot')
    m=json.loads((BASE/'HOST_MANIFEST.json').read_text())
    assert m['host_offline_passed'] and m['hardware_launch_enabled']
    pins={str(BASE/'HOST_MANIFEST.json'):sha(BASE/'HOST_MANIFEST.json')}
    for group in ('protected','source_hashes','artifact_hashes','evidence'):pins.update(m[group])
    for path,digest in pins.items():assert sha(path)==digest,path
    ready=OUT/'host'
    shutil.copytree(BASE,ready,ignore=shutil.ignore_patterns('sessions','HOST_MANIFEST.json','__pycache__','*.pyc'))
    for name in ('deployment_record.py','test_admission.py'):
        assert not (HERE/name).exists()
        value=(PARENT/name).read_text().replace('docs/wand_stopfix_tracker_ui_2026-11-02/prepared_v1/host','docs/wand_tracker_startup_fix_2026-11-02/prepared_v1/host')
        (HERE/name).write_text(value)
    (HERE/'ground_entry.py').write_text(prepare_ground_entry())
    shutil.copyfile(HERE/'deployment_record.py',ready/'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/deployment_record.py')
    shutil.copyfile(HERE/'page_warmup.js',ready/'web/page_warmup.js')
    (ready/'web/app.js').write_text(client_source())
    v=(ready/'web/index.html').read_text()
    v=replace(v,'<script src="/app.js" defer>', '<script src="/page_warmup.js" defer></script><script src="/app.js" defer>')
    v=replace(v,'<fieldset id="simulation"', '<button id="restart" hidden>返回启动检查</button>\n<fieldset id="simulation"')
    (ready/'web/index.html').write_text(v)
    v=(ready/'server.cjs').read_text()
    v=replace(v,"'/focus_evidence.js':['focus_evidence.js','text/javascript']", "'/page_warmup.js':['page_warmup.js','text/javascript'],'/focus_evidence.js':['focus_evidence.js','text/javascript']")
    v=replace(v,"send(ws,{kind:'hello',mode,synthetic,purpose:", "send(ws,{kind:'hello',mode,synthetic,launcher_url:process.env.WAND_LAUNCHER_URL||null,purpose:")
    (ready/'server.cjs').write_text(v)
    changed={str(p.relative_to(ready)) for p in ready.rglob('*') if p.is_file() and (not (BASE/p.relative_to(ready)).exists() or sha(p)!=sha(BASE/p.relative_to(ready)))}
    m.update(host_offline_passed=False,hardware_launch_enabled=False,protected=pins,source_hashes=hashes(HERE),artifact_hashes=hashes(ready),evidence={},
             inherited_runtime_hashes={p:d for p,d in m['inherited_runtime_hashes'].items() if (ready/p).exists() and sha(ready/p)==d},
             inherited_parent_runtime_hashes={str(Path(p).relative_to(BASE)):d for p,d in m['artifact_hashes'].items() if Path(p).is_relative_to(BASE) and str(Path(p).relative_to(BASE)) not in changed},
             changed_runtime_files=sorted(changed),presentation_revision='TRACKER_STARTUP_FIX_V1')
    m.pop('qualification',None);save(ready/'HOST_MANIFEST.json',m)
    save(OUT/'BUILD.json',dict(passed=True,changed_runtime_files=sorted(changed),physical_output=False,firmware_changed=False,network_settings_changed=False))
    print(ready)

if __name__=='__main__':main()
