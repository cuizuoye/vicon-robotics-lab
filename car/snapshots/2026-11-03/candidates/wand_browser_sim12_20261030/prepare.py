"""Generate an independent browser/SIM12 adapter without editing sealed sources."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
PARENT=ROOT/'docs/wand_browser_fast_2026-10-28/prepared'
OUT=ROOT/'docs/wand_browser_sim12_2026-10-30/draft'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,text):path.write_text(text)
def replace(text,old,new,count=1):
    if text.count(old)!=count:raise ValueError('unexpected sealed layout: '+old[:100])
    return text.replace(old,new)


def generate(out=OUT):
    global OUT
    OUT=out.resolve()
    spec=importlib.util.spec_from_file_location('old_fast_checker',PARENT/'check.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);module.check(PARENT)
    OUT.mkdir(parents=True,exist_ok=False);(OUT/'web').mkdir()
    for name in ('channel.py','control_channel.py','mailbox.py','view_slot.cjs'):
        shutil.copyfile(PARENT/name,OUT/name)
    for source,target in (('browser_operator_source.py','browser_operator.py'),('runtime.py','software_runtime.py')):
        shutil.copyfile(HERE/source,OUT/target)
    shutil.copyfile(HERE/'lifecycle.py',OUT/'lifecycle.py')
    server=(PARENT/'server.cjs').read_text()
    server=replace(server,"let mode='readonly',port=8765,seconds=60;","let mode='software',port=0,seconds=600,synthetic=false,occlusion=false;")
    server=replace(server,"if(args[i]==='--readonly')mode='readonly';\n  else if(args[i]==='--software')throw Error('Private ROS lab has NOT qualified; this release is READONLY ONLY');",
        "if(args[i]==='--real-sign')mode='software';\n  else if(args[i]==='--synthetic')synthetic=true;\n  else if(args[i]==='--test-occlusion'){synthetic=true;occlusion=true;}")
    server=replace(server,'seconds>120','seconds>600')
    server=replace(server,"ROS_DOMAIN_ID:'187'","ROS_DOMAIN_ID:'191',BROWSER_SIM12_SYNTHETIC:synthetic?'1':'0',BROWSER_SIM12_OCCLUSION:occlusion?'1':'0'")
    server=replace(server,"{mode,port:actualPort,session,url,hardware_entry:false}","{mode,synthetic,port:actualPort,session,url,hardware_entry:false}")
    server=replace(server,"{kind:'hello',mode,owner:","{kind:'hello',mode,synthetic,owner:")
    server=replace(server,'process.exit(0);},900).unref();','process.exit(0);},5000).unref();')
    server=replace(server,"process.on('SIGINT',()=>shutdown('SERVER_STOP'));","process.stdout.on('error',()=>shutdown('OUTPUT_PIPE_CLOSED'));\nprocess.on('SIGINT',()=>shutdown('SERVER_STOP'));")
    write(OUT/'server.cjs',server)
    worker=(PARENT/'worker.py').read_text()
    worker=replace(worker,"choices=('readonly',)","choices=('software',)")
    worker=replace(worker,'args.seconds<=120','args.seconds<=600')
    worker=replace(worker,"        if args.mode=='software':\n            for pkg in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):\n                sys.path.insert(0,str(ready/'runtime/src'/pkg))\n            from software_runtime import SoftwareRig\n            rig=SoftwareRig(ready,channel)",
        "        from software_runtime import Rig\n        rig=Rig(channel,args.session,synthetic=os.environ.get('BROWSER_SIM12_SYNTHETIC')=='1',occlusion=os.environ.get('BROWSER_SIM12_OCCLUSION')=='1')")
    worker=replace(worker,'ap=argparse.ArgumentParser();',"import signal\n    ap=argparse.ArgumentParser();")
    worker=replace(worker,"channel=Channel(readonly=args.mode=='readonly');",'channel=Channel(readonly=True);')
    worker=replace(worker,"io_failed=threading.Event();eof=threading.Event();closing=threading.Event()",
        "io_failed=threading.Event();eof=threading.Event();closing=threading.Event()\n"
        "    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:channel.stop('SERVER_TERMINATED'))")
    # Channel(readonly=True) fully validates key-only action schemas before
    # the adapter sees them. No pre-validation iteration over untrusted input.
    # Channel readonly validation forbids setpoints. Explicitly pass key events
    # to the adapter only AFTER the complete challenge reply was accepted.
    worker=replace(worker,'accepted=channel.receive(connection,data,now)',
        "accepted=channel.receive(connection,data,now)\n"
        "                    if accepted:channel.events.extend(data['events'])")
    worker=replace(worker,'events=model.events[-10:],callbacks=rig.callback_stats))','events=model.events[-10:]))')
    worker=replace(worker,'            if channel.started is not None and now-channel.started>=args.seconds:',
        '            if rig.active_since is not None and now-rig.active_since>=args.seconds:')
    worker=replace(worker,'from mailbox import Mailbox','from mailbox import Mailbox\nfrom lifecycle import waiting_expired')
    worker=replace(worker,'if now-state_at>=.045 or channel.stopped:','if now-state_at>=.020 or channel.stopped:')
    worker=replace(worker,'if now-start>180.:','if waiting_expired(start,now,rig.active_since):')
    worker=replace(worker,'ros_domain=187 if rig else None','ros_domain=191 if rig else None')
    worker=replace(worker,'authority_ever_acquired=False if not rig else rig.operator.model.activation_used',
        'authority_ever_acquired=False if not rig else rig.activation_ever')
    # No claim that a complete timed run passes without a fresh stop ACK.
    worker=replace(worker,'passed=complete and not channel.fault,completed=complete',
        "passed=bool(complete and not channel.fault and stop_evidence and stop_evidence['stop_confirmed']),completed=complete")
    write(OUT/'worker.py',worker)
    for name in ('index.html','style.css','input_guard.js'):shutil.copyfile(PARENT/'web'/name,OUT/'web'/name)
    app=(PARENT/'web/app.js').read_text()
    app=replace(app,"$('simulation').hidden=mode!=='software';$('estop').hidden=mode!=='software';", "$('simulation').hidden=true;$('estop').hidden=false;")
    app=replace(app,'私有ROS软件跟随 · 平移＋旋转1:1 · 无实车输出','SIM12 · 1:1平移＋旋转 · 直行1.20 / 斜向0.85m/s · 4.00m/s² · 无实车输出')
    app=replace(app,"$('simulation').hidden=true;", "if(v.synthetic)text('mode','自动测试手势 → ROS软件车 · 不是实时Sign · 无实车输出');\n      $('simulation').hidden=true;")
    app=replace(app,'now-lastDraw<45','now-lastDraw<33')
    app=replace(app,"function connect(){\n  if(ws||ended||document.hidden||!document.hasFocus())return;",
        "let resourcesReady=false;\nPromise.resolve(document.fonts?.ready).then(()=>{resourcesReady=true;connect();});\n"
        "function connect(){\n  if(!resourcesReady||ws||ended||document.hidden||!document.hasFocus())return;")
    app=replace(app,'dpr=Math.min(2,devicePixelRatio||1)','dpr=1')
    app=replace(app,"text('connection',v.result.passed?'本轮严格只读验收完成，通过':'本轮未通过：'+reason);",
        "text('connection',v.result.passed?'本轮完成并停稳':(reason==='CLIENT_ESC'||reason==='CLIENT_STOP_BUTTON'||reason==='OPERATOR_SECOND_SPACE'?'已手动结束并保存：':'本轮保护结束：')+reason);")
    from finalize_ui import OLD_RESULT_LINE,NEW_RESULT_LINE
    app=replace(app,OLD_RESULT_LINE,NEW_RESULT_LINE)
    write(OUT/'web/app.js',app)
    html=(OUT/'web/index.html').read_text().replace('低延迟跟随 · 控制/画面分流','真实Sign → SIM12浏览器仿真').replace('Sign跟随 · 独立浏览器操作台','SIM12 · 本机浏览器仿真')
    write(OUT/'web/index.html',html)
    manifest=dict(software_only=True,hardware_entry=False,domain=191,controller_modified=False,
        scope='BROWSER_SIM12_EXPERIMENT',qualified=False,
        original_browser_manifest_sha256=sha(PARENT/'build_manifest.json'),
        source_hashes={str(p):sha(p) for p in HERE.iterdir() if p.is_file()},
        artifact_hashes={str(p):sha(p) for p in OUT.rglob('*') if p.is_file()})
    write(OUT/'MANIFEST.json',json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(OUT)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=OUT)
    generate(parser.parse_args().output)
