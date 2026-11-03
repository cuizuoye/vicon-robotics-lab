#!/usr/bin/env python3
"""Isolated fast scheduling candidate, pin old entries and all controller bytes."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
PARENT=ROOT/'docs/wand_browser_ssh_2026-10-27/prepared'
PROBE=ROOT/'docs/wand_browser_probe_2026-10-27/prepared'
OUT=ROOT/'docs/wand_browser_fast_2026-10-28/prepared'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(d):return {str(p):sha(p) for p in d.rglob('*') if p.is_file() and not any(
    x in ('__pycache__','.pytest_cache') or x.endswith('.egg-info') for x in p.parts)}
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def edit(p,old,new):
    s=p.read_text()
    if s.count(old)!=1:raise ValueError('Parent layout changed: '+str(p)+' / '+old)
    p.write_text(s.replace(old,new))

def render(out):
    for n in ('control_channel.py','mailbox.py','server.cjs','view_slot.cjs','check.py'):
        shutil.copyfile(HERE/n,out/n)
    for n in ('channel.py','browser_operator.py','software_runtime.py','worker.py'):
        shutil.copyfile(PARENT/n,out/n)
    operator=out/'browser_operator.py'
    edit(operator,'        if self.model.held:\n            if proof[\'generation\']==self.last_published_generation:return',
         "        # Coalesce unchanged intent at50Hz, including idle/released leases.\n"
         "        # Revocation edges bypass this rate limit; no proof is retimestamped.\n"
         "        held=bool(self.model.held)\n"
         "        if held==getattr(self,'last_published_held',None) and now-getattr(self,'last_publish_steady',-float('inf'))<.020:return\n"
         "        if self.model.held:\n            if proof['generation']==self.last_published_generation:return")
    edit(operator,'        self.lease_pub.publish(String(data=json.dumps(asdict(lease))))',
         "        self.last_published_held=bool(self.model.held);self.last_publish_steady=now\n"
         "        self.lease_pub.publish(String(data=json.dumps(asdict(lease))))")
    fixture=out/'software_runtime.py'
    edit(fixture,"        self.source.create_timer(.02,self.fixture)",
         "        self.source.create_timer(.02,self.fixture)\n"
         "        self.callback_stats={}\n"
         "        for node in (self.source,self.supervisor,self.gate):\n"
         "            for timer in node.timers:\n"
         "                cb=timer.callback;key=node.get_name()+'.'+cb.__name__\n"
         "                record=self.callback_stats.setdefault(key,dict(count=0,max_gap=0.,max_duration=0.,last=None))\n"
         "                def measured(cb=cb,record=record):\n"
         "                    now=time.monotonic()\n"
         "                    if record['last'] is not None:record['max_gap']=max(record['max_gap'],now-record['last'])\n"
         "                    record['last']=now;record['count']+=1\n"
         "                    try:return cb()\n"
         "                    finally:record['max_duration']=max(record['max_duration'],time.monotonic()-now)\n"
         "                timer.callback=measured")
    old="""        start=time.monotonic()
        for _ in range(32):
            self.executor.spin_once(timeout_sec=0.)
            if time.monotonic()-start>.003:break
        self.operator.step(now,self.setpoint)"""
    edit(fixture,old,
         "        start=time.monotonic()\n"
         "        for _ in range(64):\n"
         "            self.executor.spin_once(timeout_sec=0.)\n"
         "            if time.monotonic()-start>.008:break\n"
         "        self.operator.step(time.monotonic(),self.setpoint)")
    if not (out/'web').exists():shutil.copytree(PARENT/'web',out/'web')
    else:
        for n in ('index.html','style.css','app.js'):shutil.copyfile(PARENT/'web'/n,out/'web'/n)
    shutil.copyfile(HERE/'input_guard.js',out/'web/input_guard.js')
    renderer=(PARENT/'web/app.js').read_text().split('function render(now){',1)[1]
    app=out/'web/app.js';app.write_text((HERE/'client.js').read_text()+'\nfunction render(now){'+renderer)
    edit(app,"text('timing',`浏览器主线程最大间隔 ${(clientMaxGap).toFixed(1)}ms；挑战证据≤100ms`);",
         "text('timing',`UI周期最大 ${guard.maxGap.toFixed(1)}ms；回应调度最大 ${maxReplyDispatch.toFixed(1)}ms；严格100ms`);")
    edit(out/'web/index.html','<script src="/app.js" defer>',
         '<script src="/input_guard.js" defer></script><script src="/app.js" defer>')
    edit(out/'web/index.html','Sign跟随 · Windows本地绘图','低延迟跟随 · 控制/画面分流')
    worker=out/'worker.py'
    edit(worker,"choices=('readonly','software')","choices=('readonly',)")
    edit(worker,'from channel import Channel','from control_channel import Channel\nfrom mailbox import Mailbox')
    edit(worker,'outgoing=queue.Queue(maxsize=128)','outgoing=Mailbox(maxsize=128)')
    edit(worker,'if now-challenge_at>=.018:', 'if now-challenge_at>=.008:')
    edit(worker,"elif kind=='disconnect':channel.disconnect(connection)",
         "elif kind=='disconnect':\n"
         "                    if connection==channel.owner and packet.get('reason') in ('VIEW_ACK_TIMEOUT_100MS','VIEW_DISCONNECTED','CONTROL_DISCONNECTED'):\n"
         "                        channel.stop(packet['reason'])\n"
         "                    else:channel.disconnect(connection)")
    edit(worker,'incoming.put_nowait(json.loads(line))',
         "value=json.loads(line);value['_reader_received_steady']=time.monotonic();incoming.put_nowait(value)")
    edit(worker,"                    line=json.dumps(value,allow_nan=False,ensure_ascii=False,separators=(',',':'))",
         "                    if value['kind']=='challenge' and time.monotonic()-value['issued_steady']>.100:\n"
         "                        outgoing.task_done();continue\n"
         "                    line=json.dumps(value,allow_nan=False,ensure_ascii=False,separators=(',',':'))")
    edit(worker,"if value['kind'] in ('audit','result'):","if value['kind'] in ('audit','sample','model_diag','result'):")
    edit(worker,"if value['kind']!='audit':","if value['kind'] not in ('audit','sample','model_diag'):")
    edit(worker,'complete=False;report=None;start=time.monotonic();last_state=None',
         'complete=False;report=None;start=time.monotonic();last_state=None;samples=deque(maxlen=6000);first_failure=None;last_model_diag=None')
    edit(worker,'tick_start=time.monotonic();channel.tick(tick_start)',
         'tick_start=time.monotonic();channel.begin_tick(tick_start)')
    edit(worker,"elif kind=='wire':channel.receive(connection,packet.get('data'),tick_start)",
         "elif kind=='wire':\n"
         "                    now=time.monotonic();data=packet.get('data');nonce=data.get('nonce') if isinstance(data,dict) else None\n"
         "                    issued=channel.nonces.get(nonce) if isinstance(nonce,str) else None\n"
         "                    accepted=channel.receive(connection,data,now)\n"
         "                    sample=dict(at=now,accepted=accepted,fault=channel.fault,\n"
         "                        age_s=None if issued is None else now-issued,\n"
         "                        reader_gap_s=now-packet.get('_reader_received_steady',now),\n"
         "                        events=data.get('events') if isinstance(data,dict) else None,\n"
         "                        node_receive_s=packet.get('node_received_steady'),node_event_gap_s=packet.get('node_event_gap_max_s'))\n"
         "                    samples.append(sample);emit(dict(kind='sample',**sample))")
    edit(worker,'            if rig:rig.step(time.monotonic())',
         "            channel.finish_tick(time.monotonic())\n"
         "            if channel.fault and first_failure is None:\n"
         "                first_failure=dict(at=time.monotonic(),reason=channel.fault,\n"
         "                    proof_age_s=None if channel.proof is None else time.monotonic()-channel.proof['issued'])\n"
         "            if rig:\n"
         "                rig.step(time.monotonic());model=rig.operator.model;s=model.status\n"
         "                diagnostic=(s.get('state'),s.get('reason'),model.gate.get('fault'),model.held,model.key_down,model.click_pending,model.blocked,model.notice)\n"
         "                if diagnostic!=last_model_diag:\n"
         "                    emit(dict(kind='model_diag',at=time.monotonic(),state=diagnostic,first_stop=s.get('first_stop_reason'),events=model.events[-10:],callbacks=rig.callback_stats))\n"
         "                    last_model_diag=diagnostic")
    edit(worker,"            else:state=rig.snapshot(now)",
         "            else:\n"
         "                full=rig.snapshot(now)\n"
         "                state={k:full.get(k) for k in ('state','reason','instruction','prepare_phase','car','wand','goal','reference',\n"
         "                    'trace','gate','session_requested','trial_ended','remaining_s')}\n"
         "                state['trace']=(state['trace'] or [])[-60:]\n"
         "                state['gate']={k:state['gate'].get(k) for k in ('fault','hard_fault','output_zero','speed_mps')}")
    edit(worker,'time.sleep(max(0.,.010-(time.monotonic()-tick_start)))',
         'time.sleep(max(0.,.005-(time.monotonic()-tick_start)))')
    edit(worker,"            qualification='SOFTWARE ONLY; not physical Sign/car acceptance',last_state=last_state)",
         "            qualification='SOFTWARE ONLY; not physical Sign/car acceptance',last_state=last_state,\n"
         "            first_failure=first_failure,view_coalesced=outgoing.coalesced,max_pending_views=outgoing.max_pending_views,\n"
         "            reply_dispatch='MAIN_THREAD_CHALLENGE_CALLBACK; independent periodic guard100ms',\n"
         "            input_samples=len(samples))")
    edit(worker,"        (args.session/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\\n')",
         "        result_tmp=args.session/'result.json.tmp'\n"
         "        result_tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\\n')\n"
         "        result_tmp.replace(args.session/'result.json')")

def generate(out):
    spec=importlib.util.spec_from_file_location('fast_parent_check',PARENT/'check.py')
    c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c);c.check(PARENT)
    m=json.loads((PARENT/'build_manifest.json').read_text())
    protected={str(PARENT/'build_manifest.json'):sha(PARENT/'build_manifest.json')}
    for group in ('protected_hashes','source_hashes','artifact_hashes'):protected.update(m[group])
    pm=json.loads((PROBE/'build_manifest.json').read_text())
    protected[str(PROBE/'build_manifest.json')]=sha(PROBE/'build_manifest.json')
    for group in ('protected_hashes','source_hashes','artifact_hashes'):protected.update(pm[group])
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    shutil.copytree(m['runtime_src'],out/'runtime/src',ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    write(out/'build_manifest.json',dict(schema=1,browser_fast_passed=False,hardware_entry=False,
        allowed_modes=['readonly'],software_entry_enabled=False,controller_changes=[],runtime_src=str(out/'runtime/src'),
        proof_limit_ms=100,control_gap_ms=100,view_gap_ms=100,operator_lease_ms=150,
        pose_limit_ms=200,mcu_feedback_limit_ms=300,max_active_s=120,
        protected_hashes=protected,source_hashes={},artifact_hashes={}))
    refresh(out)

def refresh(out):
    m=json.loads((out/'build_manifest.json').read_text())
    if m['browser_fast_passed']:raise ValueError('Sealed candidate cannot be refreshed')
    render(out)
    m['allowed_modes']=['readonly'];m['software_entry_enabled']=False
    changed=[str(p.relative_to(out/'runtime/src')) for p in (out/'runtime/src').rglob('*') if p.is_file()
        and '__pycache__' not in p.parts and sha(p)!=sha(PARENT/'runtime/src'/p.relative_to(out/'runtime/src'))]
    if changed:raise ValueError('Unexpected controller change '+str(changed))
    m['artifact_hashes']={str(out/n):sha(out/n) for n in ('channel.py','control_channel.py','mailbox.py',
        'server.cjs','view_slot.cjs','browser_operator.py','software_runtime.py','worker.py','check.py')}
    m['artifact_hashes'].update(hashes(out/'runtime'));m['artifact_hashes'].update(hashes(out/'web'))
    m['source_hashes']=hashes(HERE);tool=ROOT/'tools/start_car_tracking_browser_fast.sh';m['source_hashes'][str(tool)]=sha(tool)
    write(out/'build_manifest.json',m)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=OUT);ap.add_argument('--refresh-draft',action='store_true');a=ap.parse_args()
    if a.refresh_draft:refresh(a.output.resolve())
    else:generate(a.output.resolve())
    print(a.output.resolve())
