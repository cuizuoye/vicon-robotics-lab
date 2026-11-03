"""Seal software experiment, not a replacement or hardware qualification."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');HERE=Path(__file__).resolve().parent
DOCS=ROOT/'docs/wand_adaptive_servo_2026-10-30';READY=DOCS/'prepared_v1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def main():
    p=argparse.ArgumentParser();p.add_argument('--browser-session',type=Path,required=True);a=p.parse_args()
    folder=a.browser_session.resolve();ui=read(folder/'result.json');proto=read(DOCS/'ROS_PROTOCOL.json');bench=read(DOCS/'BENCHMARK_V3.json')
    if (folder.parent!=READY/'sessions' or ui['reason']!='CLIENT_ESC' or not ui['authority_ever_acquired']
            or not ui['ros_initialized'] or ui['ros_domain']!=191 or ui['max_client_gap_s']>.1):
        raise ValueError('actual-browser start/Esc evidence incomplete')
    sessions=[folder,*[Path(v['session']) for v in proto['results']]]
    required={'normal','no_start','proof_timeout','view_disconnect','control_disconnect','second_space','nonce_replay','setpoint_rejected','occlusion'}
    if (not proto['passed'] or Path(proto['root'])!=READY or {v['case'] for v in proto['results']}!=required
            or any(not v['passed'] for v in proto['results']) or not bench['passed']):raise ValueError('regressions incomplete')
    for session in sessions:
        r=read(session/'result.json');stop=r['software_stop_evidence']
        if not stop['stop_confirmed'] or not stop['gate_output_zero'] or r['hardware_socket_opened'] or r['physical_motion_started']:
            raise ValueError('stop/scope failure: '+str(session))
        if not read(session/'PROTECTED.json')['unchanged']:raise ValueError('baseline drift')
    # Prove the new real ROS supervisor actually ran the adaptive class, not
    # merely a changed page title. Every displayed target stays direct.
    normal=Path(next(v['session'] for v in proto['results'] if v['case']=='normal'))
    n=0;budgets=[]
    with (normal/'TRACE.jsonl').open() as stream:
        for line in stream:
            r=json.loads(line);s=r.get('supervisor',{});d=s.get('adaptive_servo')
            if r.get('state')=='ACTIVE' and d:
                if s['goal']!=s['mapped_goal'] or s['reference_limited']:raise ValueError('hidden reference reintroduced')
                budgets.append(d['accel_budget_mps2']);n+=1
    if n<100 or max(budgets)>4.+1e-9 or min(budgets)>=max(budgets):raise ValueError('no dynamic budget evidence')
    result=subprocess.run([sys.executable,'-B','-m','pytest','-q',str(HERE/'test_adaptive.py')],capture_output=True,text=True)
    (DOCS/'UNIT_TESTS.log').write_text(result.stdout+result.stderr)
    if result.returncode:raise ValueError('unit regressions failed')
    m=read(READY/'MANIFEST.json')
    for path,digest in m['artifact_hashes'].items():
        if sha(path)!=digest:raise ValueError('tested artifact drift: '+path)
    m['source_hashes']={str(p):sha(p) for p in HERE.iterdir() if p.is_file()}
    tool=ROOT/'tools/start_sign_browser_adaptive.sh';m['source_hashes'][str(tool)]=sha(tool)
    m['qualified']=True;save(READY/'MANIFEST.json',m)
    evidence=[DOCS/'UNIT_TESTS.log',DOCS/'BENCHMARK_V3.json',DOCS/'ROS_PROTOCOL.json',DOCS/'adaptive_browser_active.jpg']
    for session in sessions:evidence.extend(session/f for f in ('result.json','STOP.json','PROTECTED.json','CONTRACT.json'))
    save(READY/'RELEASE.json',dict(software_qualified=True,hardware_qualified=False,real_sign_human_accepted=False,
        not_promoted_over_fixed_baseline=True,manifest_sha256=sha(READY/'MANIFEST.json'),
        evidence={str(p):sha(p) for p in evidence},browser_session=str(folder),actual_adaptive_ros_samples=n,
        observed_accel_budget_range=[min(budgets),max(budgets)],physical_motion_started=False))
    print(json.dumps(dict(passed=True,software_experiment=True,not_default=True,adaptive_ros_samples=n)))
if __name__=='__main__':main()
