#!/usr/bin/env python3
"""Offline release: real local browser/private ROS; never physical Sign/car."""
import json
import os
from pathlib import Path
import re
import subprocess
from prepare import HERE,ROOT,OUT,refresh,write
from check import check

def main():
    refresh(OUT);check(OUT,True)
    logs=OUT/'validation';logs.mkdir(exist_ok=True)
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',FAST_OUTPUT=str(OUT),BROWSER_OUTPUT=str(OUT),
        ROS_DOMAIN_ID='187',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',READONLY_SECONDS='4',CONTINUOUS_SECONDS='60')
    env.pop('SOFTWARE_ONLY',None)
    env['PYTHONPATH']=':'.join([str(OUT),str(OUT/'runtime/src/mecanum_wand_4ws8_control'),
        str(OUT/'runtime/src/mecanum_wand_4ws8_hardware'),env.get('PYTHONPATH','')])
    def run(name,command,timeout=220):
        r=subprocess.run(command,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout)
        (logs/(name+'.log')).write_text(r.stdout)
        if r.returncode:raise RuntimeError(name+' failed; '+str(logs/(name+'.log')))
        return r.stdout
    bootstrap=("import os,sys;from pathlib import Path;root=Path(os.environ['FAST_OUTPUT']);sys.path.insert(0,str(root));"
        "import channel,browser_operator,control_channel,mailbox,pytest;"
        "assert all(Path(m.__file__).parent==root for m in (channel,browser_operator,control_channel,mailbox));"
        "print('UNIT_SOURCE '+str(browser_operator.__file__),flush=True);"
        "raise SystemExit(pytest.main(sys.argv[1:]))")
    unit=run('unit',['/usr/bin/python3','-c',bootstrap,'-q','--import-mode=importlib','-p','no:cacheprovider',
        str(ROOT/'candidates/wand_browser_ssh/test_channel.py'),str(HERE/'test_fast.py')])
    print(unit,flush=True)
    scheduling=json.loads(run('scheduling',['node',str(HERE/'test_scheduling.cjs')]).split('SCHEDULING_TEST ')[1].split('\n')[0])
    network=json.loads(run('network',['node',str(HERE/'test_network.cjs')]).split('NETWORK_TEST ')[1].split('\n')[0])
    print('Scheduling and real-socket regressions passed',flush=True)
    metrics=json.loads(run('metrics',['node',str(HERE/'test_metrics.cjs')]).split('METRICS_TEST ')[1].split('\n')[0])
    print('Actual Chrome60s readonly passed',flush=True)
    browser=json.loads(run('browser',['node',str(HERE/'test_browser_safety.cjs')]).split('BROWSER_SAFETY_TEST ')[1].split('\n')[0])
    assert all(v['passed'] for v in (scheduling,network,metrics,browser))
    # Preserve failed lab attempts; do NOT call an observer pass a ROS pass.
    lab=[]
    for p in sorted((OUT/'sessions').glob('fast_software_*/result.json')):
        c=json.loads(p.read_text());lab.append(dict(path=str(p),reason=c['reason'],qualification='NOT QUALIFIED; lab only'))
    write(OUT/'ROS_LAB_NOT_QUALIFIED.json',lab)
    result=dict(passed=True,unit_tests=int(re.search(r'(\d+) passed',unit)[1]),
        scheduling=scheduling,network=network,metrics=metrics,
        actual_browser_cases=browser['cases'],
        original_entries_and_runtime_unchanged=True,controller_changes=[],proof_limit_ms=100,
        actual_network_over100ms_still_fails=True,firmware_flashed=False,tenda_changed=False,car_started=False,
        modes=['readonly'],hardware_entry=False,ros_software_qualified=False,ros_software_entry_blocked=True,
        ros_lab_attempts=str(OUT/'ROS_LAB_NOT_QUALIFIED.json'),
        windows_remote_acceptance='PENDING user60s strict readonly; local Chrome/software proof does not qualify Windows or real car')
    write(OUT/'PREPARATION_RESULT.json',result);check(OUT,True)
    m=json.loads((OUT/'build_manifest.json').read_text());m['browser_fast_passed']=True;write(OUT/'build_manifest.json',m)
    print(json.dumps(check(OUT)),flush=True);print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
