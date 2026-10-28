#!/usr/bin/env python3
"""No hardware/Vicon. Offline regression + actual local Chrome/private DDS."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time
from prepare import HERE,DEFAULT,ROOT,refresh_draft,write
from check import check

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=DEFAULT);args=ap.parse_args()
    out=args.output.resolve();refresh_draft(out);check(out,False)
    logs=out/'validation';logs.mkdir(exist_ok=True)
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',BROWSER_OUTPUT=str(out),
        READONLY_SECONDS='60',CONTINUOUS_SECONDS='60',ROS_DOMAIN_ID='187',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    env.pop('SOFTWARE_ONLY',None)
    env['PYTHONPATH']=':'.join([str(HERE),str(out/'runtime/src/mecanum_wand_4ws8_control'),
        str(out/'runtime/src/mecanum_wand_4ws8_hardware'),env.get('PYTHONPATH','')])
    def run(name,cmd):
        p=subprocess.run(cmd,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=220)
        (logs/(name+'.log')).write_text(p.stdout)
        if p.returncode:raise RuntimeError(name+' failed; see '+str(logs/(name+'.log')))
        return p.stdout
    unit=run('unit',['/usr/bin/python3','-m','pytest','-q','-p','no:cacheprovider',str(HERE/'test_channel.py')])
    print(unit,flush=True)
    network=run('network',['node',str(HERE/'test_network.cjs')])
    net=json.loads(network.split('NETWORK_TEST_RESULT ')[1].split('\n')[0]);assert net['passed'];print(network,flush=True)
    browser=run('browser',['node',str(HERE/'test_browser.cjs')])
    b=json.loads(browser.split('BROWSER_TEST_RESULT ')[1].split('\n')[0]);assert b['passed']
    cases=[]
    for c in b['cases']:
        assert not c['hardware_socket_opened'] and not c['physical_motion_started']
        cases.append({k:c[k] for k in ('case','mode','reason','elapsed_s','max_client_gap_s',
            'max_control_gap_s','max_proof_age_s','ros_initialized','authority_ever_acquired','software_stop_evidence')})
    result=dict(passed=True,unit_tests=int(re.search(r'(\d+) passed',unit)[1]),network_cases=net['cases'],
        actual_browser_cases=cases,parent_runtime_bytes_unchanged=True,hardware_entry=False,
        firmware_flashed=False,tenda_changed=False,car_started=False,real_vicon_connected=False,
        windows_remote_acceptance='PENDING user 60s readonly over actual SSH; Linux Chrome is NOT Windows evidence',
        prepared_utc_s=time.time())
    write(out/'PREPARATION_RESULT.json',result)
    check(out,False)
    m=json.loads((out/'build_manifest.json').read_text());m['browser_ssh_passed']=True
    write(out/'build_manifest.json',m);print(json.dumps(check(out)),flush=True)
    print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
