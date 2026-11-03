#!/usr/bin/env python3
"""Validate observation only; no private/public ROS initialization needed."""
import json
import os
import re
import subprocess
from prepare import HERE,ROOT,OUT,hashes,sha,write
from check import check

def main():
    m=json.loads((OUT/'build_manifest.json').read_text())
    if m['prepared']:raise ValueError('Observer already sealed')
    m['source_hashes']=hashes(HERE)
    tool=ROOT/'tools/start_car_tracking_browser_probe.sh';m['source_hashes'][str(tool)]=sha(tool)
    write(OUT/'build_manifest.json',m);check(OUT,True)
    logs=OUT/'validation';logs.mkdir(exist_ok=True)
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(OUT),PROBE_OUTPUT=str(OUT),PROBE_SECONDS='60')
    def run(name,cmd):
        p=subprocess.run(cmd,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=100)
        (logs/(name+'.log')).write_text(p.stdout)
        if p.returncode:raise ValueError(name+' failed: '+str(logs/(name+'.log')))
        return p.stdout
    unit=run('unit',['/usr/bin/python3','-m','pytest','-q','-p','no:cacheprovider',str(HERE/'test_probe.py')])
    print(unit,flush=True)
    b=run('browser',['node',str(HERE/'test_browser.cjs')]);v=json.loads(b.split('PROBE_BROWSER_TEST ')[1].split('\n')[0])
    assert v['passed']
    for c in v['cases']:
        assert not any(c[k] for k in ('ros_initialized','hardware_socket_opened','authority_ever_acquired','physical_motion_started'))
    result=dict(passed=True,unit_tests=int(re.search(r'(\d+) passed',unit)[1]),cases=v['cases'],
        original_browser_entry_unchanged=True,strict_validator_byte_identical=True,
        strict_limit_ms=100,ros_initialized=False,hardware_entry=False,
        windows_remote_diagnosis='PENDING actual Windows 60s observation; local tests are not Windows evidence')
    write(OUT/'PREPARATION_RESULT.json',result);check(OUT,True)
    m=json.loads((OUT/'build_manifest.json').read_text());m['prepared']=True;write(OUT/'build_manifest.json',m)
    print(json.dumps(check(OUT)),flush=True)
    print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
