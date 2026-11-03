"""Seal only passed software evidence; never claims physical qualification."""
import hashlib
import json
import subprocess
import sys
from entry12 import HERE,REPORT,hashes
from profile12 import protected,save,contract

def main():
    REPORT.mkdir(parents=True,exist_ok=True)
    tests=subprocess.run([sys.executable,'-m','pytest','-q','-p','no:cacheprovider',str(HERE/'test_sim12.py')],capture_output=True,text=True)
    (REPORT/'UNIT_TESTS.log').write_text(tests.stdout+tests.stderr)
    if tests.returncode:raise ValueError('unit tests failed')
    paths=[REPORT/'UNIT_TESTS.log']
    for name in ('large_random_newsign_01','occlusion_newsign_01','operator_gap_newsign_01'):
        path=REPORT/name/'RESULT.json';value=json.loads(path.read_text())
        if value['automated_passed'] is not True or value['errors'] or value['hardware_output']:
            raise ValueError('failed software run '+name)
        if not json.loads((path.parent/'PROTECTED.json').read_text())['unchanged']:raise ValueError('protected drift')
        paths.extend([path,path.parent/'PROTECTED.json'])
    ready=REPORT/'sign_newgeometry_ready_01/RESULT.json'
    if not json.loads(ready.read_text())['ready']:raise ValueError('real Sign read-only qualification missing')
    paths.append(ready)
    save(REPORT/'PREPARED.json',dict(passed=True,sources=hashes(),
        protected_sha256=hashlib.sha256(json.dumps(protected(),sort_keys=True).encode()).hexdigest(),
        evidence={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},contract=contract(),
        tests=dict(unit=20,actual_ros_large_route=True,occlusion_no_auto_resume=True,operator_gap_no_auto_resume=True),
        real_sign_full_flow_accepted=False,physical_qualified=False,original_demo_modified=False))
    print('SIM12_SOFTWARE_PREPARED',flush=True)

if __name__=='__main__':main()
