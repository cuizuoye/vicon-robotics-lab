"""Serial loopback-only late-stop and audit/non-authority integration evidence."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from build import HERE,OUT,sha,save
from ground_entry import profile
def main():
    folder=Path(tempfile.mkdtemp(prefix='extra_',dir=OUT));env=dict(os.environ,PYTHONPATH=str(HERE)+os.pathsep+os.environ.get('PYTHONPATH',''),PYTHONDONTWRITEBYTECODE='1')
    def run(name,cmd,environ):
        p=subprocess.run(cmd,env=environ,text=True,capture_output=True,timeout=150)
        (folder/(name+'.stdout.log')).write_text(p.stdout);(folder/(name+'.stderr.log')).write_text(p.stderr)
        print(name+': '+str(p.returncode),flush=True)
        if p.returncode:raise ValueError(p.stdout+p.stderr)
        return p.stdout
    text=run('late_stop_ros',[sys.executable,'-B',str(HERE/'test_late_stop_ros.py')],env)
    late=json.loads(next(line for line in reversed(text.splitlines()) if line.startswith('{')))
    save(folder/'LATE_POINTER.json',late)
    audit=folder/'audit';audit.mkdir();p,consent=profile(OUT/'host',audit,True)
    browser_env=dict(env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(p),WAND_PROFILE_APPROVAL=sha(p),
        WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    run('audit_no_authority',['node',str(HERE/'test_diagnostic_authority.cjs'),str(OUT/'host'),str(audit/'RESULT.json'),'proof_timeout','8'],browser_env)
    result=dict(passed=True,physical_output=False,late_result=str(Path(late['folder'])/'RESULT.json'),audit_result=str(audit/'RESULT.json'),folder=str(folder))
    save(folder/'RESULT.json',result);print('EXTRA_RESULT '+str(folder/'RESULT.json'),flush=True)
if __name__=='__main__':main()
