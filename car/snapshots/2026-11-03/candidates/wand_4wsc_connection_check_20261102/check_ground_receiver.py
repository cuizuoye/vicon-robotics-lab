"""Exercise the new entry's real passive receiver lifecycle; no actuator IO."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0,str(ROOT/'candidates/wand_4wsc_ground_20261102'))
from ground_entry import current,check
from build import OUT,save


def main():
    ready=current();check(ready)
    sys.path.insert(0,str(ready));import bootstrap
    bootstrap.install()
    from receiver import Receiver
    folder=Path(tempfile.mkdtemp(prefix='live_receiver_',dir=OUT));receiver=Receiver()
    try:
        receiver.start(folder)
        # A separate process avoids package/global ROS context mixing. Its
        # pinned Sign geometry is byte-identical to the copied ground host's.
        p=subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('vicon_readonly.py')),
            '--output',str(folder/'source'),'--seconds','8'],
            env=dict(os.environ,ROS_DOMAIN_ID='0',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'),
            text=True,capture_output=True,timeout=25)
        (folder/'probe.stdout.log').write_text(p.stdout);(folder/'probe.stderr.log').write_text(p.stderr)
        result=json.loads((folder/'source/RESULT.json').read_text())
        result.update(passed=bool(result['passed'] and p.returncode==0),ground_receiver_lifecycle_used=True,
            ground_ready=str(ready),motor_socket_opened=False,enable_sent=False,motion_started=False)
    finally:receiver.close()
    result['owned_receiver_stopped']=receiver.child is None or receiver.child.poll() is not None
    save(folder/'RESULT.json',result)
    print(json.dumps(dict(result,folder=str(folder)),ensure_ascii=False),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
