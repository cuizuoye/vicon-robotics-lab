"""Exercise the real disabled-zero probe against exact new C, dedicated loopback."""
import json
import os
from pathlib import Path
import sys
from build import OUT,RECORD,save
from ground_entry import profile,sha

def main(folder):
    ready=OUT/'host';path,consent=profile(ready,folder,True)
    os.environ.update(WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(path),WAND_PROFILE_APPROVAL=sha(path),WAND_HARDWARE_CONSENT=consent)
    sys.path.insert(0,str(ready));import bootstrap
    from native_peer import Peer
    from idle_probe import run,CONSENT
    p=json.loads(path.read_text());peer=Peer(p['hardware']['command_port'])
    try:
        result=run(p,RECORD,folder/'PROBE',CONSENT,8.,rehearsal=True)
        sample=peer.sample()
        assert result['passed'] and result['fault_bits']==0 and not result['hardware_socket_opened'],result
        assert not peer.permit_ever and not sample['status']['selected'] and not sample['status']['permitted'],sample
        assert not peer.peak_duty and not peer.peak_rpm
        result.update(actual_native_C=True,physical_output=False,never_selected=True,never_permitted=True,peak_duty=0,peak_rpm=0)
        save(folder/'RESULT.json',result);print(json.dumps(result))
    finally:peer.close()
if __name__=='__main__':
    target=Path(sys.argv[1]);target.mkdir();main(target)
