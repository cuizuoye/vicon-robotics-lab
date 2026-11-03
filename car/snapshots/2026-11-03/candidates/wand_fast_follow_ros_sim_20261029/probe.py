"""Bounded subscriptions-only real Sign readiness record; no sim enable."""
import argparse
from pathlib import Path
import time
from dataclasses import asdict
from bootstrap import save
from live_reader import RealReader


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    reader=RealReader();valid=0;last=None;began=time.monotonic()
    try:
        reader.start()
        while time.monotonic()-began<8.:
            now=time.monotonic();ros=reader.node.get_clock().now().nanoseconds*1e-9
            packet=reader(now,ros)
            if packet:valid+=1;last=asdict(packet.pose(now,ros))
            time.sleep(.02)
        result=dict(reader.status(),valid_observations=valid,last_sign5_pose=last,
            physical_car_started=False,command_publishers=[],provenance=reader.provenance)
        save(a.output/'RESULT.json',result);print(result,flush=True)
    finally:reader.close()
