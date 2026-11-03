"""Fresh actual Sign qualification using only this SIM's rebuilt definition."""
import argparse
import json
from pathlib import Path
from profile12 import protected,save,contract
from geometry12 import install_geometry
from reader12 import Reader
from live_sign import ensure_receiver,qualify_reader
from common import diagnostic as d

def run(out):
    out.mkdir(parents=True,exist_ok=False);before=protected();receiver=reader=None
    try:
        install_geometry();receiver=ensure_receiver(out);reader=Reader();reader.start()
        result=dict(qualify_reader(reader,8.),contract=contract());save(out/'RESULT.json',result)
        print(json.dumps(result,ensure_ascii=False),flush=True);return result['ready']
    finally:
        if reader is not None:reader.close()
        if receiver is not None:d.terminate_owned(receiver)
        save(out/'PROTECTED.json',dict(unchanged=before==protected(),hardware_output=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path)
    raise SystemExit(0 if run(p.parse_args().output) else 2)
