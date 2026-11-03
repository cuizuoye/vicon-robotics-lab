"""Read-only reuse of sealed4WSB code and existing diagnostic helpers."""
import importlib.util
import json
import os
from pathlib import Path
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HOST=ROOT/'docs/wand_4wsb_host_2026-10-30/prepared_v9'
HERE=Path(__file__).resolve().parent
OUT=ROOT/'docs/wand_native_acceptance_2026-10-30'
sys.path.insert(0,str(ROOT/'candidates/wand_4wsb_host_20261030'))
os.environ['WAND_4WSB_HOST']=str(HOST)
from bootstrap import config
from prepare import save

spec=importlib.util.spec_from_file_location('readonly_resource_helpers',
    ROOT/'candidates/wand_fullscreen_replay_20261028/replay_diagnostic.py')
diagnostic=importlib.util.module_from_spec(spec);spec.loader.exec_module(diagnostic)


def protected():
    m=json.loads((HOST/'HOST_MANIFEST.json').read_text())
    if not m['host_offline_passed']:raise ValueError('sealed host required')
    expected={}
    for group in ('protected','runtime_hashes','source_hashes'):expected.update(m[group])
    expected[m['firmware']['path']]=m['firmware']['sha256']
    expected[str(ROOT/'tools/start_car_sync_4wsb.sh')]=m['file_checker_sha256']
    for p,digest in expected.items():
        if diagnostic.sha(Path(p))!=digest:raise ValueError('protected source drift: '+p)
    return expected
