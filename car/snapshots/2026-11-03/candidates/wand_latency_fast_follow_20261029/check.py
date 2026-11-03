"""File-only check. Deliberately has no hardware or live launch operation."""
import argparse
import json
from pathlib import Path
from common import ROOT,digest

RESULT=ROOT/'docs/wand_latency_fast_follow_2026-10-29/DELIVERY.json'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--check',action='store_true');p.add_argument('--launch',action='store_true')
    a=p.parse_args()
    if a.launch:p.error('SIM_ONLY: no live/hardware entry; physical measurement and real-Sign simulation pending')
    r=json.loads(RESULT.read_text())
    if digest(r['comparison'])!=r['comparison_sha256']:
        raise ValueError('comparison evidence changed')
    for field in ('protected_files','candidate_files'):
        for path,expected in r[field].items():
            if digest(path)!=expected:raise ValueError('file changed: '+path)
    if not r['offline_tests_passed'] or not r['comparison_functional_passed']:
        raise ValueError('offline verification incomplete')
    print(json.dumps(dict(file_check_passed=True,profile='SIM_ONLY_NOT_HARDWARE_APPROVED',
        protected_files=len(r['protected_files']),unit_tests=r['unit_tests_run'],
        model_runs=r['model_runs'],physical_onset_measured=False,
        live_sign_simulation_accepted=False,original_demo_unchanged=True),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
