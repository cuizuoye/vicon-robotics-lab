"""Pin this independent SIM entry only after actual private-ROS checks."""
import argparse
import json
import re
from pathlib import Path
import subprocess
import sys
from bootstrap import ROOT,input_hashes,protected_hashes,save,digest


def main():
    p=argparse.ArgumentParser();p.add_argument('--evidence',required=True,type=Path);a=p.parse_args()
    report=json.loads((a.evidence/'RESULT.json').read_text())
    if not (report['passed'] and report['cases']>=15 and report['actual_ros_chain']
            and report['original_sources_unchanged'] and report['inputs_unchanged']
            and report['physical_car_started'] is False):raise ValueError('actual ROS evidence incomplete')
    for path,sha in report['implementation_sha256'].items():
        if digest(Path(path))!=sha:raise ValueError('ROS implementation changed after evidence: '+path)
    before=protected_hashes();inputs=input_hashes();here=Path(__file__).resolve().parent
    result=subprocess.run([sys.executable,'-m','unittest','-v','test_extension.py'],cwd=here,
        capture_output=True,text=True)
    (a.evidence/'unit_tests.log').write_text(result.stdout+result.stderr)
    if result.returncode:raise RuntimeError(result.stdout+result.stderr)
    unit_tests=int(re.search(r'Ran (\d+) tests?',result.stderr).group(1))
    if unit_tests<12:raise ValueError('readiness regression coverage missing')
    if before!=protected_hashes() or inputs!=input_hashes():raise ValueError('inputs changed')
    if before!=json.loads((a.evidence/'protected_after.json').read_text()):raise ValueError('baseline drift since rehearsal')
    sources=list(here.glob('*.py'))+[ROOT/'tools/start_fast_follow_ros_sim.sh']
    manifest=dict(kind='REAL_SIGN_PRIVATE_ROS_SOFTWARE_ONLY',actual_ros_tests_passed=True,
        actual_ros_case_count=report['cases'],unit_tests=unit_tests,evidence=str(a.evidence),
        evidence_sha256=digest(a.evidence/'RESULT.json'),sources={str(v):digest(v) for v in sources},
        inputs=inputs,protected=before,firmware_flashed=False,physical_car_started=False,
        real_sign_human_acceptance_completed=False,hardware_transport_created=False)
    save(ROOT/'docs/wand_fast_follow_ros_sim_2026-10-29/PREPARED.json',manifest)
    print(json.dumps(dict(prepared=True,actual_ros_cases=report['cases'],unit_tests=unit_tests,
        protected_files=len(before),hardware_output=False),ensure_ascii=False))


if __name__=='__main__':main()
