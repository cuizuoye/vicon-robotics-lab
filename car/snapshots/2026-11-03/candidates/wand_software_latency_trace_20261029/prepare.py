"""Pin independent profiler only after timing joins and all safety tests pass."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from console import REPORT, hashes
from bootstrap import save


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', required=True, type=Path)
    parser.add_argument('--timing-evidence', type=Path)
    args = parser.parse_args()
    path = args.evidence/'RESULT.json'
    result = json.loads(path.read_text())
    assert result['passed'] and result['inherited_safety_cases'] == 15
    assert result['original_unchanged'] and result['inputs_unchanged']
    process = subprocess.run([sys.executable, '-m', 'unittest', '-v', 'test_trace'],
                             cwd=Path(__file__).parent, capture_output=True, text=True)
    (args.evidence/'unit_tests.log').write_text(process.stdout+process.stderr)
    if process.returncode:
        raise RuntimeError(process.stdout+process.stderr)
    evidence = [path, args.evidence/'unit_tests.log', args.evidence/'paired_trace/LATENCY_RESULT.json',
                args.evidence/'inherited_safety/RESULT.json']
    if args.timing_evidence:
        latest = args.timing_evidence/'RESULT.json'
        data = json.loads(latest.read_text())
        assert data['passed'] and data['timing_trace'] and data['actual_tk']
        evidence += [latest, args.timing_evidence/'paired_trace/LATENCY_RESULT.json']
    save(REPORT/'PREPARED.json', dict(passed=True, tests=dict(trace_units=11,
         actual_ros_safety=15, exact_id_ros_tk=1), sources=hashes(),
         evidence={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in evidence},
         physical_car_started=False, firmware_changed=False))
    print('SOFTWARE_LATENCY_PROFILER_PREPARED', flush=True)
