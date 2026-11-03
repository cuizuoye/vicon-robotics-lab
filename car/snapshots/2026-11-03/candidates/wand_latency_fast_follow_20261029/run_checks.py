"""Recheck unit tests plus saved comparison; produce a file-only delivery pin."""
import argparse
import io
import json
from pathlib import Path
import sys
import unittest
from common import ROOT,digest,protected_hashes,fresh_output,save


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',required=True)
    ap.add_argument('--comparison',required=True);a=ap.parse_args()
    out=fresh_output(a.output);before=protected_hashes();save(out/'protected_before.json',before)
    log=io.StringIO();suite=unittest.defaultTestLoader.discover(str(Path(__file__).parent),pattern='test_candidate.py')
    tests=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
    (out/'tests.log').write_text(log.getvalue())
    comparison=Path(a.comparison).resolve();r=json.loads(comparison.read_text())
    if digest(Path(__file__).with_name('catchup.py'))!=r['candidate_source_sha256']:
        raise ValueError('saved model comparison does not match candidate implementation')
    if before!=json.loads(comparison.with_name('protected_after.json').read_text()):
        raise ValueError('baseline moved since saved comparison')
    after=protected_hashes();save(out/'protected_after.json',after)
    good=tests.wasSuccessful() and r['functional_passed'] and r['no_large_rms_regression_on_tested_cases'] and before==after
    files=[p for p in Path(__file__).parent.iterdir() if p.is_file()]
    files.append(ROOT/'tools/check_wand_latency_fast_follow.sh')
    report=dict(schema=1,status='OFFLINE_REVIEW_READY_NOT_HARDWARE_APPROVED',
        offline_tests_passed=tests.wasSuccessful(),unit_tests_run=tests.testsRun,
        comparison_functional_passed=r['functional_passed'],model_runs=len(r['results']),
        candidate_files={str(p):digest(p) for p in sorted(files)},protected_files=after,
        comparison=str(comparison),comparison_sha256=digest(comparison),
        test_evidence=str(out/'tests.log'),original_sources_unchanged=before==after,
        physical_onset_measured=False,live_sign_simulation_accepted=False,
        actual_ros_signature_source_binding_candidate_validated=False,
        firmware_flashed=False,car_started=False,network_settings_changed=False,
        video_capture_equipment='user has normal 30/60fps or unverified; coarse onset brackets only',
        coverage_measurement='user will supply; not assumed in this delivery')
    save(out/'RESULT.json',report)
    if not good:print(log.getvalue());raise SystemExit('verification failed; no delivery pin written')
    save(ROOT/'docs/wand_latency_fast_follow_2026-10-29/DELIVERY.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('candidate_files','protected_files')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
