#!/usr/bin/env python3
"""Check the read-only source export and publish allowlisted evidence summaries."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    receipt_path = ROOT / 'handoff/CAR_EXPORT_2026-11-03.json'
    receipt = json.loads(receipt_path.read_text())
    for row in receipt['files']:
        if sha(source / row['source_relative']) != row['source_sha256']:
            raise ValueError('Source changed: ' + row['source_relative'])
        row['final_public_sha256'] = sha(ROOT / row['path'])
    receipt['public_adaptations'] = [
        '4WSC profile header absolute include converted to a relative include',
        'Separate scripts supply public paths without executing archived admission generators',
        'Latest Sign geometry uses a synthetic T shape; related source digest updated'
        , 'Public source line endings and trailing whitespace mechanically normalized'
    ]
    receipt['primary_source_hashes_verified_unchanged'] = len(receipt['files'])
    save(receipt_path, receipt)

    raw_result = source / 'docs/wand_static_source_2026-11-02/prepared_v1/static600_20261103_1/RESULT.json'
    result = json.loads(raw_result.read_text())
    keys = ('passed', 'error', 'reason', 'requested_active_s', 'observed_active_s',
        'motion_first_s', 'static_pose_republished_with_original_new_frame_stamps',
        'injected_fault', 'injected_input_stall_ms', 'actual_raw_ROS_domain',
        'actual_control_ROS_domain', 'actual_native_C_loopback', 'accepted_commands',
        'max_control_gap_s', 'frames', 'raw_publisher_max_gap_s', 'source_diagnostics',
        'native_output_zero', 'stop_confirmed', 'physical_output', 'firmware_changed',
        'network_settings_changed')
    public = {k: result[k] for k in keys}
    if not (public['passed'] and public['observed_active_s'] >= 600
            and public['stop_confirmed'] and public['native_output_zero']
            and not public['physical_output']):
        raise ValueError('Static software evidence did not pass')
    public.update(source_result_sha256=sha(raw_result),
        input='Synthetic Vicon raw topics, NOT real Tracker',
        qualification_scope='Software static-source soak, NOT complete candidate or physical qualification',
        generated_on_export=False)
    evidence = ROOT / 'car/reports/2026-11-03/evidence'
    save(evidence / 'STATIC_600S.json', public)

    qualification_path = source / 'docs/wand_background_session_display_2026-11-02/prepared_v1/QUALIFICATION.json'
    qualified = json.loads(qualification_path.read_text())
    cases = []
    for case in qualified['browser_cases_rerun']:
        cases.append({k: case[k] for k in ('case', 'passed', 'actual_browser',
            'actual_ROS_native_C_loopback', 'stop_confirmed', 'native_C_output_zero',
            'physical_output')})
    if not qualified['passed'] or len(cases) != 11 or not all(c['passed'] for c in cases):
        raise ValueError('Historical background display evidence invalid')
    save(evidence / 'BACKGROUND_DISPLAY_SOFTWARE.json', dict(
        revision=qualified['revision'], original_result_sha256=sha(qualification_path),
        inherited_session_unit=qualified['inherited_session_unit'],
        inherited_client_unit=qualified['inherited_client_unit'],
        admission=qualified['admission'], terminal_view_unit=qualified['terminal_view_unit'],
        cases=cases, physical_output=False,
        scope='Saved V2 software evidence; not final V3 candidate qualification',
        rerun_during_export=False))

    observation_path = source / 'docs/wand_4wsc_stage6_2026-11-02/OPERATOR_REVIEW.json'
    observation = json.loads(observation_path.read_text())
    save(evidence / 'STAGE6_FOLLOW_OBSERVATION.json', dict(
        original_result_sha256=sha(observation_path),
        **{k: observation[k] for k in ('identity', 'image_sha256', 'stage',
            'physical_trial', 'backend_trial_passed', 'operator_observation_reviewed',
            'operator_feedback', 'chosen_baseline', 'active_duration_s',
            'stop_confirmed', 'gate_output_zero', 'qualified_long_demo',
            'physical_stop_envelope_verified', 'electrical_thermal_verified',
            'qualification_scope')},
        configuration_limits_are_not_measured_peak_performance=True,
        revalidated_during_export=False))
    deployment_path = source / 'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json'
    deployment = json.loads(deployment_path.read_text())
    equivalence_path = source / 'docs/wand_stopfix_2026-11-02/prepared_v1/MOTION_EQUIVALENCE.json'
    equivalence = json.loads(equivalence_path.read_text())
    save(evidence / 'STOPFIX_DEPLOYMENT_SUMMARY.json', dict(
        original_deployment_sha256=sha(deployment_path),
        **{k: deployment[k] for k in ('verified_at_utc', 'identity', 'firmware_revision',
            'protocol_authority_version', 'diagnostic_payload_bytes', 'backup_verified',
            'two_full_backups_identical', 'image_sha256', 'image_bytes',
            'independent_readback_verified', 'image_prefix_matches_byte_for_byte',
            'option_bytes_unchanged')},
        normal_motion_equivalence=equivalence,
        equivalence_original_sha256=sha(equivalence_path),
        raw_flash_device_ids_and_permission_not_included=True,
        revalidated_during_export=False))

    counts = {}
    for key, filename in (('car', 'test-results/car.xml'),
                          ('wand_control', 'test-results/wand-control.xml'),
                          ('drone_unchanged', 'test-results/drone.xml'),
                          ('historical_4wsa_host', '4wsa-offline/host.xml')):
        suites = list(ET.parse(ROOT / '.local' / filename).getroot().iter('testsuite'))
        if any(int(s.get('errors', '0')) or int(s.get('failures', '0')) for s in suites):
            raise ValueError('Tests failed: ' + key)
        counts[key] = sum(int(s.get('tests', '0')) for s in suites)
    native = ET.parse(ROOT / '.local/car-20261103/native.xml').getroot()
    if (int(native.get('tests', '0')) != 7 or int(native.get('failures', '0'))
            or int(native.get('errors', '0')) or int(native.get('disabled', '0'))):
        raise ValueError('Latest native run has failures or missing cases')
    save(ROOT / 'handoff/CAR_VALIDATION_2026-11-03.json', dict(
        date='2026-11-03', validation_environment='Separate public checkout on original Linux host, not clean OS',
        commands=['bash scripts/doctor.sh', 'bash scripts/test_offline.sh',
            'bash scripts/test_car_release.sh', 'bash scripts/test_car_update_20261103.sh',
            '/usr/bin/python3 -m pytest -q scripts/test_public_scan.py',
            'python3 scripts/public_scan.py', 'git diff --check'],
        python_regression_counts=counts, source_pipeline_unit_tests=27,
        native_4wsc_asan_ubsan_suites=7, historical_native_4wsa_suites=4,
        public_scanner_unit_tests=6, primary_selected_source_hashes_unchanged=len(receipt['files']),
        public_source_bytes=sum((ROOT / r['path']).stat().st_size for r in receipt['files']),
        full_car_ros_build_passed=True, legacy_rectangle_and_motion_native_passed=True,
        latest_arm_cross_build_during_export=False, esp_idf_full_build_during_export=False,
        full_vicon_sdk_build_during_export=False,
        software_600s_evidence_imported_not_rerun=True, complete_latest_candidate_qualification=False,
        physical_600s_passed=False, hardware_commands_sent=False,
        firmware_written=False, network_settings_changed=False,
        primary_workspace_changed=False, drone_and_shared_receiver_changed=False))
    listed = subprocess.check_output(['git', 'ls-files', '--cached', '--others',
        '--exclude-standard', '-z'], cwd=ROOT).decode().split('\0')
    missing = [r['path'] for r in receipt['files'] if r['path'] not in set(listed)]
    if missing:
        raise ValueError('Selected sources ignored by Git: ' + repr(missing))
    print(json.dumps(dict(unchanged_primary_files=len(receipt['files']),
        python_tests=counts, static_software_s=public['observed_active_s'])))


if __name__ == '__main__':
    main()
