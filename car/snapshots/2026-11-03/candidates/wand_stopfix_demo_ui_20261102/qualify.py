"""Offline-only UI equivalence, admission, browser and isolated ROS regressions."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from build import ROOT, HERE, BASE, OUT, UI_PATCHES, sha, save, hashes
from ground_entry import check, profile


def main():
    ready = OUT / 'host'
    manifest = check(ready, release=False)
    if manifest['host_offline_passed']:
        raise ValueError('never overwrite a qualified snapshot')
    folder = Path(tempfile.mkdtemp(prefix='qualification_', dir=OUT))
    evidence = {}
    env = dict(os.environ, PYTHONPATH=str(HERE) + os.pathsep + os.environ.get('PYTHONPATH', ''),
               PYTHONDONTWRITEBYTECODE='1')

    def run(name, args, environ=env):
        result = subprocess.run(args, env=environ, capture_output=True, text=True, timeout=100)
        for suffix, value in (('stdout', result.stdout), ('stderr', result.stderr)):
            target = folder / (name + '.' + suffix + '.log')
            target.write_text(value)
            evidence[str(target)] = sha(target)
        print(name + ': ' + str(result.returncode), flush=True)
        if result.returncode:
            raise ValueError(result.stdout[-2000:] + result.stderr[-6000:])
        return result.stdout

    # Prove the two client files differ ONLY by the enumerated presentation edits.
    for relative, changes in UI_PATCHES.items():
        value = (BASE / relative).read_text()
        for old, new in changes:
            assert value.count(old) == 1
            value = value.replace(old, new)
        assert value == (ready / relative).read_text(), relative
    unchanged = manifest['inherited_parent_runtime_hashes']
    for relative, digest in unchanged.items():
        assert sha(ready / relative) == digest
    for name in ('idle_probe.py', 'server.cjs', 'worker.py', 'ground_runtime.py', 'hardware_runtime.py',
                 'browser_operator.py', 'control_channel.py', 'web/input_guard.js', 'web/focus_evidence.js'):
        assert sha(ready / name) == sha(BASE / name), name

    unit = json.loads(run('admission', [sys.executable, '-B', str(HERE / 'test_admission.py')]).splitlines()[-1])
    assert unit['passed'] and unit['total_tests'] == 65
    run('focus_client', ['node', str(ROOT / 'candidates/wand_stopfix_20261102/test_focus.cjs'), str(ready), str(folder / 'focus_client')])
    dom = json.loads((folder / 'focus_client/RESULT.json').read_text())
    assert dom['passed'] and dom['checks'] == 24
    visual = json.loads(run('headless_ui', ['node', str(HERE / 'test_ui.cjs'), str(ready), str(folder / 'headless_ui')]).splitlines()[-1])
    assert visual['passed'] and not visual['hardware_socket_opened']
    cases = []
    for case in ('no_start', 'normal', 'esc', 'second_space', 'blur', 'proof_timeout'):
        target = folder / case
        target.mkdir()
        path, consent = profile(ready, target, True)
        e = dict(env, WAND_4WSC_MODE='rehearsal', WAND_4WSC_PROFILE=str(path),
                 WAND_PROFILE_APPROVAL=sha(path), WAND_HARDWARE_CONSENT=consent,
                 WAND_4WSC_PURPOSE='sign', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        run('browser_' + case, ['node', str(ready / 'tests/browser_peer.cjs'), str(ready), str(target / 'RESULT.json'), case, '5'], e)
        result = json.loads((target / 'RESULT.json').read_text())
        assert result['passed'] and not result['physical_output'] and result['stop_confirmed']
        cases.append(result)
        for item in Path(result['session']).rglob('*'):
            if item.is_file() and item.suffix in ('.json', '.jsonl', '.log'):
                evidence[str(item)] = sha(item)
    for item in folder.rglob('*'):
        if item.is_file():
            evidence[str(item)] = sha(item)
    parent = json.loads((BASE.parent / 'QUALIFICATION.json').read_text())
    report = dict(passed=True, change_scope='presentation only; independent frozen host path rebound',
                  presentation_revision='DEMO_UI_V1', parent_qualification=str(BASE.parent / 'QUALIFICATION.json'),
                  unit_tests=65, DOM_VM_checks=24, headless_browser_checks=visual['checks'],
                  browser_protocol_cases_rerun=len(cases), browser_cases=cases,
                  actual_hardware_io_this_revision=False, physical_motion_started=False,
                  firmware_changed=False, network_settings_changed=False, physical_qualified=False,
                  exact_enumerated_UI_substitutions_only=True, unchanged_control_and_guard_files=True,
                  inherited_native_100ms_cases=parent['native_100ms_cadence_and_batched_feedback_cases'],
                  inherited_real_zero_only_check=parent['real_zero_only_check'],
                  inherited_checks_are_not_new_measurements=True,
                  safety_and_measurement_caveats_preserved_in_profile_and_audit=True,
                  screenshot_1280=str(folder / 'headless_ui/demo_1280x720.png'),
                  screenshot_1920=str(folder / 'headless_ui/demo_1920x1080.png'))
    manifest.update(host_offline_passed=True, hardware_launch_enabled=True,
                    evidence=evidence, source_hashes=hashes(HERE))
    wrapper = ROOT / 'tools/start_car_demo.sh'
    manifest['source_hashes'][str(wrapper)] = sha(wrapper)
    save(ready / 'HOST_MANIFEST.json', manifest)
    try:
        post = json.loads(run('post_qualified', [sys.executable, '-B', str(HERE / 'test_admission.py'), '--post-qualified']).splitlines()[-1])
        assert post['passed'] and not post['hardware_opened']
        target = OUT / 'QUALIFICATION.json'
        save(target, report)
        evidence[str(target)] = sha(target)
        manifest['evidence'] = evidence
        manifest['qualification'] = str(target)
        save(ready / 'HOST_MANIFEST.json', manifest)
        check(ready)
        save(OUT / 'LATEST_PREPARED.json', dict(prepared=str(ready), qualification=str(target),
                                              manifest_sha256=sha(ready / 'HOST_MANIFEST.json')))
    except BaseException:
        manifest.update(host_offline_passed=False, hardware_launch_enabled=False)
        save(ready / 'HOST_MANIFEST.json', manifest)
        raise
    print('DEMO_UI_QUALIFIED ' + str(target), flush=True)


if __name__ == '__main__':
    main()
