"""Retain prior native/UI protections and exercise the new actual callbacks."""
import importlib.util
import json
import os
from pathlib import Path

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');OUT=Path(os.environ['FEEDBACK_OUTPUT'])
spec=importlib.util.spec_from_file_location('bound_parent_tests',ROOT/'candidates/wand_feedback_300ms_startup/test_timeout.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
offline_only=base.offline_only
for name in dir(base):
    if name.startswith('test_') and name not in ('test_protected_parents_and_changed_files_exact',
            'test_standby_fast_path_only_accepts_never_armed_zero'):
        globals()['test_parent_'+name[5:]]=getattr(base,name)
import test_bound_unit
for name in dir(test_bound_unit):
    if name.startswith('test_'):globals()[name]=getattr(test_bound_unit,name)
spec=importlib.util.spec_from_file_location('bound_reused_remote_tests',ROOT/'candidates/wand_remote_x11/test_remote_unit.py')
remote_tests=importlib.util.module_from_spec(spec);spec.loader.exec_module(remote_tests)
for name in dir(remote_tests):
    if name.startswith('test_'):globals()['test_remote_'+name[5:]]=getattr(remote_tests,name)


def test_protected_parents_and_private_changes():
    import hashlib
    m=json.loads((OUT/'build_manifest.json').read_text())
    for p,digest in m['protected_input_hashes'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest,p
    assert {Path(p).name for p in json.loads((OUT/'PRIVATE_CHANGE_INVENTORY.json').read_text())}=={
        'bound_history.py','bound_rotation.py','rotation_nodes.py','remote_keys.py'}
    assert m['feedback_protection_ms']==300
    assert 'SHOW_BOUND_HISTORY_V1' in (OUT/'oneclick_app.py').read_text()
