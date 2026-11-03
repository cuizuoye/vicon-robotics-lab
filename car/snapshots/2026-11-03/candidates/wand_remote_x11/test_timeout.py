"""Transport/controller baseline plus private non-GUI remote regressions."""
import importlib.util
import json
import os
from pathlib import Path

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');OUT=Path(os.environ['FEEDBACK_OUTPUT'])
spec=importlib.util.spec_from_file_location('startup_parent_tests',ROOT/'candidates/wand_feedback_300ms_startup/test_timeout.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
offline_only=base.offline_only
for name in dir(base):
    if name.startswith('test_') and name!='test_protected_parents_and_changed_files_exact':
        globals()['test_parent_'+name[5:]]=getattr(base,name)
import test_remote_unit
for name in dir(test_remote_unit):
    if name.startswith('test_'):globals()[name]=getattr(test_remote_unit,name)


def test_only_console_adapter_and_three_new_modules_change():
    import hashlib
    m=json.loads((OUT/'build_manifest.json').read_text())
    for p,digest in m['protected_input_hashes'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest,p
    assert {Path(p).name for p in json.loads((OUT/'PRIVATE_CHANGE_INVENTORY.json').read_text())}=={
        'operator_console.py','async_x11.py','remote_keys.py','remote_render.py'}
    assert m['feedback_protection_ms']==300
    assert 'SHOW_REMOTE_X11_V1' in (OUT/'oneclick_app.py').read_text()
