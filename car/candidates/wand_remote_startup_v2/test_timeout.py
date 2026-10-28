"""Parent protections + observed X1140ms/startup backlog regressions."""
import importlib.util
import json
import os
from pathlib import Path
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');OUT=Path(os.environ['FEEDBACK_OUTPUT'])
sys.path.insert(0,str(ROOT/'candidates/wand_ros_bound_history'))
spec=importlib.util.spec_from_file_location('remote_v2_parent_tests',ROOT/'candidates/wand_ros_bound_history/test_timeout.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
offline_only=base.offline_only
for name in dir(base):
    if name.startswith('test_') and name not in ('test_protected_parents_and_private_changes',
            'test_expired_remote_sample_revokes_and_closes_without_a_second_read'):
        globals()['test_reused_'+name[5:]]=getattr(base,name)
import test_v2_unit
for name in dir(test_v2_unit):
    if name.startswith('test_'):globals()[name]=getattr(test_v2_unit,name)


def test_pinned_parent_and_private_change_inventory():
    import hashlib
    m=json.loads((OUT/'build_manifest.json').read_text())
    for p,d in m['protected_input_hashes'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==d,p
    assert {Path(p).name for p in json.loads((OUT/'PRIVATE_CHANGE_INVENTORY.json').read_text())}=={
        'async_x11.py','remote_keys.py','startup_candidate.py','rotation_nodes.py'}
    assert m['feedback_protection_ms']==300
    assert 'SHOW_REMOTE_STARTUP_V2' in (OUT/'oneclick_app.py').read_text()
