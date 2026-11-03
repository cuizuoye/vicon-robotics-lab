"""Retained policy regressions plus STOPFIX receipt/isolation admission. No hardware IO."""
import argparse
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from build import HERE,ROOT,OUT,RECORD,CONSENT,save
from ground_entry import profile,check

READY=OUT/'host';os.environ['GROUND_TEST_READY']=str(READY)
sys.path.insert(0,str(READY));import bootstrap
from mecanum_wand_4wsc_hardware import deployment_record as d
from mecanum_wand_4wsc_hardware.host_profile import Runtime
from mecanum_wand_4wsc_hardware.authorization import verify_flash_record
from mecanum_wand_4wsc_hardware.commissioning import AuthoritySigner,AuthorityVerifier
from mecanum_wand_4wsc_control.core import Velocity

OLD=ROOT/'candidates/wand_stopfix_20261102/test_ground.py'
spec=importlib.util.spec_from_file_location('retained_stopfix_tests',OLD)
retained=importlib.util.module_from_spec(spec);spec.loader.exec_module(retained)
REPLACED={'test_constructor_is_file_only_and_distinct_consent',
    'test_missing_measured_review_blocks_before_hardware_io','test_real_runtime_constructs_without_io',
    'test_all_independent_hardware_paths_are_closed'}

# These three checks audit the unchanged firmware, which is deliberately kept
# in the original frozen firmware folder rather than copied/rebuilt here.
for name in ('test_native_library_is_fixed_independent_candidate','test_motor_algorithms_and_parameters_are_byte_identical',
             'test_stopfix_preserves_source_guard_constants_and_positive_authority'):
    method=getattr(retained.OfflineStopfixTests,name)
    def adapted(self,original=method):
        previous=retained.READY
        try:
            retained.READY=ROOT/'docs/wand_stopfix_2026-11-02/prepared_v1/host'
            return original(self)
        finally:retained.READY=previous
    setattr(retained.OfflineStopfixTests,name,adapted)

@contextmanager
def simulated_qualification_only():
    original=Path.read_text
    def qualified(path,*args,**kw):
        text=original(path,*args,**kw)
        if path==READY/'HOST_MANIFEST.json':
            value=json.loads(text);value['host_offline_passed']=True;text=json.dumps(value)
        return text
    with patch.object(d,'verify_host_snapshot',return_value={}),patch.object(Path,'read_text',qualified):yield

def receipt():return json.loads(RECORD.read_text())

class AdmissionTests(unittest.TestCase):
    def test_new_receipt_acceptance_file_only(self):
        with patch('socket.socket',side_effect=AssertionError('no IO')),patch('subprocess.Popen',side_effect=AssertionError('no processes')):
            self.assertEqual(verify_flash_record(RECORD)['firmware_revision'],'STOPFIX_V1')
    def test_old_or_alternate_record_rejected(self):
        for value in ('',str(ROOT/'docs/wand_firmware_4wsc_2026-10-30/DEPLOYMENT_2026-11-02.json'),'/nonexistent'):
            with self.assertRaises(ValueError):verify_flash_record(value)
    def test_receipt_digest_change_rejected(self):
        with patch.object(d,'RECORD_SHA','0'*64):
            with self.assertRaisesRegex(ValueError,'receipt changed'):verify_flash_record(RECORD)
    def test_readback_flags_cannot_be_omitted_or_invented(self):
        for key in ('flashed','backup_verified','two_full_backups_identical','independent_readback_verified','image_prefix_matches_byte_for_byte','option_bytes_unchanged','final_core_halted'):
            for value in (False,1,None):
                with self.subTest(key=key,value=value):
                    r=receipt();r[key]=value
                    with self.assertRaises(ValueError):d.audit_record(r)
    def test_old_image_same_wire_identity_rejected(self):
        for key,value in (('image_sha256',d.OLD_IMAGE_SHA),('firmware_revision','4WSC'),('protocol_authority_version',7),('diagnostic_payload_bytes',204)):
            r=receipt();r[key]=value
            with self.assertRaises(ValueError):d.audit_record(r)
    def test_wrong_uid_probe_and_artifact_paths_rejected(self):
        for key,value in (('chip_uid_words',['0']*3),('stlink_sn','OTHER'),('full_readback_path','/tmp/forged.bin'),('backup_pass1_path','/tmp/old.bin')):
            r=receipt();r[key]=value
            with self.assertRaises(ValueError):d.audit_record(r)
    def test_malformed_deployment_logs_rejected(self):
        for logs in ({},{'../escape':'0'*64},dict(receipt()['log_sha256'],fake='0'*64)):
            r=receipt();r['log_sha256']=logs
            with self.assertRaises(ValueError):d.audit_record(r)
    def test_deployment_log_hash_change_rejected(self):
        r=receipt();r['log_sha256']['program_stopfix.log']='0'*64
        with self.assertRaisesRegex(ValueError,'log changed'):d.audit_record(r)
    def test_corrupt_actual_binary_backup_and_options_rejected(self):
        original=Path.read_bytes
        for name in ('stopfix_v1_image.bin','flash_before_pass2_1MiB.bin','flash_after_stopfix_1MiB.bin','option_bytes_after.bin'):
            def corrupt(path,*args,**kw):
                value=original(path,*args,**kw)
                return bytes([value[0]^1])+value[1:] if path==d.BACKUP/name else value
            with patch.object(Path,'read_bytes',corrupt):
                with self.assertRaises(ValueError):d.audit_record(receipt())
    def test_inherited_observation_cannot_claim_new_revision_qualification(self):
        p=json.loads((READY/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text());d.verify_migration(p)
        for key,value in (('new_revision_physical_qualified',True),('inherited_observation_only',False),('image_sha256',d.OLD_IMAGE_SHA)):
            q=deepcopy(p);q['firmware_deployment'][key]=value
            with self.assertRaises(ValueError):d.verify_migration(q)
    def test_runtime_rejects_draft_before_profile_or_socket(self):
        with patch('socket.socket',side_effect=AssertionError('no hardware IO')):
            with self.assertRaisesRegex(ValueError,'qualified deployed host'):Runtime('hardware','/nonexistent',consent=CONSENT)
    def test_qualified_branch_profile_and_namespace_are_file_only(self):
        # Simulate only the qualification bit here; validate the actual pinned
        # manifest without this mock in the post-qualification check below.
        with tempfile.TemporaryDirectory() as folder:
            path,consent=profile(READY,Path(folder),False,True,True)
            with simulated_qualification_only(),patch('socket.socket',side_effect=AssertionError('no IO')):
                rt=Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),consent)
            self.assertEqual(rt.namespace,'/wand_stopfix_stage6_long600');self.assertFalse(rt.software_only)
            self.assertEqual((rt.config.max_speed_mps,rt.config.max_accel_mps2),(.7,2.))
            self.assertFalse(rt.profile['physical_qualified'])
    def test_old_consent_and_stale_site_cannot_open_new_runtime(self):
        with tempfile.TemporaryDirectory() as folder:
            path,_=profile(READY,Path(folder),False,True,True)
            with simulated_qualification_only():
                for consent in ('','LOOPBACK_ONLY_C_REHEARSAL','4WSC_STAGE6_LONG600_OPERATOR_TRIAL_UNKNOWN_MEASUREMENTS_SITE_READY'):
                    with self.assertRaises(ValueError):Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),consent)
                p=json.loads(path.read_text());p['commissioning_review']['issued_utc_s']-=601;path.write_text(json.dumps(p))
                with self.assertRaises(ValueError):Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),CONSENT)
    def test_old_profile_missing_new_image_binding_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path,_=profile(READY,Path(folder),False,True,True);p=json.loads(path.read_text());p.pop('firmware_deployment');path.write_text(json.dumps(p))
            with simulated_qualification_only():
                with self.assertRaises(ValueError):Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),CONSENT)
    def test_old_authority_scope_rejected_even_with_same_key(self):
        import hmac
        from mecanum_wand_4wsc_control.safety_wire import canonical
        value=json.loads(AuthoritySigner('ab'*32,6).encode(Velocity(),1.,101.))
        self.assertIn('STOPFIX_V1',value['payload']['scope'])
        value['payload']['scope']='HARDWARE_4WSC_V8_STAGE6_LONG600_STAGE_6'
        value['signature']=hmac.new(bytes.fromhex('ab'*32),canonical(value['payload']),hashlib.sha256).hexdigest()
        with self.assertRaises(ValueError):AuthorityVerifier('ab'*32,6).decode(json.dumps(value),1.,101.)
    def test_direct_node_hardware_cannot_bypass_draft_or_mode(self):
        env=dict(os.environ,WAND_4WSC_MODE='hardware',WAND_4WSC_PROFILE='/nonexistent')
        p=subprocess.run(['node',str(READY/'server.cjs'),'--hardware'],env=env,capture_output=True,text=True,timeout=15)
        self.assertNotEqual(p.returncode,0);self.assertNotIn('BROWSER_READY',p.stdout)
        env['WAND_4WSC_MODE']='rehearsal'
        p=subprocess.run(['node',str(READY/'server.cjs'),'--hardware'],env=env,capture_output=True,text=True,timeout=5)
        self.assertNotEqual(p.returncode,0);self.assertIn('matching runtime mode',p.stderr)
    def test_zero_only_whitelist_cannot_enable_select_clear_or_move(self):
        from idle_probe import zero_only
        from native_peer import Oracle
        from mecanum_wand_4wsc_hardware.stage0_session import Stage0Session
        s=Stage0Session(lambda:123,stage=6);hello=s.new_hello(0);zero_only(hello);s.accept(Oracle().request(hello));zero_only(s.cmd_vel(0,0,0,600,False,10))
        for raw in (s.select_mode(20),s.cmd_vel(.1,0,0,600,False,30),s.cmd_vel(0,0,0,600,True,40),s.estop(50),s.control_enable(False,True,60)):
            with self.assertRaises(ValueError):zero_only(raw)
    def test_all_motion_and_browser_components_still_pinned(self):
        m=check(READY,release=False)
        for path in ('worker.py','ground_runtime.py','hardware_runtime.py','browser_operator.py','control_channel.py','web/input_guard.js','web/focus_evidence.js','focus_audit.cjs'):
            self.assertIn(path,m['inherited_runtime_hashes'])

def flatten(suite):
    for item in suite:
        if isinstance(item,unittest.TestSuite):yield from flatten(item)
        else:yield item

def main(post=False):
    if post:
        with tempfile.TemporaryDirectory() as folder:
            path,consent=profile(READY,Path(folder),False,True,True)
            with patch('socket.socket',side_effect=AssertionError('no hardware IO')),patch('subprocess.Popen',side_effect=AssertionError('no child processes')):
                rt=Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),consent)
            assert rt.namespace=='/wand_stopfix_stage6_long600'
        print(json.dumps(dict(passed=True,real_qualified_Runtime=True,hardware_opened=False,profile_is_unit_fixture_not_site_authorization=True)));return
    old=[x for x in flatten(unittest.defaultTestLoader.loadTestsFromModule(retained)) if x._testMethodName not in REPLACED]
    new=unittest.defaultTestLoader.loadTestsFromTestCase(AdmissionTests)
    total=len(old)+new.countTestCases();new_count=new.countTestCases()
    suite=unittest.TestSuite(old);suite.addTests(new)
    result=unittest.TextTestRunner(verbosity=1).run(suite)
    print(json.dumps(dict(passed=result.wasSuccessful(),retained_tests=len(old),new_tests=new_count,total_tests=total,hardware_opened=False)))
    if not result.wasSuccessful():raise SystemExit(1)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--post-qualified',action='store_true');main(p.parse_args().post_qualified)
