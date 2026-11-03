"""File-only/native-C regression. Never a physical network endpoint."""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from ground_entry import check,profile,KIND,CONSENT,RECORD

READY=Path(os.environ['GROUND_TEST_READY'])
sys.path.insert(0,str(READY));import bootstrap
from mecanum_wand_4wsc_hardware.ground_admission import common,verify_runtime_profile
from mecanum_wand_4wsc_hardware.authorization import verify_runtime_profile as qualified_demo
from mecanum_wand_4wsc_hardware.host_profile import Runtime
from mecanum_wand_4wsc_hardware.commissioning import AuthoritySigner,AuthorityVerifier
from mecanum_wand_4wsc_control.core import Pose,Velocity
from mecanum_wand_4wsc_control.pilot_core import PilotMonitor,PilotSupervisor,PilotFinalGate
from mecanum_wand_4wsc_control.tracking_config import TrackingSafetyConfig
from mecanum_wand_4wsc_control.demo_policy import POLICY,INPUT_MODE
from native_peer import Oracle
from mecanum_wand_4wsc_hardware.stage0_session import Stage0Session
from measurements import analyze
from ground_runtime import GroundRig
from control_channel import Channel


def draft():return json.loads((READY/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
def authorized():
    p=draft();p['commissioning_review']=dict(kind=KIND,issued_utc_s=time.time(),human_authorized_stage=6,
        max_active_s=600.,operator_declared_site_ready_now=True,unqualified_trial_acknowledged_now=True,flash_record=str(RECORD))
    return p


class GroundTests(unittest.TestCase):
    def test_immutable_original_and_runtime_artifacts(self):check(READY,release=False)
    def test_stage6_exact_and_no_proxy(self):
        c=common(draft());self.assertEqual((c.stage,c.max_session_s,c.max_speed_mps,c.max_accel_mps2),(6,600.,.7,2.))
        self.assertEqual((c.max_yaw_rate_radps,c.max_yaw_accel_radps2),(2.,12.))
        self.assertTrue(c.direct_sign_target);self.assertFalse(c.follower_config().workspace_radius_enabled)
    def test_stage_promotion_rejected(self):
        for value in (True,False,0,1,3,4,5,7,8,9,-1):
            p=draft();p['stage']=value
            with self.assertRaises(ValueError):common(p)
    def test_mixed_motion_limits_rejected(self):
        for key,value in (('max_speed_mps',1.2),('max_accel_mps2',4.),('max_yaw_rate_radps',1.5),('max_yaw_accel_radps2',6.)):
            p=draft();p['safety'][key]=value
            with self.assertRaises(ValueError):common(p)
    def test_time_and_feedback_cannot_be_relaxed(self):
        for key,value in (('max_session_s',601.),('feedback_timeout_s',.4)):
            p=draft();p['safety'][key]=value
            with self.assertRaises(ValueError):common(p)
    def test_independent_supervisor_and_gate_time_monitor(self):
        c=common(draft());self.assertEqual(POLICY['max_active_s'],600.)
        for owner in (PilotSupervisor(c),PilotFinalGate(c)):
            p=owner.pilot;p.used=True;p.started=1.;p.epoch='E';p.car=p.wand=p.last=Pose(0,0,c.ground_z_m,0)
            inputs=SimpleNamespace(poses={k:dict(pose=p.car) for k in ('car','wand')})
            self.assertEqual(p.active('E',inputs,p.car,600.9),'')
            self.assertEqual(p.active('E',inputs,p.car,601.),'PILOT_TIME_LIMIT_600S')
    def test_deadline_cause_precedes_logical_key_release(self):
        channel=Channel(readonly=True)
        fake=SimpleNamespace(closing=False,channel=channel,status=dict(pilot=dict(capture_started_steady=1.)))
        GroundRig.before_operator_step(fake,600.99);self.assertFalse(channel.stopped)
        GroundRig.before_operator_step(fake,601.);self.assertEqual(channel.fault,'STAGE6_LONG600_TIME_COMPLETE')
    def test_deadline_never_masks_prior_fault(self):
        channel=Channel(readonly=True);channel.stop('WAND_POSE_TIMEOUT')
        fake=SimpleNamespace(closing=False,channel=channel,status=dict(pilot=dict(capture_started_steady=1.)))
        GroundRig.before_operator_step(fake,601.);self.assertEqual(channel.fault,'WAND_POSE_TIMEOUT')
    def test_false_measurements_cannot_unlock(self):
        for key in draft()['measurements']:
            p=draft();p['measurements'][key]=.1
            with self.assertRaises(ValueError):common(p)
        p=draft();p['physical_qualified']=True
        with self.assertRaises(ValueError):common(p)
    def test_unknown_fields_retained(self):
        p=draft();self.assertFalse(p['long_session_qualified']);self.assertEqual(p['trial_review'],{})
        self.assertTrue(all(x is None for x in p['measurements'].values()))
    def test_existing_formal_long_demo_still_rejects_ground_trial(self):
        with self.assertRaises(ValueError):qualified_demo(authorized(),'sign')
    def test_draft_no_site_declaration_rejected(self):
        with self.assertRaises(ValueError):verify_runtime_profile(draft(),'sign')
    def test_fresh_site_does_not_waive_long_review(self):
        with self.assertRaisesRegex(ValueError,'600秒实机尚未开放'):verify_runtime_profile(authorized(),'sign')
    def test_stale_site_declaration_rejected(self):
        p=authorized();p['commissioning_review']['issued_utc_s']-=601
        with self.assertRaises(ValueError):verify_runtime_profile(p,'sign')
    def test_purpose_must_be_ground_sign(self):
        p=authorized();p['purpose']='suspended'
        with self.assertRaises(ValueError):verify_runtime_profile(p,'suspended')
    def test_rehearsal_endpoint_cannot_be_hardware(self):
        p=authorized();p['hardware'].update(esp_ip='127.0.0.1',network_interface='lo',command_port=18001,telemetry_port=18002)
        with self.assertRaises(ValueError):verify_runtime_profile(p,'sign')
    def test_constructor_is_file_only_and_distinct_consent(self):
        with patch('socket.socket',side_effect=AssertionError('IO forbidden')):
            for consent in (CONSENT,'LOOPBACK_ONLY_C_REHEARSAL',''):
                with self.assertRaisesRegex(ValueError,'STOPFIX_OFFLINE_ONLY'):
                    Runtime('hardware','/nonexistent',consent=consent)

    def test_distinct_signing_scope(self):
        signer=AuthoritySigner('ab'*32,6);text=signer.encode(Velocity(),1.,101.)
        p=json.loads(text);self.assertIn('LONG600',p['payload']['scope'])
        AuthorityVerifier('ab'*32,6).decode(text,1.,101.)
    def test_native_stage6_selection_and_estop(self):
        c=Oracle();s=Stage0Session(lambda:123,stage=6);s.accept(c.request(s.new_hello(0)))
        for tick in range(10,611,10):c.request(s.cmd_vel(0,0,0,600,False,tick));c.step(tick)
        c.request(s.select_mode(610));c.step(620)
        self.assertEqual(c.status['stage'],6);self.assertFalse(c.status['permitted'])
        c.request(s.estop(620));c.step(630)
        self.assertFalse(c.status['permitted']);self.assertTrue(c.wheels.fault_bits);self.assertEqual(c.status['raw_duty'],[0]*4)
    def test_measurements_are_not_full_qualification(self):
        rows=[dict(at=t,source_ros=100+t,source_age_s=.01,x=t*.1,y=0.,yaw=t*.2) for t in (0.,.1,.2,.3,.4)]
        r=analyze(rows,.2,True,True)
        self.assertAlmostEqual(r['observed_peak_xy_speed_mps'],.1)
        self.assertTrue(r['stop_observation_contiguous']);self.assertFalse(r['physical_stop_envelope_verified'])
        self.assertFalse(r['qualified_long_demo']);self.assertIsNone(r['battery_current_a'])
    def test_missing_feedback_cannot_claim_stop(self):
        r=analyze([],1.,False,True);self.assertFalse(r['stop_observation_contiguous'])
        self.assertIsNone(r['observed_path_after_stop_request_m']);self.assertFalse(r['physical_stop_envelope_verified'])

    def test_baseline_reference_cannot_be_removed_or_changed(self):
        p=draft();p.pop('baseline_observation')
        with self.assertRaises(ValueError):common(p)
        p=draft();p['baseline_observation']['sha256']='00'*32
        with self.assertRaises(ValueError):common(p)
    def test_same_stage_short_trial_is_not_long_qualification(self):
        from mecanum_wand_4wsc_hardware.ground_prior import verify
        r=verify();self.assertEqual(r['stage'],6);self.assertTrue(r['stop_confirmed'])
        self.assertFalse(r['qualified_long_demo']);self.assertFalse(r['electrical_thermal_verified'])
        self.assertFalse(draft()['physical_qualified'])
    def test_non_motion_safety_configuration_is_unchanged(self):
        old=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_ground_2026-11-02/prepared_v3/profiles/stage0.GROUND_NOT_AUTHORIZED.json')
        expected=json.loads(old.read_text())['safety']
        expected.update(max_session_s=600.,max_speed_mps=.7,max_accel_mps2=2.,max_yaw_rate_radps=2.,max_yaw_accel_radps2=12.)
        self.assertEqual(draft()['safety'],expected)
    def test_other_stage_cannot_consume_stage6_authority(self):
        text=AuthoritySigner('ab'*32,6).encode(Velocity(),1.,101.)
        for stage in (0,3,4,5,7,8,9):
            with self.assertRaises(ValueError):AuthorityVerifier('ab'*32,stage).decode(text,1.,101.)

    def test_missing_measured_review_blocks_before_hardware_io(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'p.json';p.write_text(json.dumps(authorized()))
            digest=hashlib.sha256(p.read_bytes()).hexdigest()
            old=Path.read_text
            def read(path,*a,**kw):
                if path==READY/'HOST_MANIFEST.json':return json.dumps(dict(host_offline_passed=True,identity='4WSC'))
                return old(path,*a,**kw)
            with patch.object(Path,'read_text',read),patch('socket.socket',side_effect=AssertionError('hardware IO forbidden')):
                with self.assertRaisesRegex(ValueError,'STOPFIX_OFFLINE_ONLY'):Runtime('hardware',str(p),digest,CONSENT)
    def test_short_mode_profile_cannot_open_long_gate(self):
        p=json.loads(Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_2026-11-02/prepared_v2/profiles/stage6.GROUND_NOT_AUTHORIZED.json').read_text())
        with self.assertRaises(ValueError):common(p)
    def test_review_claim_without_measurement_records_rejected(self):
        from mecanum_wand_4wsc_hardware.ground_admission import verify_long_review,REVIEW_KIND
        from mecanum_wand_4wsc_hardware.ground_prior import DIGEST,IMAGE
        p=authorized();p['long_duration_review']=dict(kind=REVIEW_KIND,human_reviewed=True,identity='4WSC',image_sha256=IMAGE,stage=6,baseline_review_sha256=DIGEST,reviewer='UNIT TEST ONLY')
        with self.assertRaises(ValueError):verify_long_review(p)
    def test_previous_timer_measurement_cannot_masquerade_as_braking(self):
        from mecanum_wand_4wsc_hardware.ground_admission import record
        p=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_2026-11-02/prepared_v2/sessions/fast_hardware_VfJ2Bc/MEASUREMENTS.json')
        with self.assertRaises(ValueError):record(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    def test_native_absolute_budget_still_630_seconds(self):
        c=Oracle();s=Stage0Session(lambda:123,stage=6);s.accept(c.request(s.new_hello(0)));c.step(100)
        self.assertEqual(c.status['absolute_run_limit_ms'],630000)
        self.assertGreater(c.status['absolute_run_limit_ms'],600000)
    def test_offline_peer_never_handles_new_packet_in_a_past_tick(self):
        from collections import deque
        from native_peer import _take_due
        p=deque([(b'HELLO','PC',335.1)])
        self.assertEqual(list(_take_due(p,330)),[])
        self.assertEqual(len(p),1)
        self.assertEqual(list(_take_due(p,340)),[(b'HELLO','PC',335.1)])
    def test_offline_peer_preserves_fractional_receipt_causality(self):
        from collections import deque
        from native_peer import _take_due
        p=deque([(b'HELLO','PC',340.001),(b'CMD','PC',345.1)])
        self.assertEqual(list(_take_due(p,340)),[])
        due=list(_take_due(p,350))
        self.assertEqual(len(due),2)
        self.assertTrue(all(v[2]<=350 for v in due))



class OperatorDurationTests(unittest.TestCase):
    def trial(self):
        with tempfile.TemporaryDirectory() as d:
            p,_=profile(READY,Path(d),False,True,True)
            return json.loads(p.read_text())
    def test_explicit_trial_accepted_file_only_no_qualification(self):
        p=self.trial()
        with patch('socket.socket',side_effect=AssertionError('no IO in admission')):
            self.assertTrue(verify_runtime_profile(p,'sign'))
        self.assertFalse(p['physical_qualified']);self.assertFalse(p['long_session_qualified'])
        self.assertEqual(p['long_duration_review'],{})
        self.assertTrue(all(v is None for v in p['measurements'].values()))
    def test_trial_declaration_required(self):
        p=self.trial();p.pop('operator_duration_trial')
        with self.assertRaises(ValueError):verify_runtime_profile(p,'sign')
    def test_trial_each_ack_required(self):
        p=self.trial()
        for key in tuple(p['operator_duration_trial']):
            q=deepcopy(p);q['operator_duration_trial'].pop(key)
            with self.assertRaises(ValueError):verify_runtime_profile(q,'sign')
    def test_trial_cannot_claim_measured_approval(self):
        p=self.trial();p['long_duration_review']={'human_reviewed':True}
        with self.assertRaises(ValueError):verify_runtime_profile(p,'sign')
    def test_trial_cannot_change_pinned_request(self):
        p=self.trial();p['operator_duration_trial']['authorization_sha256']='00'*32
        with self.assertRaises(ValueError):verify_runtime_profile(p,'sign')
    def test_real_runtime_constructs_without_io(self):
        with tempfile.TemporaryDirectory() as d:
            path,consent=profile(READY,Path(d),False,True,True)
            with patch('socket.socket',side_effect=AssertionError('no IO')):
                with self.assertRaisesRegex(ValueError,'STOPFIX_OFFLINE_ONLY'):
                    Runtime('hardware',str(path),hashlib.sha256(path.read_bytes()).hexdigest(),consent)

    def test_trial_uses_same_guards_and_full_600_parent_evidence(self):
        m=json.loads((READY/'HOST_MANIFEST.json').read_text())
        self.assertIn('ground_runtime.py',m['inherited_runtime_hashes'])
        self.assertIn('worker.py',m['inherited_runtime_hashes'])
        self.assertNotIn('server.cjs',m['inherited_runtime_hashes'])
        self.assertIn('control_channel.py',m['inherited_runtime_hashes'])
        qualification=json.loads(Path(m['inherited_600s_software_qualification']).read_text())
        self.assertTrue(qualification['passed'])
        self.assertGreater(qualification['normal_run'][0]['normal_active_s'],599.7)
        self.assertFalse(qualification['physical_motion_started'])
    def test_trial_stale_or_not_site_ready_rejected(self):
        p=self.trial();p['commissioning_review']['issued_utc_s']-=601
        with self.assertRaises(ValueError):verify_runtime_profile(p,'sign')
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):profile(READY,Path(d),False,False,True)


class PrepareReadyTests(unittest.TestCase):
    def setup_ready(self):
        from mecanum_wand_4wsc_control.auto_prepare import AutoPrepareOnce
        m=SimpleNamespace(session_requested=False,activation_used=False,pilot_used=False,closing=False,ended=False,
            status=dict(state='ARMED',gate_ready=True),gate=dict(fault='',hard_fault=False),
            transport_status={},start_ready=lambda now:True)
        p=AutoPrepareOnce();p.started=1.;p.attempted=True;p.requested_at=1.
        p.poll(m,2.,space_down=False,service_ready=True)
        self.assertEqual(p.phase,'READY');self.assertTrue(p.ready_confirmed)
        return p,m
    def test_late_ready_gap_is_not_old_prepare_timeout(self):
        p,m=self.setup_ready();m.status['gate_ready']=False;m.start_ready=lambda now:False
        self.assertFalse(p.poll(m,10.,space_down=False,service_ready=True))
        self.assertFalse(p.cancelled);self.assertEqual(p.phase,'WAITING_READY')
        self.assertFalse(m.session_requested);self.assertFalse(m.activation_used)
    def test_ready_gap_recovers_without_new_prepare_or_enable(self):
        p,m=self.setup_ready();m.status['gate_ready']=False
        p.poll(m,10.,space_down=False,service_ready=True)
        m.status['gate_ready']=True
        self.assertFalse(p.poll(m,10.4,space_down=False,service_ready=True))
        self.assertEqual(p.phase,'READY');self.assertIsNone(p.reconfirm_since)
        self.assertFalse(m.session_requested)
    def test_continuous_not_ready_is_still_bounded(self):
        p,m=self.setup_ready();m.status['gate_ready']=False
        p.poll(m,10.,space_down=False,service_ready=True)
        p.poll(m,15.001,space_down=False,service_ready=True)
        self.assertTrue(p.cancelled);self.assertFalse(m.session_requested)
    def test_first_prepare_five_second_limit_unchanged(self):
        p,m=self.setup_ready();p.ready_confirmed=False;m.status['gate_ready']=False
        p.poll(m,6.001,space_down=False,service_ready=True)
        self.assertTrue(p.cancelled)
    def test_fatal_fault_after_ready_still_cancels_immediately(self):
        for update in ('state','transport','gate'):
            p,m=self.setup_ready()
            if update=='state':m.status['state']='FAULT'
            elif update=='transport':m.transport_status['fault']='MCU_FEEDBACK_TIMEOUT'
            else:m.gate['hard_fault']=True
            p.poll(m,2.1,space_down=False,service_ready=True)
            self.assertTrue(p.cancelled);self.assertFalse(m.session_requested)
    def test_not_ready_click_cannot_delayed_enable(self):
        from mecanum_wand_4wsc_control.toggle_ui import ToggleOperatorModel
        m=ToggleOperatorModel();m.focused=True
        self.assertFalse(m.press(10.));self.assertFalse(m.release(10.1))
        self.assertFalse(m.session_requested);self.assertFalse(m.click_pending)


class OfflineStopfixTests(unittest.TestCase):
    def test_native_library_is_fixed_independent_candidate(self):
        from native_peer import LIB
        self.assertIn('wand_stopfix_2026-11-02/prepared_v1/native',str(LIB))
        self.assertFalse(json.loads((READY.parent/'flash_bundle/BUNDLE.json').read_text())['flashed'])
    def test_all_independent_hardware_paths_are_closed(self):
        import subprocess
        from ground_entry import OFFLINE
        for command in ([sys.executable,'-B',str(Path(__file__).parent/'ground_entry.py'),'--launch','--site-ready','--operator-trial'],
                        ['node',str(READY/'server.cjs'),'--hardware']):
            r=subprocess.run(command,text=True,capture_output=True,timeout=10)
            self.assertNotEqual(r.returncode,0);self.assertIn(OFFLINE,r.stdout+r.stderr)
    def test_motor_algorithms_and_parameters_are_byte_identical(self):
        parent=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_firmware_4wsc_2026-10-30/prepared_v4/firmware')
        for f in ('Src/four_wheel_speed.c','Src/four_drive_group.c','Src/drive_bench_control.c','Src/servo_policy.c','Src/mecanum_drive_plan.c',
                  'Inc/four_speed_pwm_profile.h','Inc/servo_policy.h'):
            self.assertEqual((parent/f).read_bytes(),(READY.parent/'firmware'/f).read_bytes())
    def test_stopfix_preserves_source_guard_constants_and_positive_authority(self):
        source=(READY.parent/'firmware/Src/four_link_motion.c').read_text()
        self.assertIn('!r->body.control.clear_latched_faults',source)
        self.assertIn('!(h->flags&MC_FLAG_ENABLED)',source)
        self.assertIn('disabled_zero ? 0U : SERVO_LINK_SOURCE_AGE',source)
        for key in ('last_command_ms = now','last_command_sequence = h->sequence','SERVO_SOURCE_LAG_LIMIT_MS','SERVO_SOURCE_LEAD_LIMIT_MS'):
            self.assertIn(key,source)
    def test_browser_safety_proof_code_is_unchanged(self):
        parent=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_600s_trial_2026-11-02/prepared_v2')
        for f in ('control_channel.py','channel.py','web/input_guard.js','worker.py','browser_operator.py'):
            self.assertEqual((parent/f).read_bytes(),(READY/f).read_bytes())

if __name__=='__main__':unittest.main(verbosity=2)
