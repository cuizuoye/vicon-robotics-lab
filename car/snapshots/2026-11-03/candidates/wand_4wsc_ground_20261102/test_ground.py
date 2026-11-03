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


def draft():return json.loads((READY/'profiles/stage0.GROUND_NOT_AUTHORIZED.json').read_text())
def authorized():
    p=draft();p['commissioning_review']=dict(kind=KIND,issued_utc_s=time.time(),human_authorized_stage=0,
        max_active_s=60.,operator_declared_site_ready_now=True,unqualified_trial_acknowledged_now=True,flash_record=str(RECORD))
    return p


class GroundTests(unittest.TestCase):
    def test_immutable_original_and_runtime_artifacts(self):check(READY,release=False)
    def test_stage0_exact_and_no_proxy(self):
        c=common(draft());self.assertEqual((c.stage,c.max_session_s,c.max_speed_mps,c.max_accel_mps2),(0,60.,.2,.08))
        self.assertEqual((c.max_yaw_rate_radps,c.max_yaw_accel_radps2),(.2,.8))
        self.assertTrue(c.direct_sign_target);self.assertFalse(c.follower_config().workspace_radius_enabled)
    def test_stage_promotion_rejected(self):
        for value in (True,False,1,8,9,-1):
            p=draft();p['stage']=value
            with self.assertRaises(ValueError):common(p)
    def test_mixed_motion_limits_rejected(self):
        for key,value in (('max_speed_mps',.7),('max_accel_mps2',4.),('max_yaw_rate_radps',2.),('max_yaw_accel_radps2',12.)):
            p=draft();p['safety'][key]=value
            with self.assertRaises(ValueError):common(p)
    def test_time_and_feedback_cannot_be_relaxed(self):
        for key,value in (('max_session_s',600.),('feedback_timeout_s',.4)):
            p=draft();p['safety'][key]=value
            with self.assertRaises(ValueError):common(p)
    def test_independent_supervisor_and_gate_time_monitor(self):
        c=common(draft());self.assertEqual(POLICY['max_active_s'],60.)
        for owner in (PilotSupervisor(c),PilotFinalGate(c)):
            p=owner.pilot;p.used=True;p.started=1.;p.epoch='E';p.car=p.wand=p.last=Pose(0,0,c.ground_z_m,0)
            inputs=SimpleNamespace(poses={k:dict(pose=p.car) for k in ('car','wand')})
            self.assertEqual(p.active('E',inputs,p.car,60.9),'')
            self.assertEqual(p.active('E',inputs,p.car,61.),'PILOT_TIME_LIMIT_60S')
    def test_deadline_cause_precedes_logical_key_release(self):
        channel=Channel(readonly=True)
        fake=SimpleNamespace(closing=False,channel=channel,status=dict(pilot=dict(capture_started_steady=1.)))
        GroundRig.before_operator_step(fake,60.99);self.assertFalse(channel.stopped)
        GroundRig.before_operator_step(fake,61.);self.assertEqual(channel.fault,'GROUND60_TIME_COMPLETE')
    def test_deadline_never_masks_prior_fault(self):
        channel=Channel(readonly=True);channel.stop('WAND_POSE_TIMEOUT')
        fake=SimpleNamespace(closing=False,channel=channel,status=dict(pilot=dict(capture_started_steady=1.)))
        GroundRig.before_operator_step(fake,61.);self.assertEqual(channel.fault,'WAND_POSE_TIMEOUT')
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
    def test_fresh_explicit_trial_review_only(self):verify_runtime_profile(authorized(),'sign')
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
        old=Path.read_text
        def read(path,*a,**kw):
            if path==READY/'HOST_MANIFEST.json':return json.dumps(dict(host_offline_passed=True,identity='4WSC'))
            return old(path,*a,**kw)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'p.json';path.write_text(json.dumps(authorized()));digest=hashlib.sha256(path.read_bytes()).hexdigest()
            with patch.object(Path,'read_text',read),patch('socket.socket',side_effect=AssertionError('IO before admission')):
                r=Runtime('hardware',str(path),digest,CONSENT);self.assertEqual((r.domain,r.namespace),(126,'/wand_4wsc_ground60'))
                with self.assertRaises(ValueError):Runtime('hardware',str(path),digest,'4WSC_SPACE_TOGGLE_600S_CLEAR_POWER_CUT_READY')
    def test_distinct_signing_scope(self):
        signer=AuthoritySigner('ab'*32,0);text=signer.encode(Velocity(),1.,101.)
        p=json.loads(text);self.assertIn('GROUND_COMMISSION_60S',p['payload']['scope'])
        AuthorityVerifier('ab'*32,0).decode(text,1.,101.)
    def test_native_stage0_selection_and_estop(self):
        c=Oracle();s=Stage0Session(lambda:123,stage=0);s.accept(c.request(s.new_hello(0)))
        for tick in range(10,611,10):c.request(s.cmd_vel(0,0,0,600,False,tick));c.step(tick)
        c.request(s.select_mode(610));c.step(620)
        self.assertEqual(c.status['stage'],0);self.assertFalse(c.status['permitted'])
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


if __name__=='__main__':unittest.main(verbosity=2)
