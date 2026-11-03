"""Offline protocol/admission tests. Uses exact C oracle; never a device socket."""
import hashlib
from dataclasses import asdict
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
READY=Path(__import__('os').environ.get('WAND_4WSC_TEST_READY',str(ROOT/'docs/wand_4wsc_browser_2026-11-02/prepared_v1')))
sys.path.insert(0,str(READY))
import bootstrap
from native_peer import Oracle
from idle_probe import zero_only
from mecanum_wand_4wsc_hardware.authorization import verify_flash_record,verify_runtime_profile,endpoint,IMAGE_SHA
from mecanum_wand_4wsc_hardware.stage0_session import Stage0Session
from mecanum_wand_4wsc_hardware.stage0_contract import decode_status_frame,disabled_motion_status_ready
from mecanum_wand_4wsc_hardware.startup_clock import StartupProgressSocket
from mecanum_wand_4wsc_hardware.commissioning import AuthoritySigner,AuthorityVerifier,TransportPolicy
from mecanum_wand_4wsc_control.core import Velocity
from mecanum_wand_4wsc_control.tracking_config import TrackingSafetyConfig
from mecanum_wand_4wsc_hardware.stages import get_stage
from mecanum_vicon_control.transport_protocol import decode_frame,encode_frame,MessageType,FrameFlags,ProtocolError
RECORD=ROOT/'docs/wand_firmware_4wsc_2026-10-30/DEPLOYMENT_2026-11-02.json'

class FakeSocket:
    def __init__(self):self.sent=[]
    def sendto(self,data,dest):self.sent.append(data);return len(data)

class HostTests(unittest.TestCase):
    def test_all_ten_exact_stage_configs(self):
        for i in range(10):
            p=json.loads((READY/'profiles'/f'stage{i}.NOT_APPROVED.json').read_text())
            c=TrackingSafetyConfig(**p['safety']);self.assertEqual(c.stage,i)
            self.assertTrue(c.direct_sign_target);self.assertFalse(c.follower_config().workspace_radius_enabled)
            self.assertEqual(asdict(c.limits),asdict(get_stage(i)))
    def test_fresh_flash_proof_verified(self):
        r=verify_flash_record(RECORD);self.assertEqual(r['image_sha256'],IMAGE_SHA)
    def test_old_image_record_not_admitted(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.json';r=json.loads(RECORD.read_text());r['identity']='4WSB';p.write_text(json.dumps(r))
            with self.assertRaises(ValueError):verify_flash_record(p)
    def test_drafts_block_before_io(self):
        for i in range(10):
            p=json.loads((READY/'profiles'/f'stage{i}.NOT_APPROVED.json').read_text())
            with self.assertRaises(ValueError):verify_runtime_profile(p,'sign')
    def test_loopback_cannot_be_real_endpoint(self):
        p={'hardware':dict(esp_ip='127.0.0.1',network_interface='lo',command_port=52001,telemetry_port=52002)}
        self.assertEqual(endpoint(p,rehearsal=True)[0],'127.0.0.1')
        with self.assertRaises(ValueError):endpoint(p)
        p['hardware']['esp_ip']='192.168.50.1'
        with self.assertRaises(ValueError):endpoint(p,rehearsal=True)
    def test_stage_boolean_and_mixed_limits_rejected(self):
        for value in (True,False,-1,10,1.0):
            with self.assertRaises(ValueError):get_stage(value)
        p=json.loads((READY/'profiles/stage0.NOT_APPROVED.json').read_text());p['safety']['max_accel_mps2']=4.
        with self.assertRaises(ValueError):TrackingSafetyConfig(**p['safety'])
    def test_c_payloads_all_ten_disabled_selection(self):
        for i in range(10):
            c=Oracle();s=Stage0Session(lambda:123,stage=i);s.accept(c.request(s.new_hello(0)))
            for t in range(10,611,10):
                c.request(s.cmd_vel(0,0,0,600,False,t));c.step(t)
            c.request(s.select_mode(610));c.step(620)
            d=decode_status_frame(c.packet(0x31));self.assertEqual(d['tag'],'4WSC')
            self.assertEqual(d['output_authority_version'],8);self.assertEqual(d['stage'],i)
            self.assertTrue(d['selected']);self.assertTrue(disabled_motion_status_ready(d))
    def test_all_fixed_stage_words_pass_clock_guard(self):
        for i in range(10):
            s=Stage0Session(lambda:123,stage=i);s.new_hello(0);s.connected=True
            fake=FakeSocket();w=StartupProgressSocket(fake,'127.0.0.1',lambda:123,stage=i)
            w.clock.session_id=123;w.clock.qualified=True;w.clock.frozen=True;w.clock.ready=lambda _:True
            for raw in (s.select_mode(1000),s.control_enable(True,False,1001),s.control_enable(False,False,1002)):
                w.sendto(raw,('127.0.0.1',52000))
            self.assertEqual(len(fake.sent),3)
    def test_other_stage_and_old_word_rejected(self):
        w=StartupProgressSocket(FakeSocket(),'127.0.0.1',lambda:123,stage=9)
        w.clock.session_id=123;w.clock.qualified=True;w.clock.frozen=True;w.clock.ready=lambda _:True
        for word in (0x7804,0x7B00,0x7C08,0x7C80,0):
            raw=encode_frame(MessageType.CONTROL_ENABLE,sequence=2,session_id=123,sender_time_ms=1000,
                flags=FrameFlags.ACK_REQUIRED,payload=struct.pack('<BBH',1,0,word))
            with self.assertRaises(OSError):w.sendto(raw,('127.0.0.1',52000))
    def test_probe_has_no_enable_select_clear_path(self):
        s=Stage0Session(lambda:123);hello=s.new_hello(0);zero_only(hello);s.connected=True
        zero_only(s.cmd_vel(0,0,0,600,False,10))
        for raw in (s.select_mode(20),s.control_enable(True,False,30),s.control_enable(False,True,40),
                s.cmd_vel(.01,0,0,600,False,50),s.cmd_vel(0,0,0,600,True,60),s.estop(70)):
            with self.assertRaises(ValueError):zero_only(raw)
    def test_authority_all_ten_tiers_and_cross_scope(self):
        for i in range(10):
            signer=AuthoritySigner('ab'*32,i);v=AuthorityVerifier('ab'*32,i)
            v.decode(signer.encode(Velocity(),1.,101.),1.,101.)
            p=json.loads(signer.encode(Velocity(),1.01,101.01));self.assertIn('4WSC_V8',p['payload']['scope'])
            other=AuthorityVerifier('ab'*32,(i+1)%10)
            with self.assertRaises(ValueError):other.decode(json.dumps(p),1.01,101.01)
    def test_replay_future_signature_rejected(self):
        signer=AuthoritySigner('ab'*32,0);v=AuthorityVerifier('ab'*32,0)
        text=signer.encode(Velocity(),1.,101.);v.decode(text,1.,101.)
        with self.assertRaises(ValueError):v.decode(text,1.01,101.01)
        text=signer.encode(Velocity(),3.,103.)
        with self.assertRaises(ValueError):v.decode(text,1.,101.)
        bad=json.loads(signer.encode(Velocity(),1.,101.));bad['signature']='0'*64
        with self.assertRaises(ValueError):AuthorityVerifier('ab'*32,0).decode(json.dumps(bad),1.,101.)
    def test_zero_never_acquires_motion_epoch(self):
        policy=TransportPolicy('ab'*32,0);signer=AuthoritySigner('ab'*32,0)
        for j in range(20):
            t=1.+j*.02;policy.receive(signer.encode(Velocity(),t,100.+t),t,100.+t)
            self.assertFalse(policy.active(t));self.assertFalse(policy.epoch)
    def test_feedback_pair_cannot_repeat_minimum_stamp(self):
        from mecanum_wand_4wsc_hardware.feedback_pair import paired_advance
        self.assertTrue(paired_advance((1.,1.),None))
        self.assertFalse(paired_advance((1.02,1.),(1.,1.)))
        self.assertFalse(paired_advance((1.,1.02),(1.,1.)))
        self.assertTrue(paired_advance((1.02,1.02),(1.,1.)))
        self.assertFalse(paired_advance((.99,1.03),(1.,1.)))
    def test_used_budget_only_admits_disabled_preparation(self):
        from mecanum_wand_4wsc_control.idle_admission import zero_preparation_ready
        self.assertTrue(zero_preparation_ready(dict(healthy_budget_used=True,native_disabled_zero=True,native_fault_bits=0)))
        for t in (dict(healthy_budget_used=True,native_disabled_zero=False,native_fault_bits=0),
                dict(healthy_budget_used=True,native_disabled_zero=True,native_fault_bits=1)):
            self.assertFalse(zero_preparation_ready(t))

if __name__=='__main__':unittest.main(verbosity=2)
