"""Offline backend-authority tests. No hardware sockets or ROS initialization."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest
from build import HERE,OUT
sys.path.insert(0,str(OUT/'host'))
import bootstrap
from session_channel import Channel
from session_model import SessionOperatorModel
from mecanum_wand_4wsc_control.demo_policy import INPUT_MODE
from mecanum_wand_4wsc_control.pilot_core import POLICY


def live_channel(now=10.):
    c=Channel();c.open('owner',now);c.begin_tick(now);c.backend_tick(now,now);c.ready=True
    v=c.challenge(now);return c,dict(type='command',action='start',nonce=v['nonce'],sequence=1)


class ChannelTests(unittest.TestCase):
    def test_no_browser_packet_does_not_start(self):
        c,_=live_channel()
        for n in range(100):
            t=10+n*.02;c.begin_tick(t);c.backend_tick(t,t);c.finish_tick(t)
        self.assertFalse(c.session_authorized(t));self.assertFalse(c.stopped)
    def test_single_explicit_start_continues_without_ui_messages(self):
        c,v=live_channel();self.assertTrue(c.receive('owner',v,10.))
        for n in range(1,3001):
            t=10+n*.02;c.begin_tick(t);c.backend_tick(t,t);c.finish_tick(t)
            self.assertTrue(c.session_authorized(t))
        self.assertEqual(c.accepted_commands,1);self.assertIsNone(c.proof)
    def test_expired_start_never_late_starts(self):
        c,v=live_channel();c.begin_tick(11.01);c.backend_tick(11.01,11.01)
        self.assertFalse(c.receive('owner',v,11.01));self.assertEqual(c.fault,'START_EXPIRED_OR_REPLAYED')
        self.assertFalse(c.intent)
    def test_duplicate_start_cannot_resume(self):
        c,v=live_channel();c.receive('owner',v,10.);self.assertFalse(c.receive('owner',v,10.01))
        self.assertTrue(c.stopped);self.assertFalse(c.intent)
    def test_stop_works_without_nonce_or_new_sequence(self):
        c,v=live_channel();c.receive('owner',v,10.)
        c.receive('owner',dict(type='stop',reason='ESC'),10.01)
        self.assertEqual(c.fault,'CLIENT_ESC');self.assertFalse(c.session_authorized(10.01))
        c.ready=True;self.assertFalse(c.receive('owner',v,10.02))
    def test_estop_preserves_latch_request(self):
        c,_=live_channel();c.receive('owner',dict(type='stop',reason='ESTOP'),10.)
        self.assertEqual(c.fault,'OPERATOR_ESTOP')
    def test_wrong_owner_has_no_authority(self):
        c,v=live_channel();self.assertFalse(c.receive('other',v,10.));self.assertFalse(c.intent)
    def test_web_disconnect_still_stops(self):
        c,v=live_channel();c.receive('owner',v,10.);c.disconnect('owner');self.assertTrue(c.stopped)
    def test_control_gap_stops_before_new_input(self):
        c,v=live_channel();c.receive('owner',v,10.);c.begin_tick(10.101);c.backend_tick(10.101,10.101)
        self.assertEqual(c.fault,'SERVER_CONTROL_LOOP_GAP');self.assertFalse(c.session_authorized(10.101))
    def test_backend_command_path_timeout_stops(self):
        c,v=live_channel();c.receive('owner',v,10.)
        for t in (10.04,10.08,10.101):c.begin_tick(t);c.finish_tick(t)
        self.assertEqual(c.fault,'BACKEND_COMMAND_CHANNEL_TIMEOUT')
    def test_stale_queued_backend_tick_cannot_refresh(self):
        c,v=live_channel();c.receive('owner',v,10.);self.assertFalse(c.backend_tick(10.,10.101))
        self.assertFalse(c.session_authorized(10.101))
    def test_old_ui_pulse_and_external_setpoint_rejected(self):
        for v in (dict(type='pulse',nonce='old'),dict(type='setpoint',x=1,y=2),dict(type='command',action='start',nonce='x',sequence=True)):
            c,_=live_channel();self.assertFalse(c.receive('owner',v,10.));self.assertTrue(c.stopped)


def model_ready():
    m=SessionOperatorModel();m.backend_available=True
    m.receive_status(dict(state='ARMED',gate_ready=True,pilot=dict(POLICY,used=False,operator_input_mode=INPUT_MODE)),10.)
    m.receive_gate(dict(fault='',hard_fault=False),10.)
    return m


class ModelTests(unittest.TestCase):
    def test_unowned_startup_delay_is_not_a_motion_session(self):
        m=model_ready();m.backend_available=False;m.tick(5.);m.tick(10.)
        self.assertFalse(m.ended);self.assertFalse(m.held)
        m.backend_available=True;self.assertTrue(m.request_session(10.))
    def test_no_fake_focus_or_key_is_needed(self):
        m=model_ready();self.assertFalse(m.focused);self.assertFalse(m.key_down)
        self.assertTrue(m.request_session(10.));self.assertFalse(m.focused);self.assertFalse(m.key_down)
    def test_stale_status_and_fault_block_start(self):
        m=model_ready();self.assertFalse(m.request_session(10.251))
        for target,key,value in (('status','hard_fault',True),('status','input_health','WAND_POSE_TIMEOUT'),('gate','fault','COMMAND_TIMEOUT')):
            m=model_ready();getattr(m,target)[key]=value;self.assertFalse(m.request_session(10.))
    def test_main_loop_gap_revokes_logical_session(self):
        m=model_ready();m.request_session(10.);m.tick(10.);m.tick(10.101)
        self.assertTrue(m.ended);self.assertFalse(m.held)
    def test_no_automatic_recovery(self):
        m=model_ready();m.request_session(10.);m.end_trial(10.01,'test stop')
        self.assertFalse(m.request_session(10.02));self.assertFalse(m.can_recover(10.02))
    def test_backend_unavailable_revokes(self):
        m=model_ready();m.request_session(10.);m.backend_available=False;m.tick(10.01)
        self.assertFalse(m.held);self.assertTrue(m.ended)


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__]);n=suite.countTestCases()
    r=unittest.TextTestRunner(verbosity=1).run(suite)
    print(json.dumps(dict(passed=r.wasSuccessful(),checks=n,hardware_output=False,ROS_initialized=False)))
    if not r.wasSuccessful():raise SystemExit(1)
