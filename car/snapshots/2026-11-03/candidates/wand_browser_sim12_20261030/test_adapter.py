"""Pure mocked publication/clock checks, no ROS context or desktop creation."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0,str(ROOT/'candidates/wand_sim12_20261030'))
import profile12
spec=importlib.util.spec_from_file_location('browser_sim12_operator',Path(__file__).with_name('browser_operator_source.py'))
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
sys.path.insert(0,str(ROOT/'docs/wand_browser_fast_2026-10-28/prepared'))
from control_channel import Channel
from lifecycle import waiting_expired


def packet(channel,t,sequence=1,events=None):
    return dict(type='pulse',nonce=channel.challenge(t)['nonce'],sequence=sequence,
        focused=True,visible=True,space=False,ui_gap_ms=20,events=events or [])


class AdapterTests(unittest.TestCase):
    def test_no_180s_cutoff_after_explicit_activation(self):
        self.assertFalse(waiting_expired(10.,191.,15.))
        self.assertFalse(waiting_expired(10.,609.,15.))
        self.assertTrue(waiting_expired(10.,191.,None))

    def test_bad_wait_clock_fails_closed(self):
        self.assertTrue(waiting_expired(10.,float('nan'),None))
        self.assertTrue(waiting_expired(10.,9.,None))

    def node(self):
        channel=Channel(readonly=True);channel.open('one',10.);channel.tick(10.)
        channel.receive('one',packet(channel,10.),10.005)
        node=mod.BrowserOperator.__new__(mod.BrowserOperator)
        node.channel=channel;node.last_generation=-1;node.last_held=None;node.last_publish=-float('inf')
        node.sequence=0;node.owner='test';out=[]
        node.lease_pub=NS(publish=lambda m:out.append(json.loads(m.data)))
        node.model=NS(held=True,tick=lambda _:None)
        node.model.end_trial=lambda *_:setattr(node.model,'held',False)
        return node,out

    def test_original_challenge_time_not_pump_time(self):
        n,out=self.node();n.clocks=lambda:(10.03,110.03);n.heartbeat()
        self.assertEqual(out[-1]['issued_steady'],10.)
        self.assertAlmostEqual(out[-1]['issued_ros'],110.)
        n.clocks=lambda:(10.06,110.06);n.heartbeat();self.assertEqual(len(out),1)

    def test_expired_proof_immediate_release(self):
        n,out=self.node();n.clocks=lambda:(10.099,110.099);n.heartbeat()
        n.clocks=lambda:(10.101,110.101);n.heartbeat()
        self.assertFalse(out[-1]['held']);self.assertEqual(len(out),2)

    def test_non_key_inputs_forbidden_by_channel(self):
        c=Channel(True);c.open('one',10.);c.tick(10.)
        self.assertFalse(c.receive('one',packet(c,10.,events=[dict(kind='setpoint',x=0.,y=0.,yaw=0.,visible=True)]),10.01))
        self.assertTrue(c.stopped)

    def test_stopped_owner_never_restores(self):
        c=Channel(True);c.open('one',10.);c.tick(10.)
        p=packet(c,10.);c.stop('TEST')
        self.assertFalse(c.receive('one',p,10.01));self.assertIsNone(c.evidence(10.01))

    def test_no_clock_refresh_after_real_task_gap(self):
        c=Channel(True);c.open('one',10.);c.tick(10.)
        c.receive('one',packet(c,10.),10.005);c.begin_tick(10.265)
        self.assertEqual(c.fault,'SERVER_CONTROL_LOOP_GAP')


if __name__=='__main__':unittest.main()
