"""No physical IO. Test actual raw-frame intake and original cache deadlines."""
import copy
import json
import math
import queue
import sys
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from build import OUT

sys.path.insert(0, str(OUT/'host'))
import bootstrap
bootstrap.install()
from geometry import synthetic_packet
from mecanum_wand_4wsc_hardware import source_pipeline as p
from mecanum_wand_4wsc_hardware.source_relay import SourceRelay
from mecanum_wand_4wsc_control.rotation_source import Raw
from mecanum_wand_4wsc_control.rotation_geometry import REQUIRED


def clock(ros):
    return NS(now=lambda:NS(nanoseconds=round(ros*1e9)))

def header(ros):
    sec = int(ros)
    return NS(frame_id='map', stamp=NS(sec=sec, nanosec=round((ros-sec)*1e9)))

def raw_fixture(ros):
    node = object.__new__(p.BufferedRawSource)
    node.frames = {};node.packet = None;node.completed_frames = 0
    node.callback_generation = 0;node.discarded_unselected = 0
    node.samples = {};node.last_stamps = {};node.rejected = {}
    node.get_clock = lambda:clock(ros)
    return node


class IntakeTests(unittest.TestCase):
    def test_complete_same_frame_cohort_is_used(self):
        now = time.monotonic();ros = time.time();node = raw_fixture(ros)
        packet = synthetic_packet(1., 1., 0., now, ros)
        for s in packet.samples:
            node.raw_sample(s.name, s.position, s.quaternion, NS(header=header(ros)))
        self.assertEqual(node.completed_frames, 1)
        self.assertEqual(tuple(s.name for s in node.packet.samples), REQUIRED)
        self.assertEqual(node.packet.stamp, packet.stamp)
        self.assertLess(abs(node.packet.pose(now+.01, ros+.01).x-1.), 1e-9)

    def test_same_pose_new_stamp_is_a_new_frame(self):
        node = raw_fixture(1000.)
        for i in range(20):
            ros = 1000.+i*.005;node.get_clock = lambda ros=ros:clock(ros)
            packet = synthetic_packet(1., 1., 0., time.monotonic(), ros)
            for s in packet.samples:
                node.raw_sample(s.name, s.position, s.quaternion, NS(header=header(ros)))
        self.assertEqual(node.completed_frames, 20)
        self.assertEqual(node.rejected, {})

    def test_duplicate_does_not_renew_receipt(self):
        ros = 1000.;node = raw_fixture(ros)
        packet = synthetic_packet(1., 1., 0., time.monotonic(), ros)
        s = packet.samples[0];msg = NS(header=header(ros))
        node.raw_sample(s.name, s.position, s.quaternion, msg)
        before = node.frames[ros][s.name]
        node.raw_sample(s.name, s.position, s.quaternion, msg)
        self.assertIs(node.frames[ros][s.name], before)

    def test_same_stamp_conflict_rejected(self):
        ros = 1000.;node = raw_fixture(ros);msg = NS(header=header(ros))
        node.raw_sample('Sign1', (0., 0., 0.), None, msg)
        with self.assertRaisesRegex(ValueError, 'SAME_STAMP_CONFLICT'):
            node.raw_sample('Sign1', (.01, 0., 0.), None, msg)

    def test_unselected_expired_is_discarded_not_retimestamped(self):
        node = raw_fixture(1000.3)
        node.raw_sample('Sign1', (0., 0., 0.), None, NS(header=header(1000.)))
        self.assertIsNone(node.packet);self.assertEqual(node.frames, {})
        self.assertEqual(node.discarded_unselected, 1)

    def test_future_frame_rejected(self):
        node = raw_fixture(1000.)
        with self.assertRaises(ValueError):
            node.raw_sample('Sign1', (0., 0., 0.), None, NS(header=header(1000.02)))

    def test_selected_cache_still_expires_at_200ms(self):
        now = time.monotonic();ros = time.time()
        packet = synthetic_packet(1., 1., 0., now, ros)
        packet.pose(now+.199, ros+.199)
        with self.assertRaisesRegex(ValueError, 'AGE_OR_MIXED_FRAME'):
            packet.pose(now+.201, ros+.201)

    def test_no_mixed_frames(self):
        node = raw_fixture(1000.01)
        packet = synthetic_packet(1., 1., 0., time.monotonic(), 1000.)
        for i,s in enumerate(packet.samples):
            stamp = 1000. if i%2 else 1000.005
            node.raw_sample(s.name, s.position, s.quaternion, NS(header=header(stamp)))
        self.assertIsNone(node.packet)

    def test_geometry_rejection_kept(self):
        ros = 1000.;node = raw_fixture(ros)
        packet = synthetic_packet(1., 1., 0., time.monotonic(), ros)
        with self.assertRaisesRegex(ValueError, 'GEOMETRY_OR_ATTITUDE_FLIP'):
            for i,s in enumerate(packet.samples):
                pos = (s.position[0]+.05, *s.position[1:]) if i==2 else s.position
                node.raw_sample(s.name, pos, s.quaternion, NS(header=header(ros)))

    def test_drains_cohort_not_one_callback(self):
        node = NS(callback_generation=0);remaining = [30]
        def spin(**kw):
            if remaining[0]:remaining[0]-=1;node.callback_generation+=1
        count, _ = p.drain_callbacks(NS(spin_once=spin), node)
        self.assertEqual(count, 30)

    def test_drain_is_bounded(self):
        node = NS(callback_generation=0)
        def spin(**kw):node.callback_generation+=1
        count, _ = p.drain_callbacks(NS(spin_once=spin), node, maximum=12)
        self.assertEqual(count, 12)

    def test_ipc_drops_old_before_fresh(self):
        q = queue.Queue();now = time.monotonic()
        q.put(dict(kind='data',issued=now-1));q.put(dict(kind='data',issued=now))
        reader = NS(channel=q, value=None, process=NS(is_alive=lambda:True), ipc_rejected=0, discarded_ipc=0)
        self.assertEqual(p.poll_latest(reader)['issued'], now)
        self.assertEqual(reader.discarded_ipc, 1);self.assertEqual(reader.ipc_rejected, 0)

    def test_ipc_error_still_rejected(self):
        q = queue.Queue();q.put(dict(kind='error',error='SOURCE_FAULT'))
        reader = NS(channel=q, value=None, process=NS(is_alive=lambda:True), ipc_rejected=0)
        with self.assertRaisesRegex(ValueError, 'SOURCE_IPC_ERROR'):p.poll_latest(reader)
        self.assertEqual(reader.ipc_rejected, 1)

    def test_ipc_replay_cannot_renew(self):
        q = queue.Queue();old = dict(kind='data',issued=time.monotonic()-.3)
        reader = NS(channel=q, value=old, process=NS(is_alive=lambda:True), ipc_rejected=0, discarded_ipc=0)
        self.assertIsNone(p.poll_latest(reader));self.assertEqual(reader.value, old)

    def test_graph_audit_does_not_block_callbacks(self):
        audit = p.GraphAudit(lambda:(time.sleep(.12) or []))
        try:
            node = NS(callback_generation=0)
            def spin(**kw):node.callback_generation+=1
            started = time.monotonic()
            count, _ = p.drain_callbacks(NS(spin_once=spin), node, maximum=64)
            self.assertEqual(count, 64);self.assertLess(time.monotonic()-started, .05)
            deadline = time.monotonic()+1
            while not audit.snapshot(time.monotonic())[1]['ready'] and time.monotonic()<deadline:time.sleep(.005)
            self.assertTrue(audit.snapshot(time.monotonic())[1]['ready'])
            self.assertTrue(audit.snapshot(time.monotonic()+1)[0])
        finally:self.assertTrue(audit.close())


class RelayTests(unittest.TestCase):
    def fixture(self):
        now = time.monotonic();ros = time.time()
        data = dict(packet=synthetic_packet(1.,1.,0.,now,ros).payload(), car=None, conflicts=[],rejected={})
        publications = []
        pub = NS(publish=lambda msg:publications.append(msg))
        relay = NS(reader=NS(poll=lambda:data),runtime=NS(yaw_offset=0.),
            last_car=None,last_packet=None,cached_wire=None,cached_packet=None,
            rejected=0,last_status=None,status_at=-math.inf,
            packet_pub=pub,wand_pub=pub,car_pub=pub,mode_pub=pub,info=pub,
            get_clock=lambda:clock(ros), header=lambda v:header(v).stamp)
        return relay,data,now,ros

    def test_expired_validated_reuse_not_geometry_fault(self):
        relay, data, now, ros = self.fixture()
        SourceRelay.tick(relay)
        stamp = relay.last_packet
        relay.get_clock = lambda:clock(ros+.21)
        with patch('mecanum_wand_4wsc_hardware.source_relay.time.monotonic',return_value=now+.21):
            SourceRelay.tick(relay)
        self.assertEqual(relay.rejected, 0)
        self.assertEqual(relay.last_packet, stamp)

    def test_new_mixed_packet_is_a_real_rejection(self):
        relay, data, _, _ = self.fixture()
        data['packet']['samples'][1]['stamp']-=.001
        SourceRelay.tick(relay)
        self.assertEqual(relay.rejected, 1)

    def test_new_bad_geometry_is_a_real_rejection(self):
        relay, data, _, _ = self.fixture()
        point = data['packet']['samples'][1]['position']
        data['packet']['samples'][1]['position'] = (point[0]+.05, *point[1:])
        SourceRelay.tick(relay)
        self.assertEqual(relay.rejected, 1)


class ControlGraphTests(unittest.TestCase):
    def test_slow_graph_does_not_block_main_control(self):
        core = NS(inputs=NS(graph_ok=False))
        fixture = NS(core=core,inspect_graph=lambda:(time.sleep(.12) or []),
                     runtime=NS(domain=196,namespace='/test',expected={}))
        with patch.object(p,'GraphProcessAudit',side_effect=lambda *args:p.GraphAudit(fixture.inspect_graph)):
            started=time.monotonic();p.AsyncGraphMixin.check_graph(fixture)
        self.assertLess(time.monotonic()-started,.05)
        self.assertFalse(core.inputs.graph_ok)
        try:
            deadline=time.monotonic()+1.
            while not core.inputs.graph_ok and time.monotonic()<deadline:
                p.AsyncGraphMixin.check_graph(fixture);time.sleep(.005)
            self.assertTrue(core.inputs.graph_ok)
            # Thread may inspect only; it must not mutate the core itself.
            core.inputs.graph_ok=False;time.sleep(.13)
            self.assertFalse(core.inputs.graph_ok)
            p.AsyncGraphMixin.check_graph(fixture);self.assertTrue(core.inputs.graph_ok)
        finally:fixture._independent_graph_audit.close()

    def test_identity_conflict_still_revokes(self):
        fixture=NS(core=NS(inputs=NS(graph_ok=True)),inspect_graph=lambda:['wrong publisher'],
                   runtime=NS(domain=196,namespace='/test',expected={}))
        with patch.object(p,'GraphProcessAudit',side_effect=lambda *args:p.GraphAudit(fixture.inspect_graph)):
            p.AsyncGraphMixin.check_graph(fixture)
        try:
            time.sleep(.02);p.AsyncGraphMixin.check_graph(fixture)
            self.assertFalse(fixture.core.inputs.graph_ok)
        finally:fixture._independent_graph_audit.close()


class DuplicateZeroTests(unittest.TestCase):
    def check(self, change=None, expected=1, second=1.001):
        from mecanum_wand_4wsc_hardware.optimized_nodes import Gate,RotationGateNode
        from mecanum_wand_4wsc_control.safety_core import ZERO
        node=object.__new__(Gate)
        frame=NS(source='test',sequence=1,epoch='epoch',state='ACTIVE',enabled=True)
        node.core=NS(frame=frame,fault='',hard_fault=False,primed_epoch='epoch')
        calls=[]
        with patch.object(RotationGateNode,'publish_output',lambda _,output:calls.append(output)):
            with patch('mecanum_wand_4wsc_hardware.optimized_nodes.time.monotonic',return_value=1.):node.publish_output(ZERO)
            if change:change(node)
            with patch('mecanum_wand_4wsc_hardware.optimized_nodes.time.monotonic',return_value=second):node.publish_output(ZERO)
        self.assertEqual(len(calls),expected)
    def test_same_active_zero_frame_coalesced(self):self.check()
    def test_new_frame_still_published(self):self.check(lambda n:setattr(n.core.frame,'sequence',2),2)
    def test_fault_zero_always_published(self):self.check(lambda n:setattr(n.core,'fault','FAULT'),2)
    def test_stop_zero_always_published(self):self.check(lambda n:setattr(n.core.frame,'state','STOPPING'),2)
    def test_disabled_zero_always_published(self):self.check(lambda n:setattr(n.core.frame,'enabled',False),2)
    def test_zero_timer_before_next_event_is_also_coalesced(self):self.check(second=1.015)
    def test_valid_zero_has_bounded_heartbeat(self):self.check(expected=2,second=1.025)


if __name__ == '__main__':
    import math
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    print(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,physical_output=False)))
    raise SystemExit(0 if result.wasSuccessful() else 1)
