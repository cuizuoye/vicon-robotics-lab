import struct
import unittest
import contextlib
import io
from types import SimpleNamespace
from four_link_bench_probe import Probe, TransitionGuard, MessageType, FrameFlags, pack_cmd_vel, parse_args
from mecanum_vicon_control.bench_link_status import decode_bench_status, STATUS, TAG
from mecanum_vicon_control.transport_protocol import decode_frame, pack_control_enable

class Sink:
    def __init__(self): self.frames=[]
    def send(self,raw): self.frames.append(decode_frame(raw))
    def close(self): pass

class BenchTests(unittest.TestCase):
    def test_readonly_outbound_guard(self):
        for mode in ('no-power','logic-power','powered-stationary','post-run'):
            with self.subTest(mode=mode):
                s=Sink(); p=Probe(SimpleNamespace(mode=mode),s)
                p.zero()
                with self.assertRaises(ValueError): p.forward()
                with self.assertRaises(ValueError): p.send(MessageType.CONTROL_ENABLE,pack_control_enable(True))
                with self.assertRaises(ValueError): p.send(MessageType.CMD_VEL,pack_cmd_vel(1,0,0,150))
                self.assertEqual(len(s.frames),1); self.assertEqual(s.frames[0].flags,0)

    def test_power_confirmations_cannot_be_substituted(self):
        required={
            'no-power':'--confirm-battery-disconnected',
            'post-run':'--confirm-battery-disconnected',
            'logic-power':'--confirm-battery-powered-motors-disconnected',
            'powered-stationary':'--confirm-powered-suspended-stationary',
            'forward50':'--confirm-powered-suspended-stationary',
        }
        base=['--esp-ip','192.0.2.2','--bind-ip','192.0.2.1','--interface','test0','--output','test.json']
        confirms=sorted(set(required.values()))
        for mode,correct in required.items():
            for confirmation in confirms:
                with self.subTest(mode=mode,confirmation=confirmation):
                    argv=base+['--mode',mode,confirmation]
                    if confirmation==correct:
                        args=parse_args(argv)
                        self.assertEqual(args.mode,mode)
                        self.assertIsNone(args.serial)
                    else:
                        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                            parse_args(argv)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(base+['--mode',mode])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(base+['--mode','logic-power']+confirms)

    def test_logic_power_record_does_not_claim_battery_disconnected(self):
        args=parse_args(['--esp-ip','192.0.2.2','--bind-ip','192.0.2.1',
            '--interface','test0','--output','test.json','--mode','logic-power',
            '--confirm-battery-powered-motors-disconnected'])
        probe=Probe(args,Sink())
        self.assertTrue(probe.record['battery_powered_motors_disconnected_confirmed'])
        self.assertFalse(probe.record['battery_disconnected_confirmed'])
        self.assertFalse(probe.record['powered_suspended_stationary_confirmed'])
        self.assertFalse(probe.record['motion_attempted'])

    def test_motion_whitelist(self):
        s=Sink(); p=Probe(SimpleNamespace(mode='forward50'),s)
        p.forward()
        with self.assertRaises(ValueError): p.send(MessageType.CMD_VEL,pack_cmd_vel(0,0.05,0,150),FrameFlags.ENABLED)
        with self.assertRaises(ValueError): p.send(MessageType.CONTROL_ENABLE,pack_control_enable(False,True))
        self.assertEqual(len(s.frames),1)

    def test_transition_is_bounded_and_does_not_hide_new_fault_state(self):
        for moving in (False,True):
            g=TransitionGuard(moving,0.2)
            self.assertFalse(g.accept('telemetry',not moving,0.05))
            self.assertTrue(g.accept('telemetry',moving,0.06))
            with self.assertRaises(ValueError): g.accept('telemetry',not moving,0.07)
            self.assertFalse(g.accept('diagnostics',not moving,0.1))
            with self.assertRaises(ValueError): g.accept('diagnostics',not moving,0.2)
        with self.assertRaises(ValueError): TransitionGuard(False,0.201)
        with self.assertRaises(ValueError): TransitionGuard(False).accept('telemetry',True,0)

    def test_duplicate_reordered_and_wrapped_feedback(self):
        p=Probe(SimpleNamespace(mode='no-power'),Sink())
        kind=MessageType.WHEEL_TELEMETRY
        for seq,accepted in [(0xfffffffe,True),(0xfffffffe,False),(0xfffffffd,False),(0,True),(1,True)]:
            self.assertEqual(p.new_feedback(SimpleNamespace(message_type=kind,sequence=seq)),accepted)
        self.assertTrue(p.new_feedback(SimpleNamespace(message_type=MessageType.LINK_STATUS,sequence=0)))

    def test_status(self):
        v=[TAG,100,100,10,10,80,0,500]+[0]*8+[0]*4+[0]*4+[0]*4+[0]*3+[0,0,0,0]
        self.assertFalse(decode_bench_status(STATUS.pack(*v))['active'])
        for field,value in [(0,0),(4,31),(5,1001),(7,501),(8,1),(16,10),(20,280),(28,1),(31,16),(32,5),(33,1),(34,2)]:
            bad=v.copy(); bad[field]=value
            with self.assertRaises(ValueError): decode_bench_status(STATUS.pack(*bad))
        a=v.copy(); a[28]=50; a[34]=1; a[31]=15
        a[8:16]=[100,92,92,100,100,92,92,100]
        a[16:20]=[80,-80,80,-80]; a[20:24]=[281,278,280,281]
        self.assertTrue(decode_bench_status(STATUS.pack(*a))['active'])
        a[9]=99
        with self.assertRaises(ValueError): decode_bench_status(STATUS.pack(*a))

if __name__=='__main__': unittest.main()
