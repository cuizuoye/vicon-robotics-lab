import math
import socket
import subprocess
import unittest
from unittest.mock import patch
from model import (Limits, Harness, AcceptedHarness, ZERO, shape, Velocity,
                   accepted_config, protected_hashes, wheel_rpm)


class ModelTests(unittest.TestCase):
    def test_accepted_profile_equivalence(self):
        old, new = AcceptedHarness(True), Harness(Limits())
        old.start();new.start()
        for i in range(800):
            t = i*.02
            for h in (old,new):
                h.gesture(.2*math.sin(t),.12*(1-math.cos(1.4*t)),.04*math.sin(t))
                h.step()
            for a,b in zip(vars(old.output).values(),vars(new.output).values()):
                self.assertAlmostEqual(a,b,places=12)
            self.assertEqual(old.sup.state,new.sup.state)
            self.assertEqual(old.gate.fault,new.gate.fault)

    def test_guards_unchanged(self):
        accepted = accepted_config()
        c = Limits(1.2,4.,800.)
        for name in vars(accepted):
            if name not in ('max_speed_mps','max_accel_mps2'):
                self.assertEqual(getattr(c,name),getattr(accepted,name),name)
        self.assertEqual(c.follower_config().scale,1.)
        self.assertFalse(c.follower_config().workspace_radius_enabled)

    def test_profile_enumeration_only(self):
        for args in ((True,2.,400.),(.7,float('nan'),400.),(1.3,2.,400.),(.7,2.,900.)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                Limits(*args)

    def test_coupled_shaping_respects_profile_and_rpm(self):
        c = Limits(1.5,6.,800.).follower_config()
        old = ZERO
        for i in range(500):
            v = shape(Velocity(2.*math.sin(i*.1),2.*math.cos(i*.1),.15*math.sin(i*.05)),old,.02,c)
            self.assertLessEqual(math.hypot(v.vx,v.vy),1.5+1e-9)
            self.assertLessEqual(math.hypot(v.vx-old.vx,v.vy-old.vy),6*.02+1e-9)
            self.assertLessEqual(max(abs(x) for x in wheel_rpm(v)),800+1e-9)
            old = v

    def test_occlusion_stops_without_auto_resume(self):
        for c in (Limits(),Limits(.7,2.,400.),Limits(1.2,4.,800.)):
            with self.subTest(profile=c.name):
                h = Harness(c);h.start()
                for i in range(50):h.gesture(i*.005,0.);h.step()
                h.pose_on['wand'] = False;h.run(.3)
                self.assertEqual(h.output,ZERO)
                self.assertNotEqual(h.sup.state,'ACTIVE')
                h.pose_on['wand'] = True;h.run(1.5)
                self.assertEqual(h.output,ZERO)

    def test_time_lease_feedback_guards_survive_high_limits(self):
        for mode in ('lease','feedback','car','jump'):
            h = Harness(Limits(1.2,4.,800.));h.start()
            for i in range(25):h.gesture(i*.004,0.);h.step()
            if mode == 'lease':h.lease_on=False
            elif mode == 'feedback':h.feedback_on=False
            elif mode == 'car':h.pose_on['car']=False
            else:h.gesture(1.,1.)
            h.run(.5)
            self.assertEqual(h.output,ZERO,mode)
            self.assertNotEqual(h.sup.state,'ACTIVE',mode)

    def test_no_live_io(self):
        before = protected_hashes()
        def deny(*args,**kwargs):raise AssertionError('unexpected external IO')
        with patch.object(socket,'socket',deny),patch.object(subprocess,'Popen',deny):
            h=Harness(Limits(.7,2.,400.));h.start();h.gesture(.1,0.);h.step()
        self.assertEqual(before,protected_hashes())


if __name__ == '__main__':unittest.main(verbosity=2)
