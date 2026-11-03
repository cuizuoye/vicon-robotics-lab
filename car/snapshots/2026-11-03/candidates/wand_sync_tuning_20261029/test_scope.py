import json
import unittest
from model import Limits,ZERO,Velocity
from ros_tuning import Signer,Verifier,commissioning


class ScopeTests(unittest.TestCase):
    def test_old_and_sim_channels_mutually_reject(self):
        key='12'*32
        sim=Signer(key,Limits(1.2,4.,800.))
        old=commissioning.AuthoritySigner(key,4)
        with self.assertRaises(ValueError):commissioning.AuthorityVerifier(key,4).decode(sim.encode(ZERO,10.,100.),10.,100.)
        with self.assertRaises(ValueError):Verifier(key,Limits(1.2,4.,800.)).decode(old.encode(ZERO,10.,100.),10.,100.)

    def test_profile_mismatch_rejected(self):
        key='12'*32
        signer=Signer(key,Limits(.7,2.,400.))
        with self.assertRaises(ValueError):Verifier(key,Limits(1.2,4.,800.)).decode(signer.encode(ZERO,10.,100.),10.,100.)

    def test_expiry_owner_replay_tamper_and_caps_preserved(self):
        for mode in ('expiry','owner','replay','tamper','speed','rpm'):
            with self.subTest(mode=mode):
                key='12'*32;c=Limits(.7,2.,400.);s=Signer(key,c);v=Verifier(key,c)
                zero=s.encode(ZERO,10.,100.);v.decode(zero,10.,100.)
                if mode=='expiry':wire=s.encode(ZERO,9.,99.)
                elif mode=='owner':wire=Signer(key,c).encode(ZERO,10.01,100.01)
                elif mode=='replay':wire=zero
                elif mode=='speed':wire=s.encode(Velocity(.8,0.,0.),10.01,100.01,epoch='e',state='ACTIVE',permit=True)
                elif mode=='rpm':
                    # RPM test uses a higher vector cap but retains 400RPM.
                    c=Limits(1.2,2.,400.);s=Signer(key,c);v=Verifier(key,c)
                    v.decode(s.encode(ZERO,10.,100.),10.,100.)
                    wire=s.encode(Velocity(.8,.8,0.),10.01,100.01,epoch='e',state='ACTIVE',permit=True)
                else:
                    data=json.loads(s.encode(ZERO,10.01,100.01));data['signature']='0'*64;wire=json.dumps(data)
                with self.assertRaises(ValueError):v.decode(wire,10.01,100.01)


if __name__=='__main__':unittest.main(verbosity=2)
