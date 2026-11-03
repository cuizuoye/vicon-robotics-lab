"""New boundary/unit regressions. No real Sign or motor output required."""
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import unittest

from bootstrap import ZERO,Velocity,config,protected_hashes
from ros_chain import synthetic_packet
from live_reader import RealReader
from mecanum_wand_4ws8_hardware.commissioning import AuthoritySigner,AuthorityVerifier


def reader_fixture():
    reader=object.__new__(RealReader)
    reader.lock=threading.Lock();reader.packet=None;reader.rejected=reader.received=0
    reader.frames={};reader.latest={};reader.flags={'sign':True};reader.error=''
    reader.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=time.time_ns())))
    return reader


def deliver(reader,packet):
    for sample in packet.samples:
        sec=int(sample.stamp);ns=round((sample.stamp-sec)*1e9)
        msg=NS(header=NS(frame_id='map',stamp=NS(sec=sec,nanosec=ns)))
        p=NS(x=sample.position[0],y=sample.position[1],z=sample.position[2])
        if sample.name=='sign':
            q=NS(**dict(zip(('x','y','z','w'),sample.quaternion)));msg.pose=NS(position=p,orientation=q)
        else:msg.point=p
        reader.receive(sample.name,msg)


class ExtensionTests(unittest.TestCase):
    def test_original_limits_and_new_module_separation(self):
        c=config()
        self.assertEqual((c.max_speed_mps,c.max_accel_mps2,c.pose_timeout_s,c.command_timeout_s),(.7,.8,.2,.1))
        self.assertEqual((c.max_yaw_rate_radps,c.max_yaw_accel_radps2),(.1,.4))
        self.assertGreater(len(protected_hashes()),230)

    def test_real_boundary_has_no_control_publisher_or_hardware_api(self):
        text=Path(__file__).with_name('live_reader.py').read_text()
        for forbidden in ('create_publisher(', 'create_client(', 'sendto(', 'Serial(', 'HardwareBridge('):
            self.assertNotIn(forbidden,text)

    def test_real_packet_reused_without_stamp_refresh(self):
        r=reader_fixture();p=synthetic_packet(0.,0.,0.,time.monotonic(),time.time())
        deliver(r,p);self.assertIsNotNone(r.packet);old=r.packet
        deliver(r,p);self.assertIs(r.packet,old)
        self.assertEqual(r.received,1)

    def test_mixed_raw_frames_do_not_form_a_packet(self):
        r=reader_fixture();p=synthetic_packet(0.,0.,0.,time.monotonic(),time.time())
        rows=list(p.samples);rows[-1]=replace(rows[-1],stamp=rows[-1].stamp+.01)
        deliver(r,type(p)(tuple(rows)));self.assertIsNone(r.packet)

    def test_geometry_and_future_frames_are_rejected(self):
        for mode in ('geometry','future'):
            with self.subTest(mode=mode):
                r=reader_fixture();p=synthetic_packet(0.,0.,0.,time.monotonic(),time.time()+(.03 if mode=='future' else 0.))
                rows=list(p.samples)
                if mode=='geometry':rows[-1]=replace(rows[-1],position=(99.,99.,1.))
                deliver(r,type(p)(tuple(rows)))
                self.assertIsNone(r.packet);self.assertGreater(r.rejected,0)

    def test_z_ignored_and_real_sign5_translation_unchanged(self):
        a=synthetic_packet(.3,-.2,0.,time.monotonic(),time.time(),1.)
        b=synthetic_packet(.3,-.2,0.,time.monotonic(),time.time(),1.4)
        pa=a.pose(time.monotonic(),time.time());pb=b.pose(time.monotonic(),time.time())
        self.assertAlmostEqual(pa.x,pb.x);self.assertAlmostEqual(pa.y,pb.y)
        self.assertAlmostEqual(pb.z-pa.z,.4)

    def test_signed_replay_stale_owner_and_hard_caps(self):
        for mode in ('replay','stale','owner','speed','unsigned'):
            with self.subTest(mode=mode):
                key='12'*32;signer=AuthoritySigner(key,4);v=AuthorityVerifier(key,4)
                zero=signer.encode(ZERO,10.,100.);v.decode(zero,10.,100.)
                if mode=='replay':text=zero
                elif mode=='owner':text=AuthoritySigner(key,4).encode(ZERO,10.01,100.01)
                elif mode=='speed':text=signer.encode(Velocity(.71,0.,0.),10.01,100.01,epoch='e',state='ACTIVE',permit=True)
                elif mode=='unsigned':text=json.dumps(dict(vx=.7,vy=0.,wz=0.))
                else:text=signer.encode(ZERO,9.,99.)
                with self.assertRaises((ValueError,KeyError)):v.decode(text,10.01,100.01)

    def test_actual_tk_key_routing_repeat_stop_and_layout(self):
        import tkinter as tk
        from app import Window
        class Worker:
            snapshot={};ui_stamp=0.
            def __init__(self):self.actions=[];self.done=threading.Event()
            def post(self,action):self.actions.append(action)
        root=tk.Tk();root.withdraw();worker=Worker();window=Window(root,worker)
        try:
            root.update_idletasks()
            self.assertLessEqual(root.winfo_reqheight(),720)
            window.press();window.press();self.assertEqual(worker.actions,['toggle'])
            window.release();window.press();self.assertEqual(worker.actions,['toggle'])
            window.clear_key();window.press();self.assertEqual(worker.actions,['toggle','toggle'])
            script=root.bind_class('FastSimKeys','<KeyPress-space>')
            self.assertIn('break',script)
            self.assertIn('FastSimKeys',root.bindtags())
        finally:window.close()

    def test_tk_controls_never_offer_start_before_gate_ready(self):
        import tkinter as tk
        from app import Window
        class Worker:
            snapshot={};ui_stamp=0.
            def __init__(self):self.actions=[];self.done=threading.Event()
            def post(self,action):self.actions.append(action)
        root=tk.Tk();root.withdraw();worker=Worker();window=Window(root,worker)
        try:
            window.controls(dict(state='STANDBY',ready=False,can_prepare=False,prepare_pending=False))
            window.prepare_button.invoke();window.toggle_button.invoke()
            self.assertEqual(worker.actions,[])
            window.controls(dict(state='STANDBY',ready=False,can_prepare=True,prepare_pending=False))
            window.prepare_button.invoke();self.assertEqual(worker.actions,['prepare'])
            window.controls(dict(state='ARMED',ready=False,can_prepare=False,prepare_pending=True))
            window.toggle_button.invoke();self.assertEqual(worker.actions,['prepare'])
            self.assertNotIn('READY',window.prepare_button.cget('text'))
            window.controls(dict(state='ARMED',ready=True,can_prepare=False,prepare_pending=False))
            window.toggle_button.invoke();self.assertEqual(worker.actions,['prepare','toggle'])
            self.assertIn('READY',window.prepare_button.cget('text'))
        finally:window.close()

    def test_expired_ui_action_rejected_but_stop_and_focus_never_expire(self):
        from app import Worker
        class Fake:
            def __init__(self):self.events=[];self.actions=[];self.message='';self.requested=False
            def prepare(self):self.actions.append('prepare')
            def engage(self):self.actions.append('engage')
            def stop(self,reason):self.actions.append('stop')
            def focus_lost(self):self.actions.append('focus_lost')
        c=Fake()
        Worker.dispatch(c,'prepare',time.monotonic()-.2)
        self.assertEqual(c.actions,[]);self.assertIn('100ms',c.message)
        Worker.dispatch(c,'toggle',time.monotonic()+.2);self.assertEqual(c.actions,[])
        Worker.dispatch(c,'stop',time.monotonic()-1.)
        Worker.dispatch(c,'focus_lost',time.monotonic()-1.)
        self.assertEqual(c.actions,['stop','focus_lost'])

    def test_preparation_ready_message_requires_final_gate_not_just_armed(self):
        from ros_chain import Chain
        c=object.__new__(Chain);c.prepare_pending=True;c.prepare_requested=time.monotonic()
        c.sup=NS(core=NS(state='ARMED',hard_fault=False));c.gate=NS(core=NS(hard_fault=False))
        c.plant=NS(fault_bits=0);c.source_issue=lambda:'';c.events=[]
        c.ready=lambda:False
        c.progress_preparation(time.monotonic(),time.time())
        self.assertTrue(c.prepare_pending);self.assertNotIn('READY',c.message)
        c.ready=lambda:True
        c.progress_preparation(time.monotonic(),time.time())
        self.assertFalse(c.prepare_pending);self.assertIn('READY',c.message)
        self.assertEqual(c.events[-1]['event'],'PREPARE_READY_AFTER_GATE')

    def test_idle_focus_loss_does_not_manufacture_a_stop(self):
        from ros_chain import Chain
        c=object.__new__(Chain);c.prepare_pending=c.requested=False
        c.sup=NS(core=NS(state='STANDBY'));c.events=[];calls=[];c.stop=lambda reason:calls.append(reason)
        c.focus_lost();self.assertEqual(calls,[])
        c.prepare_pending=True;c.focus_lost();self.assertEqual(len(calls),1)
        c.prepare_pending=False;c.sup.core.state='ARMED';c.focus_lost()
        self.assertEqual(len(calls),2)


if __name__=='__main__':unittest.main(verbosity=2)
