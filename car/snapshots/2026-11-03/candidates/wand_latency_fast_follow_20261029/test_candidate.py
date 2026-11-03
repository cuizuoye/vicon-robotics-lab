import json
from dataclasses import replace
import math
from pathlib import Path
import random
import socket
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

from common import ZERO,Velocity,config,wheel_rpm,protected_hashes
from benchmark import Harness,synthetic_cases,compare_case
from catchup import FastCatchupSession
from measure import video_intervals,velocity_alignment,rows


def meta(fps=60):
    return dict(capture_fps=fps,same_camera=True,constant_frame_spacing=True,
        no_missing_frames=True,capture_fps_verified=True,initial_standstill_verified=True,
        onset_definition='first visible motion, same fixed camera')


def annotation():
    return dict(trial_id='SYNTHETIC',event='wheel',sign_last_still=10,sign_first_moving=11,
                output_last_still=16,output_first_moving=17)


class MeasurementTests(unittest.TestCase):
    def test_30_60_240fps_intervals_not_fake_precision(self):
        for fps in (30,60,240):
            r=video_intervals([annotation()],meta(fps))[0]
            self.assertAlmostEqual(r['interval_width_ms'],2000/fps)
            self.assertEqual(r['low_frame_rate_warning'],fps<120)
            self.assertFalse(r['camera_exposure_to_motor_electrical_onset_measured'])

    def test_metadata_defaults_never_qualify(self):
        template=json.loads(Path(__file__).with_name('VIDEO_METADATA_TEMPLATE.json').read_text())
        with self.assertRaises(ValueError):video_intervals([annotation()],template)

    def test_every_unverified_clock_condition_rejected(self):
        for key in ('same_camera','constant_frame_spacing','no_missing_frames',
                    'capture_fps_verified','initial_standstill_verified'):
            m=meta();m[key]=False
            with self.subTest(key=key),self.assertRaises(ValueError):video_intervals([annotation()],m)

    def test_empty_annotation_is_not_a_measurement(self):
        with self.assertRaises(ValueError):video_intervals([],meta())

    def test_clearly_noncausal_or_reversed_bracket_rejected(self):
        for fields in [dict(sign_first_moving=10),dict(output_first_moving=15),
                       dict(output_last_still=1,output_first_moving=2)]:
            a=annotation();a.update(fields)
            with self.assertRaises(ValueError):video_intervals([a],meta())

    def test_same_frame_onsets_remain_unresolved(self):
        a=annotation();a.update(output_last_still=10,output_first_moving=11)
        r=video_intervals([a],meta())[0]
        self.assertTrue(r['includes_zero']);self.assertLess(r['delay_lower_ms'],0)

    def test_static_curve_does_not_report_zero_latency(self):
        t=np.arange(0,10,.01);v=np.zeros((len(t),2))
        self.assertFalse(velocity_alignment(t,v,v)['qualified'])

    def test_known_synthetic_lag_with_matched_filters(self):
        t=np.arange(0,15,.01)
        g=np.column_stack([np.sin(t*2),np.cos(t*1.3)])
        c=np.column_stack([np.sin((t-.13)*2),np.cos((t-.13)*1.3)])
        for width in (5,10,20):
            r=velocity_alignment(t,g,c,width)
            self.assertTrue(r['qualified']);self.assertAlmostEqual(r['best_lag_ms'],130,delta=10)

    def test_loss_or_truncated_json_never_silently_filled(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'rows.jsonl';row=json.dumps(dict(topic='car_pose',data={}))+'\n'
            p.write_text(row+'\0'*10);self.assertEqual(len(list(rows(p))),1)
            p.write_text(row+'\0\n'+row)
            with self.assertRaises(ValueError):list(rows(p))
            p.write_text(row+'{broken')
            with self.assertRaises(ValueError):list(rows(p))


class SafetyTests(unittest.TestCase):
    def test_caps_mapping_and_spatial_policy_unchanged(self):
        c=config();f=c.follower_config()
        self.assertEqual((c.max_speed_mps,c.max_accel_mps2,c.max_yaw_rate_radps,c.max_yaw_accel_radps2),(.7,.8,.1,.4))
        self.assertEqual((f.scale,f.reference_lead_m,f.filter_tau_s),(1.,.20,0.))
        self.assertFalse(f.workspace_radius_enabled)
        self.assertEqual((c.pose_timeout_s,c.command_timeout_s,c.feedback_timeout_s,c.lease_timeout_s),(.2,.1,.3,.15))

    def moving(self):
        h=Harness(True);h.start()
        for i in range(50):h.gesture(.005*i,0);h.step()
        self.assertNotEqual(h.output,ZERO)
        return h

    def test_prepare_is_not_enable(self):
        h=Harness(True);h.run(1.8)
        self.assertTrue(h.sup.request_start(h.now,h.ros)[0]);h.run(.2)
        self.assertEqual(h.output,ZERO);self.assertEqual(h.sup.state,'ARMED')

    def test_recorded_heading_is_preserved_without_rotating_world_xy(self):
        h=Harness(True,origin_yaw=.8);h.start();h.gesture(.1,0.,0.);h.step()
        self.assertAlmostEqual(h.sup.follower.mapping.car_origin.yaw,.8)
        self.assertAlmostEqual(h.sup.follower.mapped_goal.x,.1)
        self.assertAlmostEqual(h.sup.follower.mapped_goal.y,0.)
        self.assertAlmostEqual(h.sup.follower.mapped_goal.yaw,.8)

    def test_release_zero_then_no_automatic_restart(self):
        h=self.moving();h.held=False;self.assertEqual(h.step(),ZERO)
        h.run(2.);h.held=True;h.run(.5)
        self.assertNotEqual(h.sup.state,'ACTIVE');self.assertEqual(h.output,ZERO)

    def test_source_lease_feedback_command_loss_and_faults(self):
        for kind in ('wand','car','lease','feedback','command','graph','actuator','lift','tilt','jump','duplicate'):
            with self.subTest(kind=kind):
                h=self.moving()
                if kind in ('wand','car'):h.pose_on[kind]=False
                elif kind=='lease':h.lease_on=False
                elif kind=='feedback':h.feedback_on=False
                elif kind=='command':h.command_on=False
                elif kind=='graph':h.graph_ok=False
                elif kind=='actuator':h.fault_bits=0x20
                elif kind=='lift':h.plant.pose=type(h.plant.pose)(h.plant.pose.x,h.plant.pose.y,h.c.ground_z_m+.11,0.)
                elif kind=='tilt':h.roll=.35
                elif kind=='jump':h.gesture(2.,0.)
                elif kind=='duplicate':h.pose_stamp['wand']=h.ros
                h.run(.40)
                self.assertEqual(h.output,ZERO)
                self.assertTrue(h.gate.fault or h.sup.reason)
                h.pose_on={'car':True,'wand':True};h.lease_on=h.feedback_on=h.command_on=True
                h.graph_ok=True;h.pose_stamp.clear();h.run(.2)
                self.assertEqual(h.output,ZERO)

    def test_real_actuator_fault_stays_hard_latched(self):
        h=self.moving();h.fault_bits=0x20;h.step();h.fault_bits=0;h.run(.5)
        self.assertTrue(h.gate.hard_fault);self.assertEqual(h.output,ZERO)

    def test_independent_gate_rejects_speed_violation(self):
        h=self.moving();h.now+=.02;h.ros+=.02
        f=replace(h.gate.frame,sequence=h.gate.last_sequence+1,issued_steady=h.now,
                  issued_ros=h.ros,command=Velocity(1.,0.,0.))
        h.gate.receive(f,h.now,h.ros)
        self.assertEqual(h.gate.tick(h.now,h.ros),ZERO)
        self.assertEqual(h.gate.fault,'FINAL_VECTOR_LIMIT')

    def test_independent_gate_rejects_acceleration_violation(self):
        h=self.moving();h.now+=.02;h.ros+=.02;old=h.output
        f=replace(h.gate.frame,sequence=h.gate.last_sequence+1,issued_steady=h.now,
                  issued_ros=h.ros,command=Velocity(old.vx+.10,old.vy,old.wz))
        h.gate.receive(f,h.now,h.ros)
        self.assertEqual(h.gate.tick(h.now,h.ros),ZERO)
        self.assertEqual(h.gate.fault,'FINAL_ACCELERATION_LIMIT')

    def test_independent_gate_rejects_foreign_owner(self):
        h=self.moving();f=replace(h.gate.frame,sequence=h.gate.last_sequence+1,source='foreign')
        self.assertFalse(h.gate.receive(f,h.now,h.ros));self.assertEqual(h.gate.output,ZERO)
        self.assertEqual(h.gate.fault,'COMMAND_OWNER_CHANGED')

    def test_clock_jump_and_future_pose_remain_protected(self):
        for kind in ('clock','future_pose'):
            h=self.moving()
            if kind=='clock':h.ros+=.2
            else:h.pose_stamp['wand']=h.ros+1.
            h.step();self.assertEqual(h.output,ZERO)
            self.assertTrue(h.gate.fault or h.sup.reason)

    def test_no_progress_guard_retained(self):
        h=Harness(True);h.start();h.freeze=True
        for i in range(220):h.gesture(min(i*.01,1.),0.);h.step()
        self.assertEqual(h.output,ZERO)
        self.assertTrue('PROGRESS' in h.sup.reason or 'PROGRESS' in h.gate.fault)

    def test_quiet_catchup_does_not_accept_stale_velocity(self):
        h=Harness(True);h.start()
        for i in range(50):h.gesture(i*.02,0.);h.step()
        h.pose_stamp['wand']=h.ros;h.run(.4)
        self.assertFalse(h.sup.state=='ACTIVE');self.assertEqual(h.output,ZERO)

    def test_old_unsafe_fast_reversal_regression(self):
        samples=synthetic_cases()['fast_reversal']
        b,_=compare_case('reverse',samples,False);n,_=compare_case('reverse',samples,True)
        self.assertTrue(b['passed'] and n['passed'])
        self.assertLessEqual(n['xy_rms_m'],b['xy_rms_m']*1.05)

    def test_never_increases_any_command_or_wheel_cap(self):
        h=Harness(True);h.start();old=ZERO
        for i in range(500):
            t=i*.02;h.gesture(min(t,1.),.4*math.sin(t),.08*math.sin(t))
            v=h.step()
            self.assertLessEqual(math.hypot(v.vx,v.vy),.7+1e-8)
            self.assertLessEqual(math.hypot(v.vx-old.vx,v.vy-old.vy)/.02,.8+1e-6)
            self.assertLessEqual(max(abs(x) for x in wheel_rpm(v)),400+1e-7);old=v

    def test_randomized_fast_sources_do_not_add_unexpected_stops(self):
        for seed in range(12):
            with self.subTest(seed=seed):
                rng=random.Random(seed);samples=[];x=y=vx=vy=0.
                ax=ay=0.
                from benchmark import Sample
                for i in range(1,801):
                    if i%20==1:ax,ay=rng.uniform(-2,2),rng.uniform(-2,2)
                    vx=max(-1.2,min(1.2,vx+ax*.02));vy=max(-1.2,min(1.2,vy+ay*.02))
                    x+=vx*.02;y+=vy*.02
                    samples.append(Sample(i*.02,x,y,.04*math.sin(i*.02)))
                b,_=compare_case('random',samples,False);n,_=compare_case('random',samples,True)
                if b['passed']:self.assertTrue(n['passed'],n)

    def test_offline_core_works_with_sockets_and_subprocesses_forbidden(self):
        def forbidden(*a,**k):raise AssertionError('unexpected live IO')
        before=protected_hashes()
        with patch.object(socket,'socket',forbidden),patch.object(subprocess,'Popen',forbidden):
            h=Harness(True);h.start();h.gesture(.001,0.);h.step()
        self.assertEqual(protected_hashes(),before)


if __name__=='__main__':unittest.main()
