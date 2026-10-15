import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from rr_pwm_fine_probe import FIRMWARE, Probe, parse_status, parse_trace
from rr_diagnostic_probe import parse_status as old_status, parse_trace as old_trace
from test_rr_diagnostic_probe import FakeSerial as OldSerial, fixture as old_fixture, trace_fixture
from test_speed_pi_probe import Clock


def convert(line):
    prefix,*parts=line.split(',')
    if prefix not in ('TELSP','STATUSSP','RRTRACE'): return line
    f=dict(p.split('=',1) for p in parts)
    if prefix=='RRTRACE':
        if f['en']=='1':
            f['d']='-80' if f['assist']=='8' else '-49'
            f['p1']=str(8400-8400*abs(int(f['d']))//1000); f['p2']='8400'
    else:
        if f['fw']=='rr_diag_start8_v1': f['fw']=FIRMWARE
        f.update(pwm_rr_arr='8399',pwm_rr_psc='0')
        if f['en']=='1':
            f['d_rr']='-80' if f['start_mask']=='8' else '-49'
            f['p_rr1']=str(8400-8400*abs(int(f['d_rr']))//1000); f['p_rr2']='8400'
    return prefix+','+','.join(f'{k}={v}' for k,v in f.items())


class FakeSerial(OldSerial):
    def queue(self,text):
        super().queue(convert(text))


class FineProbeTests(unittest.TestCase):
    def exercise(self,motion=None,after_run=False,powered_stationary=False,**kw):
        clock=Clock(); link=FakeSerial(clock,**kw); link.used=after_run
        with patch('drive_pi_probe.time.monotonic',clock):
            r=Probe(link,motion=motion,after_run=after_run,
                    powered_stationary=powered_stationary).run()
        self.assertLessEqual(link.commands.count('run RR164 S10'),1)
        self.assertLessEqual(link.commands.count('trace'),1)
        return r,link

    def test_new_identity_and_actual_timer(self):
        line=convert(old_fixture())
        self.assertEqual(parse_status(line)['fw'],FIRMWARE)
        for old in (old_fixture(),line.replace('8399','99'),line.replace('pwm_rr_psc=0','pwm_rr_psc=83')):
            with self.assertRaises(ValueError): parse_status(old)
        with self.assertRaises(ValueError): old_status(line)

    def test_fine_duty_schema_and_trace_mapping(self):
        line=convert(old_fixture(en=1,sel=4,mode=5,used=1,want_rr=164,target_rr=164,
                                 d_rr=-40,p_rr1=96,p_rr2=100))
        self.assertEqual(parse_status(line)['d_rr'],-49)
        with self.assertRaises(ValueError): parse_status(line.replace('p_rr1=7989','p_rr1=7990'))
        tr=convert(trace_fixture())
        self.assertEqual(parse_trace(tr)['d'],-49)
        with self.assertRaises(ValueError): old_trace(tr)
        with self.assertRaises(ValueError): parse_trace(tr.replace('p1=7989','p1=7990'))
        with self.assertRaises(ValueError): parse_trace(tr.replace('d=-49','d=-101'))

    def test_one_unique_motion_and_export(self):
        r,l=self.exercise(motion='rr164')
        self.assertTrue(r['passed'],r['errors'])
        self.assertEqual(l.commands.count('run RR164 S10'),1)
        self.assertEqual(l.commands.count('trace'),1)
        self.assertEqual(r['pwm_profile']['duty_quantum_permille'],1)
        self.assertIn(-49,r['rr_recording']['duty_levels_permille'])

    def test_readonly_and_fault_paths_unchanged(self):
        for kw in ({},{'after_run':True},{'powered_stationary':True}):
            r,l=self.exercise(**kw)
            self.assertTrue(r['passed'],r['errors'])
            self.assertEqual(set(l.commands),{'stop','status'})
        for kw in ({'fault':True},{'zero_moves':True},{'drop_ack':True},
                   {'export_error':'missing'},{'old':True}):
            r,l=self.exercise(motion='rr164',**kw)
            self.assertFalse(r['passed'])
            self.assertIsNone(l.started)
            if kw.get('old'): self.assertNotIn('run RR164 S10',l.commands)

    def test_confirmation_before_serial(self):
        script=Path(__file__).with_name('rr_pwm_fine_probe.py')
        r=subprocess.run([sys.executable,'-B',str(script),'unused','--mode','rr164'],
                         capture_output=True,text=True)
        self.assertEqual(r.returncode,2)
        self.assertIn('serial unopened',r.stderr)

    def test_real_fine_c_schema_safety_and_quantization(self):
        root=Path(__file__).resolve().parents[1]
        sources=['Src/rr_diagnostic_trace.c','Src/drive_bench_control.c','Src/four_drive_group.c',
                 'Src/four_wheel_speed.c','Src/mecanum_drive_plan.c','Src/mecanum_kinematics.c']
        with tempfile.TemporaryDirectory(prefix='rr_fine_schema_') as tmp:
            for entry in ('rr_schema_fixture','test_rr_diagnostic','test_rr_pwm_fine'):
                exe=Path(tmp)/entry
                subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-fanalyzer',
                                '-DDRIVE_STARTUP_OBSERVE=1','-DRR_DIAGNOSTIC=1','-DRR_PWM_FINE=1',
                                '-IInc',f'tests/{entry}.c',*sources,'-lm','-o',str(exe)],
                               cwd=root,check=True,capture_output=True)
                out=subprocess.run([str(exe)],check=True,capture_output=True,text=True).stdout
                if entry=='rr_schema_fixture':
                    a,t,b,u=out.splitlines()
                    self.assertEqual(parse_status(a)['pwm_rr_arr'],8399)
                    self.assertEqual(parse_status(b)['en'],0)
                    parse_trace(t); parse_trace(u)


if __name__=='__main__': unittest.main()
