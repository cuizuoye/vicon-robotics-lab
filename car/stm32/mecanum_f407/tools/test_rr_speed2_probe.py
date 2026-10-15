import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from rr_speed2_probe import FIRMWARE, Probe, parse_status, parse_trace
from rr_pwm_fine_probe import parse_status as old_status
from test_rr_pwm_fine_probe import convert as fine_convert
from test_rr_diagnostic_probe import FakeSerial as OldSerial, fixture, trace_fixture
from test_speed_pi_probe import Clock


def convert(line):
    line=fine_convert(line)
    prefix,*parts=line.split(',')
    if prefix not in ('TELSP','STATUSSP','RRTRACE'): return line
    f=dict(p.split('=',1) for p in parts)
    if prefix=='RRTRACE':
        f['enc']=str(2*int(f['enc']))
        if f['en']=='1':
            for key in ('target','cps','fcps'): f[key]=str(2*int(f[key]))
            f['d']='-80' if f['assist']=='8' else '-96'
            f['p1']=str(8400-8400*abs(int(f['d']))//1000)
    else:
        if f['fw']=='rr_diag_pwm01_v1': f['fw']=FIRMWARE
        f['enc_rr']=str(2*int(f['enc_rr']))
        if f['en']=='1':
            for key in ('want_rr','target_rr','cps_rr','fcps_rr','rpm_rr','mms_rr'):
                f[key]=str(2*int(f[key]))
            f['d_rr']='-80' if f['start_mask']=='8' else '-96'
            f['p_rr1']=str(8400-8400*abs(int(f['d_rr']))//1000)
    return prefix+','+','.join(f'{k}={v}' for k,v in f.items())


class FakeSerial(OldSerial):
    """只测协议隔离/一次性/停止，不模拟实际达速或机械阻力。"""
    def queue(self,text): super().queue(convert(text))

    def write(self,data):
        if data==b'run RR328 S10\n':
            result=super().write(b'run RR164 S10\n')
            self.commands[-1]='run RR328 S10'
            return result
        return super().write(data)


class Speed2Tests(unittest.TestCase):
    def exercise(self,motion=None,after_run=False,powered_stationary=False,**kw):
        clock=Clock(); link=FakeSerial(clock,**kw); link.used=after_run
        with patch('drive_pi_probe.time.monotonic',clock):
            r=Probe(link,motion=motion,after_run=after_run,
                    powered_stationary=powered_stationary).run()
        self.assertLessEqual(link.commands.count('run RR328 S10'),1)
        self.assertNotIn('run RR164 S10',link.commands)
        return r,link

    def test_identity_target_timer_and_cap(self):
        line=convert(fixture(en=1,sel=4,mode=5,used=1,want_rr=164,target_rr=164,
                             cps_rr=-164,fcps_rr=164,d_rr=-40,p_rr1=96,p_rr2=100))
        self.assertEqual(parse_status(line)['want_rr'],328)
        for bad in (line.replace(FIRMWARE,'rr_diag_pwm01_v1'),
                    line.replace('want_rr=328','want_rr=164'),
                    line.replace('target_rr=328','target_rr=329'),
                    line.replace('d_rr=-96','d_rr=-101'),
                    line.replace('pwm_rr_arr=8399','pwm_rr_arr=99')):
            with self.assertRaises(ValueError): parse_status(bad)
        with self.assertRaises(ValueError): old_status(line)
        tr=convert(trace_fixture()); self.assertEqual(parse_trace(tr)['target'],328)
        with self.assertRaises(ValueError): parse_trace(tr.replace('target=328','target=329'))

    def test_unique_run_and_readonly_modes(self):
        r,l=self.exercise(motion='rr328')
        self.assertTrue(r['passed'],r['errors'])
        self.assertEqual(l.commands.count('run RR328 S10'),1)
        self.assertEqual(l.commands.count('trace'),1)
        self.assertEqual(r['speed_profile']['target_cps'],328)
        for kw in ({},{'after_run':True},{'powered_stationary':True}):
            r,l=self.exercise(**kw); self.assertTrue(r['passed'],r['errors'])
            self.assertEqual(set(l.commands),{'stop','status'})

    def test_fault_missing_ack_and_other_wheel_do_not_retry(self):
        for kw in ({'fault':True},{'zero_moves':True},{'drop_ack':True},
                   {'export_error':'missing'},{'old':True}):
            r,l=self.exercise(motion='rr328',**kw)
            self.assertFalse(r['passed']); self.assertIsNone(l.started)
            self.assertLessEqual(l.commands.count('trace'),1)

    def test_exact_host_whitelist_and_confirmation(self):
        p=Probe(FakeSerial(Clock()),motion='rr328')
        for command in ('run RR164 S10','run RR492 S10','run RR328 S30','clear 5A','run F50 S10'):
            with self.assertRaises(ValueError): p.send(command)
        script=Path(__file__).with_name('rr_speed2_probe.py')
        for mode in ('rr328','rr164'):
            r=subprocess.run([sys.executable,'-B',str(script),'unused','--mode',mode],
                             capture_output=True,text=True)
            self.assertEqual(r.returncode,2)

    def test_real_c_schema_and_safety(self):
        root=Path(__file__).resolve().parents[1]
        sources=['Src/rr_diagnostic_trace.c','Src/drive_bench_control.c','Src/four_drive_group.c',
                 'Src/four_wheel_speed.c','Src/mecanum_drive_plan.c','Src/mecanum_kinematics.c']
        with tempfile.TemporaryDirectory(prefix='rr_speed2_schema_') as tmp:
            for entry in ('rr_schema_fixture','test_rr_diagnostic','test_rr_pwm_fine','test_rr_speed2'):
                exe=Path(tmp)/entry
                subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-fanalyzer',
                                '-fsanitize=address,undefined','-fno-omit-frame-pointer','-no-pie',
                                '-DDRIVE_STARTUP_OBSERVE=1','-DRR_DIAGNOSTIC=1',
                                '-DRR_PWM_FINE=1','-DRR_SPEED2=1','-IInc',
                                f'tests/{entry}.c',*sources,'-lm','-o',str(exe)],
                               cwd=root,check=True,capture_output=True)
                out=subprocess.run([str(exe)],check=True,capture_output=True,text=True).stdout
                if entry=='rr_schema_fixture':
                    a,t,b,u=out.splitlines()
                    self.assertEqual(parse_status(a)['want_rr'],328)
                    self.assertEqual(parse_status(b)['en'],0)
                    parse_trace(t); parse_trace(u)


if __name__=='__main__': unittest.main()
