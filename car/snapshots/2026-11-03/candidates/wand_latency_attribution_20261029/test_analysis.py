"""Offline checks for measurement semantics, not controller performance."""
import unittest
import numpy as np
from analyze import lag_search, common_blocks, stats


class AnalysisTests(unittest.TestCase):
    def test_known_130ms_lag_both_xy_axes(self):
        t=np.arange(0.,14.,.01)
        def signal(t):return np.column_stack((np.sin(t*1.7)+.1*np.sin(t*4.2),.2*np.cos(t*2.3)))
        for window in (50,100,200):
            r=lag_search(t,signal(t),signal(t-.13),window)
            self.assertTrue(r['qualified']);self.assertEqual(r['best_lag_ms'],130.)

    def test_static_does_not_report_zero_latency(self):
        t=np.arange(0,12,.01);r=lag_search(t,t*0,t*0)
        self.assertFalse(r['qualified']);self.assertEqual(r['reason'],'insufficient_excitation')

    def test_boundary_optimum_not_a_fixed_delay_claim(self):
        t=np.arange(0,12,.01);r=lag_search(t,np.sin(t),np.sin(t))
        self.assertFalse(r['qualified']);self.assertEqual(r['best_lag_ms'],0.)

    def test_no_interpolation_across_source_gap(self):
        a=np.r_[np.arange(0.,6.,.02),np.arange(7.,14.,.02)]
        blocks,gaps=common_blocks((a,),0.,14.)
        self.assertTrue(gaps)
        for lo,hi in blocks:self.assertFalse(any(x<hi and y>lo for x,y in gaps))

    def test_nonfinite_and_bad_interval_rejected(self):
        with self.assertRaises(ValueError):stats([1.,np.nan])
        with self.assertRaises(ValueError):common_blocks((np.arange(4),),4.,0.)
        t=np.arange(0,12,.01)
        with self.assertRaises(ValueError):lag_search(t,t,np.full_like(t,np.nan))

    def test_short_record_cannot_qualify(self):
        t=np.arange(0,2,.01);self.assertFalse(lag_search(t,np.sin(t),np.sin(t-.1))['qualified'])


if __name__=='__main__':unittest.main(verbosity=2)
