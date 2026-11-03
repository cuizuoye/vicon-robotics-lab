import unittest
from ros_observation_stats import valid_duration,feedback_gaps_ms

class ObservationTests(unittest.TestCase):
    def test_duration_only_bound(self):
        for value in (5,10,30,120):self.assertTrue(valid_duration(value))
        for value in (0,4.99,120.01,float('inf'),float('nan')):
            self.assertFalse(valid_duration(value))
    def test_feedback_not_command_gap(self):
        s=feedback_gaps_ms({'wheel_rpm':[{'time_s':0},{'time_s':.1},{'time_s':.27}]})
        self.assertAlmostEqual(s['wheel_rpm'],170)
        self.assertIsNone(s['diagnostics'])
    def test_bad_time_order(self):
        with self.assertRaises(ValueError):
            feedback_gaps_ms({'sequence':[{'time_s':2},{'time_s':1}]})

if __name__=='__main__':unittest.main()
