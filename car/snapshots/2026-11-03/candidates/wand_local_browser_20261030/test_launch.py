import argparse
import unittest
from launch import duration,check


class LocalLaunchTests(unittest.TestCase):
    def test_duration_bounds(self):
        for value in ('nan','inf','0','121'):
            with self.assertRaises(argparse.ArgumentTypeError):duration(value)
        self.assertEqual(duration('60'),60.)

    def test_reused_candidate_is_readonly(self):
        result=check()
        self.assertTrue(result['passed'])
        self.assertFalse(result['ros_initialized'])
        self.assertFalse(result['hardware_entry'])
        self.assertEqual(result['proof_limit_ms'],100)


if __name__=='__main__':unittest.main()
