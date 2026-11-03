import unittest
from live_analysis import session_outcome


class OutcomeTests(unittest.TestCase):
    def session(self, reason, seconds=60., hard=False):
        return dict(events=[dict(at=10., state=['ACTIVE', '', '']),
                            dict(at=10.+seconds, state=['STOPPING', reason, '']),
                            dict(at=12.+seconds, state=['FAULT' if hard else 'PAUSED',
                                                       'CONTROL_TIMER_GAP' if hard else reason, ''])],
                    snapshot=dict(state='FAULT' if hard else 'PAUSED', hard_fault=hard,
                                  stop_confirmed=True))

    def test_first_fault_is_not_overwritten_by_later_latch(self):
        value = session_outcome(self.session('FINAL_GATE_STATUS_TIMEOUT', 12.5, True))
        self.assertFalse(value['passed'])
        self.assertEqual(value['first_interruption']['first_stop_reason'], 'FINAL_GATE_STATUS_TIMEOUT')
        self.assertAlmostEqual(value['longest_active_s'], 12.5)

    def test_expected_stop_still_requires_full_observation(self):
        self.assertTrue(session_outcome(self.session('操作员Esc/停止'))['passed'])
        self.assertTrue(session_outcome(self.session('再次空格停止'))['passed'])
        self.assertFalse(session_outcome(self.session('操作员Esc/停止', 5.))['passed'])

    def test_focus_and_other_safety_stops_do_not_pass(self):
        for reason in ('操作台失去焦点，准备/跟随已取消', 'UI_HEARTBEAT_TIMEOUT', 'WAND_POSE_TIMEOUT'):
            with self.subTest(reason=reason):
                self.assertFalse(session_outcome(self.session(reason))['passed'])
        value=self.session('操作员Esc/停止')
        value['events']=value['events'][:1]
        self.assertFalse(session_outcome(value)['passed'])


if __name__ == '__main__': unittest.main(verbosity=2)
