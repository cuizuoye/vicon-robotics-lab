import copy
import unittest
from ros_link_trace_analysis import analyze


class TraceAnalysisTests(unittest.TestCase):
    def fixture(self):
        observation = dict(observation_end_s=1, topics={'fault': [dict(time_s=1, value=1)]},
            host_send_timing=[dict(kind=16, sequence=seq, session_id=7,
                start_s=at, end_s=at+0.001, disabled_zero=True)
                for seq, at in [(30, 0.7), (31, 0.72), (32, 0.74)]])
        text = ('TRACE_FAULT sid=7 esp_ms=2000 mcu_ms=1000 last_cmd=30 fault=1\n'
            'TRACE_EVENT kind=1 seq=30 at_ms=1800 duration_ms=0 detail=1\n'
            'TRACE_EVENT kind=2 seq=30 at_ms=1800 duration_ms=1 detail=1\nTRACE_END\n')
        return observation, [dict(text=text[:20]), dict(text=text[20:])]

    def test_split_lines_and_trailing_silence(self):
        result = analyze(*self.fixture())
        self.assertEqual(result['timing']['esp_rx_silence_at_fault_ms'], 200)
        self.assertEqual(result['host_sent_newer_sequences_before_fault_callback'], [31, 32])
        self.assertTrue(result['last_rx_equals_mcu_last_command'])
        self.assertTrue(result['recent_uart_writes_all_successful'])

    def test_missing_dump_or_mismatched_session_rejected(self):
        observation, chunks = self.fixture()
        for altered in [[], [dict(text=''.join(x['text'] for x in chunks).replace('sid=7', 'sid=8'))]]:
            with self.assertRaises(ValueError):
                analyze(observation, altered)

    def test_completed_gap_matched_by_sequence(self):
        observation, chunks = self.fixture()
        text = ''.join(row['text'] for row in chunks).replace('TRACE_END',
            'TRACE_EVENT kind=1 seq=31 at_ms=1951 duration_ms=0 detail=2\nTRACE_END')
        result = analyze(observation, [dict(text=text)])
        gap = result['largest_completed_esp_rx_gap']
        self.assertEqual(gap['esp_gap_ms'], 151)
        self.assertAlmostEqual(gap['matched_host_send_gap_ms'], 20)
        self.assertTrue(gap['preceding_rx_equals_mcu_last_command'])

    def test_nonzero_send_rejected_and_post_fault_excluded(self):
        observation, chunks = self.fixture()
        unsafe = copy.deepcopy(observation)
        unsafe['host_send_timing'][0]['disabled_zero'] = False
        with self.assertRaises(ValueError):
            analyze(unsafe, chunks)
        observation['host_send_timing'].append(dict(kind=16, sequence=99, session_id=7,
            start_s=2, end_s=2.001, disabled_zero=False))
        self.assertNotIn(99, analyze(observation, chunks)['host_sent_newer_sequences_before_fault_callback'])


if __name__ == '__main__':
    unittest.main()
