import unittest
from radio_link_analysis import summarize_radio


def dump(rows):
    return ''.join(f'TRACE_RADIO cb_ms={cb} hw_us={hw} seq={seq} retry={retry} len=120\n'
        for cb, hw, seq, retry in rows) + f'TRACE_RADIO_END count={len(rows)}\n'


class RadioAnalysisTests(unittest.TestCase):
    def test_both_layers_silent(self):
        result = summarize_radio(dump([(1000, 500000, 30, 0), (1151, 651000, 31, 1)]), 1000, 1151)
        self.assertEqual(result['max_callback_gap_ms'], 151)
        self.assertEqual(result['max_hardware_gap_us'], 151000)
        self.assertEqual(result['radio_callbacks_strictly_inside_app_gap'], [])
        self.assertEqual(result['radio_retry_flag_count'], 1)

    def test_radio_activity_during_application_gap(self):
        result = summarize_radio(dump([(1000, 500000, 30, 0), (1020, 520000, 31, 0),
            (1151, 651000, 32, 0)]), 1000, 1151)
        self.assertEqual(result['radio_callbacks_strictly_inside_app_gap'][0]['sequence'], 31)

    def test_wrap(self):
        result = summarize_radio(dump([(0xfffffff0, 0xfffffff0, 4095, 0),
            (4, 19984, 0, 0)]), 0xfffffff0, 4)
        self.assertEqual(result['max_callback_gap_ms'], 20)
        self.assertEqual(result['max_hardware_gap_us'], 20000)

    def test_incomplete_or_invalid_rejected(self):
        valid = dump([(1, 100, 1, 0), (2, 200, 2, 0)])
        for text in ['', valid.replace('count=2', 'count=3'),
                     valid.replace('retry=0', 'retry=2'), valid.replace('hw_us=200', 'hw_us=100')]:
            with self.assertRaises(ValueError):
                summarize_radio(text, 1, 2)

    def test_tid_and_trailing_silence(self):
        text = dump([(1000, 500000, 30, 0), (1020, 520000, 31, 0)]).replace('len=120', 'len=120 tid=6')
        result = summarize_radio(text, 1000, 1020, 1180)
        self.assertEqual(result['observed_qos_tids'], {'6': 2})
        self.assertEqual(result['trailing_radio_callback_silence_at_fault_ms'], 160)
        with self.assertRaises(ValueError):
            summarize_radio(text.replace('tid=6', 'tid=16'), 1000, 1020)


if __name__ == '__main__':
    unittest.main()
