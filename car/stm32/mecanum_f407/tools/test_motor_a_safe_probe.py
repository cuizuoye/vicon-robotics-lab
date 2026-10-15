"""仅测试串口行同步；不打开真实串口，不驱动电机。"""

import unittest

from motor_a_safe_probe import SerialLineFramer, parse_status


class SerialLineFramerTests(unittest.TestCase):
    def test_initial_fragment_is_recorded_not_parsed(self):
        framer = SerialLineFramer()
        self.assertEqual(framer.feed(b'cps=-200,fault=0x00000000\r\nOK,stopped\n'),
                         [b'OK,stopped'])
        self.assertEqual(framer.startup_discarded,
                         {'text': 'cps=-200,fault=0x00000000', 'truncated': False})
        self.assertEqual(framer.invalid_lines, 0)

    def test_initial_full_line_is_also_sync_only(self):
        framer = SerialLineFramer()
        self.assertEqual(framer.feed(b'READY,motor_a_safe_off\nSTATUS,complete\n'),
                         [b'STATUS,complete'])

    def test_chunk_boundaries_preserve_complete_lines(self):
        framer = SerialLineFramer()
        self.assertEqual(framer.feed(b'frag'), [])
        self.assertFalse(framer.synchronized)
        self.assertEqual(framer.feed(b'ment\nOK,sto'), [])
        self.assertEqual(framer.feed(b'pped\r\nnext\n'), [b'OK,stopped', b'next'])

    def test_long_startup_fragment_is_bounded(self):
        framer = SerialLineFramer()
        self.assertEqual(framer.feed(b'x' * 3000 + b'\nvalid\n'), [b'valid'])
        self.assertEqual(len(framer.startup_discarded['text']), 512)
        self.assertTrue(framer.startup_discarded['truncated'])

    def test_long_regular_line_cannot_create_valid_tail(self):
        framer = SerialLineFramer()
        framer.feed(b'\n')
        self.assertEqual(framer.feed(b'x' * 513 + b'OK,stopped\nreal\n'), [b'real'])
        self.assertEqual(framer.invalid_lines, 1)

    def test_exact_limit_and_empty_lines(self):
        framer = SerialLineFramer()
        self.assertEqual(framer.feed(b'\n' + b'x' * 512 + b'\n\n'), [b'x' * 512, b''])
        self.assertEqual(framer.invalid_lines, 0)

    def test_regular_non_ascii_is_not_silently_repaired(self):
        framer = SerialLineFramer()
        frames = framer.feed(b'\n\xff\n')
        with self.assertRaises(UnicodeDecodeError):
            frames[0].decode('ascii')

    def test_status_parser_preserves_unsafe_state(self):
        row = parse_status('STATUS,ms=100,en=1,hiz=0,mode=duty,'
                           'target_cps=0,duty=100,enc=0,cps=0,fault=0x00000001')
        self.assertEqual((row['en'], row['hiz'], row['duty'], row['fault']), (1, 0, 100, 1))


if __name__ == '__main__':
    unittest.main()
