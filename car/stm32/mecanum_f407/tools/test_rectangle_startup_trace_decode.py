import math
import unittest

from rectangle_startup_trace_decode import CAPACITY, HEADER, MAGIC, ROW, decode


def row(ms, enabled, duty=0, counts=(0, 0, 0, 0)):
    duties = [duty, -duty, duty, -duty]
    compares = sum(([100, 100-d//10] if d > 0 else
                    [100+d//10, 100] if d < 0 else [0, 0] for d in duties), [])
    return ROW.pack(ms, *counts, *([0.0]*12), *duties, *compares,
                    0 if enabled else 32, 5 if enabled else 9, enabled, 0,
                    15 if duty else 0, 0 if enabled else 4, 0, 5, 0)


def blob(records=(), complete=False, post=0):
    unpacked = [ROW.unpack(r) for r in records]
    stopped = [r for r in unpacked if not r[31]]
    head = HEADER.pack(MAGIC, 1, ROW.size, CAPACITY, len(records), post,
                       bool(records) and not complete, complete, 0, 0,
                       281.0, 278.0, 280.0, 281.0,
                       unpacked[0][0] if records else 0, stopped[0][0] if stopped else 0)
    return head + b''.join(records) + bytes((CAPACITY-len(records))*ROW.size)


class DecoderTests(unittest.TestCase):
    def test_idle(self):
        result = decode(blob())
        self.assertEqual(result['count'], 0)
        self.assertFalse(result['complete'])
        self.assertIsNone(result['runtime_to_stop_ms'])

    def test_complete(self):
        records = [row(1000, 1), row(1010, 1, 80, (1, -1, 1, -1))]
        records += [row(1020+i*10, 0, counts=(1, -1, 1, -1)) for i in range(20)]
        result = decode(blob(records, True, 20))
        self.assertTrue(result['complete'])
        self.assertEqual(result['runtime_to_stop_ms'], 20)
        self.assertEqual(result['last_fault'], 32)
        self.assertEqual(result['wheels']['RR']['raw_count_change'], -1)
        self.assertEqual(result['wheels']['FL']['maximum_abs_duty_permille'], 80)

    def test_bad_abi_and_truncation(self):
        with self.assertRaises(ValueError):
            decode(blob()[:-1])
        wrong = bytearray(blob()); wrong[:4] = bytes(4)
        with self.assertRaises(ValueError):
            decode(wrong)

    def test_nonfinite_and_ccr_mismatch(self):
        values = list(ROW.unpack(row(1000, 1, 80)))
        values[5] = math.nan
        with self.assertRaises(ValueError):
            decode(blob([ROW.pack(*values)]))
        values = list(ROW.unpack(row(1000, 1, 80))); values[21] = 101
        with self.assertRaises(ValueError):
            decode(blob([ROW.pack(*values)]))

    def test_invalid_complete_and_disabled_output(self):
        with self.assertRaises(ValueError):
            decode(blob([row(1000, 1, 80)], True, 20))
        with self.assertRaises(ValueError):
            decode(blob([row(1000, 0, 80)]))

    def test_tick_wrap(self):
        records = [row(0xfffffff8, 1), row(2, 1, 80, (1, -1, 1, -1))]
        self.assertEqual(decode(blob(records))['rows'][1]['raw_cps_from_count_delta'][0], 100)

    def test_partial_stop_and_reenable(self):
        records = [row(1000, 1), row(1010, 0)]
        self.assertEqual(decode(blob(records, post=1))['post_stop_rows'], 1)
        with self.assertRaises(ValueError):
            decode(blob(records + [row(1020, 1)], post=1))
        with self.assertRaises(ValueError):
            decode(blob(records, post=0))

    def test_timestamp_and_inactive_metadata(self):
        data = bytearray(blob([row(1000, 1)]))
        values = list(HEADER.unpack_from(data)); values[-2] = 999
        data[:HEADER.size] = HEADER.pack(*values)
        with self.assertRaises(ValueError):
            decode(data)
        values[-2] = 1000; values[6] = 0
        data[:HEADER.size] = HEADER.pack(*values)
        with self.assertRaises(ValueError):
            decode(data)


if __name__ == '__main__':
    unittest.main()
