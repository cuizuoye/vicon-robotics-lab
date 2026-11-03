"""Pure file parsing checks; no desktop/ROS/robot imports."""
import json
from pathlib import Path
import tempfile
import unittest
from audit import load_trace


class TraceTests(unittest.TestCase):
    def parse(self, raw):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'trace.jsonl'
            path.write_bytes(raw)
            result=load_trace(path)
            self.assertEqual(path.read_bytes(),raw)
            return result

    def test_complete(self):
        rows, integrity=self.parse(b'{"t":1}\n{"t":2}\n')
        self.assertEqual(len(rows),2)
        self.assertTrue(integrity['source_complete'])

    def test_nul_tail_preserved_and_not_counted(self):
        rows, integrity=self.parse(b'{"t":1}\n'+b'\0'*1526)
        self.assertEqual(len(rows),1)
        self.assertFalse(integrity['source_complete'])
        self.assertEqual(integrity['anomalies'][0]['bytes'],1526)

    def test_middle_corruption_fails(self):
        with self.assertRaises(ValueError):
            self.parse(b'{"t":1}\n\0\n{"t":2}\n')

    def test_incomplete_json_not_silently_repaired(self):
        with self.assertRaises(ValueError):
            self.parse(b'{"t":1}\n{"t":')

    def test_nonobject_rejected(self):
        with self.assertRaises(ValueError):
            self.parse(b'[]\n')


if __name__ == '__main__':
    unittest.main()
