#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mapping_replay', ROOT / 'tools/mapping/replay.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


class DatasetAdapter(unittest.TestCase):
    def scan(self, ranges=None, pose=None):
        ranges = ranges or [1] * 180
        pose = pose or [0] * 6
        return 'FLASER 180 ' + ' '.join(map(str, ranges + pose)) + ' 1.000001 host 0'

    def test_geometry_and_clock(self):
        result = replay.parse(self.scan(), 42)
        self.assertEqual(result['stamp'], 1000001)
        self.assertEqual(result['line'], 42)
        self.assertEqual(result['pose'], [0, 0, 0])
        paired = replay.parse(self.scan(pose=[1, 2, 0.3, 1, 2, 0.3]), 1)
        self.assertEqual(paired['pose'], [1, 2, 0.3])
        self.assertEqual(len(result['hits']), 180)
        self.assertAlmostEqual(result['hits'][0][0], 0)
        self.assertAlmostEqual(result['hits'][0][1], -1)
        self.assertAlmostEqual(result['hits'][-1][1], 1)
        self.assertLess(result['hits'][89][1], 0)
        self.assertGreater(result['hits'][90][1], 0)
        odom = replay.parse('ODOM 1 2 3 0 0 0 1.000002 host 0', 43)
        self.assertEqual(odom['pose'], [1, 2, 3])
        self.assertEqual(odom['stamp'], 1000002)

    def test_reject_and_filter(self):
        self.assertEqual(replay.parse('# comment', 1), None)
        ranges = [1] * 180
        ranges[0:3] = [0, 81.83, 20]
        self.assertEqual(replay.parse(self.scan(ranges), 1)['filtered'], 3)
        for line in [self.scan(pose=[1, 0, 0, 0, 0, 0]), self.scan([float('nan')] * 180),
                     self.scan([-1] * 180), 'FLASER 180 1 2', 'ODOM nan 0 0 0 0 0 1 host 0']:
            with self.assertRaises(ValueError):
                replay.parse(line, 1)
        for stamp in ['-1', 'nan', '1.0000001', '1000000000000000']:
            with self.assertRaises(ValueError):
                replay.timestamp(stamp)


if __name__ == '__main__':
    unittest.main()
