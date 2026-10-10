#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import gzip
import hashlib
import tempfile
from unittest.mock import patch

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

    def test_corrected_profile_and_paired_pose(self):
        # Exported ODOM can have sub-microsecond garbage; it must never replace paired FLASER poses.
        text = ('ODOM 99 99 99 0 0 0 6.12103e-62 host 0\n' +
                self.scan().replace('1.000001', '2.000001') + '\n' + self.scan() + '\n')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'corrected.gz'
            path.write_bytes(gzip.compress(text.encode()))
            profile = dict(replay.DATASETS['corrected'], sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            with patch.dict(replay.DATASETS, corrected=profile):
                records, backwards = replay.load(path, 'corrected')
                self.assertEqual([r['stamp'] for r in records], [1000001, 2000001])
                self.assertTrue(all(r['kind'] == 'scan' and r['pose'] == [0, 0, 0] for r in records))
                self.assertEqual(backwards, dict(odom=0, scan=1))
                with self.assertRaisesRegex(ValueError, 'checksum'):
                    replay.load(path, 'raw')
                path.write_bytes(gzip.compress((text + '# changed').encode()))
                with self.assertRaisesRegex(ValueError, 'checksum'):
                    replay.load(path, 'corrected')

    def test_pairing_uses_measurements_and_preserves_raw_odometry(self):
        raw = [replay.parse(self.scan().replace('1.000001', '10.000001'), 1),
               replay.parse(self.scan([2]*180).replace('1.000001', '11.000001'), 2)]
        corrected = [replay.parse(self.scan(pose=[5, 2, 0.2, 5, 2, 0.2]), 42)]
        paired = replay.pair_references(raw, corrected)
        self.assertEqual(paired[0]['pose'], [0, 0, 0])
        self.assertEqual(paired[0]['stamp'], 10000001)
        self.assertEqual(paired[0]['reference_source_stamp'], 1000001)
        self.assertEqual(paired[0]['map_reference'], [5, 2, 0.2])
        self.assertNotIn('map_reference', paired[1])
        self.assertAlmostEqual(paired[1]['display_pose'][0], 5)
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            replay.pair_references([replay.parse(self.scan(), 1)]*2, corrected)
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            replay.pair_references([], corrected)

    def test_reject_and_filter(self):
        self.assertEqual(replay.parse('# comment', 1), None)
        ranges = [1] * 180
        ranges[0:3] = [0, 81.83, 20]
        filtered = replay.parse(self.scan(ranges), 1)
        self.assertEqual(filtered['filtered'], 3)
        full = replay.parse(self.scan(), 1)
        self.assertEqual(filtered['hits'], full['hits'][3:])
        self.assertEqual(filtered['pose'], full['pose'])
        self.assertEqual(filtered['stamp'], full['stamp'])
        for line in [self.scan(pose=[1, 0, 0, 0, 0, 0]), self.scan([float('nan')] * 180),
                     self.scan([-1] * 180), 'FLASER 180 1 2', 'ODOM nan 0 0 0 0 0 1 host 0']:
            with self.assertRaises(ValueError):
                replay.parse(line, 1)
        for stamp in ['-1', 'nan', '1.0000001', '1000000000000000']:
            with self.assertRaises(ValueError):
                replay.timestamp(stamp)


if __name__ == '__main__':
    unittest.main()
