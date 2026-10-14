#!/usr/bin/env python3
"""Real IPC: runtime lifetime, task lifetime and bounded computation are independent."""
import importlib.util
import json
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
import stdio_publication_test as ipc

class Execution(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.s = ipc.Session(self.directory.name, delay=50)

    def tearDown(self):
        self.s.close()
        self.directory.cleanup()

    def stats(self):
        return {k: int(v) for k, v in
                (x.split('=') for x in self.s.req('RESOURCES').split()[1:])}

    def start(self):
        self.assertEqual(self.s.req('CONFIG 20 20 1 -5 -5 100'), 'OK CONFIG')
        self.assertEqual(self.s.req('RUNTIME'), 'RUNTIME IDLE 0')
        self.assertTrue(self.s.req('BEGIN 1').startswith('ERROR'))
        self.s.req('RUN')
        self.s.req('SUBSCRIBE ON')

    def test_reception_overload_cancel_and_new_task(self):
        self.start()
        s = self.s
        s.req('BEGIN 1')
        for i in range(200):
            s.write(f'DATA SCAN {i} 1 2 0\nDATA ODOM {i} 0 0 0\n')
            if i % 20 == 19: s.req('STATUS')
        stats = self.stats()
        self.assertEqual(stats['compute_peak_retained'], 8)
        self.assertGreater(stats['compute_rejected'], 0)
        self.assertTrue(any(e[2:4] == ['ODOM', '199'] for e in s.events))
        s.req('CANCEL'); s.req('BEGIN 2')
        self.assertEqual(s.req('RUNTIME'), 'RUNTIME RUNNING 1')
        s.wait_compute()
        self.assertEqual(int(s.req('STATUS').split()[4]), 0)
        self.assertGreater(self.stats()['detached_task_results'], 0)
        s.write('DATA ODOM 200 3 0 0\nDATA SCAN 200 1 2 0\n')
        s.req('FINISH')
        self.assertEqual(s.req('STATUS').split()[1:5], ['ready', '2', '1', '1'])
        self.assertTrue(s.req('GET_MAP baseline').startswith('MAP baseline'))
        s.req('STOP')
        self.assertEqual(s.req('RUNTIME'), 'RUNTIME IDLE 1')
        self.assertTrue(s.req('ODOM 201 0 0 0').startswith('ERROR'))
        s.req('RESET'); s.req('RUN')
        self.assertEqual(s.req('RUNTIME'), 'RUNTIME RUNNING 2')
        self.assertEqual(s.req('TF'), 'TF unavailable')
        s.write('DATA ODOM 0 0 0 0\nDATA SCAN 0 1 2 0\n')
        s.wait_compute()
        self.assertEqual(self.stats()['local_updates'], 1)

    def test_dataset_continues_without_task_and_after_cancel(self):
        # Reuse the authoritative dataset adapter; no second parser or artificial trajectory.
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location('mapping_replay', root/'tools/mapping/replay.py')
        replay = importlib.util.module_from_spec(spec); spec.loader.exec_module(replay)
        records, _ = replay.load(replay.DATA)
        references, _ = replay.load(replay.DATASETS['corrected']['path'], 'corrected')
        scans = replay.pair_references([r for r in records if r['kind'] == 'scan'], references)[:320]
        s = self.s
        self.assertEqual(s.req('CONFIG 1000 1000 0.2 -100 -100 250000 MAP_TF 20000000'), 'OK CONFIG')
        s.req('RUN')
        start = scans[0]['stamp']
        def feed(begin, end):
            for i in range(begin, end):
                scan = scans[i]; stamp = scan['stamp'] - start
                pose = ' '.join(map(str, scan['pose']))
                s.write(f'DATA ODOM {stamp} {pose}\n')
                if 'map_reference' in scan:
                    pose = ' '.join(map(str, scan['map_reference']))
                    s.write(f'DATA MAP_POSE {stamp} {pose}\n')
                points = ' '.join(str(v) for xy in scan['hits'] for v in xy)
                s.write(f'DATA SCAN {stamp} {len(scan["hits"])} {points}\n')
                if i % 4 == 3: s.wait_compute()
            s.wait_compute()
        feed(0, 200)
        before = self.stats()
        self.assertEqual(s.req('STATUS').split()[1:3], ['idle', '0'])
        self.assertEqual(before['slam_scans'], 200)
        self.assertEqual(before['local_updates'], 200)  # Includes the 168 scans without global TF.
        self.assertGreater(before['publications'], 0)
        s.req('BEGIN 1'); feed(200, 216); s.req('CANCEL')
        canceled = s.req('STATUS')
        feed(216, 240)
        after = self.stats()
        self.assertEqual(s.req('STATUS'), canceled)
        self.assertEqual(after['local_updates'], 240)
        self.assertEqual(s.req('RUNTIME'), 'RUNTIME RUNNING 1')
        self.assertNotEqual(s.req('TF'), 'TF unavailable')
        s.req('BEGIN 2'); feed(240, 280); s.req('FINISH')
        baseline = s.req('GET_MAP baseline')
        feed(280, 320)
        final = self.stats()
        self.assertEqual(s.req('GET_MAP baseline'), baseline)
        self.assertEqual(final['slam_scans'], 320)
        self.assertEqual(final['local_updates'], 320)
        self.assertEqual(final['compute_rejected'], 0)
        self.assertLessEqual(final['compute_peak_retained'], 8)
        print(json.dumps(dict(dataset='intel raw + recorded corrected poses',
            no_task_local_updates=before['local_updates'],
            after_cancel_local_updates=after['local_updates'],
            final_local_updates=final['local_updates'],
            map_updates=final['live_updates']+int(s.req('STATUS').split()[4]),
            missing_global_tf=final['tf_dropped'], peak=final['compute_peak_retained'])))
        s.req('QUIT')
        self.assertEqual(s.p.wait(timeout=1), 0)

    def test_invalid_delay_is_rejected(self):
        for value in ('-1', 'x', '5001', ''):
            result = subprocess.run([ipc.BINARY, self.directory.name, '--map-delay-ms='+value],
                                    capture_output=True, timeout=2)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, b'')

    def test_close_during_computation(self):
        self.s.close()
        for mode in ('quit', 'eof', 'disconnect'):
            with self.subTest(mode=mode):
                s = ipc.Session(self.directory.name, delay=5000)
                try:
                    s.req('CONFIG 20 20 1 -5 -5 100'); s.req('RUN')
                    s.write('DATA ODOM 0 0 0 0\nDATA SCAN 0 1 2 0\n')
                    s.req('STATUS')
                    start = time.monotonic()
                    if mode == 'quit': self.assertEqual(s.req('QUIT'), 'BYE')
                    elif mode == 'eof': s.p.stdin.close()
                    else: s.p.stdout.close()
                    code = s.p.wait(timeout=1)
                    self.assertLess(time.monotonic()-start, 1)
                    self.assertEqual(code == 0, mode != 'disconnect')
                    self.assertEqual(list(Path(self.directory.name).iterdir()), [])
                finally: s.close()

if __name__ == '__main__':
    ipc.BINARY = str(Path(sys.argv.pop(1)).resolve())
    unittest.main()
