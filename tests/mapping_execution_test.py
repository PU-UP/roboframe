#!/usr/bin/env python3
"""Exercise slow computation through real IPC; do not use helpers that wait for FINISH."""
import subprocess
import sys
import time
import tempfile
import unittest
from pathlib import Path
import stdio_publication_test as ipc

class Execution(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.s = ipc.Session(self.directory.name, delay=1000)
        self.s.req('CONFIG 20 20 1 -5 -5 100')
        self.s.req('SUBSCRIBE ON'); self.s.req('BEGIN 1')

    def tearDown(self):
        self.s.close(); self.directory.cleanup()

    def stats(self):
        return {k: int(v) for k, v in (x.split('=') for x in self.s.req('RESOURCES').split()[1:])}

    def test_reception_overload_cancel_and_new_session(self):
        s = self.s
        start = time.monotonic()
        for i in range(200):
            s.write(f'DATA SCAN {i} 1 2 0\nDATA ODOM {i} 0 0 0\n')
            if i % 20 == 19: s.req('STATUS')  # Drain output; this is input/compute pressure.
        status = s.req('STATUS').split()
        stats = self.stats()
        self.assertLess(time.monotonic() - start, 0.7)
        self.assertEqual(status[3], '200')
        self.assertEqual(stats['compute_peak_retained'], 8)
        self.assertGreater(stats['compute_rejected'], 0)
        self.assertTrue(any(e[2:4] == ['ODOM', '199'] for e in s.events))
        self.assertEqual(s.response(s.submit('FINISH')), 'ACCEPTED FINISH')
        self.assertEqual(s.req('STATUS').split()[1], 'finishing')
        self.assertEqual(s.req('CANCEL'), 'OK CANCEL')
        self.assertEqual(s.req('BEGIN 2'), 'OK BEGIN')
        s.write('DATA ODOM 200 3 0 0\nDATA SCAN 200 1 2 0\n')
        self.assertEqual(s.response(s.submit('FINISH')), 'ACCEPTED FINISH')
        # No further input: owner must still collect and publish completion.
        while True:
            fields = s.line().split()
            self.assertEqual(fields[0], 'EVENT')
            if fields[2:4] == ['MAPPING', 'finished']:
                self.assertEqual(fields[4:6], ['0', '2']); break
        self.assertEqual(s.req('STATUS').split()[1], 'ready')
        self.assertGreaterEqual(self.stats()['stale_results'], 1)
        self.assertEqual(s.req('GET_MAP baseline').split()[3], '1')
        self.assertEqual(s.req('QUIT'), 'BYE')
        self.assertEqual(s.p.wait(timeout=1), 0)

    def test_invalid_delay_is_rejected(self):
        for value in ('-1', 'x', '5001', ''):
            result = subprocess.run([ipc.BINARY, self.directory.name, '--map-delay-ms=' + value],
                                    capture_output=True, timeout=2)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, b'')

    def test_close_during_computation(self):
        # Exercise each transport exit with an occupied worker.
        self.s.close()
        for mode in ('quit', 'eof', 'disconnect'):
            with self.subTest(mode=mode):
                s = ipc.Session(self.directory.name, delay=5000)
                try:
                    s.req('CONFIG 20 20 1 -5 -5 100'); s.req('BEGIN 1')
                    s.write('DATA ODOM 0 0 0 0\nDATA SCAN 0 1 2 0\n')
                    s.req('STATUS')
                    start = time.monotonic()
                    if mode == 'quit': self.assertEqual(s.req('QUIT'), 'BYE')
                    elif mode == 'eof': s.p.stdin.close()
                    else: s.p.stdout.close()
                    code = s.p.wait(timeout=1)
                    self.assertLess(time.monotonic() - start, 1)
                    self.assertEqual(code == 0, mode != 'disconnect')
                    self.assertEqual(list(Path(self.directory.name).iterdir()), [])
                finally: s.close()

if __name__ == '__main__':
    ipc.BINARY = str(Path(sys.argv.pop(1)).resolve())
    unittest.main()
