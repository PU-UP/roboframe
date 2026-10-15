#!/usr/bin/env python3
"""Real pipes: unsolicited output, bounded backpressure, EOF, disconnect and shutdown."""
import fcntl
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import time
import unittest

BINARY = None

class Session:
    def __init__(self, directory, delay=0):
        self.p = subprocess.Popen([BINARY, directory, f'--map-delay-ms={delay}'], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        fcntl.fcntl(self.p.stdout, fcntl.F_SETPIPE_SZ, 4096)
        self.buffer = b''
        self.events = []
        self.ident = 0
        assert self.line() == 'READY roboframe-v4'

    def write(self, text):
        data = text.encode()
        while data:
            n = os.write(self.p.stdin.fileno(), data)
            data = data[n:]

    def line(self):
        deadline = time.monotonic() + 5
        while b'\n' not in self.buffer:
            left = deadline - time.monotonic()
            if left <= 0 or not select.select([self.p.stdout], [], [], left)[0]:
                raise TimeoutError('output timeout')
            data = os.read(self.p.stdout.fileno(), 65536)
            if not data:
                raise EOFError(self.p.stderr.read().decode())
            self.buffer += data
        line, self.buffer = self.buffer.split(b'\n', 1)
        return line.decode()

    def submit(self, command):
        self.ident += 1
        self.write(f'REQ {self.ident} {command}\n')
        return self.ident

    def response(self, ident):
        while True:
            line = self.line()
            if line.startswith('EVENT '):
                event = line.split()
                if self.events:
                    assert int(event[1]) > int(self.events[-1][1])
                self.events.append(event)
                continue
            if line.startswith('RES 0 ERROR'):
                return line.split(' ', 2)[2]
            prefix = f'RES {ident} '
            assert line.startswith(prefix), line[:200]
            return line[len(prefix):]

    def req(self, command):
        response = self.response(self.submit(command))
        if response == 'ACCEPTED FINISH':
            deadline = time.monotonic() + 5
            while self.req('STATUS').split()[1] == 'finishing':
                if time.monotonic() >= deadline: raise TimeoutError('finish timeout')
                time.sleep(0.001)
        return response

    def wait_compute(self):
        deadline = time.monotonic() + 5
        while True:
            fields = self.req('RESOURCES').split()[1:]
            values = dict(x.split('=') for x in fields)
            if not int(values['pipeline_pending']): return
            if time.monotonic() >= deadline: raise TimeoutError('compute timeout')
            time.sleep(0.001)

    def prepare(self, size=20):
        assert self.req('SUBSCRIBE ON') == 'OK SUBSCRIBE'
        assert self.req(f'CONFIG {size} {size} 1 -5 -5 100') == 'OK CONFIG'
        assert self.req('RUN') == 'OK RUN'
        assert self.req('BEGIN 1') == 'OK BEGIN'
        self.write('DATA ODOM 0 0 0 0\nDATA SCAN 0 1 2 0\n')
        assert self.req('FINISH') == 'ACCEPTED FINISH'

    def close(self):
        if self.p.poll() is None:
            self.p.kill()
        self.p.wait(timeout=5)
        self.p.stdin.close(); self.p.stdout.close(); self.p.stderr.close()

class Publication(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.s = Session(self.directory.name)

    def tearDown(self):
        self.s.close(); self.directory.cleanup()

    def test_odom_without_scan_and_unsubscribe(self):
        s = self.s
        s.req('CONFIG 20 20 1 -5 -5 100')
        s.req('RUN')
        s.req('SUBSCRIBE ON')
        s.write('DATA ODOM 0 1 2 0.5\n')
        event = s.line().split()  # No STATUS or any other request to elicit output.
        self.assertEqual(event[2:], ['ODOM', '0', '1', '2', '0.5'])
        s.req('SUBSCRIBE OFF')
        s.write('DATA ODOM 1 2 2 0.5\n')
        self.assertTrue(s.req('TF').startswith('TF '))
        self.assertTrue(all(e[2] == 'TF' for e in s.events))
        self.assertEqual(s.req('QUIT'), 'BYE')
        self.assertEqual(s.p.wait(timeout=3), 0)

    def test_map_payload_events_and_generation(self):
        s = self.s; s.prepare()
        s.req('SAVE first')
        map1 = s.req('GET_MAP baseline').split()
        self.assertEqual(map1[:2], ['MAP', 'baseline'])
        cells = bytes.fromhex(map1[-1])
        self.assertEqual(len(cells), 400)
        self.assertEqual(cells[5 * 20 + 7], 0)
        kinds = [e[3] for e in s.events if e[2] == 'MAPPING']
        self.assertTrue(all(k in kinds for k in ['started', 'finished', 'saved']))
        s.write('DATA ODOM 100000 0 0 0\nDATA SCAN 100000 1 3 0\n')
        self.assertEqual(s.req('GET_MAP baseline').split(), map1)
        s.wait_compute()
        self.assertNotEqual(s.req('GET_MAP live').split()[-1], map1[-1])
        s.req('STOP'); s.req('LOAD first')
        map2 = s.req('GET_MAP baseline').split()
        self.assertEqual(int(map2[2]), 0)
        self.assertEqual(map2[-1], map1[-1])
        self.assertEqual(s.req('TF'), 'TF unavailable')
        s.req('RUN'); s.req('BEGIN 2'); s.req('CANCEL')
        self.assertEqual(s.req('RUNTIME').split()[1], 'RUNNING')
        self.assertEqual(s.req('GET_MAP baseline').split()[-1], map1[-1])
        self.assertEqual(s.req('QUIT'), 'BYE')
        self.assertEqual(s.p.wait(timeout=3), 0)
        self.assertTrue(any(e[2:4] == ['RUNTIME', 'IDLE'] for e in s.events))

    def test_slow_reader_coalesces_without_blocking_input(self):
        s = self.s
        s.req('SUBSCRIBE ON'); s.req('CONFIG 20 20 1 -5 -5 100'); s.req('RUN'); s.req('BEGIN 1')
        s.write('DATA ODOM 0 0 0 0\nDATA SCAN 0 1 2 0\n')
        s.req('STATUS')
        s.wait_compute()
        # Stop reading; DATA, task completion and SAVE must still be processed.
        s.write(''.join(f'DATA ODOM {i} 0 0 0\n' for i in range(1, 2001)))
        finish_id = s.submit('FINISH')
        ident = s.submit('SAVE while_slow')
        deadline = time.monotonic() + 1
        marker = Path(self.directory.name) / 'while_slow.rfgrid'
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(marker.exists(), 'input stalled behind output')
        self.assertIsNone(s.p.poll())
        self.assertEqual(s.response(finish_id), 'ACCEPTED FINISH')
        self.assertEqual(s.response(ident), 'OK SAVE')
        stats = dict(field.split('=') for field in s.req('IPC_STATUS').split()[1:])
        self.assertGreater(int(stats['replaced']), 0)
        self.assertLess(int(stats['peak_bytes']), 10000)
        self.assertTrue(any(e[2:4] == ['ODOM', '2000'] for e in s.events))
        self.assertTrue(any(e[2:4] == ['MAPPING', 'saved'] for e in s.events))
        self.assertTrue(any(e[2:4] == ['MAPPING', 'finished'] and int(e[4]) == 0 for e in s.events))
        s.req('QUIT')
        self.assertEqual(s.p.wait(timeout=3), 0)

    def test_stalled_reader_times_out_while_input_is_idle(self):
        s = self.s; s.prepare(1000)
        s.submit('GET_MAP live')  # Two MB cannot fit in the unread 4 KiB pipe.
        self.assertNotEqual(s.p.wait(timeout=5), 0)
        self.assertIn('reader stalled', s.p.stderr.read().decode())

    def test_reliable_overflow_is_connection_failure(self):
        s = self.s; s.prepare()
        try:
            s.write(''.join(f'REQ {100+i} STATUS\n' for i in range(600)))
        except BrokenPipeError:
            pass
        self.assertNotEqual(s.p.wait(timeout=5), 0)
        self.assertIn('queue exhausted', s.p.stderr.read().decode())

    def test_output_disconnect_without_new_input(self):
        s = self.s; s.prepare()
        s.p.stdout.close()
        self.assertNotEqual(s.p.wait(timeout=3), 0)
        self.assertIn('disconnected', s.p.stderr.read().decode())

    def test_eof_emits_stop_without_saving(self):
        s = self.s
        s.req('CONFIG 20 20 1 -5 -5 100'); s.req('SUBSCRIBE ON'); s.req('RUN'); s.req('BEGIN 1')
        s.write('DATA ODOM 0 0 0 0\nDATA SCAN 0 1 2 0\n')
        s.p.stdin.close()
        lines = []
        try:
            while True: lines.append(s.line())
        except EOFError:
            pass
        self.assertEqual(s.p.wait(timeout=3), 0)
        self.assertTrue(any('MAPPING stopped' in line for line in lines))
        self.assertFalse(any('MAPPING finished' in line for line in lines))
        self.assertEqual(list(Path(self.directory.name).iterdir()), [])

    def test_shutdown_drain_has_deadline(self):
        s = self.s; s.prepare(1000)
        s.submit('GET_MAP live'); s.submit('QUIT')
        self.assertNotEqual(s.p.wait(timeout=5), 0)
        self.assertRegex(s.p.stderr.read().decode(), 'stalled|drain timed out')

    def test_ids_malformed_data_and_partial_input(self):
        s = self.s
        s.req('CONFIG 20 20 1 -5 -5 100')
        s.write('REQ 1 BEGIN 9\n')
        self.assertTrue(s.line().startswith('RES 0 ERROR request id'))
        s.write('DATA QUIT\n')
        self.assertTrue(s.line().startswith('RES 0 ERROR DATA'))
        s.write('DATA ODOM nan 0 0 0\n')
        self.assertTrue(s.line().startswith('RES 0 ERROR'))
        s.write('REQ 2 BEGIN 1')
        s.p.stdin.close()
        self.assertEqual(s.line(), 'RES 0 ERROR incomplete line')
        self.assertEqual(s.p.wait(timeout=3), 0)

if __name__ == '__main__':
    BINARY = str(Path(sys.argv.pop(1)).resolve())
    unittest.main()
