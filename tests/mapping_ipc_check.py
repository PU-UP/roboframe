#!/usr/bin/env python3
import pathlib
import subprocess
import sys
import tempfile
import unittest

BINARY = str(pathlib.Path(sys.argv.pop(1)).resolve())
CONFIG = 'CONFIG 20 20 1 -5 -5 100\n'


class MappingIPC(unittest.TestCase):
    def run_session(self, commands, directory):
        result = subprocess.run([BINARY, directory], input=commands, text=True,
                                capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        return result.stdout.splitlines()

    def test_workflow_freeze_and_load(self):
        with tempfile.TemporaryDirectory() as directory:
            lines = self.run_session(CONFIG + 'BEGIN 1\nODOM 0 0 0 0\nSCAN 50 1 2 0\n'
                'STATUS\nODOM 100 2 0 0\nSNAPSHOT progress\nFINISH\nSAVE final\n'
                'ODOM 200 3 0 0\nSCAN 150 1 2 0\nSAVE frozen\nSTATUS\nQUIT\n', directory)
            self.assertNotIn('ERROR', '\n'.join(lines))
            self.assertIn('STATUS mapping 1 1 0 0 0 1 0 0 0 0 0 0 0', lines)
            self.assertIn('STATUS ready 1 1 1 1 0 0 1 0 1 0 0 0 0', lines)
            final = (pathlib.Path(directory) / 'final.rfgrid').read_bytes()
            self.assertEqual(final, (pathlib.Path(directory) / 'frozen.rfgrid').read_bytes())
            lines = self.run_session(CONFIG + 'LOAD final\nSAVE reloaded\nQUIT\n', directory)
            self.assertIn('OK LOAD', lines)
            self.assertEqual(final, (pathlib.Path(directory) / 'reloaded.rfgrid').read_bytes())

    def test_invalid_input_does_not_mutate(self):
        with tempfile.TemporaryDirectory() as directory:
            lines = self.run_session('BEGIN 1\n' + CONFIG + 'BEGIN 1\nODOM 0 0 0 0\n'
                'SCAN 50 3000\nSCAN 50 1 30 0\nSCAN 50 1 2 0 extra\n'
                'SCAN 50 1 nan 0\nODOM -1 0 0 0\n' + 'X' * 70000 + '\n'
                'STATUS\nSCAN 50 1 2 0\nODOM 100 2 0 0\nFINISH\nSAVE ../escape\n'
                'SAVE valid\nSAVE valid\nSTATUS\nQUIT\n', directory)
            self.assertEqual(sum(line.startswith('ERROR ') for line in lines), 9)
            self.assertIn('STATUS mapping 1 0 0 0 0 0 0 0 0 0 0 0 0', lines)
            self.assertIn('OK SAVE', lines)
            self.assertEqual(len(list(pathlib.Path(directory).iterdir())), 1)

    def test_configuration_retry_and_duplicate(self):
        with tempfile.TemporaryDirectory() as directory:
            lines = self.run_session('STATUS\nCONFIG 0 20 1 -5 -5 100\n' + CONFIG +
                'BEGIN 1\n' + CONFIG + 'STATUS\nQUIT\n', directory)
            self.assertEqual(sum(line.startswith('ERROR ') for line in lines), 3)
            self.assertEqual(lines.count('OK CONFIG'), 1)
            self.assertIn('STATUS mapping 1 0 0 0 0 0 0 0 0 0 0 0 0', lines)
            self.assertEqual(list(pathlib.Path(directory).iterdir()), [])
            self.assertEqual(self.run_session('QUIT\n', directory), ['READY roboframe-v1', 'BYE'])

    def test_eof_is_not_finish_or_save(self):
        with tempfile.TemporaryDirectory() as directory:
            lines = self.run_session(CONFIG + 'BEGIN 1\nODOM 0 0 0 0\nSCAN 0 1 2 0\nSAVE partial', directory)
            self.assertEqual(lines[-1], 'ERROR incomplete line')
            self.assertEqual(list(pathlib.Path(directory).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
