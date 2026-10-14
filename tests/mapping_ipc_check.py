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
        import stdio_publication_test as ipc
        ipc.BINARY = BINARY
        host = ipc.Session(directory)
        lines = ['READY roboframe-v3']
        try:
            for command in commands.splitlines(keepends=True):
                if not command.endswith('\n'):
                    host.ident += 1
                    host.write(f'REQ {host.ident} {command}')
                    break
                body = command.strip()
                response = host.req(body)
                lines.append(response)
                if body == 'QUIT': break
                if body.split()[0] in ('ODOM', 'SCAN') and not response.startswith('ERROR'):
                    host.wait_compute()
            host.p.stdin.close()
            try:
                while True:
                    line = host.line()
                    lines.append(line.split(' ', 2)[2] if line.startswith('RES ') else line)
            except EOFError:
                pass
            self.assertEqual(host.p.wait(timeout=5), 0)
            return lines
        finally:
            host.close()

    def test_workflow_freeze_and_load(self):
        with tempfile.TemporaryDirectory() as directory:
            lines = self.run_session(CONFIG + 'BEGIN 1\nODOM 0 0 0 0\nSCAN 50 1 2 0\n'
                'STATUS\nODOM 100 2 0 0\nSNAPSHOT progress\nFINISH\nSAVE final\n'
                'ODOM 200 3 0 0\nSCAN 150 1 2 0\nSAVE frozen\nSTATUS\nQUIT\n', directory)
            self.assertNotIn('ERROR', '\n'.join(lines))
            self.assertIn('STATUS mapping 1 1 0 0 0 1 0 0 0 0 0 0 0', lines)
            self.assertIn('STATUS ready 1 1 1 0 0 0 1 0 1 0 0 0 0', lines)
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
            self.assertEqual(self.run_session('QUIT\n', directory), ['READY roboframe-v3', 'BYE'])

    def test_live_map_preserves_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            commands = CONFIG + 'BEGIN 1\nODOM 0 0 0 0\nSCAN 0 1 2 0\nFINISH\nSAVE baseline\n'
            for i in range(1, 40):
                commands += f'ODOM {i * 100000} 0 0 0\nSCAN {i * 100000} 1 3 0\n'
            commands += 'RESOURCES\nSAVE preserved\nSNAPSHOT live\nQUIT\n'
            lines = self.run_session(commands, directory)
            self.assertFalse(any(line.startswith('ERROR') for line in lines))
            record = next(line for line in lines if line.startswith('RESOURCES '))
            resources = {key: int(value) for key, value in
                         (field.split('=') for field in record.split()[1:])}
            self.assertEqual(resources['live_updates'], 39)
            root = pathlib.Path(directory)
            self.assertEqual((root / 'baseline.rfgrid').read_bytes(), (root / 'preserved.rfgrid').read_bytes())
            self.assertNotEqual((root / 'baseline.rfgrid').read_bytes(), (root / 'live.rfgrid').read_bytes())

    def test_tf_chain_expiry_reset_and_persistence(self):
        config = 'CONFIG 30 30 1 -10 -10 100 MAP_TF 20\n'
        with tempfile.TemporaryDirectory() as directory:
            lines = self.run_session(config +
                'BEGIN 1\nODOM 0 1 0 0\nSCAN 0 1 2 0\nTF\n'
                'MAP_POSE 10 5 0 0\nSCAN 10 1 2 0\nSTATUS\nODOM 10 2 0 0\nTF\n'
                'ODOM 20 3 0 0\nSCAN 20 1 2 0\nTF\n'
                'ODOM 31 4 0 0\nSCAN 31 1 2 0\nTF\n'
                'MAP_POSE 40 7 0 0\nODOM 40 5 0 0\nSCAN 40 1 2 0\nTF\n'
                'FINISH\nSAVE mapped\nRESOURCES\n'
                'BEGIN 2\nODOM 50 6 0 0\nSCAN 50 1 2 0\nTF\nCANCEL\nQUIT\n', directory)
            self.assertFalse(any(line.startswith('ERROR') for line in lines))
            self.assertEqual(lines.count('TF unavailable'), 3)  # Initial, expired, and new session.
            self.assertIn('TF 10 10 2 0 0 3 0 0 5 0 0', lines)
            self.assertIn('TF 20 10 3 0 0 3 0 0 6 0 0', lines)  # Old correction, current odom.
            self.assertIn('TF 40 40 5 0 0 2 0 0 7 0 0', lines)
            resources = next(line for line in lines if line.startswith('RESOURCES '))
            self.assertIn('correction_outputs=2', resources)
            self.assertIn('tf_dropped=2', resources)
            file = pathlib.Path(directory) / 'mapped.rfgrid'
            self.assertTrue(file.read_bytes().startswith(b'RFGRID2\n'))
            lines = self.run_session(config + 'LOAD mapped\nTF\nSAVE copy\nQUIT\n', directory)
            self.assertIn('TF unavailable', lines)  # Loading a map is not localization.
            self.assertEqual(file.read_bytes(), (pathlib.Path(directory)/'copy.rfgrid').read_bytes())
            lines = self.run_session(CONFIG + 'LOAD mapped\nQUIT\n', directory)
            self.assertTrue(any('frame differs' in line for line in lines))
            lines = self.run_session(config + 'BEGIN 1\nMAP_POSE 10 0 0 0 extra\n'
                                    'MAP_POSE 10 nan 0 0\nQUIT\n', directory)
            self.assertEqual(sum(line.startswith('ERROR') for line in lines), 2)

    def test_eof_is_not_finish_or_save(self):
        with tempfile.TemporaryDirectory() as directory:
            lines = self.run_session(CONFIG + 'BEGIN 1\nODOM 0 0 0 0\nSCAN 0 1 2 0\nSAVE partial', directory)
            self.assertEqual(lines[-1], 'ERROR incomplete line')
            self.assertEqual(list(pathlib.Path(directory).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
