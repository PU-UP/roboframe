#!/usr/bin/env python3
"""Real process pipes: task ownership, typed parsing, results and shutdown."""
from pathlib import Path
import subprocess
import sys
import unittest
BINARY = str(Path(sys.argv.pop(1)).resolve())

class IpcCheck(unittest.TestCase):
    def run_host(self, text):
        result = subprocess.run([BINARY], input=text, text=True, capture_output=True, timeout=3)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('[integration] shutdown', result.stderr)
        return result.stdout.splitlines()

    def test_completion_and_cancel(self):
        self.assertEqual(self.run_host(
            'GOAL 1 2\nSAMPLE 3\nSAMPLE 5\nSTATUS\nGOAL 2 3\nSAMPLE -2\nCANCEL 2\nQUIT\n'),
            ['READY', 'ACCEPTED 1', 'FEEDBACK 1 1 3', 'FEEDBACK 1 2 8',
             'RESULT 1 succeeded 2 8 none', 'STATUS active 1 succeeded',
             'ACCEPTED 2', 'FEEDBACK 2 1 -2', 'RESULT 2 canceled 1 -2 none', 'BYE'])

    def test_rejections_leave_task_unchanged(self):
        self.assertEqual(self.run_host(
            'SAMPLE 1\nGOAL 1 2\nGOAL 2 3\nCANCEL 2\nDEACTIVATE\nSAMPLE 4\n'
            'CANCEL 1\nGOAL 1 1\nSAMPLE 9\nSTATUS\nQUIT\n'),
            ['READY', 'ERROR no running goal', 'ACCEPTED 1', 'ERROR component busy',
             'ERROR wrong goal id', 'ERROR cancel task before deactivation', 'FEEDBACK 1 1 4',
             'RESULT 1 canceled 1 4 none', 'ERROR goal id must increase',
             'ERROR no running goal', 'STATUS active 1 canceled', 'BYE'])

    def test_lifecycle_is_separate_from_task(self):
        self.assertEqual(self.run_host(
            'DEACTIVATE\nGOAL 1 1\nSTATUS\nACTIVATE\nGOAL 1 1\nSAMPLE 7\nDEACTIVATE\nSTATUS\nQUIT\n'),
            ['READY', 'OK DEACTIVATE', 'ERROR component inactive', 'STATUS inactive 0 idle',
             'OK ACTIVATE', 'ACCEPTED 1', 'FEEDBACK 1 1 7', 'RESULT 1 succeeded 1 7 none',
             'OK DEACTIVATE', 'STATUS inactive 1 succeeded', 'BYE'])

    def test_malformed_requests_do_not_start_task(self):
        lines = ['GOAL -1 2', 'GOAL 1 0', 'GOAL 0 1', 'GOAL 1 2 extra',
                 'GOAL 18446744073709551616 1', 'SAMPLE', 'SAMPLE 1.5',
                 'SAMPLE 9223372036854775808', 'UNKNOWN', '', 'X'*10000]
        replies = self.run_host('\n'.join(lines+['STATUS','QUIT'])+'\n')
        self.assertEqual(replies[0], 'READY')
        self.assertEqual(len(replies), len(lines)+3)
        self.assertTrue(all(r.startswith('ERROR ') for r in replies[1:-2]))
        self.assertEqual(replies[-2:], ['STATUS active 0 idle', 'BYE'])

    def test_failure_has_one_result_and_component_can_run_again(self):
        self.assertEqual(self.run_host(
            'GOAL 1 2\nSAMPLE 9223372036854775807\nSAMPLE 1\nSAMPLE 2\n'
            'GOAL 2 1\nSAMPLE -4\nQUIT\n'),
            ['READY', 'ACCEPTED 1', 'FEEDBACK 1 1 9223372036854775807',
             'RESULT 1 failed 1 9223372036854775807 overflow', 'ERROR no running goal',
             'ACCEPTED 2', 'FEEDBACK 2 1 -4', 'RESULT 2 succeeded 1 -4 none', 'BYE'])

    def test_eof_cancels_and_incomplete_request_is_not_executed(self):
        self.assertEqual(self.run_host('GOAL 1 2\nSAMPLE 3\nSAMPLE 4'),
            ['READY', 'ACCEPTED 1', 'FEEDBACK 1 1 3', 'ERROR incomplete line',
             'RESULT 1 canceled 1 3 none'])
        self.assertEqual(self.run_host('GOAL 1 2\nQUIT\n'),
            ['READY', 'ACCEPTED 1', 'RESULT 1 canceled 0 0 none', 'BYE'])
        self.assertEqual(self.run_host(''), ['READY'])

if __name__ == '__main__':
    unittest.main()
