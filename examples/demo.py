#!/usr/bin/env python3
"""External caller: goal/feedback/result over real pipes, with readable layer traces."""
import argparse
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('binary', nargs='?', type=Path,
                    default=Path(__file__).resolve().parents[1]/'build/robot_runtime')
args = parser.parse_args()
commands = ['GOAL 1 2', 'SAMPLE 3', 'SAMPLE 5', 'STATUS',
            'GOAL 2 3', 'GOAL 3 1', 'SAMPLE -2', 'CANCEL 2', 'SAMPLE 9',
            'DEACTIVATE', 'STATUS', 'ACTIVATE', 'GOAL 3 1', 'SAMPLE 7', 'QUIT']
print('[product] pipe input:\n  ' + '\n  '.join(commands), flush=True)
result = subprocess.run([str(args.binary.resolve())], input='\n'.join(commands)+'\n',
                        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        timeout=5, check=True)
print(result.stdout, end='')
responses = [line for line in result.stdout.splitlines() if not line.startswith('[')]
expected = ['READY', 'ACCEPTED 1', 'FEEDBACK 1 1 3', 'FEEDBACK 1 2 8',
            'RESULT 1 succeeded 2 8 none', 'STATUS active 1 succeeded',
            'ACCEPTED 2', 'ERROR component busy', 'FEEDBACK 2 1 -2',
            'RESULT 2 canceled 1 -2 none', 'ERROR no running goal',
            'OK DEACTIVATE', 'STATUS inactive 2 canceled', 'OK ACTIVATE',
            'ACCEPTED 3', 'FEEDBACK 3 1 7', 'RESULT 3 succeeded 1 7 none', 'BYE']
if responses != expected:
    raise RuntimeError(f'unexpected responses: {responses}')
print('PASS: completion / cancellation / busy rejection / lifecycle through IPC')
