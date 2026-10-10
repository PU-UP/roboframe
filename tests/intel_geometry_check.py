#!/usr/bin/env python3
"""Compare actual IPC maps against polar-coordinate endpoints from unmodified Intel records."""
import argparse
import gzip
import math
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/mapping'))
import replay

def check(binary):
    raw = []
    with gzip.open(replay.DATA, 'rt') as stream:
        for line in stream:
            if line.startswith('FLASER '):
                raw.append(line)
    # Cover the full recording, including large translations and different headings.
    chosen = [raw[i * (len(raw) - 1) // 11] for i in range(12)]
    with tempfile.TemporaryDirectory(prefix='roboframe-intel-geometry-') as directory:
        host = replay.Host(binary.resolve(), Path(directory), 'geometry')
        try:
            w, h, res, ox, oy = 1085, 990, 0.1, -73, -58
            host.send(f'CONFIG {w} {h} {res} {ox} {oy} 250000')
            for number, line in enumerate(chosen, 1):
                fields = line.split()
                pose = list(map(float, fields[185:188]))
                ranges = list(map(float, fields[2:182]))
                expected = set()
                # Independent polar form, not the adapter's Cartesian rotation.
                angles = list(range(-90, 0)) + list(range(1, 91))
                for distance, degrees in zip(ranges, angles):
                    if not 0.01 <= distance < 20:
                        continue
                    angle = pose[2] + degrees * math.pi / 180
                    x = math.floor((pose[0] + distance * math.cos(angle) - ox) / res)
                    y = math.floor((pose[1] + distance * math.sin(angle) - oy) / res)
                    if not 0 <= x < w or not 0 <= y < h:
                        raise AssertionError('reference endpoint outside map')
                    expected.add(y*w+x)
                scan = replay.parse(line, number)
                host.send(f'BEGIN {number}')
                host.send('ODOM ' + str(number) + ' ' + ' '.join(format(v, '.17g') for v in pose))
                host.send('SCAN ' + str(number) + ' ' + str(len(scan['hits'])) + ' ' +
                          ' '.join(format(v, '.17g') for hit in scan['hits'] for v in hit))
                host.send('FINISH')
                host.send(f'SAVE scan-{number}')
                grid = replay.read_map(Path(directory) / f'scan-{number}.rfgrid')
                actual = {i for i, cell in enumerate(grid['cells']) if cell == 0}
                if actual != expected:
                    raise AssertionError(f'scan {number}: {len(actual ^ expected)} mismatched endpoint cells')
            host.send('QUIT')
            if host.process.wait(timeout=5) != 0:
                raise RuntimeError('host failed')
        finally:
            host.close()
    print('PASS: 12 recorded scans across the full log; all occupied cells match independent polar projection')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    check(parser.parse_args().binary)
