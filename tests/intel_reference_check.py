#!/usr/bin/env python3
"""Optional real-data acceptance: provenance and conservative free-space connectivity, not a planner component."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/mapping'))
import replay


def check(path):
    scans, _ = replay.load(replay.DATASETS['corrected']['path'], 'corrected')
    raw, _ = replay.load(replay.DATA)
    raw_points = {tuple(map(tuple, s['hits'])) for s in raw if s['kind'] == 'scan'}
    assert len(scans) == 910
    assert all(tuple(map(tuple, s['hits'])) in raw_points for s in scans)
    grid = replay.read_map(path)
    assert grid['frame'] == 'map', 'regenerate the corrected reference with explicit map-frame TF'
    assert grid['revision'] == len(scans), 'use the live map containing all corrected scans'
    w, h, res, cells = grid['width'], grid['height'], grid['resolution'], grid['cells']
    # Conservative square clearance plus one cell for boundaries/segments; unknown is blocked.
    radius = 0.2
    margin = math.ceil(radius / res) + 1
    clear = set()
    for index, value in enumerate(cells):
        x, y = index % w, index // w
        if value != 254 or not margin <= x < w-margin or not margin <= y < h-margin:
            continue
        if all(cells[(y+dy)*w+x+dx] == 254
               for dy in range(-margin, margin+1) for dx in range(-margin, margin+1)):
            clear.add(index)
    def cell(pose):
        x = math.floor((pose[0]-grid['origin'][0])/res)
        y = math.floor((pose[1]-grid['origin'][1])/res)
        return y*w+x if 0 <= x < w and 0 <= y < h else -1
    candidates = [cell(s['pose']) for s in scans if cell(s['pose']) in clear]
    assert candidates, 'no recorded pose has conservative clearance'
    start = candidates[0]
    queue = deque([start])
    parent = {start: None}
    while queue:
        current = queue.popleft()
        # All clear cells have a border margin, so these edges cannot wrap rows.
        for neighbor in (current-1, current+1, current-w, current+w):
            if neighbor in clear and neighbor not in parent:
                parent[neighbor] = current
                queue.append(neighbor)
    end = current  # Farthest reachable cell in this four-connected diagnostic graph.
    path_cells = []
    while current is not None:
        path_cells.append(current)
        current = parent[current]
    path_cells.reverse()
    assert all(i in clear for i in path_cells)
    length = (len(path_cells)-1)*res
    assert length >= 10, 'insufficient connected space for the next planning exercise'
    def center(index):
        return [grid['origin'][0]+(index%w+0.5)*res,
                grid['origin'][1]+(index//w+0.5)*res]
    return dict(passed=True, dataset='corrected', matched_raw_scans=len(scans),
                map_revision=grid['revision'], map_frame=grid['frame'],
                robot_half_width_m=radius, conservative_margin_cells=margin,
                unknown_is_blocked=True, reachable_cells=len(parent),
                path_length_m=round(length, 3), start=center(start), goal=center(end),
                path=[center(i) for i in path_cells],
                note='Offline connectivity check only; not runtime planning, localization or control')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map', type=Path)
    parser.add_argument('--output', type=Path, help='new JSON artifact path')
    args = parser.parse_args()
    result = check(args.map)
    if args.output:
        with args.output.open('x') as stream:
            json.dump(result, stream, indent=2)
            stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'path'}, indent=2))
