#!/usr/bin/env python3
"""Replay archived Intel odometry/laser data through IPC; save, reload and inspect maps."""
import argparse
import base64
from decimal import Decimal
import gzip
import hashlib
import json
import math
from pathlib import Path
import select
import struct
import subprocess
import zlib

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / 'history/2026-10-09-full-prototype/data/intel-lab/intel.log.gz'
SHA256 = '449a9813e9709b6fd053ba0d006ea69c5f4485ba08dd021448e653e3af2e6ff9'
DATASETS = {
    'raw': dict(path=DATA, sha256=SHA256,
                pose_source='FLASER associated raw odometry; unchanged',
                label='Intel raw-log / original odometry', frame='recorded_odom'),
    'corrected': dict(path=ROOT / 'data/intel-lab/intel.gfs.log.gz',
                      sha256='152d4db32761a71ac4a557d2bd383d4ba054864c290b799df92096ba34b70dea',
                      pose_source='Publisher corrected FLASER poses; offline reference, not online SLAM or ground truth',
                      label='Intel corrected-log / publisher corrected poses', frame='corrected_reference'),
}

DATASETS['tf'] = dict(DATASETS['raw'], label='Intel raw odom + sparse lidar-map correction',
                      pose_source='Raw paired odom; lidar-map computes T_map_odom from matched recorded map poses',
                      frame='map')


def timestamp(text):
    value = Decimal(text) * 1000000
    if not value.is_finite() or value < 0 or value != int(value) or value > 2**63 - 1:
        raise ValueError('expected nonnegative integer microsecond timestamp')
    return int(value)


def parse(line, number):
    fields = line.split()
    if not fields or fields[0] not in ('ODOM', 'FLASER'):
        return None
    if fields[0] == 'ODOM':
        if len(fields) != 10:
            raise ValueError('invalid ODOM record')
        pose = list(map(float, fields[1:4]))
        stamp = timestamp(fields[7])
        if not all(math.isfinite(x) for x in pose):
            raise ValueError('nonfinite odometry')
        return dict(kind='odom', stamp=stamp, pose=pose, line=number)
    if len(fields) != 191 or fields[1] != '180':
        raise ValueError('expected Intel FLASER with 180 readings')
    ranges = list(map(float, fields[2:182]))
    laser = list(map(float, fields[182:185]))
    odom = list(map(float, fields[185:188]))
    if not all(math.isfinite(x) for x in ranges + laser + odom) or any(x < 0 for x in ranges):
        raise ValueError('invalid scan values')
    if any(abs(a - b) > 1e-6 for a, b in zip(laser, odom)):
        raise ValueError('adapter requires identity laser/base extrinsics')
    hits = []
    for i, distance in enumerate(ranges):
        if 0.01 <= distance < 20:
            # Archived Intel adapter convention: legacy CARMEN -90..-1,+1..+90 degrees.
            angle = math.radians(i - 90 if i < 90 else i - 89)
            hits.append([distance * math.cos(angle), distance * math.sin(angle)])
    if not hits:
        raise ValueError('scan contains no usable hit')
    return dict(kind='scan', stamp=timestamp(fields[188]), hits=hits,
                filtered=180 - len(hits), pose=odom, line=number, range_key=tuple(ranges))


def load(path, dataset='raw'):
    if hashlib.sha256(path.read_bytes()).hexdigest() != DATASETS[dataset]['sha256']:
        raise ValueError('Intel dataset checksum mismatch')
    records, previous = [], {}
    backwards = dict(odom=0, scan=0)
    with gzip.open(path, 'rt', encoding='ascii') as stream:
        for number, line in enumerate(stream, 1):
            # Corrected poses belong to each FLASER; its separate ODOM stream is not an input.
            if dataset == 'corrected' and not line.startswith('FLASER '):
                continue
            record = parse(line, number)
            if record is None:
                continue
            kind = record['kind']
            if kind in previous and record['stamp'] < previous[kind]:
                backwards[kind] += 1
            previous[kind] = record['stamp']
            records.append(record)
    records.sort(key=lambda r: (r['stamp'], r['kind'] != 'odom'))
    last = {}
    for record in records:
        if last.get(record['kind']) == record['stamp']:
            raise ValueError('duplicate per-stream timestamp')
        last[record['kind']] = record['stamp']
    return records, backwards



def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    yaw = a[2] + b[2]
    return [a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], math.atan2(math.sin(yaw), math.cos(yaw))]


def pair_references(raw, corrected):
    # Match measurements, not timestamps/row numbers: the publisher exported a different clock.
    index = {}
    for scan in raw:
        key = scan['range_key']
        if key in index:
            index[key] = None
        else:
            index[key] = scan
    for reference in corrected:
        scan = index.get(reference['range_key'])
        if scan is None or 'map_reference' in scan:
            raise ValueError('corrected scan must match exactly one distinct raw scan')
        scan['map_reference'] = reference['pose']
        scan['reference_source_stamp'] = reference['stamp']
    correction = None
    for scan in raw:
        pose = scan['pose']
        if 'map_reference' in scan:
            c, sn = math.cos(pose[2]), math.sin(pose[2])
            correction = compose(scan['map_reference'], [-c*pose[0]-sn*pose[1], sn*pose[0]-c*pose[1], -pose[2]])
        scan['display_pose'] = compose(correction, pose) if correction is not None else None
    return raw


def read_map(path):
    with path.open('rb') as stream:
        magic = stream.readline()
        if magic not in (b'RFGRID1\n', b'RFGRID2\n'):
            raise ValueError('invalid map magic')
        metadata = stream.readline().rstrip(b'\n')
        checksum = int(stream.readline())
        cells = stream.read()
    fields = metadata.decode().split()
    frame = 'odom'
    if magic == b'RFGRID2\n':
        if len(fields) != 8 or fields[-1] != 'map':
            raise ValueError('invalid map frame')
        frame = fields.pop()
    w, h, res, ox, oy, revision, stamp = fields
    w, h = int(w), int(h)
    if len(cells) != w * h or set(cells) - {0, 205, 254}:
        raise ValueError('invalid map cells')
    value = 14695981039346656037
    for byte in metadata + cells:
        value = ((value ^ byte) * 1099511628211) & ((1 << 64) - 1)
    if value != checksum:
        raise ValueError('map checksum mismatch')
    return dict(width=w, height=h, resolution=float(res), origin=[float(ox), float(oy)],
                revision=int(revision), stamp_us=int(stamp), cells=cells, frame=frame)


def png(grid):
    def chunk(kind, payload):
        return struct.pack('!I', len(payload)) + kind + payload + struct.pack('!I', zlib.crc32(kind + payload))
    w, h, cells = grid['width'], grid['height'], grid['cells']
    rows = b''.join(b'\0' + cells[y*w:(y+1)*w] for y in reversed(range(h)))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', w, h, 8, 0, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))


class Host:
    def __init__(self, binary, directory, label):
        self.log = (directory / (label + '-stderr.log')).open('w')
        self.requests = gzip.open(directory / (label + '-input.txt.gz'), 'wt')
        self.events = (directory / (label + '-events.jsonl')).open('w')
        self.process = subprocess.Popen([str(binary), str(directory)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=self.log, text=True, bufsize=1)
        try:
            if self.receive() != 'READY roboframe-v1':
                raise RuntimeError('mapping host not ready')
        except Exception:
            self.close()
            raise

    def receive(self):
        if not select.select([self.process.stdout], [], [], 30)[0]:
            raise TimeoutError('mapping host response timeout')
        response = self.process.stdout.readline()
        if not response:
            raise RuntimeError('mapping host terminated; inspect stderr log')
        return response.rstrip('\n')

    def send(self, command, record=None):
        self.requests.write(command + '\n')
        self.process.stdin.write(command + '\n')
        self.process.stdin.flush()
        response = self.receive()
        self.events.write(json.dumps(dict(command=command.split()[0], response=response,
            source_line=record['line'] if record else None,
            source_stamp_us=record['stamp'] if record else None)) + '\n')
        if response.startswith('ERROR'):
            raise RuntimeError(response)
        return response

    def status(self):
        values = self.send('STATUS').split()
        keys = ['session', 'received', 'integrated', 'ignored', 'dropped', 'pending',
                'used_rays', 'outside_rays', 'map_revision', 'drop_sync', 'drop_queue', 'drop_bounds', 'drop_cancel']
        if len(values) != 15 or values[0] != 'STATUS':
            raise RuntimeError('malformed status')
        return dict(phase=values[1], **dict(zip(keys, map(int, values[2:]))))

    def resources(self):
        values = self.send('RESOURCES').split()
        if values[0] != 'RESOURCES':
            raise RuntimeError('malformed resource response')
        return {key: int(value) for key, value in (field.split('=') for field in values[1:])}

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=5)
        self.process.stdin.close()
        self.process.stdout.close()
        self.log.close()
        self.requests.close()
        self.events.close()


def run(args):
    dataset = DATASETS[args.dataset]
    records, backwards = load(args.data, args.dataset)
    all_scans = [r for r in records if r['kind'] == 'scan']
    use_tf = args.dataset in ('tf', 'corrected')
    if use_tf:
        if args.dataset == 'corrected':
            corrected = all_scans
            raw, backwards = load(DATA, 'raw')
        else:
            raw = records
            corrected, _ = load(DATASETS['corrected']['path'], 'corrected')
        all_scans = pair_references([r for r in raw if r['kind'] == 'scan'], corrected)
        if args.dataset == 'corrected':
            all_scans = [r for r in all_scans if 'map_reference' in r]

    count = min(3000, len(all_scans)) if args.scans is None else (len(all_scans) if args.scans == 0 else args.scans)
    if count < 2 or count > len(all_scans):
        raise ValueError(f'scans must be 2..{len(all_scans)}, or 0 for all')
    finish = args.finish_after if args.finish_after is not None else count * 4 // 5
    if not 1 <= finish < count:
        raise ValueError('finish-after must be before the last selected scan')
    gap_us = args.max_odom_gap_ms * 1000
    if not 1 <= args.max_odom_gap_ms <= 10000:
        raise ValueError('max-odom-gap-ms must be 1..10000')
    selected = all_scans[:count]
    records = []
    for scan in selected:
        records.append(dict(kind='odom', stamp=scan['stamp'], pose=scan['pose'], line=scan['line']))
        records.append(scan)
    odometry = [r for r in records if r['kind'] == 'odom']
    if not odometry:
        raise ValueError('no odometry')
    start_us = records[0]['stamp']
    bounds = [s.get('display_pose', s['pose']) for s in selected]
    bounds = [p for p in bounds if p is not None]
    if not bounds:
        raise ValueError('selected prefix has no map correction; select more scans')
    ox = math.floor(min(p[0] for p in bounds) - 21)
    oy = math.floor(min(p[1] for p in bounds) - 21)
    width = math.ceil((max(p[0] for p in bounds) + 21 - ox) / 0.1)
    height = math.ceil((max(p[1] for p in bounds) + 21 - oy) / 0.1)
    config = f'CONFIG {width} {height} 0.1 {ox} {oy} {gap_us}'
    if use_tf:
        config += ' MAP_TF 20000000'
    args.output.mkdir(parents=True, exist_ok=False)
    summary_path = args.output / 'summary.json'
    summary_path.write_text(json.dumps(dict(passed=False, reason='run incomplete')))
    host = Host(args.binary.resolve(), args.output.resolve(), 'mapping')
    frames, trajectory, resource_samples, tf_trace = [], [], [], []
    scan_number, finished = 0, False
    checkpoints = max(1, finish // 12)

    def frame(name, number):
        grid = read_map(args.output / (name + '.rfgrid'))
        picture = png(grid)
        (args.output / (name + '.png')).write_bytes(picture)
        frames.append(dict(after_scan=number, revision=grid['revision'],
            stamp_us=grid['stamp_us'], known=sum(c != 205 for c in grid['cells']),
            image='data:image/png;base64,' + base64.b64encode(picture).decode()))
        resource_samples.append(dict(after_scan=number, **host.resources()))
        return grid

    def finish_mapping():
        host.send('FINISH')
        status = host.status()
        host.send('SAVE final')
        grid = frame('final', scan_number)
        print(f"FINISH: {status['integrated']} mapped; {status['dropped']} dropped; final map saved", flush=True)
        return status, grid

    try:
        host.send(config)
        host.send('BEGIN 1')
        print(f'MODULES READY; BEGIN; replay {count} scans, finish after {finish}', flush=True)
        for record in records:
            t = record['stamp'] - start_us
            if record['kind'] == 'odom':
                host.send('ODOM ' + str(t) + ' ' + ' '.join(format(v, '.17g') for v in record['pose']), record)
                continue
            if scan_number == finish and not finished:
                before, final_grid = finish_mapping()
                finished = True
            if 'map_reference' in record:
                host.send('MAP_POSE ' + str(t) + ' ' + ' '.join(format(v, '.17g') for v in record['map_reference']), record)
            host.send('SCAN ' + str(t) + ' ' + str(len(record['hits'])) + ' ' +
                      ' '.join(format(v, '.12g') for point in record['hits'] for v in point), record)
            scan_number += 1
            displayed = record.get('display_pose', record['pose'])
            trajectory.append([t, displayed[0] if displayed else None, displayed[1] if displayed else None])
            if use_tf and ('map_reference' in record or scan_number % 100 == 0):
                response = host.send('TF').split()
                if displayed is None:
                    if response != ['TF', 'unavailable']:
                        raise RuntimeError('TF used before first correction')
                else:
                    if len(response) != 12 or int(response[1]) != t or int(response[2]) > t:
                        raise RuntimeError('TF time/direction response mismatch')
                    if list(map(float, response[3:6])) != record['pose']:
                        raise RuntimeError('raw odometry changed inside TF chain')
                    actual = list(map(float, response[-3:]))
                    expected = record.get('map_reference', displayed)
                    error = math.hypot(actual[0]-expected[0], actual[1]-expected[1])
                    yaw_error = abs(math.atan2(math.sin(actual[2]-expected[2]), math.cos(actual[2]-expected[2])))
                    if error > 1e-7 or yaw_error > 1e-7:
                        raise RuntimeError('composed TF disagrees with paired map pose')
                    tf_trace.append(dict(scan=scan_number, stamp_us=t, raw_stamp_us=record['stamp'],
                        reference_source_stamp_us=record.get('reference_source_stamp'), correction_stamp_us=int(response[2]),
                        odom_from_base=list(map(float, response[3:6])), map_from_odom=list(map(float, response[6:9])),
                        map_from_base=actual, paired_reference='map_reference' in record,
                        error_m=error, yaw_error_rad=yaw_error))
            if not finished and scan_number < finish and scan_number % checkpoints == 0:
                if host.status()['integrated']:
                    name = f'progress-{scan_number:05d}'
                    host.send('SNAPSHOT ' + name)
                    frame(name, scan_number)
            if finished and scan_number < count and scan_number % checkpoints == 0:
                name = f'live-{scan_number:05d}'
                host.send('SNAPSHOT ' + name)
                frame(name, scan_number)
            if scan_number % 1000 == 0:
                print(f'IPC scan {scan_number}/{count}', flush=True)
        if not finished:
            before, final_grid = finish_mapping()
        after = host.status()
        host.send('SAVE frozen')
        host.send('SNAPSHOT live')
        live_grid = frame('live', scan_number)
        host.send('WAIT_PLANNER')
        resources = host.resources()
        resource_samples.append(dict(after_scan=scan_number, **resources))
        original = (args.output / 'final.rfgrid').read_bytes()
        if original != (args.output / 'frozen.rfgrid').read_bytes():
            raise RuntimeError('map changed after FINISH')
        if (before['integrated'] != after['integrated'] or after['received'] != finish or
                after['integrated'] + after['dropped'] != finish or after['pending'] != 0 or
                after['ignored'] != 0 or after['used_rays'] == 0):
            raise RuntimeError('workflow accounting mismatch')
        if (resources['live_updates'] != count - finish or resources['live_dropped'] or live_grid['revision'] != count - after['dropped'] or
                resources['planner_inconsistent'] or resources['planner_peak_retained'] > 2 or
                resources['planner_pending'] or resources['planner_active'] or
                resources['planner_revision'] != live_grid['revision'] or
                resources['planner_completed'] + resources['planner_replaced'] != resources['planner_received']):
            raise RuntimeError('shared-resource or continued-update check failed')
        host.send('QUIT')
        if host.process.wait(timeout=5) != 0:
            raise RuntimeError('mapping host failed')
    finally:
        host.close()

    reload_host = Host(args.binary.resolve(), args.output.resolve(), 'reload')
    try:
        reload_host.send(config)
        reload_host.send('LOAD final')
        reload_host.send('SAVE reloaded')
        reload_host.send('QUIT')
        if reload_host.process.wait(timeout=5) != 0:
            raise RuntimeError('reload host failed')
    finally:
        reload_host.close()
    if original != (args.output / 'reloaded.rfgrid').read_bytes():
        raise RuntimeError('map did not survive process restart')
    summary = dict(passed=True, dataset=args.dataset, dataset_label=dataset['label'],
                   dataset_sha256=dataset['sha256'], reference_frame='map' if use_tf else dataset['frame'],
                   corrected_sha256=DATASETS['corrected']['sha256'] if use_tf else None,
                   raw_sha256=SHA256 if use_tf else None,
                   tf_checks=len(tf_trace), reference_checks=sum(f['paired_reference'] for f in tf_trace),
                   max_tf_error_m=max((f['error_m'] for f in tf_trace), default=0),
                   map_frame=live_grid['frame'],
                   total_dataset_scans=len(all_scans),
                   replayed_scans=count, finish_after=finish, replayed_odom=len(odometry),
                   pose_source=dataset['pose_source'],
                   max_odom_gap_ms=args.max_odom_gap_ms, reordered_records=backwards, before=before, after=after,
                   saved_map_sha256=hashlib.sha256(original).hexdigest(),
                   frozen_equal=True, reloaded_equal=True, resources=resources,
                   live_revision=live_grid['revision'],
                   changed_cells_after_finish=sum(a != b for a, b in zip(final_grid['cells'], live_grid['cells'])),
                   full_snapshot_equivalent_bytes=resources['publications'] * resources['map_bytes'],
                   known_cells=sum(c != 205 for c in final_grid['cells']),
                   occupied_cells=final_grid['cells'].count(0),
                   filtered_ranges=sum(r['filtered'] for r in records if r['kind'] == 'scan'),
                   width=width, height=height, resolution=0.1, origin=[ox, oy])
    if use_tf:
        (args.output / 'tf-trace.json').write_text(json.dumps(tf_trace, indent=2) + '\n')
        expected_refs = sum('map_reference' in scan for scan in selected)
        if resources['correction_outputs'] != expected_refs or resources['missed_references']:
            raise RuntimeError('recorded correction not consumed by lidar-map')
    report = dict(summary=summary, frames=frames, trajectory=trajectory, resource_samples=resource_samples)
    html = Path(__file__).with_name('replay.html').read_text()
    (args.output / 'replay.html').write_text(html.replace('__REPORT__', json.dumps(report).replace('<', '\\u003c')))
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    print('PASS: baseline preserved, live updates, bounded shared planner reads, save/reload', flush=True)
    print(json.dumps(resources, indent=2), flush=True)
    print(args.output.resolve() / 'replay.html')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', choices=tuple(DATASETS), default='raw',
                        help='raw: local odom; corrected: matched keyframes; tf: all raw scans + sparse map correction')
    parser.add_argument('--data', type=Path, help='override file path; selected dataset checksum still enforced')
    parser.add_argument('--binary', type=Path, default=ROOT / 'build-product/robot_runtime')
    parser.add_argument('--output', type=Path, default=ROOT / 'build-product/result')
    parser.add_argument('--scans', type=int, help='default: up to 3000; 0 = full dataset')
    parser.add_argument('--finish-after', type=int)
    parser.add_argument('--max-odom-gap-ms', type=int, default=250)
    args = parser.parse_args()
    args.data = args.data or DATASETS[args.dataset]['path']
    if not args.data.is_file():
        parser.error('dataset file missing; see data/intel-lab/README.md')
    if not args.binary.is_file():
        parser.error('build robot_runtime first')
    if args.output.exists():
        parser.error('output already exists; use a new --output directory to preserve previous maps')
    run(args)


if __name__ == '__main__':
    main()
