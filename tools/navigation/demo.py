#!/usr/bin/env python3
"""Plan on Intel's reference map and publish unconsumed velocity using an explicit stationary TF fixture."""
import argparse
import base64
import json
import math
from pathlib import Path
import shutil
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tests'))
import intel_reference_check
import replay

def run(binary,map_path,output):
    reference=intel_reference_check.check(map_path)
    grid=replay.read_map(map_path)
    output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(map_path,output/'reference.rfgrid')
    start=reference['start'];goal=reference['goal']
    host=replay.Host(binary.resolve(),output.resolve(),'navigation')
    stamp=0
    yaw=0.0
    def locate():
        nonlocal stamp
        stamp+=100000
        # Deliberately non-identity odom -> map. No velocity is integrated into this fixed pose.
        host.send(f'ODOM {stamp} 1 2 0.3')
        host.send(f'MAP_POSE {stamp} {start[0]} {start[1]} {yaw}')
        host.send(f'SCAN {stamp} 1 2 0')
        host.wait_idle()
    def status():
        fields=host.send('NAV_STATUS').split()
        assert len(fields)==5 and fields[0]=='NAV'
        return fields[1]
    def wait_phase(wanted,refresh=True):
        deadline=time.monotonic()+5
        while True:
            if refresh: locate()
            phase=status()
            if phase==wanted: return
            assert phase not in ('failed','succeeded','canceled'),phase
            assert time.monotonic()<deadline,'navigation phase timeout'
            time.sleep(0.02)
    try:
        w,h=grid['width'],grid['height']
        res=grid['resolution'];ox,oy=grid['origin']
        host.send(f'CONFIG {w} {h} {res} {ox} {oy} 250000 MAP_TF 20000000')
        host.send('LOAD reference');host.send('RUN');locate()
        host.send(f'NAVIGATE 1 {goal[0]} {goal[1]} 0')
        wait_phase('following')
        fields=host.send('GET_PATH').split()
        assert fields[0]=='PATH' and fields[1]=='1' and fields[4]=='map'
        count=int(fields[5]);assert count>2 and len(fields)==6+count*3
        path=[list(map(float,fields[i:i+3])) for i in range(6,len(fields),3)]
        assert math.dist(path[0][:2],start)<1e-8 and math.dist(path[-1][:2],goal)<1e-8
        margin=reference['conservative_margin_cells']
        for pose in path:
            x=math.floor((pose[0]-ox)/res);y=math.floor((pose[1]-oy)/res)
            assert margin<=x<w-margin and margin<=y<h-margin
            assert all(grid['cells'][(y+dy)*w+x+dx]==254
                       for dy in range(-margin,margin+1) for dx in range(-margin,margin+1))
        following=next(p for p in path if math.dist(p[:2],start)>0.025)
        yaw=math.atan2(following[1]-start[1],following[0]-start[0])
        for _ in range(8):
            locate();time.sleep(0.06)
        assert status()=='following','stationary robot must not report success'
        host.send('NAV_CANCEL');wait_phase('canceled',refresh=False)
        locate();host.send(f'NAVIGATE 2 {goal[0]} {goal[1]} 0');wait_phase('following')
        # Stop feeding odometry. The receive-time watchdog must stop output even with valid stored TF.
        wait_phase('failed',refresh=False)
        assert host.send('NAV_STATUS').endswith('tf_unavailable')
        host.send('QUIT');assert host.process.wait(timeout=5)==0
    finally:
        host.close()
    messages=[json.loads(line)['message'].split() for line in (output/'navigation-events.jsonl').read_text().splitlines()]
    velocity=[m for m in messages if len(m)>2 and m[2]=='CMD_VEL']
    assert velocity and any(float(m[7])>0 for m in velocity)
    assert all(abs(float(m[7]))<=0.3 and abs(float(m[8]))<=1.0 for m in velocity)
    for session in ('1','2'):
        commands=[m for m in velocity if m[3]==session]
        assert commands and float(commands[-1][7])==0 and float(commands[-1][8])==0
    assert not any(m[2:4]==['NAV','succeeded'] for m in messages)
    result=dict(passed=True,map_source='Intel corrected-log / 910 matched scans',
                localization_source='stationary explicit test fixture with non-identity map/odom TF',
                motion_closed_loop=False,robot_radius_m=0.2,unknown_is_blocked=True,
                map_revision=grid['revision'],path_points=count,
                path_length_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:])),
                velocity_messages=len(velocity),cancel_zero=True,stale_tf_zero=True,start=start,goal=goal,path=path)
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    # Static diagnostic overlay, not a simulated moving robot.
    image=base64.b64encode(replay.png(grid)).decode()
    points=' '.join(f'{(p[0]-ox)/res},{h-(p[1]-oy)/res}' for p in path)
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}"><image width="{w}" height="{h}" href="data:image/png;base64,{image}"/><polyline points="{points}" fill="none" stroke="#08a4ff" stroke-width="2"/></svg>'
    (output/'path.svg').write_text(svg)
    print(json.dumps({k:v for k,v in result.items() if k!='path'},indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,default=ROOT/'build-nav/robot_runtime')
    parser.add_argument('--map',type=Path,required=True,help='live.rfgrid from a full corrected replay')
    parser.add_argument('--output',type=Path,required=True,help='new output directory')
    args=parser.parse_args();run(args.binary,args.map,args.output)
