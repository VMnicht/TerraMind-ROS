#!/usr/bin/env python3
"""从真实故障注入记录绘图；不连接 ROS、不控制车辆。"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    directory=parser.parse_args().directory
    meta=json.loads((directory/'metadata.json').read_text())
    assert meta['source']=='real_vehicle_fault_injection'
    plan=json.loads((directory/'plan.json').read_text())
    records=json.loads((directory/'telemetry.json').read_text())
    origin=plan[0]['start'];c=math.cos(origin['yaw']);s=math.sin(origin['yaw'])
    def local(x,y):
        dx=x-origin['x'];dy=y-origin['y'];return dx*c+dy*s,-dx*s+dy*c
    points=[];errors=[];times=[];indices=[];initial=records[0]['t']
    for record in records:
        if record['pose'] is None:continue
        x,y,yaw=record['pose'];points.append(local(x,y));index=record['index']
        if index is None or index>=len(plan):continue
        seg=plan[index]
        if seg['kind']=='line' and seg['mowing']:
            a=seg['start'];errors.append(-(x-a['x'])*math.sin(a['yaw'])+(y-a['y'])*math.cos(a['yaw']))
            times.append(record['t']-initial);indices.append(index)
    metrics=dict(source='real_vehicle_fault_injection',maximum_coverage_cross_track_m=max(map(abs,errors)),
                 p95_coverage_cross_track_m=float(np.percentile(np.abs(errors),95)),
                 duration_s=records[-1]['t']-initial)
    (directory/'metrics.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2))
    fig,(ax,err)=plt.subplots(1,2,figsize=(12,5))
    ax.plot([.2,2.2,2.2,.2,.2],[-.135,-.135,1.865,1.865,-.135],color='#8a969e',label='2 x 2 m area')
    first=True
    for seg in plan:
        if seg['kind']=='rotate':continue
        a=local(seg['start']['x'],seg['start']['y']);b=local(seg['end']['x'],seg['end']['y'])
        ax.plot([a[0],b[0]],[a[1],b[1]],'--',color='#168b83',lw=1,label='Planned route' if first else None);first=False
    points=np.asarray(points);ax.plot(points[:,0],points[:,1],color='#cf3f42',lw=1.5,label='Real measured trajectory')
    ax.scatter([0],[0],color='black',s=25);ax.set_aspect('equal');ax.legend(fontsize=8)
    ax.set_xlabel('Forward at capture (m)');ax.set_ylabel('Left at capture (m)');ax.grid(alpha=.25)
    for index in sorted(set(indices)):
        selected=np.asarray(indices)==index
        err.plot(np.asarray(times)[selected],np.asarray(errors)[selected]*100,color='#cf3f42',lw=1)
    err.axhline(0,color='#168b83',ls='--');err.set_xlabel('Time (s)');err.set_ylabel('Coverage cross-track error (cm)');err.grid(alpha=.25)
    fig.suptitle('Real vehicle fault-injection DEMO — blade OFF\nIntentional reference bias; NOT normal tracking performance')
    fig.tight_layout();fig.savefig(directory/'trajectory-real.png',dpi=160)


if __name__=='__main__':main()
