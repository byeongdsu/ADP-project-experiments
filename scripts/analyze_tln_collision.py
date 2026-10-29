"""Offline TLN collision inspection using bag receipt times."""
import argparse
import bisect
import json
import math
import sqlite3
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    run = parser.parse_args().run
    names = ['/drive', '/car_state/odom_GT', '/sim/obstacles', '/sim/collision', '/scan', '/parameter_events']
    data = {}
    with sqlite3.connect(f'file:{run / "telemetry/telemetry_0.db3"}?mode=ro', uri=True) as db:
        for ident, name, typ in db.execute('select id,name,type from topics'):
            if name in names:
                cls = get_message(typ)
                data[name] = [(t/1e9, deserialize_message(b, cls)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (ident,))]
    times = {n: [t for t,m in rows] for n,rows in data.items()}
    def at(n,t):
        i = bisect.bisect_right(times.get(n, []), t)-1
        return data[n][i] if i >= 0 else (None,None)
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    hits = []
    prev = 'none'
    for t,m in data['/sim/collision']:
        if m.data in ('wall','obstacle') and prev == 'none':
            hits.append((t,m.data))
        prev = m.data
    layouts = []
    for t,m in data['/sim/obstacles']:
        poses = [[p.position.x,p.position.y,yaw(p.orientation)] for p in m.poses]
        if not layouts or poses != layouts[-1]['poses']:
            layouts.append({'time':t,'poses':poses})
    params = []
    for t,m in data.get('/parameter_events', []):
        if m.node.endswith('/tln_inference'):
            params.append({'time':t,'node':m.node,'values':{p.name: {'type':p.value.type,'double':p.value.double_value,'integer':p.value.integer_value,'string':p.value.string_value,'bool':p.value.bool_value} for p in list(m.new_parameters)+list(m.changed_parameters)}})
    out = {'alignment':'latest preceding bag receipt timestamps; asynchronous, not internal callback times', 'layouts':layouts, 'runtime_parameter_events':params, 'collisions':hits, 'snapshots':[]}
    for hit,kind in hits[:1]:
        for dt in [-3,-2,-1,-.5,-.2,-.05,0]:
            t = hit+dt
            ages = {}
            msgs = {}
            for n in names[:5]:
                ts,m = at(n,t); msgs[n] = m
                ages[n] = t-ts if ts else None
            g = msgs['/car_state/odom_GT']; d = msgs['/drive']; ob = msgs['/sim/obstacles']; scan = msgs['/scan']
            p = g.pose.pose.position
            vals = [r for i,r in enumerate(scan.ranges) if abs(scan.angle_min+i*scan.angle_increment)<.15 and math.isfinite(r)]
            out['snapshots'].append({'dt':dt,'xy':[p.x,p.y],'yaw_deg':math.degrees(yaw(g.pose.pose.orientation)), 'actual_vx':g.twist.twist.linear.x, 'command_speed':d.drive.speed,'steer':d.drive.steering_angle,'obstacle_center_distances':[math.hypot(z.position.x-p.x,z.position.y-p.y) for z in ob.poses], 'front_scan_min':min(vals) if vals else None,'sample_ages':ages})
        fig,axes = plt.subplots(1,2,figsize=(11,5))
        gt = [(t,m) for t,m in data['/car_state/odom_GT'] if hit-5 <= t <= hit]
        axes[0].plot([m.pose.pose.position.x for t,m in gt],[m.pose.pose.position.y for t,m in gt],label='GT last 5 s')
        _,ob = at('/sim/obstacles',hit)
        for p in ob.poses:
            a=yaw(p.orientation); corners=[(-.15,-.15),(.15,-.15),(.15,.15),(-.15,.15),(-.15,-.15)]
            axes[0].plot([p.position.x+x*math.cos(a)-y*math.sin(a) for x,y in corners],[p.position.y+x*math.sin(a)+y*math.cos(a) for x,y in corners],color='orange')
        axes[0].set_aspect('equal',adjustable='datalim'); axes[0].legend();axes[0].set_title('Obstacle squares assume 0.30 m size')
        drive=[(t,m) for t,m in data['/drive'] if hit-5<=t<=hit]
        axes[1].plot([t-hit for t,m in gt],[m.twist.twist.linear.x for t,m in gt],label='GT vx (m/s)')
        axes[1].plot([t-hit for t,m in drive],[m.drive.speed for t,m in drive],label='command (m/s)')
        ax=axes[1].twinx();ax.plot([t-hit for t,m in drive],[m.drive.steering_angle for t,m in drive],color='green',alpha=.6);ax.set_ylabel('steering (rad)')
        axes[1].set_xlabel('seconds relative to collision');axes[1].legend();fig.tight_layout();fig.savefig(run/'tln_collision.png',dpi=150);plt.close(fig)
    (run/'tln_collision.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({'snapshots':out['snapshots'],'last_layout':layouts[-1] if layouts else None,'parameter_event_count':len(params)},indent=2))


if __name__ == '__main__':
    main()
