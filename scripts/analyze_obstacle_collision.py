"""Inspect collision timeline using latest preceding bag samples; no replay or writes to ROS."""
import argparse
import bisect
import json
import math
import sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    args=p.parse_args()
    db=sqlite3.connect(f'file:{args.run/"telemetry/telemetry_0.db3"}?mode=ro',uri=True)
    names=['/drive','/state','/car_state/odom_GT','/pf/viz/inferred_pose',
           '/car_state/frenet/odom','/local_waypoints','/sim/obstacles',
           '/sim/collision','/perception/obstacles','/planner/avoidance/otwpnts']
    data={}
    for i,n,typ in db.execute('SELECT id,name,type FROM topics'):
        if n in names:
            cls=get_message(typ)
            data[n]=[(t/1e9,deserialize_message(b,cls)) for t,b in db.execute(
                'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(i,))]
    times={n:[t for t,m in rows] for n,rows in data.items()}
    def at(n,t):
        rows=data.get(n,[])
        i=bisect.bisect_right(times.get(n,[]),t)-1
        return rows[i] if i>=0 else (None,None)
    hits=[];prev='none'
    for t,m in data['/sim/collision']:
        if m.data in ('obstacle','wall') and prev=='none':hits.append({'time_s':t,'kind':m.data})
        prev=m.data
    changes=[];prev=None
    for t,m in data['/state']:
        if m.data!=prev:changes.append([t,m.data]);prev=m.data
    layouts=[];prev=None
    for t,m in data['/sim/obstacles']:
        layout=[[z.position.x,z.position.y,yaw(z.orientation)] for z in m.poses]
        if layout!=prev:layouts.append({'time_s':t,'xy_yaw':layout});prev=layout
    out={'alignment':'latest preceding bag receive time; asynchronous inputs, not internal controller capture',
         'collisions':hits,'state_changes':changes,'layout_changes':layouts,'snapshots':[]}
    for hit in hits[:1]:
        for dt in [-3,-2,-1,-.5,-.2,-.05,0]:
            t=hit['time_s']+dt;s={'relative_to_collision_s':dt,'time_s':t,'sample_age_s':{}}
            msgs={}
            for n in names:
                ts,m=at(n,t);msgs[n]=m
                if ts is not None:s['sample_age_s'][n]=t-ts
            g=msgs['/car_state/odom_GT'];pf=msgs['/pf/viz/inferred_pose'];gp=g.pose.pose.position
            s['gt']={'xy':[gp.x,gp.y],'yaw':yaw(g.pose.pose.orientation),'vx':g.twist.twist.linear.x}
            if pf:
                pp=pf.pose.position
                s['pf']={'xy':[pp.x,pp.y],'yaw':yaw(pf.pose.orientation),'position_error_m':math.hypot(pp.x-gp.x,pp.y-gp.y),
                         'yaw_error_deg':math.degrees(math.atan2(math.sin(yaw(pf.pose.orientation)-s['gt']['yaw']),math.cos(yaw(pf.pose.orientation)-s['gt']['yaw'])))}
            state=msgs['/state'];s['state']=state.data if state else None
            drive=msgs['/drive'];s['drive']={'speed':drive.drive.speed,'steer':drive.drive.steering_angle} if drive else None
            fr=msgs['/car_state/frenet/odom'];s['frenet']={'s':fr.pose.pose.position.x,'d':fr.pose.pose.position.y} if fr else None
            for n,key in [('/local_waypoints','local_path'),('/planner/avoidance/otwpnts','avoidance')]:
                m=msgs[n];ws=m.wpnts if m else []
                s[key]={'points':len(ws)}
                if ws:
                    w=min(ws,key=lambda w:(w.x_m-gp.x)**2+(w.y_m-gp.y)**2)
                    s[key].update(nearest_gt_distance_m=math.hypot(w.x_m-gp.x,w.y_m-gp.y),nearest_gt_wp={'s':w.s_m,'d':w.d_m,'v':w.vx_mps})
            ob=msgs['/perception/obstacles'];s['perceived_obstacles']=[{'id':z.id,'s':z.s_center,'d':z.d_center,'size':z.size,'vs':z.vs,'visible':z.is_visible} for z in ob.obstacles] if ob else []
            ob=msgs['/sim/obstacles'];s['actual_obstacles']=[{'xy':[z.position.x,z.position.y],'center_distance_gt_m':math.hypot(z.position.x-gp.x,z.position.y-gp.y)} for z in ob.poses] if ob else []
            out['snapshots'].append(s)
    (args.run/'collision_diagnostics.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out,indent=2))


if __name__=='__main__':main()
