"""Spatially align first traversals using recorded topics; source ROS first."""
from pathlib import Path
import sqlite3
import json
import csv
import math
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
RUNS=[('R06-retry5',.75,1792486590.752354707,1792486599.332275630),
      ('R07-retry3',.85,1792486809.055456815,1792486816.755263259),
      ('R08',.95,1792486958.765860334,1792486962.354154119),
      ('R09',1.,1792487126.295871014,1792487129.765204723)]
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
all_rows={}; summary={}
for run,scale,start,end in RUNS:
    db=sqlite3.connect(f'file:{ROOT / "runs/EXP-001" / run / "telemetry/telemetry_0.db3"}?mode=ro',uri=True)
    data={}
    for tid,name,typ in db.execute('SELECT id,name,type FROM topics'):
        cls=get_message(typ)
        data[name]=[(t/1e9,deserialize_message(b,cls)) for t,b in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(tid,))]
    times={n:np.array([t for t,m in rows]) for n,rows in data.items()}
    def latest(name,t):
        i=int(np.searchsorted(times[name],t,side='right'))-1
        if i<0 or t-times[name][i]>.12:return None
        return data[name][i][1]
    global_w=data['/global_waypoints'][0][1].wpnts
    reference={w.id:w for w in global_w}
    rows=[]
    for t,fr in data['/car_state/frenet/odom']:
        if not start<=t<end:continue
        gt=latest('/car_state/odom_GT',t);odom=latest('/car_state/odom',t)
        cmd=latest('/drive',t);local=latest('/local_waypoints',t)
        if any(x is None for x in [gt,odom,cmd,local]) or not local.wpnts:continue
        x,y=gt.pose.pose.position.x,gt.pose.pose.position.y
        if gt.twist.twist.linear.x<.1:continue
        wp=local.wpnts
        idx=min(range(len(wp)),key=lambda i:(wp[i].x_m-x)**2+(wp[i].y_m-y)**2)
        w=wp[idx];d=fr.pose.pose.position.y;s=fr.pose.pose.position.x
        heading=yaw(gt.pose.pose.orientation)
        # Estimate only: parameters read from installed NUC2 YAML, no runtime dump.
        l1=float(np.clip(.04+.575*odom.twist.twist.linear.x,max(.1,math.sqrt(2)*abs(d)),6.))
        target=wp[min(len(wp)-1,idx+int(l1/.1+.5))]
        eta=wrap(math.atan2(target.y_m-y,target.x_m-x)-heading)
        ref=reference[w.id]
        rows.append(dict(run=run,scale=scale,time=t-start,s=s,d=d,
            gt_v=gt.twist.twist.linear.x,controller_v=odom.twist.twist.linear.x,
            command_v=cmd.drive.speed,steer_deg=math.degrees(cmd.drive.steering_angle),
            heading_error_deg=math.degrees(wrap(heading-w.psi_rad)),
            curvature=w.kappa_radpm,path_v=w.vx_mps,l1_est_m=l1,
            target_s_est=target.s_m,eta_est_deg=math.degrees(eta),
            local_geometry_delta_m=math.hypot(w.x_m-ref.x_m,w.y_m-ref.y_m),
            path_speed_ratio=w.vx_mps/ref.vx_mps if ref.vx_mps else 0.,
            heading_speed_scale_est=1.-.3*float(np.clip((abs(wrap(heading-w.psi_rad))-math.pi/9)/(math.pi/2-math.pi/9),0,1))))
    all_rows[run]=rows
    first=data['/car_state/odom_GT'][0][1]
    summary[run]=dict(initial_pose=[first.pose.pose.position.x,first.pose.pose.position.y,yaw(first.pose.pose.orientation)],
        initial_v=first.twist.twist.linear.x,max_local_geometry_delta_m=max(r['local_geometry_delta_m'] for r in rows),
        median_path_speed_ratio=float(np.median([r['path_speed_ratio'] for r in rows])))
    with (OUT/f'{run}_aligned.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)

# Equal spatial bins avoid overweighting slow sections and differing sample rates.
bins={}
for run,rows in all_rows.items():
    bins[run]={}
    for r in rows:
        k=int(r['s']/.5)
        bins[run].setdefault(k,[]).append(r)
common=sorted(set.intersection(*(set(v) for v in bins.values())))
spatial={}
for run in all_rows:
    spatial[run]={}
    for name,keys in [('common_early', [k for k in common if .5<=k*.5<19.5]),
                      ('late_corner_35_41', [k for k in bins[run] if 35<=k*.5<41.5])]:
        if not keys:continue
        spatial[run][name]=dict(bins=len(keys),mean_abs_d_m=float(np.mean([np.mean([abs(r['d']) for r in bins[run][k]]) for k in keys])),
             mean_gt_v=float(np.mean([np.mean([r['gt_v'] for r in bins[run][k]]) for k in keys])))
    spatial[run]['stations']={}
    for s in [4,8,10,12,14,16,18,19,36,38,40,41]:
        a=[r for r in all_rows[run] if s<=r['s']<s+.5]
        if a:
            spatial[run]['stations'][str(s)]={k:float(np.mean([r[k] for r in a])) for k in
                ['d','gt_v','controller_v','command_v','steer_deg','heading_error_deg','curvature','path_v','l1_est_m','eta_est_deg','heading_speed_scale_est']}
(OUT/'comparison.json').write_text(json.dumps(dict(validation=summary,spatial=spatial),indent=2)+'\n')
fig,axs=plt.subplots(5,1,figsize=(12,13),sharex=True)
for (run,scale,_,_),color in zip(RUNS,['tab:blue','tab:orange','tab:green','tab:red']):
    ks=sorted(bins[run]);x=np.array([k*.5+.25 for k in ks])
    for ax,key in zip(axs,['d','gt_v','heading_error_deg','steer_deg','l1_est_m']):
        y=[np.mean([r[key] for r in bins[run][k]]) for k in ks]
        ax.plot(x,y,label=str(scale),color=color)
for ax,label in zip(axs,['Signed tracking error [m]','GT speed [m/s]','Heading error [deg]','Command steer [deg]','Estimated lookahead [m]']):
    ax.set_ylabel(label);ax.grid(alpha=.3);ax.axvspan(16,20,color='gray',alpha=.1)
    ax.axvspan(38,42,color='gray',alpha=.1)
axs[0].legend(title='Speed scale');axs[-1].set_xlabel('Reference path s [m] (first traversal, 0.5 m bins)')
fig.tight_layout();fig.savefig(OUT/'spatial_comparison.png',dpi=160)
print(json.dumps(dict(validation=summary,windows={r:{k:v for k,v in x.items() if k!='stations'} for r,x in spatial.items()}),indent=2))
