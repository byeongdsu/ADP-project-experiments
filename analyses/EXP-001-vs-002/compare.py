"""Compare first GT/PF traversals using the same GT-to-polyline metric."""
from pathlib import Path
import json,sqlite3,csv,math
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
# Start/end from scorer logs; successful runs use only the first credited lap.
RUNS=[('GT-0.75','EXP-001','R06-retry5',1792486590.752354707,1792486599.332275630),
      ('PF-0.75','EXP-002','R06',1792494337.138501991,1792494345.709719028),
      ('GT-0.85','EXP-001','R07-retry3',1792486809.055456815,1792486816.755263259),
      ('PF-0.85','EXP-002','R07',1792494439.501581824,1792494447.200582263)]


def read_topic(db,name):
    tid,typ=db.execute('SELECT id,type FROM topics WHERE name=?',(name,)).fetchone()
    cls=get_message(typ)
    return [(t/1e9,deserialize_message(b,cls)) for t,b in db.execute(
        'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(tid,))]


def main():
    all_bins={};initial={};raw={};reference=None
    for label,exp,run,start,end in RUNS:
        path=ROOT/'runs'/exp/run/'telemetry/telemetry_0.db3'
        db=sqlite3.connect(f'file:{path}?mode=ro',uri=True)
        gt=read_topic(db,'/car_state/odom_GT')
        w=read_topic(db,'/global_waypoints')[0][1].wpnts
        xy=np.array([(p.x_m,p.y_m) for p in w]);s=np.array([p.s_m for p in w])
        if reference is None:reference=xy.copy()
        assert np.array_equal(reference,xy),'Reference geometry differs'
        a=xy[:-1];v=xy[1:]-a;v2=np.sum(v*v,axis=1)
        valid=v2>1e-12;a=a[valid];v=v[valid];v2=v2[valid];s0=s[:-1][valid];ds=np.diff(s)[valid]
        m=gt[0][1];p=m.pose.pose;q=p.orientation
        initial[label]=dict(x=p.position.x,y=p.position.y,
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),vx=m.twist.twist.linear.x)
        bins={};rows=[]
        for t,m in gt:
            if not start<=t<end or m.twist.twist.linear.x<.1:continue
            pos=np.array([m.pose.pose.position.x,m.pose.pose.position.y]);offset=pos-a
            u=np.clip(np.sum(offset*v,axis=1)/v2,0,1);delta=offset-u[:,None]*v
            dist2=np.sum(delta*delta,axis=1);i=int(np.argmin(dist2));d=float(math.sqrt(dist2[i]))
            cross=v[i,0]*delta[i,1]-v[i,1]*delta[i,0];signed=math.copysign(d,cross)
            progress=float(s0[i]+u[i]*ds[i]);speed=m.twist.twist.linear.x
            rows.append([t-start,progress,signed,speed]);bins.setdefault(int(progress/.5),[]).append([signed,speed])
        raw[label]=dict(window_start=start,window_end=end,samples=len(rows))
        all_bins[label]=bins
        np.savetxt(OUT/f'{label}_trace.csv',rows,delimiter=',',header='seconds_from_timing_start,s_m,gt_signed_polyline_distance_m,gt_vx_mps',comments='')
    poses=list(initial.values());assert all(p==poses[0] for p in poses),'Initial poses differ'
    common=sorted(k for k in set.intersection(*(set(b) for b in all_bins.values())) if .5<=k*.5<41.5)
    assert len(common)==82,f'Unexpected common coverage: {len(common)} bins'
    result=dict(method='First traversal, GT-to-global-polyline distance, equal 0.5m spatial bins; 0.5<=s<41.5',
                shared_bins=len(common),initial_pose=initial,reference_geometry_equal=True,runs={})
    fig,axs=plt.subplots(2,1,figsize=(11,7),sharex=True)
    styles=[('tab:blue','-'),('tab:blue','--'),('tab:orange','-'),('tab:orange','--')]
    for (label,bins),(color,style) in zip(all_bins.items(),styles):
        result['runs'][label]=dict(**raw[label],
            spatial_mean_abs_gt_tracking_m=float(np.mean([np.mean(np.abs(np.array(bins[k])[:,0])) for k in common])),
            spatial_mean_gt_vx_mps=float(np.mean([np.mean(np.array(bins[k])[:,1]) for k in common])))
        x=[k*.5+.25 for k in common]
        axs[0].plot(x,[np.mean(np.array(bins[k])[:,0]) for k in common],style,color=color,label=label)
        axs[1].plot(x,[np.mean(np.array(bins[k])[:,1]) for k in common],style,color=color,label=label)
    for ax in axs:ax.grid(alpha=.3)
    axs[0].set_ylabel('GT signed tracking distance [m]');axs[0].legend(ncol=2)
    axs[1].set_ylabel('GT speed [m/s]');axs[1].set_xlabel('Global racing-line progress s [m]')
    fig.tight_layout();fig.savefig(OUT/'gt_pf_spatial_comparison.png',dpi=160)
    (OUT/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
