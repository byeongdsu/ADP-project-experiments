"""Analyze three completed laps with ROS environment sourced; no replay."""
from pathlib import Path
import sqlite3
import json
import math
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
c = sqlite3.connect(f'file:{root / "telemetry/telemetry_0.db3"}?mode=ro', uri=True)
data = {}
for tid, name, typ in c.execute('SELECT id,name,type FROM topics'):
    data[name] = [(t/1e9, deserialize_message(b, get_message(typ))) for t,b in
                  c.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(tid,))]
edges = [1792486590.752354707,1792486599.332275630,1792486607.812575672,1792486616.295163228]
fr = np.array([(t,m.pose.pose.position.x,m.pose.pose.position.y) for t,m in data['/car_state/frenet/odom']])
gt = np.array([(t,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x) for t,m in data['/car_state/odom_GT']])
drive = np.array([(t,m.drive.speed,m.drive.steering_angle) for t,m in data['/drive']])
def stats(a,b):
    f=fr[(fr[:,0]>=a)&(fr[:,0]<b)]
    g=gt[(gt[:,0]>=a)&(gt[:,0]<b)]
    d=drive[(drive[:,0]>=a)&(drive[:,0]<b)]
    return dict(samples=len(f),mean_abs_d_m=float(abs(f[:,2]).mean()),
                rms_d_m=float(np.sqrt(np.mean(f[:,2]**2))),
                p95_abs_d_m=float(np.percentile(abs(f[:,2]),95)),
                max_abs_d_m=float(abs(f[:,2]).max()),gt_vx_max_mps=float(g[:,3].max()),
                command_speed_max_mps=float(d[:,1].max()),
                command_steering_max_abs_rad=float(abs(d[:,2]).max()))
_,m=data['/car_state/odom_GT'][0];q=m.pose.pose.orientation
initial=dict(x=m.pose.pose.position.x,y=m.pose.pose.position.y,
             yaw_rad=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),vx=m.twist.twist.linear.x)
result=dict(window='finish-line timing start through third completed lap; bag reception timestamps',
            initial_gt_pose=initial,overall=stats(edges[0],edges[-1]),
            laps=[stats(a,b) for a,b in zip(edges,edges[1:])],
            message_counts={n:len(v) for n,v in data.items()})
(root/'tracking_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
np.savetxt(root/'frenet_trace.csv',fr,delimiter=',',header='bag_time_s,s_m,d_m',comments='')
fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True)
axs[0].plot(fr[:,0]-edges[0],fr[:,2]);axs[0].set_ylabel('Frenet d [m]')
axs[1].plot(drive[:,0]-edges[0],drive[:,1],label='Command')
axs[1].plot(gt[:,0]-edges[0],gt[:,3],label='GT vx');axs[1].set_ylabel('Speed [m/s]');axs[1].legend()
axs[2].plot(drive[:,0]-edges[0],drive[:,2]);axs[2].set_ylabel('Command steer [rad]')
for ax in axs:
    for t in edges:ax.axvline(t-edges[0],color='gray',linestyle=':')
    ax.set_xlim(0,edges[-1]-edges[0]);ax.grid(alpha=.3)
axs[-1].set_xlabel('Seconds from finish-line timing start')
fig.tight_layout();fig.savefig(root/'tracking.png',dpi=150)
print(json.dumps(result,indent=2))
