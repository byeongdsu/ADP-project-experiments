"""Run with ROS environment sourced; analyze recorded data without replay."""
from pathlib import Path
import csv
import json
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
db = sqlite3.connect(f'file:{root / "telemetry/telemetry_0.db3"}?mode=ro', uri=True)
data = {}
for tid, name, typ in db.execute('SELECT id,name,type FROM topics'):
    cls = get_message(typ)
    data[name] = [(t / 1e9, deserialize_message(b, cls)) for t, b in
                  db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))]
# Absolute wall times from scorer logs, independent of JSON's rounded elapsed time.
start, hit = 1792485987.781774433, 1792485996.428340034
fr = np.array([(t, m.pose.pose.position.x, m.pose.pose.position.y,
                m.twist.twist.linear.x) for t, m in data['/car_state/frenet/odom']])
drive = np.array([(t, m.drive.speed, m.drive.steering_angle) for t, m in data['/drive']])
window = fr[(fr[:, 0] >= start) & (fr[:, 0] < hit)]
cmd = drive[(drive[:, 0] >= start) & (drive[:, 0] < hit)]
metrics = dict(window='finish-line timing start to first collision; bag reception timestamps',
               samples=len(window), mean_abs_d_m=float(np.mean(abs(window[:, 2]))),
               rms_d_m=float(np.sqrt(np.mean(window[:, 2]**2))),
               p95_abs_d_m=float(np.percentile(abs(window[:, 2]), 95)),
               max_abs_d_m=float(np.max(abs(window[:, 2]))),
               command_speed_min_mps=float(cmd[:, 1].min()),
               command_speed_max_mps=float(cmd[:, 1].max()),
               command_steering_max_abs_rad=float(abs(cmd[:, 2]).max()),
               commanded_steering_fraction_above_sim_limit=float(np.mean(abs(cmd[:, 2]) > .4189)),
               gt_odometry_recorded='/car_state/odom_GT' in data)
with (root / 'frenet_trace.csv').open('w') as f:
    w = csv.writer(f); w.writerow(['bag_time_s','time_from_timing_start_s','s_m','d_m','frenet_vs_estimate_mps'])
    w.writerows((t,t-start,s,d,v) for t,s,d,v in fr)
gt = np.array([(t, m.pose.pose.position.x, m.pose.pose.position.y,
                m.twist.twist.linear.x, m.twist.twist.linear.y) for t,m in data['/car_state/odom_GT']])
gwin = gt[(gt[:,0]>=start) & (gt[:,0]<hit)]
moving = gwin[gwin[:,3] > 0.1]
metrics.update(gt_vx_max_mps=float(gwin[:,3].max()),
               gt_speed_magnitude_max_mps=float(np.hypot(gwin[:,3],gwin[:,4]).max()),
               last_pre_collision_gt_vx_mps=float(gwin[-1,3]),
               last_pre_collision_gt_xy=gwin[-1,1:3].tolist(),
               last_pre_collision_d_m=float(window[-1,2]))
metrics.update(last_moving_gt_vx_mps=float(moving[-1,3]),
               last_moving_gt_time_from_start_s=float(moving[-1,0]-start),
               last_moving_gt_xy=moving[-1,1:3].tolist())
with (root/'gt_trace.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['bag_time_s','x_m','y_m','vx_mps','vy_mps']);w.writerows(gt)
(root / 'tracking_metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
fig, axs = plt.subplots(3,1,figsize=(10,8),sharex=True)
axs[0].plot(fr[:,0]-start,fr[:,2]); axs[0].set_ylabel('Frenet d [m]')
axs[1].plot(drive[:,0]-start,drive[:,1]); axs[1].plot(gt[:,0]-start,gt[:,3],label='GT vx'); axs[1].set_ylabel('Speed [m/s]'); axs[1].legend(['Command','GT vx'])
axs[2].plot(drive[:,0]-start,drive[:,2]); axs[2].set_ylabel('Command steer [rad]')
for limit in [-.4189,.4189]: axs[2].axhline(limit,color='gray',linestyle=':')
for ax in axs:
    ax.axvline(hit-start,color='red',linestyle='--',label='First collision')
    ax.set_xlim(-.6,hit-start+1); ax.grid(alpha=.3)
axs[0].legend(); axs[-1].set_xlabel('Seconds from finish-line timing start')
fig.tight_layout(); fig.savefig(root/'tracking.png',dpi=150)
print(json.dumps(metrics, indent=2))
print('Last pre-collision Frenet samples:',window[-5:].tolist())
print('Last pre-collision commands:',cmd[-5:].tolist())
