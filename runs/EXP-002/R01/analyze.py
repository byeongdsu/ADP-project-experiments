"""Compare PF and GT around controller start using bag reception times."""
from pathlib import Path
import sqlite3,json,math
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
db=sqlite3.connect(f'file:{root/"telemetry/telemetry_0.db3"}?mode=ro',uri=True)
data={}
for tid,n,typ in db.execute('SELECT id,name,type FROM topics'):
    cls=get_message(typ)
    data[n]=[(t/1e9,deserialize_message(b,cls)) for t,b in db.execute(
        'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(tid,))]
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
gt=np.array([(t,m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)) for t,m in data['/car_state/odom_GT']])
pf=np.array([(t,m.pose.position.x,m.pose.position.y,yaw(m.pose.orientation)) for t,m in data['/pf/viz/inferred_pose']])
t0,drive=data['/drive'][0]
hit=next(t for t,m in data['/sim/collision'] if m.data=='wall')
rows=[]
for t,x,y,h in pf:
    i=np.searchsorted(gt[:,0],t,side='right')-1
    if i<0 or t-gt[i,0]>.1:continue
    g=gt[i];dh=math.atan2(math.sin(h-g[3]),math.cos(h-g[3]))
    rows.append([t-t0,math.hypot(x-g[1],y-g[2]),math.degrees(dh)])
a=np.array(rows)
pre=a[a[:,0]<=0][-1]
result=dict(first_command_time=t0,collision_time=hit,seconds_command_to_collision=hit-t0,
    last_pre_command_position_error_m=float(pre[1]),last_pre_command_heading_error_deg=float(pre[2]),
    first_command_speed_mps=drive.drive.speed,first_command_steering_rad=drive.drive.steering_angle,
    pre_command_pf_xy=pf[pf[:,0]<=t0][-1,1:3].tolist(),
    pre_command_gt_xy=gt[gt[:,0]<=t0][-1,1:3].tolist(),
    pre_command_pf_yaw_deg=float(np.degrees(pf[pf[:,0]<=t0][-1,3])),
    pre_command_gt_yaw_deg=float(np.degrees(gt[gt[:,0]<=t0][-1,3])))
(root/'localization_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
np.savetxt(root/'localization_error.csv',a,delimiter=',',header='seconds_from_first_command,position_error_m,heading_error_deg',comments='')
fig,axs=plt.subplots(2,1,figsize=(10,6),sharex=True)
axs[0].plot(a[:,0],a[:,1]);axs[0].set_ylabel('PF-GT position error [m]')
axs[1].plot(a[:,0],a[:,2]);axs[1].set_ylabel('Wrapped yaw error [deg]')
for ax in axs:
    ax.axvline(0,color='black',linestyle=':',label='First command')
    ax.axvline(hit-t0,color='red',linestyle='--',label='Wall collision');ax.grid(alpha=.3)
axs[0].legend();axs[-1].set_xlabel('Seconds from first drive command')
fig.tight_layout();fig.savefig(root/'localization_error.png',dpi=150)
print(json.dumps(result,indent=2))
