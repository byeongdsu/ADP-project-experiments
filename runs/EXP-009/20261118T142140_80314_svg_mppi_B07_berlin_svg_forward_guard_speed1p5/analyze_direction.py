"""Offline direction review; does not change the controller or raw bag."""
from pathlib import Path
import sqlite3,json,math,bisect
import numpy as np
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
run=Path(__file__).resolve().parent
s=json.loads((run/'session.json').read_text());wp=json.loads(Path(s['waypoints_file']).read_text())['global_traj_wpnts_iqp']['wpnts'];length=wp[-1]['s_m'];wp=wp[:-1]
points=np.array([[w['x_m'],w['y_m']] for w in wp]);tree=cKDTree(points)
db=next((run/'bag').glob('*.db3'));c=sqlite3.connect(f'file:{db}?mode=ro',uri=True);topics={name:(i,get_message(cls)) for i,name,cls in c.execute('select id,name,type from topics')}
def read(name):
 i,cls=topics[name];return [(t/1e9,deserialize_message(raw,cls)) for t,raw in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
od=read('/car_state/odom_GT');cmd=read('/mppi_sim/drive');risk=read('/mppi/collision_rate');paths=read('/mppi/best_path');nonzero=[(t,m) for t,m in cmd if m.drive.speed>0.01];start=nonzero[0][0];end=nonzero[-1][0];od=[(t,m) for t,m in od if start<=t<=end];t=np.array([v[0]-start for v in od]);xy=np.array([[m.pose.pose.position.x,m.pose.pose.position.y] for _,m in od]);q=[m.pose.pose.orientation for _,m in od];yaw=np.array([math.atan2(2*(v.w*v.z+v.x*v.y),1-2*(v.y*v.y+v.z*v.z)) for v in q]);distance,ix=tree.query(xy);refyaw=np.array([wp[i]['psi_rad'] for i in ix]);error=np.arctan2(np.sin(yaw-refyaw),np.cos(yaw-refyaw));arc=np.array([wp[i]['s_m'] for i in ix]);progress=np.unwrap(arc/length*2*math.pi)*length/(2*math.pi);progress-=progress[0];risk_t=[a for a,m in risk];cmd_t=[a for a,m in cmd];path_t=[a for a,m in paths]
def preceding(rows,times,stamp):return rows[max(0,bisect.bisect_right(times,stamp)-1)][1]
def snapshot(i):
 stamp=t[i]+start;command=preceding(cmd,cmd_t,stamp);rate=preceding(risk,risk_t,stamp);return {'time_after_first_nonzero_drive_s':float(t[i]),'xy':xy[i].tolist(),'waypoint_index':int(ix[i]),'distance_to_nearest_waypoint_m':float(distance[i]),'actual_heading_deg':float(np.degrees(yaw[i])),'reference_heading_deg':float(np.degrees(refyaw[i])),'heading_error_deg':float(np.degrees(error[i])),'gt_vx_m_s':float(od[i][1].twist.twist.linear.x),'speed_command_m_s':command.drive.speed,'steering_command_rad':command.drive.steering_angle,'candidate_collision_fraction':rate.data,'nearest_reference_progress_m':float(progress[i])}
opp=np.flatnonzero(np.abs(error)>math.pi/2);first=int(opp[0]) if len(opp) else None
peak=int(np.argmax(progress));samples=[snapshot(int(np.argmin(abs(t-v)))) for v in np.arange(0,t[-1],2)]
if first is not None:
 for dt in [-3,-2,-1,0,1,2,4]:samples.append(snapshot(int(np.argmin(abs(t-(t[first]+dt))))))
last=snapshot(len(t)-1);result={'alignment':'Latest preceding receipt samples, nearest waypoint direction; not exact internal callback synchronization. Nearest-reference progress approximate off path.','first_opposite_heading':snapshot(first) if first is not None else None,'maximum_forward_progress':snapshot(peak),'last':last,'progress_change_after_peak_m':float(progress[-1]-progress[peak]),'opposite_heading_duration_approx_s':float(np.diff(t,append=t[-1])[np.abs(error)>math.pi/2].sum()),'drive_speed_min_m_s':min(m.drive.speed for _,m in cmd),'drive_speed_max_m_s':max(m.drive.speed for _,m in cmd),'negative_drive_count':sum(m.drive.speed<-.001 for _,m in cmd),'negative_gt_velocity_count':sum(m.twist.twist.linear.x<-.001 for _,m in od),'samples':samples}
(run/'direction_detail.json').write_text(json.dumps(result,indent=2))
fig,ax=plt.subplots(2,2,figsize=(13,10));ax[0,0].plot(points[:,0],points[:,1],'--',c='gray',label='reference');ax[0,0].plot(xy[:,0],xy[:,1],label='GT');
if first is not None:ax[0,0].scatter(*xy[first],c='red',label='first heading error >90 deg')
ax[0,0].quiver(xy[::150,0],xy[::150,1],np.cos(yaw[::150]),np.sin(yaw[::150]),angles='xy',scale_units='xy',scale=2,width=.004);ax[0,0].axis('equal');ax[0,0].legend(fontsize=8);ax[0,0].set_title('Trajectory and actual vehicle headings')
ax[0,1].plot(t,np.degrees(error));ax[0,1].axhline(90,c='red',ls='--');ax[0,1].axhline(-90,c='red',ls='--');ax[0,1].set_title('Heading error from nearest reference [deg]')
ax[1,0].plot(t,progress);ax[1,0].set_title('Unwrapped nearest-reference progress [m]')
ax[1,1].plot([v-start for v,m in risk],[m.data for v,m in risk],label='candidate collision fraction');ax[1,1].set_ylim(0,1.05);right=ax[1,1].twinx();right.plot([v-start for v,m in cmd],[m.drive.steering_angle for v,m in cmd],c='green',alpha=.5,label='steer [rad]');ax[1,1].legend();right.set_ylabel('steering [rad]')
for a in ax.flat:a.grid(alpha=.2)
for a in [ax[0,1],ax[1,0],ax[1,1]]:
 a.set_xlabel('s after first nonzero drive')
 if first is not None:a.axvline(t[first],c='red',alpha=.4)
fig.tight_layout();fig.savefig(run/'direction_detail.png',dpi=150);plt.close(fig)
print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2));c.close()
