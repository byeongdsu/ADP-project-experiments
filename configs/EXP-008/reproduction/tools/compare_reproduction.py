"""Compare the third-layout obstacle encounter from two finalized ROS 2 bags."""
import argparse,bisect,json,math,sqlite3
from pathlib import Path
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
p=argparse.ArgumentParser();p.add_argument('previous',type=Path);p.add_argument('repeat',type=Path);p.add_argument('--previous-label',default='R03 collision');p.add_argument('--repeat-label',default='R04 completed');a=p.parse_args();result={'runs':[]}
fig,axs=plt.subplots(1,3,figsize=(17,5));mapdir=Path('/home/misys/forza_ws/race_stack/stack_master/maps/hyhall12');meta=yaml.safe_load((mapdir/'hyhall12_orig.yaml').read_text());im=Image.open(mapdir/'hyhall12_orig.png');ox,oy,_=meta['origin'];r=meta['resolution'];axs[0].imshow(im,extent=[ox,ox+im.width*r,oy,oy+im.height*r],origin='upper',cmap='gray')
all_layouts=[]
for run,color,label in [(a.previous,'red',a.previous_label),(a.repeat,'blue',a.repeat_label)]:
 db=sqlite3.connect(f'file:{next((run/"bag").glob("*.db3"))}?mode=ro',uri=True);topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
 def load(n,begin=None,end=None):
  i,cls=topics[n];sql='select timestamp,data from messages where topic_id=?';args=[i]
  if begin is not None:sql+=' and timestamp>=?';args.append(int(begin*1e9))
  if end is not None:sql+=' and timestamp<=?';args.append(int(end*1e9))
  return [(t/1e9,deserialize_message(data,cls)) for t,data in db.execute(sql+' order by timestamp',args)]
 def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 layouts=[]
 for t,m in load('/sim/obstacles'):
  xy=[(q.position.x,q.position.y,q.orientation.z,q.orientation.w) for q in m.poses]
  if not layouts or layouts[-1]['poses']!=xy:layouts.append({'time':t,'poses':xy})
 all_layouts.append(layouts);start=layouts[2]['time'];obs=layouts[2]['poses'][0];lap3=[t for t,m in load('/sim/lap_count') if m.data==3];hits=[t for t,m in load('/sim/collision') if m.data in ['wall','obstacle']];end=lap3[0] if lap3 else hits[0]
 odom=load('/car_state/odom_GT',start,end);xy=np.array([(m.pose.pose.position.x,m.pose.pose.position.y) for t,m in odom]);dist=np.hypot(xy[:,0]-obs[0],xy[:,1]-obs[1]);i=int(dist.argmin());center_time=odom[i][0];near=odom[i][1];solves=load('/mppi/calculation_time',None,start);before=load('/mppi_sim/drive',start,center_time)
 entry={'run':run.name,'outcome':json.loads((run/'summary.json').read_text())['reason'],'third_layout_start_receipt':start,'closest_obstacle_center_time':center_time,'elapsed_from_third_layout_s':center_time-start,'closest_center_distance_m':float(dist[i]),'closest_xy':xy[i].tolist(),'closest_yaw_rad':yaw(near.pose.pose.orientation),'calculations_recorded_before_third_layout':len(solves),'GT_pose_at_third_layout_start':[float(xy[0,0]),float(xy[0,1]),yaw(odom[0][1].pose.pose.orientation)],'snapshots':[]}
 for dt in [-2,-1,-.5,0,.5,1]:
  target=center_time+dt
  if target<start or target>end:continue
  idx=bisect.bisect_right([t for t,m in odom],target)-1;t,m=odom[max(0,idx)];row={'relative_to_closest_s':dt,'xy':[m.pose.pose.position.x,m.pose.pose.position.y],'yaw':yaw(m.pose.pose.orientation),'vx':m.twist.twist.linear.x}
  for n,k in [('/mppi_sim/drive','drive'),('/mppi/collision_rate','sample_collision_fraction')]:
   records=load(n,target-.2,target)
   if records:
    _,val=records[-1]
    if k=='drive':row.update(speed_cmd=val.drive.speed,steer_cmd=val.drive.steering_angle)
    else:row[k]=val.data
  entry['snapshots'].append(row)
 reference_x=8.232072024153762
 eligible=[(j,abs(pos[0]-reference_x)) for j,pos in enumerate(xy) if pos[1]<0 and 7<pos[0]<9]
 if eligible:
  j=min(eligible,key=lambda pair:pair[1])[0];msg=odom[j][1];entry['at_previous_collision_x']={'target_x_m':reference_x,'xy':xy[j].tolist(),'yaw_rad':yaw(msg.pose.pose.orientation),'vx':msg.twist.twist.linear.x}
 result['runs'].append(entry)
 segment=[(t,m) for t,m in odom if center_time-3<=t<=center_time+1.5];axs[0].plot([m.pose.pose.position.x for t,m in segment],[m.pose.pose.position.y for t,m in segment],color=color,label=label);axs[0].scatter(*xy[i],color=color,marker='x')
 for n,k,ax in [('/mppi_sim/drive','steer',axs[1]),('/mppi/collision_rate','risk',axs[2])]:
  vals=load(n,center_time-3,min(center_time+1.5,end));ax.plot([t-center_time for t,m in vals],[m.drive.steering_angle if k=='steer' else m.data for t,m in vals],color=color,label=label)
 db.close()
result['actual_first_three_layouts_identical']=[all_layouts[0][i]['poses']==all_layouts[1][i]['poses'] for i in range(3)];result['controller_yaml_identical']=(a.previous/'controller.yaml').read_bytes()==(a.repeat/'controller.yaml').read_bytes();result['simulator_yaml_identical']=(a.previous/'simulator.yaml').read_bytes()==(a.repeat/'simulator.yaml').read_bytes();result['alignment']='latest preceding receipt samples; plots aligned by each run closest center approach, not a shared physical timestamp';result['clearance_note']='center distance is not body surface clearance'
ob=all_layouts[0][2]['poses'][0];theta=2*math.atan2(ob[2],ob[3]);rot=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]]);corners=np.array([[-.15,-.15],[.15,-.15],[.15,.15],[-.15,.15]])@rot.T+ob[:2];axs[0].add_patch(Polygon(corners,facecolor='orange',edgecolor='black'));wp=json.loads((mapdir/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts'];axs[0].plot([q['x_m'] for q in wp],[q['y_m'] for q in wp],'--',color='gray',label='reference');axs[0].set_xlim(5,10);axs[0].set_ylim(-2,.5);axs[0].set_aspect('equal');axs[0].legend();axs[0].set_title('Same third-layout obstacle')
axs[1].set_ylabel('steering command [rad]');axs[2].set_ylabel('candidate collision fraction');axs[2].set_ylim(0,1.05)
for ax in axs[1:]:ax.set_xlabel('seconds relative to closest center approach');ax.legend()
fig.tight_layout();fig.savefig(a.repeat/'reproduction_comparison.png',dpi=170);plt.close(fig);(a.repeat/'reproduction_comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
