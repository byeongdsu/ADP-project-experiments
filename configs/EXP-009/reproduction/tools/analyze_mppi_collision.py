"""Read-only bag collision inspection using latest preceding receipt timestamps."""
import argparse,bisect,json,math,sqlite3
from pathlib import Path
import numpy as np,yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();run=a.run;dbfile=next((run/'bag').glob('*.db3'))
conn=sqlite3.connect(f'file:{dbfile}?mode=ro',uri=True);topics={n:(i,get_message(t)) for i,n,t in conn.execute('select id,name,type from topics')}
def load(name,begin=None,end=None):
 i,cls=topics[name];sql='select timestamp,data from messages where topic_id=?';params=[i]
 if begin is not None:sql+=' and timestamp>=?';params.append(int(begin*1e9))
 if end is not None:sql+=' and timestamp<=?';params.append(int(end*1e9))
 return [(t/1e9,deserialize_message(b,cls)) for t,b in conn.execute(sql+' order by timestamp',params)]
collisions=load('/sim/collision');prev='none';hits=[]
for t,m in collisions:
 if m.data in ['wall','obstacle'] and prev=='none':hits.append((t,m.data))
 prev=m.data
assert hits,'No collision event';hit,kind=hits[0]
names=['/car_state/odom_GT','/mppi_sim/drive','/mppi/best_path','/mppi/collision_rate','/mppi/calculation_time','/mppi/local_costmap','/scan']
data={n:load(n,hit-6,hit+.2) for n in names};data['/sim/obstacles']=load('/sim/obstacles',None,hit+.2);times={n:[t for t,_ in rows] for n,rows in data.items()}
def at(name,t):
 i=bisect.bisect_right(times[name],t)-1;return data[name][i] if i>=0 else (None,None)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def poly(x,y,angle,length,width):
 pts=np.array([[-length/2,-width/2],[length/2,-width/2],[length/2,width/2],[-length/2,width/2]]);c,s=math.cos(angle),math.sin(angle);return pts@np.array([[c,-s],[s,c]]).T+[x,y]
layouts=[]
for t,m in data['/sim/obstacles']:
 poses=[{'x':q.position.x,'y':q.position.y,'yaw':yaw(q.orientation)} for q in m.poses]
 if not layouts or poses!=layouts[-1]['poses']:layouts.append({'receipt_time':t,'poses':poses})
snapshots=[]
for dt in [-3,-2,-1,-.5,-.2,-.05,0]:
 t=hit+dt;ages={};msgs={}
 for n in data:
  ts,msg=at(n,t);ages[n]=None if ts is None else t-ts;msgs[n]=msg
 od=msgs['/car_state/odom_GT'];cmd=msgs['/mppi_sim/drive'];scan=msgs['/scan'];gm=msgs['/mppi/local_costmap'];pos=od.pose.pose.position
 ranges=np.asarray(scan.ranges);angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment;sel=np.isfinite(ranges)&(np.abs(angles)<.15)
 mat=gm.data[gm.layers.index('collision_layer')];nx=mat.layout.dim[1].size;ny=mat.layout.dim[0].size;cost=np.asarray(mat.data).reshape((nx,ny),order='F');ix=int((gm.info.pose.position.x+gm.info.length_x/2)/gm.info.resolution);iy=int((gm.info.pose.position.y+gm.info.length_y/2)/gm.info.resolution)
 snapshots.append({'dt_s':dt,'xy':[pos.x,pos.y],'yaw_rad':yaw(od.pose.pose.orientation),'actual_vx':od.twist.twist.linear.x,'speed_command':cmd.drive.speed,'steering_command_rad':cmd.drive.steering_angle,'predicted_collision_fraction':msgs['/mppi/collision_rate'].data,'solver_ms':msgs['/mppi/calculation_time'].data,'forward_scan_min_0_15rad_m':float(ranges[sel].min()),'costmap_at_robot_origin':float(cost[ix,iy]),'obstacle_center_distances_m':[math.hypot(q.position.x-pos.x,q.position.y-pos.y) for q in msgs['/sim/obstacles'].poses],'sample_age_s':ages})
out={'alignment':'latest preceding bag receipt samples; asynchronous, not internal callback alignment','collision_receipt_time':hit,'simulator_collision_kind':kind,'snapshots':snapshots,'layouts':layouts,'nearest_layout_age_at_collision_s':hit-layouts[-1]['receipt_time'],'interpretation':'Collision classification is the simulator LiDAR iTTC trigger beam label, not a direct rectangle intersection test.'}
(run/'collision_detail.json').write_text(json.dumps(out,indent=2))
fig,(ax,axcmd,axrisk)=plt.subplots(1,3,figsize=(18,6));session=json.loads((run/'session.json').read_text()) if (run/'session.json').exists() else {};mapyaml=Path(session.get('map_yaml','/home/misys/forza_ws/race_stack/stack_master/maps/hyhall12/hyhall12_orig.yaml'));mapdir=mapyaml.parent;m=yaml.safe_load(mapyaml.read_text());im=Image.open(mapdir/m['image']);ox,oy,_=m['origin'];r=m['resolution'];ax.imshow(im,extent=[ox,ox+im.width*r,oy,oy+im.height*r],origin='upper',cmap='gray')
odoms=[(t,m) for t,m in data['/car_state/odom_GT'] if hit-4<=t<=hit];xy=np.array([(m.pose.pose.position.x,m.pose.pose.position.y) for t,m in odoms]);ax.plot(xy[:,0],xy[:,1],color='blue',label='GT last 4 s')
wp=json.loads(Path(session.get('waypoints_file',str(mapdir/'global_waypoints.json'))).read_text())['global_traj_wpnts_iqp']['wpnts'];ax.plot([v['x_m'] for v in wp],[v['y_m'] for v in wp],'--',color='gray',label='reference')
for ob in layouts[-1]['poses']:ax.add_patch(Polygon(poly(ob['x'],ob['y'],ob['yaw'],.3,.3),facecolor='orange',edgecolor='red'))
for dt,color in [(-2,'green'),(-1,'purple'),(-.2,'red')]:
 _,path=at('/mppi/best_path',hit+dt);ax.plot([p.pose.position.x for p in path.poses],[p.pose.position.y for p in path.poses],color=color,label=f'prediction at {dt}s')
 row=next(x for x in snapshots if x['dt_s']==dt);ax.add_patch(Polygon(poly(*row['xy'],row['yaw_rad'],.58,.2032),fill=False,edgecolor=color))
end=snapshots[-1];ax.scatter(*end['xy'],color='red',marker='x',label='collision');ax.set_xlim(xy[:,0].min()-1,xy[:,0].max()+1);ax.set_ylim(xy[:,1].min()-1,xy[:,1].max()+1);ax.set_aspect('equal');ax.legend(fontsize=8);ax.set_title('Final layout; footprint outlines are illustrative')
axcmd.plot([t-hit for t,m in odoms],[m.twist.twist.linear.x for t,m in odoms],label='GT vx');cmds=[(t,m) for t,m in data['/mppi_sim/drive'] if hit-4<=t<=hit+.1];axcmd.plot([t-hit for t,m in cmds],[m.drive.speed for t,m in cmds],label='speed cmd');other=axcmd.twinx();other.plot([t-hit for t,m in cmds],[m.drive.steering_angle for t,m in cmds],color='green',label='steer');other.set_ylabel('steering [rad]');axcmd.legend();axcmd.set_xlabel('s relative to collision');axcmd.set_title('Speed and steering')
for name,label in [('/mppi/collision_rate','predicted collision fraction')]:
 vals=[(t,m.data) for t,m in data[name] if hit-4<=t<=hit];axrisk.plot([t-hit for t,v in vals],[v for t,v in vals],label=label)
axrisk.set_ylim(0,1.05);axrisk.set_xlabel('s relative to collision');axrisk.legend();axrisk.set_title('Sampled candidate collision fraction')
fig.tight_layout();fig.savefig(run/'collision_detail.png',dpi=170);plt.close(fig);print(json.dumps(out,indent=2));conn.close()
