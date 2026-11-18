"""Offline audit that simulator obstacles were present in scan/costmap observations.
Near-center LiDAR returns are a geometric consistency check, not semantic labels.
"""
import json,sys,math
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
p=Path(sys.argv[1]);reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(p/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan','/car_state/odom_GT','/sim/obstacles','/mppi/local_costmap','/mppi/reference_sdf']))
odom=None;obs=[];counts={};scan_proximity_frames=0;scan_samples=0;cost_hits=0;cost_samples=0;min_scan_distance=None
while reader.has_next():
 topic,data,t=reader.read_next();counts[topic]=counts.get(topic,0)+1
 # Subsample scan/costmap only; keep every odom and layout event.
 if topic in ['/scan','/mppi/local_costmap'] and counts[topic]%10:continue
 msg=deserialize_message(data,types[topic])
 if topic=='/car_state/odom_GT':odom=msg
 elif topic=='/sim/obstacles':obs=[(q.position.x,q.position.y) for q in msg.poses]
 elif topic in ['/scan','/mppi/local_costmap'] and obs and odom is not None:
  pose=odom.pose.pose;q=pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c,s=math.cos(yaw),math.sin(yaw)
  if topic=='/scan':
   r=np.asarray(msg.ranges);angles=msg.angle_min+np.arange(len(r))*msg.angle_increment;valid=np.isfinite(r)&(r<msg.range_max-.01)&(r>msg.range_min);lx=r[valid]*np.cos(angles[valid])+.27;ly=r[valid]*np.sin(angles[valid]);gx=pose.position.x+c*lx-s*ly;gy=pose.position.y+s*lx+c*ly
   best=min(float(np.hypot(gx-x,gy-y).min()) for x,y in obs);min_scan_distance=best if min_scan_distance is None else min(min_scan_distance,best);scan_samples+=1;scan_proximity_frames+=int(best<.25)
  else:
   a=msg.data[msg.layers.index('collision_layer')];nx=a.layout.dim[1].size;ny=a.layout.dim[0].size;m=np.asarray(a.data).reshape((nx,ny),order='F');maxx=msg.info.pose.position.x+msg.info.length_x/2;maxy=msg.info.pose.position.y+msg.info.length_y/2
   for x,y in obs:
    dx=x-pose.position.x;dy=y-pose.position.y;lx=c*dx+s*dy;ly=-s*dx+c*dy
    if math.hypot(dx,dy)>3:continue
    ix=int(math.floor((maxx-lx)/msg.info.resolution));iy=int(math.floor((maxy-ly)/msg.info.resolution))
    if 0<=ix<nx and 0<=iy<ny:cost_samples+=1;cost_hits+=int(m[ix,iy]>0)
result={'topic_counts':counts,'subsampled_scan_frames':scan_samples,'frames_with_return_within_0_25m_of_obstacle_center':scan_proximity_frames,'minimum_return_distance_to_obstacle_center_m':min_scan_distance,'costmap_obstacle_center_samples_within_3m':cost_samples,'costmap_samples_with_positive_cost_at_obstacle_center':cost_hits,'note':'Geometric scan consistency; obstacle size 0.30 m. Samples joined to most recent bag odom, not exact callback synchronization.'}
(p/'input_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
