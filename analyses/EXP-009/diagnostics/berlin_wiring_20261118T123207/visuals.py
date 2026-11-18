#!/usr/bin/env python3
"""Read-only visualization for live simulation or rosbag replay. Publishes no controls."""
from pathlib import Path
import math
import numpy as np,yaml
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid,Odometry,Path as NavPath
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32
class Visuals(Node):
 def __init__(self):
  super().__init__('mppi_sim_visuals');latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.map_pub=self.create_publisher(OccupancyGrid,'/mppi/map',latched);self.cost_pub=self.create_publisher(OccupancyGrid,'/mppi/costmap_view',10);self.markers=self.create_publisher(MarkerArray,'/mppi/vehicle',10);self.trail_pub=self.create_publisher(NavPath,'/mppi/gt_trail',10)
  self.odom=None;self.ms=0.;self.trail=NavPath();self.last_trail=0.
  self.create_subscription(Odometry,'/car_state/odom_GT',self.on_odom,qos_profile_sensor_data)
  self.create_subscription(GridMap,'/mppi/local_costmap',self.on_cost,qos_profile_sensor_data)
  self.create_subscription(Float32,'/mppi/calculation_time',self.on_ms,qos_profile_sensor_data)
  path=Path('/home/misys/forza_ws/race_stack/stack_master/maps/hyhall12/hyhall12_orig.yaml');m=yaml.safe_load(path.read_text());im=np.asarray(Image.open(path.parent/m['image']).convert('L'))
  prob=(255.-im[::-1,:])/255.;values=np.full(im.shape,-1,dtype=np.int8);values[prob<m['free_thresh']]=0;values[prob>m['occupied_thresh']]=100
  self.map=OccupancyGrid();self.map.header.frame_id='map';self.map.info.resolution=float(m['resolution']);self.map.info.width=im.shape[1];self.map.info.height=im.shape[0];self.map.info.origin.position.x=float(m['origin'][0]);self.map.info.origin.position.y=float(m['origin'][1]);self.map.info.origin.orientation.w=1.;self.map.data=values.ravel().tolist()
  self.create_timer(1.,self.publish_map);self.create_timer(.1,self.publish_car);self.publish_map()
 def publish_map(self):self.map.header.stamp=self.get_clock().now().to_msg();self.map_pub.publish(self.map)
 def on_ms(self,msg):self.ms=msg.data
 def on_odom(self,msg):self.odom=msg
 def on_cost(self,msg):
  a=msg.data[msg.layers.index('collision_layer')];nx=a.layout.dim[1].size;ny=a.layout.dim[0].size;m=np.asarray(a.data).reshape((nx,ny),order='F')
  out=OccupancyGrid();out.header=msg.header;out.info.resolution=msg.info.resolution;out.info.width=nx;out.info.height=ny;out.info.origin.position.x=msg.info.pose.position.x-msg.info.length_x/2;out.info.origin.position.y=msg.info.pose.position.y-msg.info.length_y/2;out.info.origin.orientation.w=1.;out.data=np.clip(m[::-1,::-1].T,0,100).astype(np.int8).ravel().tolist();self.cost_pub.publish(out)
 def publish_car(self):
  if self.odom is None:return
  od=self.odom;stamp=od.header.stamp;array=MarkerArray()
  for i,kind in enumerate([Marker.CUBE,Marker.ARROW,Marker.TEXT_VIEW_FACING]):
   m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='mppi_vehicle';m.id=i;m.type=kind;m.action=Marker.ADD;m.pose=od.pose.pose
   # Copy individual pose values; ROS Python message assignment aliases the object.
   import copy
   m.pose=copy.deepcopy(od.pose.pose)
   m.color.a=1.
   if kind==Marker.CUBE:m.scale.x=.58;m.scale.y=.2032;m.scale.z=.13;m.pose.position.z=.1;m.color.b=1.;m.color.g=.35
   elif kind==Marker.ARROW:m.scale.x=.7;m.scale.y=.06;m.scale.z=.06;m.pose.position.z=.23;m.color.r=1.;m.color.g=.9
   else:m.pose.position.z=.8;m.scale.z=.22;m.color.r=.05;m.color.g=.05;m.color.b=.05;m.text=f'GT {od.twist.twist.linear.x:.2f} m/s | solve {self.ms:.1f} ms'
   array.markers.append(m)
  self.markers.publish(array)
  pose=PoseStamped();pose.header=od.header;pose.pose=od.pose.pose;self.trail.header=od.header
  if not self.trail.poses or math.hypot(pose.pose.position.x-self.trail.poses[-1].pose.position.x,pose.pose.position.y-self.trail.poses[-1].pose.position.y)>.03:self.trail.poses.append(pose)
  self.trail.poses=self.trail.poses[-10000:];self.trail_pub.publish(self.trail)
def main():
 rclpy.init();node=Visuals()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
