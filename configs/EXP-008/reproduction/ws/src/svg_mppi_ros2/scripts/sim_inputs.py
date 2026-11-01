#!/usr/bin/env python3
"""Simulation adapter: global nearest-waypoint fields and scan-only local obstacles.
The original CPU solvers are unchanged; field generation is a ROS 2 adaptation.
"""
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped

def make_grid(layers, resolution, center, frame, stamp):
    msg=GridMap(); msg.header.frame_id=frame; msg.header.stamp=stamp
    nx,ny=next(iter(layers.values())).shape
    msg.info.resolution=float(resolution);msg.info.length_x=nx*resolution;msg.info.length_y=ny*resolution
    msg.info.pose.position.x=float(center[0]);msg.info.pose.position.y=float(center[1]);msg.info.pose.orientation.w=1.
    for name,matrix in layers.items():
        a=Float32MultiArray()
        a.layout.dim=[MultiArrayDimension(label='column_index',size=ny,stride=nx*ny),MultiArrayDimension(label='row_index',size=nx,stride=nx)]
        a.data=np.asarray(matrix,dtype=np.float32).ravel(order='F').tolist()
        msg.layers.append(name);msg.data.append(a)
    return msg

class Inputs(Node):
    def __init__(self):
        super().__init__('mppi_sim_inputs')
        waypoint_file=self.declare_parameter('waypoints_file','').value
        self.speed=float(self.declare_parameter('reference_speed',1.).value)
        data=json.loads(Path(waypoint_file).read_text())['global_traj_wpnts_iqp']['wpnts']
        points=np.array([(p['x_m'],p['y_m']) for p in data]); yaw=np.array([p['psi_rad'] for p in data])
        self.reference_points=points
        resolution=.1; lo=points.min(axis=0)-4.;hi=points.max(axis=0)+4.
        size=np.ceil((hi-lo)/resolution).astype(int); center=(lo+hi)/2
        x=center[0]+size[0]*resolution/2-(np.arange(size[0])+.5)*resolution
        y=center[1]+size[1]*resolution/2-(np.arange(size[1])+.5)*resolution
        xx,yy=np.meshgrid(x,y,indexing='ij'); distance,index=cKDTree(points).query(np.column_stack([xx.ravel(),yy.ravel()]))
        shape=xx.shape
        self.ref_layers={'distance_field':distance.reshape(shape),'angle_field':yaw[index].reshape(shape),'speed_field':np.full(shape,self.speed)}
        self.ref_center=center
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.ref_pub=self.create_publisher(GridMap,'/mppi/reference_sdf',latched)
        self.path_pub=self.create_publisher(NavPath,'/mppi/reference_path',latched)
        self.cost_pub=self.create_publisher(GridMap,'/mppi/local_costmap',qos_profile_sensor_data)
        self.scan_sub=self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.res=.05;self.n=200;self.center=(3.,0.);self.scan=None;self.seen=False
        # Physical footprint .58 x .2032 m, plus 0.06 m longitudinal / 0.0484 m lateral margin.
        self.inflation=np.ones((15,7),dtype=bool)
        self.create_timer(.05,self.publish_costmap)
        self.create_timer(1.,self.publish_reference)
        self.publish_reference()
        self.get_logger().info(f'GT simulation fields: {len(points)} waypoints, {shape} reference cells, scan-only local obstacles')
    def publish_reference(self):
        stamp=self.get_clock().now().to_msg()
        self.ref_pub.publish(make_grid(self.ref_layers,.1,self.ref_center,'map',stamp))
        path=NavPath();path.header.stamp=stamp;path.header.frame_id='map'
        for x,y in self.reference_points:
            p=PoseStamped();p.header=path.header;p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.;path.poses.append(p)
        self.path_pub.publish(path)
    def scan_cb(self,msg):
        if msg.header.frame_id!='car_state/laser':
            self.get_logger().error('Unexpected scan frame: '+msg.header.frame_id);return
        self.scan=msg;self.scan_at=self.get_clock().now()
    def publish_costmap(self):
        if self.scan is None or (self.get_clock().now()-self.scan_at).nanoseconds>500_000_000:return
        scan=self.scan;r=np.asarray(scan.ranges);angles=scan.angle_min+np.arange(len(r))*scan.angle_increment
        valid=np.isfinite(r)&(r>scan.range_min)&(r<scan.range_max-.01)
        # Match this simulator's published laser transform, not the xacro's .275 m.
        x=r[valid]*np.cos(angles[valid])+.27;y=r[valid]*np.sin(angles[valid])
        ix=np.floor((8.-x)/self.res).astype(int);iy=np.floor((5.-y)/self.res).astype(int)
        inside=(ix>=0)&(ix<self.n)&(iy>=0)&(iy<self.n)
        occupied=np.zeros((self.n,self.n),dtype=bool);occupied[ix[inside],iy[inside]]=True
        inflated=binary_dilation(occupied,structure=self.inflation)
        msg=make_grid({'collision_layer':inflated.astype(np.float32)*100},self.res,self.center,'car_state/base_link',scan.header.stamp)
        self.cost_pub.publish(msg)
        if not self.seen:
            self.get_logger().info(f'First costmap: {len(r)} scan beams, {int(occupied.sum())} occupied, {int(inflated.sum())} inflated cells');self.seen=True

def main():
    rclpy.init();node=Inputs()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
