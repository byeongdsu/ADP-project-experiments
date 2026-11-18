"""Synthetic ROS inputs only: no simulator and no live moving vehicle."""
import importlib.util,os,json,signal,subprocess,time
from pathlib import Path
import numpy as np,yaml
os.environ['ROS_DOMAIN_ID']='79';os.environ['ROS_LOCALHOST_ONLY']='1'
root=Path('/home/misys/shared_dir/svg_mppi_ros2_20261031');out=Path(__file__).resolve().parent;os.environ['ROS_LOG_DIR']=str(out/'guard_ros_logs')
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,String
from ackermann_msgs.msg import AckermannDriveStamped
spec=importlib.util.spec_from_file_location('inputs',root/'ws_forward_guard/src/svg_mppi_ros2/scripts/sim_inputs.py');inputs=importlib.util.module_from_spec(spec);spec.loader.exec_module(inputs)
rclpy.init();node=rclpy.create_node('synthetic_guard_check');latest={};commands=[];statuses=[]
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
refpub=node.create_publisher(GridMap,'/mppi/reference_sdf',latched);costpub=node.create_publisher(GridMap,'/mppi/local_costmap',qos_profile_sensor_data);odompub=node.create_publisher(Odometry,'/car_state/odom_GT',qos_profile_sensor_data);enable=node.create_publisher(Bool,'/mppi/enable',10)
sub1=node.create_subscription(String,'/mppi/status',lambda m:statuses.append(m.data),latched);sub2=node.create_subscription(AckermannDriveStamped,'/mppi_sim/drive',lambda m:commands.append(m.drive.speed),qos_profile_sensor_data)
for _ in range(4):rclpy.spin_once(node,timeout_sec=.1)
assert set(node.get_node_names())<={'synthetic_guard_check'},'Domain occupied'
base=yaml.safe_load((root/'manual_runs/20261118T140204_66033_forward_mppi_B05_berlin_footprint_speed1p5/controller.yaml').read_text());results=[]
for mode in ['forward_mppi','svg_mppi']:
 commands.clear();statuses.clear();cfg=dict(base);cfg['mpc_mode']=mode;config=out/f'{mode}_guard_test.yaml';config.write_text(yaml.safe_dump(cfg));h=(out/f'{mode}_guard_node.log').open('w');child=subprocess.Popen([str(root/'ws_forward_guard/install/svg_mppi_ros2/lib/svg_mppi_ros2/mppi_sim_node'),'--ros-args','-p','config_file:='+str(config)],stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
 def publish(blocked):
  stamp=node.get_clock().now().to_msg();z=np.zeros((200,200),dtype=np.float32);raw=z.copy();
  if blocked:raw[150,:]=100.
  costpub.publish(inputs.make_grid({'raw_occupancy':raw,'collision_layer':z,'clearance_cost':z},.05,(3.,0.),'car_state/base_link',stamp));refpub.publish(inputs.make_grid({'distance_field':z,'angle_field':z,'speed_field':z+1.5},.05,(3.,0.),'map',stamp));odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='map';odom.child_frame_id='car_state/base_link';odom.pose.pose.orientation.w=1.;odom.twist.twist.linear.x=1.5;odompub.publish(odom)
 try:
  deadline=time.monotonic()+10
  while time.monotonic()<deadline and 'ready_disabled' not in statuses:
   assert child.poll() is None,'Controller exited';publish(True);rclpy.spin_once(node,timeout_sec=.025)
  assert 'ready_disabled' in statuses,statuses
  deadline=time.monotonic()+10
  while time.monotonic()<deadline and not any(s.startswith('guard_stop:') for s in statuses):
   publish(True);enable.publish(Bool(data=True));rclpy.spin_once(node,timeout_sec=.025)
  expected='guard_stop: no_collision_free_forward_candidate';assert expected in statuses,statuses
  # Removing obstacle and repeatedly re-enabling must not auto restart a latched stop.
  until=time.monotonic()+1.
  while time.monotonic()<until:publish(False);enable.publish(Bool(data=True));rclpy.spin_once(node,timeout_sec=.025)
  assert commands and all(abs(v)<1e-12 for v in commands),commands
  results.append({'mode':mode,'passed':True,'statuses':statuses.copy(),'zero_drive_messages':len(commands),'nonzero_drive_messages':0,'latch_survived_clear_map_and_reenable':True})
 finally:
  if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
  try:child.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
  h.close()
  for _ in range(10):rclpy.spin_once(node,timeout_sec=.03)
(out/'guard_node_test.json').write_text(json.dumps({'synthetic_inputs_only':True,'simulator_started':False,'results':results},indent=2));print(json.dumps(results,indent=2));node.destroy_node();rclpy.try_shutdown()
