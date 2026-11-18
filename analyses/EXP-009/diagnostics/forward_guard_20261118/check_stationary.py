import os,signal,subprocess,time,json,math
from pathlib import Path
import numpy as np,yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
from sensor_msgs.msg import LaserScan
from grid_map_msgs.msg import GridMap
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
root=Path('/home/misys/shared_dir/svg_mppi_ros2_20261031');out=Path(__file__).resolve().parent
session=sorted((root/'manual_runs').glob('*B07_berlin_svg_forward_guard_speed1p5'))[-1];s=json.loads((session/'session.json').read_text());race=Path('/home/misys/forza_ws/race_stack')
os.environ['ROS_DOMAIN_ID']='78';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
rclpy.init();node=rclpy.create_node('berlin_stationary_check');latest={};counts={};nonzero=[];children=[];handles=[]
def cb(topic,msg):
 latest[topic]=msg;counts[topic]=counts.get(topic,0)+1
 if topic=='/mppi_sim/drive' and abs(msg.drive.speed)>1e-8:nonzero.append(msg.drive.speed)
subscriptions=[]
for topic,cls,latched in [('/scan',LaserScan,False),('/car_state/odom_GT',Odometry,False),('/mppi/local_costmap',GridMap,False),('/mppi/reference_sdf',GridMap,True),('/mppi/reference_path',NavPath,True),('/mppi/map',OccupancyGrid,True),('/sim/obstacles',PoseArray,False),('/mppi/status',String,True),('/mppi_sim/drive',AckermannDriveStamped,False)]:
 qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if latched else qos_profile_sensor_data
 subscriptions.append(node.create_subscription(cls,topic,lambda m,t=topic:cb(t,m),qos))
for _ in range(4):rclpy.spin_once(node,timeout_sec=.1)
assert set(node.get_node_names())<={'berlin_stationary_check'},'Test domain occupied'
commands=[('simulator',['ros2','run','f1tenth_gym_ros','gym_bridge','--ros-args','-r','__node:=bridge','--params-file',str(session/'simulator.yaml')]),('inputs',['python3',str(Path(s['solver_workspace'])/'install/svg_mppi_ros2/lib/svg_mppi_ros2/sim_inputs.py'),'--ros-args','-p','waypoints_file:='+s['waypoints_file'],'-p','reference_speed:='+str(s['reference_speed'])]),('visuals',['python3',str(root/'tools/visuals.py'),'--ros-args','-p','map_yaml:='+s['map_yaml']]),('obstacles',['ros2','run','race_manager','obstacle_spawner','--ros-args','--params-file',str(race/'utilities/nodes/race_manager/config/race_manager_params.yaml'),'-p','waypoints_file:='+s['waypoints_file'],'-p','num_obstacles:=4','-p','seed:=2026']),('controller',[s['controller_executable'],'--ros-args','-p','config_file:='+str(session/'controller.yaml')])]
result={'stationary_only':True,'controller_enable_published':False,'domain':78,'session':str(session)}
try:
 for label,cmd in commands:
  h=(out/(label+'.check.log')).open('w');handles.append(h);children.append((label,subprocess.Popen(cmd,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)))
 deadline=time.monotonic()+35
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  for label,p in children:
   if p.poll() is not None:raise RuntimeError(label+' exited '+str(p.returncode))
  if len(latest)==9 and latest['/mppi/status'].data=='ready_disabled':break
 assert len(latest)==9,('Missing inputs',set(t for t,_,_ in [('/scan',LaserScan,False)])-set(latest),list(latest))
 assert latest['/mppi/status'].data=='ready_disabled',latest['/mppi/status'].data
 pose=yaml.safe_load(Path(s['start_pose_file']).read_text());odom=latest['/car_state/odom_GT'];pos=odom.pose.pose.position
 assert math.hypot(pos.x-pose['sx'],pos.y-pose['sy'])<.001
 assert abs(odom.twist.twist.linear.x)<1e-8 and not nonzero
 grid=latest['/mppi/local_costmap'];assert {'raw_occupancy','collision_layer','clearance_cost'}<=set(grid.layers)
 ref=latest['/mppi/reference_sdf'];speeds=np.array(ref.data[ref.layers.index('speed_field')].data);assert np.allclose(speeds,s['reference_speed'])
 mp=latest['/mppi/map'];assert mp.info.width==600 and mp.info.height==600 and abs(mp.info.resolution-.04)<1e-8
 assert abs(mp.info.origin.position.x+9.285232)<1e-8 and abs(mp.info.origin.position.y+21.2166344)<1e-8
 assert len(latest['/sim/obstacles'].poses)==4
 assert len(latest['/mppi/reference_path'].poses)==489
 assert len(node.get_publishers_info_by_topic('/mppi_sim/drive'))==1 and not node.get_publishers_info_by_topic('/drive')
 result.update(passed=True,counts=counts,initial_xy=[pos.x,pos.y],map_shape=[mp.info.width,mp.info.height],reference_points=489,costmap_layers=grid.layers,obstacles=4,controller_status=latest['/mppi/status'].data,reference_speed_min=float(speeds.min()),reference_speed_max=float(speeds.max()),nonzero_drive_count=len(nonzero),obstacle_poses=[{'x':p.position.x,'y':p.position.y,'qz':p.orientation.z,'qw':p.orientation.w} for p in latest['/sim/obstacles'].poses])
except Exception as e:
 result.update(passed=False,error=repr(e));raise
finally:
 for label,p in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for h in handles:h.close()
 (out/'stationary_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));node.destroy_node();rclpy.try_shutdown()
