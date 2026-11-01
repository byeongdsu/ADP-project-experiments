#!/usr/bin/env python3
"""Runs this adapter only against the existing Gym simulator, in a private domain."""
import argparse,csv,hashlib,json,math,os,signal,subprocess,time
from pathlib import Path
import numpy as np,yaml
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import LaserScan
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Bool,Float32,String,Int32
from geometry_msgs.msg import PoseArray
from ackermann_msgs.msg import AckermannDriveStamped

root=Path(__file__).resolve().parents[1];race=Path('/home/misys/forza_ws/race_stack')
ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['forward_mppi','svg_mppi']);ap.add_argument('--duration',type=float,default=90);ap.add_argument('--obstacles',type=int,default=0);ap.add_argument('--seed',type=int,default=2026);ap.add_argument('--laps',type=int,default=1);ap.add_argument('--session-dir',type=Path);ap.add_argument('--external',action='store_true');args=ap.parse_args()
out=args.session_dir.resolve() if args.session_dir else root/'results'/(time.strftime('%Y%m%dT%H%M%S')+f'_{os.getpid()}_{args.mode}_obs{args.obstacles}')
if args.external:
 assert args.session_dir and (out/'session.json').exists(), 'Prepare a manual session first'
 assert not (out/'summary.json').exists(), 'This session already has results; prepare a new session'
else:out.mkdir(parents=True)
wp=race/'stack_master/maps/hyhall12/global_waypoints.json'
config=out/'controller.yaml' if args.external else root/'ws/src/svg_mppi_ros2/config'/f'{args.mode}.yaml'
reference_speed=float(yaml.safe_load(config.read_text())['reference_speed'])
sim=yaml.safe_load((race/'stack_master/config/SIM/sim.yaml').read_text());params=sim['bridge']['ros__parameters'];params.update(yaml.safe_load((race/'stack_master/maps/hyhall12/start_pose.yaml').read_text()))
params.update(map_path=str(race/'stack_master/maps/hyhall12/hyhall12_orig.yaml'),sim_params=str(race/'stack_master/config/SIM/sim_params.yaml'),ego_drive_topic='/mppi_sim/drive',kb_teleop=False)
if not args.external:
 (out/'simulator.yaml').write_text(yaml.safe_dump(sim));(out/'controller.yaml').write_text(config.read_text())
(out/'runner.py').write_text(Path(__file__).read_text())
provenance={'mode':args.mode,'obstacles':args.obstacles,'seed':args.seed,'duration_limit_wall_s':args.duration,'target_laps':args.laps,'startup_obstacle_barrier':bool(args.obstacles),'pose_source':'GT','speed_m_s':reference_speed,'domain':os.environ.get('ROS_DOMAIN_ID'),'upstream_commit':'f4fb7b74f12d4568790213aab26dd6f0538d1925','source_files':{}}
for f in [wp,race/'stack_master/config/SIM/sim_params.yaml',race/'base_system/f110_simulator/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py',config]:provenance['source_files'][str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
for f in sorted((root/'ws/src/svg_mppi_ros2').rglob('*')):
 if f.is_file() and '__pycache__' not in str(f):provenance['source_files'][str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
(out/'conditions.json').write_text(json.dumps(provenance,indent=2))
processes=[];logs=[]
def launch(label,cmd):
 h=(out/(label+'.log')).open('w');logs.append(h);p=subprocess.Popen(cmd,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);processes.append((label,p))
 with (out/'commands.jsonl').open('a') as f:f.write(json.dumps({'label':label,'argv':cmd})+'\n')
 return p
class Observer(Node):
 def __init__(self):
  super().__init__('mppi_sim_observer');self.latest={};self.counts={};self.calc=[];self.cycles=[];self.collisions=[];self.layouts=[];self.laps=0;self.status='';self.started=None;self.rows=[];self.stop_since=None
  self.enable=self.create_publisher(Bool,'/mppi/enable',10)
  self.subs=[]
  specs=[('/car_state/odom_GT',Odometry),('/scan',LaserScan),('/mppi/local_costmap',GridMap),('/mppi/calculation_time',Float32),('/mppi/cycle_time',Float32),('/mppi/collision_rate',Float32),('/mppi_sim/drive',AckermannDriveStamped),('/sim/collision',String),('/sim/lap_count',Int32),('/sim/obstacles',PoseArray)]
  for topic,cls in specs:self.subs.append(self.create_subscription(cls,topic,lambda m,t=topic:self.callback(t,m),qos_profile_sensor_data))
  for topic,cls in [('/mppi/status',String),('/mppi/reference_sdf',GridMap),('/mppi/reference_path',NavPath)]:self.subs.append(self.create_subscription(cls,topic,lambda m,t=topic:self.callback(t,m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
 def callback(self,t,m):
  self.counts[t]=self.counts.get(t,0)+1;self.latest[t]=m
  if t=='/mppi/status':self.status=m.data
  if t=='/sim/lap_count':self.laps=m.data
  if t=='/sim/obstacles':
   layout=[{'x':p.position.x,'y':p.position.y,'qz':p.orientation.z,'qw':p.orientation.w} for p in m.poses]
   if not self.layouts or self.layouts[-1]['poses']!=layout:self.layouts.append({'wall':time.time(),'lap':self.laps,'poses':layout})
  if self.started is None:return
  elapsed=time.monotonic()-self.started
  if t=='/mppi/calculation_time':self.calc.append((elapsed,m.data))
  if t=='/mppi/cycle_time':self.cycles.append((elapsed,m.data))
  if t=='/sim/collision' and m.data in ('wall','obstacle'):
   if not self.collisions or self.collisions[-1]['kind']!=m.data:self.collisions.append({'wall_s':elapsed,'kind':m.data})
  if t=='/car_state/odom_GT':
   p=m.pose.pose.position;cmd=self.latest.get('/mppi_sim/drive');self.rows.append((elapsed,p.x,p.y,m.twist.twist.linear.x,cmd.drive.speed if cmd else 0.,cmd.drive.steering_angle if cmd else 0.,self.latest.get('/mppi/collision_rate',Float32()).data))
   if abs(m.twist.twist.linear.x)<.05:
    if self.stop_since is None:self.stop_since=elapsed
   else:self.stop_since=None
 def arm(self,value):self.enable.publish(Bool(data=value))
 def spin(self,seconds=.05):rclpy.spin_once(self,timeout_sec=seconds)
 def healthy(self):
  for label,p in processes:
   if p.poll() is not None:raise RuntimeError(f'{label} exited {p.returncode}; see {label}.log')
 def wait(self,check,limit):
  end=time.monotonic()+limit
  while time.monotonic()<end:
   self.spin();self.healthy()
   if check():return
  raise TimeoutError(f'Input readiness timeout; status={self.status}, counts={self.counts}')
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Observer();reason='error';error=None
try:
 for _ in range(15):node.spin(.1)
 others=[n for n in node.get_node_names() if n!=node.get_name()]
 if args.external:
  expected={'bridge','mppi_sim_inputs','mppi_sim_visuals','rviz2','rviz','scorer','obstacle_spawner','mppi_manual_recorder'}
  unexpected={n for n in others if n not in expected and not n.startswith('transform_listener_impl_')}
  assert not unexpected, 'Unexpected nodes in manual domain: '+str(unexpected)
  node.wait(lambda:{'bridge','mppi_sim_inputs','scorer','mppi_manual_recorder'}<=set(node.get_node_names()),30)
 else:
  if others:raise RuntimeError(f'ROS domain is not empty: {others}; choose MPPI_DOMAIN_ID')
 if not args.external:
  launch('bag',['ros2','bag','record','-o',str(out/'bag'),'/scan','/car_state/odom_GT','/mppi_sim/drive','/mppi/local_costmap','/mppi/reference_sdf','/mppi/reference_path','/mppi/best_path','/mppi/calculation_time','/mppi/cycle_time','/mppi/collision_rate','/mppi/status','/sim/collision','/sim/obstacles','/sim/lap_count','/tf'])
  launch('simulator',['ros2','run','f1tenth_gym_ros','gym_bridge','--ros-args','-r','__node:=bridge','--params-file',str(out/'simulator.yaml')])
  launch('inputs',['ros2','run','svg_mppi_ros2','sim_inputs.py','--ros-args','-p','waypoints_file:='+str(wp),'-p','reference_speed:='+str(reference_speed)])
  launch('scorer',['ros2','run','race_manager','scorer','--ros-args','-p','waypoints_file:='+str(wp),'-p','map_name:=hyhall12','-p','log_dir:='+str(out),'-p','seed:='+str(args.seed)])
  if args.obstacles:
   launch('obstacles',['ros2','run','race_manager','obstacle_spawner','--ros-args','--params-file',str(race/'utilities/nodes/race_manager/config/race_manager_params.yaml'),'-p','waypoints_file:='+str(wp),'-p','num_obstacles:='+str(args.obstacles),'-p','seed:='+str(args.seed)])
 node.wait(lambda:all(t in node.latest for t in ['/scan','/car_state/odom_GT','/mppi/local_costmap','/mppi/reference_sdf']) and (not args.obstacles or ('/sim/obstacles' in node.latest and len(node.latest['/sim/obstacles'].poses)==args.obstacles)),120)
 launch('controller',['ros2','run','svg_mppi_ros2','mppi_sim_node','--ros-args','-p','config_file:='+str(out/'controller.yaml')])
 node.wait(lambda:node.status=='ready_disabled',60)
 scan=node.latest['/scan'];odom=node.latest['/car_state/odom_GT'];ref=node.latest['/mppi/reference_sdf'];cost=node.latest['/mppi/local_costmap']
 ready={'scan_beams':len(scan.ranges),'scan_frame':scan.header.frame_id,'odom_frame':odom.header.frame_id,'initial_pose':[odom.pose.pose.position.x,odom.pose.pose.position.y],'reference_layers':ref.layers,'costmap_layers':cost.layers,'publishers':[{ 'node':p.node_name,'type':p.topic_type} for p in node.get_publishers_info_by_topic('/mppi_sim/drive')]}
 assert len(ready['publishers'])==1 and not node.get_publishers_info_by_topic('/drive')
 assert math.hypot(ready['initial_pose'][0]-params['sx'],ready['initial_pose'][1]-params['sy'])<.01
 (out/'ready.json').write_text(json.dumps(ready,indent=2))
 if args.external:
  session=json.loads((out/'session.json').read_text());assert session['mode']==args.mode
  assert session['config_sha256']==hashlib.sha256(config.read_bytes()).hexdigest(), 'Session config changed after preparation; prepare a new session'
  assert 'mppi_manual_recorder' in node.get_node_names(), 'Start terminal 4 recording first'
  print('Recording and inputs ready. Starting MPPI; Ctrl+C stops this controller.',flush=True)
 node.started=time.monotonic();node.arm(True);print('RUN',out,flush=True)
 next_report=0
 while True:
  node.spin();node.healthy();elapsed=time.monotonic()-node.started
  if elapsed>=next_report:print(f'{elapsed:.1f}s status={node.status} laps={node.laps} calculations={len(node.calc)} collisions={len(node.collisions)}',flush=True);next_report+=15
  if node.collisions:reason='first_collision';break
  if args.laps>0 and node.laps>=args.laps:reason='lap_target';break
  if elapsed>=args.duration:reason='duration';break
  if node.stop_since is not None and elapsed-node.stop_since>=15:reason='stopped_15_wall_s';break
  if elapsed>10 and len(node.calc)<2:raise RuntimeError('No solver output')
  if node.status.startswith('solver_error'):raise RuntimeError(node.status)
except KeyboardInterrupt:reason='interrupted'
except Exception as exc:error=repr(exc);print(error,flush=True)
finally:
 if rclpy.ok() and any(label=='controller' for label,_ in processes):
  node.arm(False)
  for _ in range(8):node.spin(.05)
 for label,p in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=15)
   except subprocess.TimeoutExpired:
    os.killpg(p.pid,signal.SIGTERM)
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for h in logs:h.close()
 with (out/'trajectory.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['wall_s','x','y','vx','command_speed','command_steer','predicted_collision_rate']);w.writerows(node.rows)
 summary={'reason':reason,'error':error,'mode':args.mode,'obstacles':args.obstacles,'topic_counts':node.counts,'collisions':node.collisions,'laps_completed':node.laps,'distance_m':0.}
 if len(node.rows)>1:
  a=np.asarray(node.rows);distance=np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1).sum();summary.update(distance_m=float(distance),observed_wall_s=float(a[-1,0]),max_speed_m_s=float(a[:,3].max()),stopped_wall_s=float(np.diff(a[:,0])[np.abs(a[:-1,3])<.05].sum()))
  points=json.loads(wp.read_text())['global_traj_wpnts_iqp']['wpnts'];dist=cKDTree([(p['x_m'],p['y_m']) for p in points]).query(a[:,1:3])[0];summary['gt_nearest_waypoint_distance_m']={'mean':float(dist.mean()),'max':float(dist.max())}
 for name,values in [('solver_ms',node.calc),('cycle_ms',node.cycles)]:
  if values:
   v=np.array(values)[:,1];summary[name]={'n':len(v),'mean':float(v.mean()),'median':float(np.median(v)),'p95':float(np.percentile(v,95)),'max':float(v.max()),'over_25ms_fraction':float((v>25).mean())}
  with (out/(name+'.csv')).open('w',newline='') as f:w=csv.writer(f);w.writerow(['wall_s',name]);w.writerows(values)
 (out/'summary.json').write_text(json.dumps(summary,indent=2));(out/'obstacle_layouts.json').write_text(json.dumps(node.layouts,indent=2))
 print(json.dumps(summary,indent=2),flush=True);print('RESULT',out,flush=True)
 node.destroy_node();rclpy.try_shutdown()
 if node.rows:subprocess.run(['python3',str(root/'tools/plot_run.py'),str(out)],check=True)
if error:raise SystemExit(1)
