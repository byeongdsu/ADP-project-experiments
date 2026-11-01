#!/usr/bin/env python3
"""Four-terminal simulation workflow. Existing stacks and previous runs stay untouched."""
import argparse,ctypes,fcntl,hashlib,json,os,shlex,signal,subprocess,sys,threading,time
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[1];race=Path('/home/misys/forza_ws/race_stack');wp=race/'stack_master/maps/hyhall12/global_waypoints.json'
p=argparse.ArgumentParser();sub=p.add_subparsers(dest='phase',required=True)
prep=sub.add_parser('prepare');prep.add_argument('mode',choices=['forward_mppi','svg_mppi']);prep.add_argument('--config',type=Path);prep.add_argument('--obstacles',type=int,default=2);prep.add_argument('--seed',type=int,default=2026);prep.add_argument('--laps',type=int,default=3);prep.add_argument('--duration',type=float,default=240);prep.add_argument('--domain',type=int,default=76);prep.add_argument('--label',default='manual');prep.add_argument('--solver-version',choices=['upstream','repro_v1','speed_v1'],default='upstream')
for phase in ['sim','scenario','record','controller','rviz']:
 a=sub.add_parser(phase);a.add_argument('session',type=Path)
 if phase=='sim':a.add_argument('--headless',action='store_true')
args=p.parse_args()
if args.phase=='prepare':
 assert 0<=args.domain<=232 and args.obstacles>=0 and args.laps>=0 and args.duration>0
 solver_workspace=root/({'upstream':'ws','repro_v1':'ws_repro','speed_v1':'ws_speed'}[args.solver_version])
 controller_executable=solver_workspace/'install/svg_mppi_ros2/lib/svg_mppi_ros2/mppi_sim_node'
 assert controller_executable.is_file(),'Build the selected solver version first'
 original=(args.config or root/'ws/src/svg_mppi_ros2/config'/f'{args.mode}.yaml').resolve();cfg=yaml.safe_load(original.read_text());assert cfg['mpc_mode']==args.mode,'Config mpc_mode must match requested mode'
 assert 0<float(cfg['reference_speed'])<=(3. if args.solver_version=='speed_v1' else 2.),'Selected simulation adapter speed limit exceeded'
 assert cfg['common']['prediction_step_size']>=3 and cfg['common']['prediction_interval']>0
 assert cfg['common']['lf']==.15875 and cfg['common']['lr']==.17145,'Keep existing simulator geometry'
 assert 1<=cfg['common']['thread_num']<=22
 label=''.join(c for c in args.label if c.isalnum() or c in '_-');out=root/'manual_runs'/(time.strftime('%Y%m%dT%H%M%S')+f'_{os.getpid()}_{args.mode}_{label}');out.mkdir()
 (out/'controller.yaml').write_text(original.read_text())
 sim=yaml.safe_load((race/'stack_master/config/SIM/sim.yaml').read_text());params=sim['bridge']['ros__parameters'];params.update(yaml.safe_load((race/'stack_master/maps/hyhall12/start_pose.yaml').read_text()));params.update(map_path=str(race/'stack_master/maps/hyhall12/hyhall12_orig.yaml'),sim_params=str(race/'stack_master/config/SIM/sim_params.yaml'),ego_drive_topic='/mppi_sim/drive',kb_teleop=False)
 (out/'simulator.yaml').write_text(yaml.safe_dump(sim))
 session={'solver_version':args.solver_version,'solver_workspace':str(solver_workspace),'controller_executable':str(controller_executable),'controller_binary_sha256':hashlib.sha256(controller_executable.read_bytes()).hexdigest(),'mode':args.mode,'original_config':str(original),'config_sha256':hashlib.sha256((out/'controller.yaml').read_bytes()).hexdigest(),'obstacles':args.obstacles,'seed':args.seed,'laps':args.laps,'duration':args.duration,'domain':args.domain,'reference_speed':float(cfg['reference_speed'])}
 (out/'session.json').write_text(json.dumps(session,indent=2));cmds={1:f'bash manual.sh sim {out}',3:f'bash manual.sh scenario {out}',4:f'bash manual.sh record {out}',2:f'bash manual.sh controller {out}'}
 (out/'TERMINALS.txt').write_text('\n\n'.join(f'Terminal {i}:\ncd {root}\n{cmd}' for i,cmd in cmds.items())+'\n')
 print('SESSION',out);print((out/'TERMINALS.txt').read_text());sys.exit(0)
out=args.session.resolve()
if not (out/'session.json').is_file():
 p.error('세션 폴더에 session.json이 없습니다: '+str(out)+'\n$R 같은 셸 변수가 비어 있을 수 있습니다. manual_runs/... 세션의 전체 경로를 직접 지정하세요.')
session=json.loads((out/'session.json').read_text());os.environ['ROS_DOMAIN_ID']=str(session['domain']);os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
lock=(out/(args.phase+'.lock')).open('w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('This phase is already running')
if args.phase in ['sim','scenario','controller'] and (out/'summary.json').exists():raise SystemExit('Completed session: prepare a new run to preserve these results')
if args.phase in ['rviz','sim'] and not getattr(args,'headless',False):
 display_hint=root/'visual/host-display.txt'
 if os.getenv('MPPI_DISPLAY'):os.environ['DISPLAY']=os.environ['MPPI_DISPLAY']
 elif display_hint.exists():os.environ['DISPLAY']=display_hint.read_text().strip()
 x=ctypes.CDLL('libX11.so.6');x.XOpenDisplay.argtypes=[ctypes.c_char_p];x.XOpenDisplay.restype=ctypes.c_void_p;x.XCloseDisplay.argtypes=[ctypes.c_void_p];d=x.XOpenDisplay(None)
 if not d:raise SystemExit('Cannot open host DISPLAY. On Ubuntu HOST terminal run: xhost +si:localuser:$(id -un)\nThen retry this command inside the existing Humble container. No simulation was started.')
 x.XCloseDisplay(d)
# Refuse duplicate worlds/recorders even when a different session directory is used.
if args.phase in ['sim','scenario','record']:
 import rclpy
 from rclpy.signals import SignalHandlerOptions
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO);probe=rclpy.create_node('mppi_manual_preflight')
 for _ in range(5):rclpy.spin_once(probe,timeout_sec=.1)
 names=set(probe.get_node_names())-{'mppi_manual_preflight'}
 probe.destroy_node();rclpy.try_shutdown()
 if args.phase=='sim' and names:raise SystemExit('ROS domain is occupied: '+str(sorted(names))+'; stop previous run first')
 if args.phase=='scenario' and names&{'scorer','obstacle_spawner'}:raise SystemExit('A scenario is already running in this domain')
 if args.phase=='record' and 'mppi_manual_recorder' in names:raise SystemExit('A manual recorder is already running in this domain')
commands=[]
if args.phase=='sim':
 commands=[('simulator',['ros2','run','f1tenth_gym_ros','gym_bridge','--ros-args','-r','__node:=bridge','--params-file',str(out/'simulator.yaml')]),('inputs',['ros2','run','svg_mppi_ros2','sim_inputs.py','--ros-args','-p','waypoints_file:='+str(wp),'-p','reference_speed:='+str(session['reference_speed'])]),('visuals',['python3',str(root/'tools/visuals.py')])]
 if not args.headless:commands.append(('rviz',['rviz2','-d',str(root/'visual/mppi.rviz')]))
elif args.phase=='rviz':commands=[('rviz',['rviz2','-d',str(root/'visual/mppi.rviz')])]
elif args.phase=='scenario':
 commands=[('scorer',['ros2','run','race_manager','scorer','--ros-args','-p','waypoints_file:='+str(wp),'-p','map_name:=hyhall12','-p','log_dir:='+str(out),'-p','seed:='+str(session['seed'])])]
 if session['obstacles']:commands.append(('obstacles',['ros2','run','race_manager','obstacle_spawner','--ros-args','--params-file',str(race/'utilities/nodes/race_manager/config/race_manager_params.yaml'),'-p','waypoints_file:='+str(wp),'-p','num_obstacles:='+str(session['obstacles']),'-p','seed:='+str(session['seed'])]))
elif args.phase=='record':
 if (out/'bag').exists():raise SystemExit('Bag already exists; prepare a new session')
 commands=[('bag',['ros2','bag','record','-o',str(out/'bag'),'--node-name','mppi_manual_recorder','/scan','/car_state/odom_GT','/mppi_sim/drive','/mppi/local_costmap','/mppi/reference_sdf','/mppi/reference_path','/mppi/best_path','/mppi/calculation_time','/mppi/cycle_time','/mppi/collision_rate','/mppi/status','/sim/collision','/sim/obstacles','/sim/obstacles/markers','/sim/lap_count','/tf','/mppi/map','/mppi/costmap_view','/mppi/vehicle','/mppi/gt_trail'])]
elif args.phase=='controller':
 if 'controller_binary_sha256' in session:
  assert hashlib.sha256(Path(session['controller_executable']).read_bytes()).hexdigest()==session['controller_binary_sha256'],'Solver binary changed after preparation; prepare a new session'
 assert hashlib.sha256((out/'controller.yaml').read_bytes()).hexdigest()==session['config_sha256'],'Prepared config changed. Prepare a new session from edited tuning YAML.'
 commands=[('controller-terminal',['python3',str(root/'tools/run_sim.py'),session['mode'],'--external','--session-dir',str(out),'--obstacles',str(session['obstacles']),'--seed',str(session['seed']),'--laps',str(session['laps']),'--duration',str(session['duration'])])]
children=[];threads=[];handles=[];returncode=0
# Preserve rclpy-independent signal handling so child groups are always cleaned up.
def pump(pipe,handle,label):
 for line in iter(pipe.readline,''):handle.write(line);handle.flush();print(f'[{label}] {line}',end='',flush=True)
try:
 for label,cmd in commands:
  h=(out/(label+'.manual.log')).open('a');handles.append(h)
  with (out/'commands.manual.jsonl').open('a') as f:f.write(json.dumps({'phase':args.phase,'label':label,'wall':time.time(),'argv':cmd})+'\n')
  child=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True);children.append((label,child));t=threading.Thread(target=pump,args=(child.stdout,h,label),daemon=True);t.start();threads.append(t)
  print('START',shlex.join(cmd),flush=True)
 while True:
  ended=[(name,c) for name,c in children if c.poll() is not None]
  if ended:
   name,c=ended[0];returncode=c.returncode;print(f'{name} exited {returncode}',flush=True);break
  time.sleep(.2)
except KeyboardInterrupt:print('Stopping this terminal phase...',flush=True)
finally:
 for label,c in reversed(children):
  if c.poll() is None:
   os.killpg(c.pid,signal.SIGINT)
   try:c.wait(timeout=20)
   except subprocess.TimeoutExpired:
    os.killpg(c.pid,signal.SIGTERM)
    try:c.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(c.pid,signal.SIGKILL);c.wait()
 for t in threads:t.join(timeout=2)
 for h in handles:h.close()
 (out/(args.phase+'-exit.json')).write_text(json.dumps({'wall':time.time(),'returncode':returncode},indent=2))
raise SystemExit(returncode if returncode>=0 else 1)
