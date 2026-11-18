"""Offline Gym numerical braking check; no ROS publisher/GUI/bag driving."""
from pathlib import Path
import json,math
import gymnasium as gym
import f110_gym,numpy as np,yaml
r=Path('/home/misys/shared_dir/svg_mppi_ros2_20261031');o=Path(__file__).resolve().parent
s=yaml.safe_load((r/'manual_runs/20261118T140922_71360_svg_mppi_B06_berlin_footprint_svg_speed1p5/simulator.yaml').read_text())['bridge']['ros__parameters'];params={k:float(v) for k,v in yaml.safe_load(Path(s['sim_params']).read_text()).items()};results=[]
env=gym.make('f110_gym:f110-v1',map=str(Path(s['map_path']).with_suffix('')),map_ext=s['map_img_ext'],params=params,num_agents=1,disable_env_checker=True)
for steer in [0.,-.4189,.4189]:
 env.reset(poses=np.array([[s['sx'],s['sy'],s['stheta']]]))
 for _ in range(100):obs,_,_,_=env.step(np.array([[0.,1.5]]))
 for _ in range(15):obs,_,_,_=env.step(np.array([[steer,1.5]]))
 start=np.array([obs['poses_x'][0],obs['poses_y'][0]]);previous=start.copy();distance=0.;stopped=None;rows=[]
 for i in range(200):
  obs,_,_,_=env.step(np.array([[steer,0.]]));pos=np.array([obs['poses_x'][0],obs['poses_y'][0]]);distance+=np.linalg.norm(pos-previous);previous=pos;v=float(obs['linear_vels_x'][0]);rows.append({'t_s':(i+1)*.01,'v_m_s':v,'path_distance_m':float(distance),'xy':pos.tolist()})
  if stopped is None and abs(v)<.05:stopped={'t_s':(i+1)*.01,'distance_m':float(distance)}
 assert stopped is not None and abs(v)<.01
 results.append({'held_steering_rad':steer,'initial_speed_target_m_s':1.5,'stop_below_0_05':stopped,'final_speed_m_s':v,'total_distance_after_stop_request_m':float(distance),'trajectory':rows})
env.close();(o/'braking_test.json').write_text(json.dumps({'offline_only':True,'dt_s':.01,'results':results,'limitation':'Fresh zero target, no communication delay; finite test conditions, not guaranteed collision-free braking.'},indent=2));print(json.dumps([{k:v for k,v in x.items() if k!='trajectory'} for x in results],indent=2))
