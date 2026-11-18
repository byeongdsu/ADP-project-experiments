"""Offline forward-only kinematic search with conservative body disk clearance."""
from pathlib import Path
import json,math,heapq,time
import numpy as np,yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
from matplotlib.path import Path as PolyPath
root=Path('/home/misys/shared_dir/svg_mppi_ros2_20261031');run=root/'manual_runs/20261118T123305_18673_forward_mppi_B01_berlin_speed1p0';out=run/'passage_analysis';d=json.loads((out/'analysis.json').read_text());s=json.loads((run/'session.json').read_text());meta=yaml.safe_load(Path(s['map_yaml']).read_text());im=np.asarray(Image.open(s['map_image']).convert('L'));r=meta['resolution'];ox,oy,_=meta['origin'];gx=ox+(np.arange(im.shape[1])+.5)*r;gy=oy+(im.shape[0]-np.arange(im.shape[0])-.5)*r;X,Y=np.meshgrid(gx,gy);points=np.column_stack([X.ravel(),Y.ravel()]);occ=(255-im.astype(float))/255>=meta['free_thresh'];obs=json.loads((run/'collision_detail.json').read_text())['layouts'][-1]['poses']
for o in obs:
 c,ss=math.cos(o['yaw']),math.sin(o['yaw']);poly=np.array([[-.15,-.15],[.15,-.15],[.15,.15],[-.15,.15]])@np.array([[c,ss],[-ss,c]])+[o['x'],o['y']];occ|=PolyPath(poly).contains_points(points).reshape(im.shape)
dist=distance_transform_edt(~occ)*r;radius=math.hypot(.58,.2032)/2+r/math.sqrt(2)
wp=json.loads(Path(s['waypoints_file']).read_text())['global_traj_wpnts_iqp']['wpnts'];target=wp[268];goal=(target['x_m'],target['y_m'],target['psi_rad']);start=(d['diagnostic_start_xy'][0],d['diagnostic_start_xy'][1],math.radians(d['recorded_costmap_tests'][0]['yaw_deg']),.04)
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def safe(x,y):
 if not (-5.5<x<.5 and -7<y<-2):return False
 ix=int((x-ox)/r);iy=im.shape[0]-1-int((y-oy)/r)
 return 0<=ix<im.shape[1] and 0<=iy<im.shape[0] and dist[iy,ix]>radius
def key(p):return (round(p[0]/.06),round(p[1]/.06),round(wrap(p[2])/.13),round(p[3]/.14))
def heuristic(p):return math.hypot(p[0]-goal[0],p[1]-goal[1])
front=[(heuristic(start),0,0,start)];best={key(start):0.};nodes={0:(None,start)};count=0;expanded=0;terminal=None;t0=time.monotonic()
while front and expanded<100000 and time.monotonic()-t0<25:
 _,cost,idx,p=heapq.heappop(front)
 if cost>best.get(key(p),1e20)+1e-8:continue
 expanded+=1
 if heuristic(p)<.22 and abs(wrap(p[2]-goal[2]))<.35:terminal=idx;break
 if cost>12:continue
 for command in [-.4189,-.21,0.,.21,.4189]:
  x,y,yaw,steer=p;valid=True
  for _ in range(4):
   beta=math.atan(.15875/(.15875+.17145)*math.tan(steer));x+=math.cos(yaw+beta)*.05;y+=math.sin(yaw+beta)*.05;yaw=wrap(yaw+math.sin(beta)/.17145*.05);steer+=np.clip((command-steer)/.1,-3.2,3.2)*.05
   if not safe(x,y):valid=False;break
  if not valid:continue
  q=(x,y,yaw,float(steer));c=cost+.2+.015*abs(command-p[3]);k=key(q)
  if c>=best.get(k,1e20):continue
  count+=1;best[k]=c;nodes[count]=(idx,q);heapq.heappush(front,(c+1.15*heuristic(q),c,count,q))
path=[]
if terminal is not None:
 while terminal is not None:
  parent,q=nodes[terminal];path.append(q);terminal=parent
 path.reverse()
result={'found':bool(path),'expanded_states':expanded,'wall_s':time.monotonic()-t0,'start_xy_yaw_steer':start,'goal_xy_yaw':goal,'body_disk_radius_with_pixel_margin_m':radius,'fixed_speed_m_s':1.,'model':'kinematic bicycle, first-order steer tau .1s, steering-rate cap3.2rad/s, forward only','path_states':path,'limitations':'Offline simplified model and static occupancy; not a Gym dynamic replay or guaranteed MPPI candidate. Failure to find would not prove impossibility.'};(out/'kinematic_probe.json').write_text(json.dumps(result,indent=2));print({k:v for k,v in result.items() if k!='path_states'})
