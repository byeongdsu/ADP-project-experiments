"""Geometry and recorded-costmap checks; no controller/settings changes.

Connectivity is a 2-D centre-path test, not a steering-feasible trajectory proof.
"""
import bisect
import collections
import json
import math
import sqlite3
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as PolyPath
from matplotlib.patches import Polygon
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'manual_runs/20261118T123305_18673_forward_mppi_B01_berlin_speed1p0'
OUT=RUN/'passage_analysis'
OUT.mkdir(exist_ok=True)
session=json.loads((RUN/'session.json').read_text())
meta=yaml.safe_load(Path(session['map_yaml']).read_text())
im=np.asarray(Image.open(session['map_image']).convert('L'))
free=((255.-im)/255.)<meta['free_thresh']
res=float(meta['resolution']);ox,oy,_=meta['origin']
wp=json.loads(Path(session['waypoints_file']).read_text())['global_traj_wpnts_iqp']['wpnts']
xy=np.array([(v['x_m'],v['y_m']) for v in wp]);arc=np.array([v['s_m'] for v in wp]);tree=cKDTree(xy)
db=sqlite3.connect(f'file:{next((RUN/"bag").glob("*.db3"))}?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
cache={}
def read(n):
    if n not in cache:
        i,c=topics[n]
        cache[n]=[(t/1e9,deserialize_message(d,c)) for t,d in db.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
    return cache[n]
def at(n,t):
    rows=read(n);i=bisect.bisect_right([tt for tt,m in rows],t)-1
    assert i>=0
    return rows[i]
def yaw(q):return math.atan2(2*q.w*q.z,1-2*q.z*q.z)
def square(x,y,angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[-.15,-.15],[.15,-.15],[.15,.15],[-.15,.15]])@np.array([[c,s],[-s,c]])+[x,y]
def image_index(x,y):
    return np.floor((np.asarray(x)-ox)/res).astype(int),im.shape[0]-1-np.floor((np.asarray(y)-oy)/res).astype(int)
def check_connection(mask,start,goal,return_path=False):
    if not all(0<=v<mask.shape[j] for p in [start,goal] for j,v in enumerate(p)):
        return {'connected':False,'reason':'outside grid'}
    if not mask[start] or not mask[goal]:
        return {'connected':False,'reason':'start or goal blocked','start_free':bool(mask[start]),'goal_free':bool(mask[goal])}
    labels,_=label(mask)
    connected=bool(labels[start]==labels[goal])
    result={'connected':connected,'reason':'4-neighbour centre connectivity'}
    if connected and return_path:
        queue=collections.deque([start]);parents={start:None}
        while queue:
            p=queue.popleft()
            if p==goal:break
            for di,dj in [(1,0),(-1,0),(0,1),(0,-1)]:
                q=p[0]+di,p[1]+dj
                if 0<=q[0]<mask.shape[0] and 0<=q[1]<mask.shape[1] and mask[q] and q not in parents:
                    parents[q]=p;queue.append(q)
        path=[];p=goal
        while p is not None:path.append(p);p=parents[p]
        result['path_indices']=path[::-1]
    return result

hit=next(t for t,m in read('/sim/collision') if m.data in ['wall','obstacle'])
_,obmsg=at('/sim/obstacles',hit)
polys=[square(p.position.x,p.position.y,yaw(p.orientation)) for p in obmsg.poses]
ob=obmsg.poses[2];obxy=np.array([ob.position.x,ob.position.y]);_,wi=tree.query(obxy)
normal=np.array([-math.sin(wp[wi]['psi_rad']),math.cos(wp[wi]['psi_rad'])])
invrot=np.array([[math.cos(yaw(ob.orientation)),math.sin(yaw(ob.orientation))],[-math.sin(yaw(ob.orientation)),math.cos(yaw(ob.orientation))]])
samples=np.arange(0.,3.001,.001);gaps={}
for side,sign in [('left',1),('right',-1)]:
    pts=obxy+samples[:,None]*normal*sign
    ix,iy=image_index(pts[:,0],pts[:,1]);inside=(ix>=0)&(ix<im.shape[1])&(iy>=0)&(iy<im.shape[0])
    local=(pts-obxy)@invrot.T;in_ob=(abs(local[:,0])<=.15)&(abs(local[:,1])<=.15)
    wall=np.ones(len(samples),dtype=bool);wall[inside]=~free[iy[inside],ix[inside]]
    wall_distance=float(samples[np.flatnonzero(wall)[0]])
    obstacle_exit=float(samples[np.flatnonzero(~in_ob)[0]])
    gaps[side]={'obstacle_center_to_wall_ray_m':wall_distance,'obstacle_surface_exit_ray_m':obstacle_exit,
                'surface_gap_ray_m':wall_distance-obstacle_exit,
                'gap_minus_vehicle_width_m':wall_distance-obstacle_exit-.2032}

# Rasterize all four actual obstacles on the published map.
gx=ox+(np.arange(im.shape[1])+.5)*res
gy=oy+(im.shape[0]-1-np.arange(im.shape[0])+.5)*res
XX,YY=np.meshgrid(gx,gy);flat=np.column_stack([XX.ravel(),YY.ravel()])
obmask=np.zeros(im.shape,dtype=bool)
for poly in polys:obmask|=PolyPath(poly).contains_points(flat).reshape(im.shape)
distance=distance_transform_edt(free&~obmask)*res
_,nearest=tree.query(flat);arcs=arc[nearest].reshape(im.shape)
roi=(arcs>=21.)&(arcs<=28.)
_,odom_start=at('/car_state/odom_GT',hit-3)
startxy=np.array([odom_start.pose.pose.position.x,odom_start.pose.pose.position.y])
goalxy=xy[268];sx,sy=image_index(*startxy);tx,ty=image_index(*goalxy)
global_tests={}
for name,radius in [('half_width_necessary_only',.2032/2),('circumscribed_body_disk_conservative',math.hypot(.58,.2032)/2)]:
    global_tests[name]={'radius_m':radius,**check_connection((distance>radius)&roi,(int(sy),int(sx)),(int(ty),int(tx)),True)}

snapshots=[]
for dt in [-3.,-2.5,-2.,-1.5]:
    tg,gm=at('/mppi/local_costmap',hit+dt);to,od=at('/car_state/odom_GT',tg)
    pose=od.pose.pose;angle=yaw(pose.orientation);c,s=math.cos(angle),math.sin(angle)
    a=gm.data[gm.layers.index('collision_layer')];nx,ny=a.layout.dim[1].size,a.layout.dim[0].size
    coll=np.asarray(a.data).reshape((nx,ny),order='F');soft=np.asarray(gm.data[gm.layers.index('clearance_cost')].data).reshape((nx,ny),order='F')
    resolution=gm.info.resolution
    maxx=gm.info.pose.position.x+gm.info.length_x/2;maxy=gm.info.pose.position.y+gm.info.length_y/2
    lx=maxx-(np.arange(nx)+.5)*resolution;ly=maxy-(np.arange(ny)+.5)*resolution
    LX,LY=np.meshgrid(lx,ly,indexing='ij');WX=pose.position.x+c*LX-s*LY;WY=pose.position.y+s*LX+c*LY
    ix,iy=image_index(WX,WY);inside=(ix>=0)&(ix<im.shape[1])&(iy>=0)&(iy<im.shape[0]);physical=np.zeros((nx,ny),dtype=bool)
    physical[inside]=distance[iy[inside],ix[inside]]>.2032/2
    _,nearest=tree.query(np.column_stack([WX.ravel(),WY.ravel()]));along=arc[nearest].reshape((nx,ny));physical&=(along>=21)&(along<=28)
    dx,dy=goalxy-[pose.position.x,pose.position.y];goal_local=(c*dx+s*dy,-s*dx+c*dy)
    start=(int(maxx/resolution),int(maxy/resolution));goal=(int((maxx-goal_local[0])/resolution),int((maxy-goal_local[1])/resolution))
    tests={}
    for ceiling in [0.,.25,.5,1.,2.,5.]:
        tests[str(ceiling)]=check_connection((coll==0)&physical&(soft<=ceiling+1e-6),start,goal)
    binary=check_connection((coll==0)&physical,start,goal,True)
    row={'dt_from_hit_s':dt,'costmap_sample_age_s':hit+dt-tg,'odom_to_grid_age_s':tg-to,
         'pose_xy':[pose.position.x,pose.position.y],'yaw_deg':angle*180/math.pi,
         'collision_free_connectivity_with_half_width_map_filter':binary,
         'connectivity_at_maximum_soft_cost':tests}
    separation=angle-wp[wi]['psi_rad']
    projected_full_width=2*(.35*abs(math.sin(separation))+.15*abs(math.cos(separation)))
    row['inflation_projected_width_on_obstacle_cross_section_m']=projected_full_width
    row['soft_margin_sensitivity']={}
    empty_dist=distance_transform_edt(coll==0)*resolution
    for margin in [.30,.15]:
        alternative=5*np.maximum(0.,1-empty_dist/margin)**2
        if binary['connected']:
            lo,hi=0.,5.
            for _ in range(12):
                mid=(lo+hi)/2
                if check_connection((coll==0)&physical&(alternative<=mid+1e-6),start,goal)['connected']:hi=mid
                else:lo=mid
            row['soft_margin_sensitivity'][str(margin)]={'minimum_bottleneck_soft_cost':hi}
        else:
            row['soft_margin_sensitivity'][str(margin)]={'minimum_bottleneck_soft_cost':None,'reason':'hard collision layer already disconnected'}
    snapshots.append(row)
    if dt==-2.:
        fig,axs=plt.subplots(1,3,figsize=(17,6))
        axs[0].imshow(im,extent=[ox,ox+im.shape[1]*res,oy,oy+im.shape[0]*res],origin='upper',cmap='gray')
        axs[0].plot(xy[:,0],xy[:,1],'--',color='gray',label='reference')
        traj=[m.pose.pose.position for t,m in read('/car_state/odom_GT') if hit-5<=t<=hit]
        axs[0].plot([p.x for p in traj],[p.y for p in traj],color='tab:red',label='failed GT')
        for poly in polys:axs[0].add_patch(Polygon(poly,facecolor='orange',edgecolor='black'))
        for name,test in global_tests.items():
            if test['connected']:
                indices=np.array(test['path_indices']);axs[0].plot(gx[indices[:,1]],gy[indices[:,0]],label=name)
        axs[0].set_xlim(-5,1);axs[0].set_ylim(-7,-2);axs[0].set_aspect('equal');axs[0].legend(fontsize=7);axs[0].set_title('Map geometry / 2-D centre routes')
        for ax,matrix,title in [(axs[1],coll,'Recorded inflated collision layer'),(axs[2],soft,'Recorded soft clearance cost')]:
            art=ax.pcolormesh(WX,WY,matrix,shading='nearest',cmap='magma');fig.colorbar(art,ax=ax)
            ax.plot(xy[:,0],xy[:,1],'--',color='cyan');ax.scatter(*row['pose_xy'],color='lime',label='vehicle');ax.scatter(*goalxy,color='blue',label='diagnostic goal')
            if binary['connected']:
                indices=np.array(binary['path_indices']);ax.plot(WX[indices[:,0],indices[:,1]],WY[indices[:,0],indices[:,1]],color='lime',label='2-D free route')
            ax.set_xlim(-5,1);ax.set_ylim(-7,-2);ax.set_aspect('equal');ax.set_title(title);ax.legend(fontsize=7)
        fig.tight_layout();fig.savefig(OUT/'geometry_vs_costmap.png',dpi=150);plt.close(fig)

result={'vehicle_length_m':.58,'vehicle_width_m':.2032,'inflation_half_lengths_current_ego_axes_m':[.35,.15],
        'soft_margin_m':.30,'soft_peak':5.,'obstacle_center_xy':obxy.tolist(),'reference_waypoint':int(wi),
        'cross_section_gap_measurements':gaps,'diagnostic_start_xy':startxy.tolist(),'diagnostic_goal_xy':goalxy.tolist(),
        'global_geometric_tests':global_tests,'recorded_costmap_tests':snapshots,
        'limitations':['Grid connectivity is not a nonholonomic or dynamic path feasibility proof.',
         'Half-width disk is necessary only; circumscribed disk is a conservative body-fit test, without steering constraints.',
         'Soft cost thresholds are diagnostic contours, not hard constraints in MPPI.',
         'Local grids are joined to preceding odom by receipt time; static map filter removes optimistic unknown-space shortcuts.',
         'Cross-section wall rays at the obstacle do not measure the narrowest gap along a whole manoeuvre.']}
(OUT/'analysis.json').write_text(json.dumps(result,indent=2))
compact={**result,'global_geometric_tests':{k:{kk:vv for kk,vv in v.items() if kk!='path_indices'} for k,v in global_tests.items()},
         'recorded_costmap_tests':[{**v,'collision_free_connectivity_with_half_width_map_filter':{k:x for k,x in v['collision_free_connectivity_with_half_width_map_filter'].items() if k!='path_indices'}} for v in snapshots]}
print(json.dumps(compact,indent=2))
