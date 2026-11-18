"""Compare failed encounters to the same lap in R25, using read-only bags.

Future path errors describe a replanned closed loop, not isolated model accuracy.
"""
import bisect
import json
import math
import sqlite3
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'diagnostics/speed3_failure_review_20261101'
OUT.mkdir(exist_ok=True)
RUNS = {label: next((ROOT/'manual_runs').glob('*'+suffix)) for label, suffix in [
    ('R24', 'R24_clearance_speed3p0'), ('R25', 'R25_clearance_speed3p0_repeat'),
    ('R26', 'R26_clearance_speed3p0_repeat3')]}

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))

class Bag:
    def __init__(self, run):
        self.db = sqlite3.connect(f'file:{next((run/"bag").glob("*.db3"))}?mode=ro', uri=True)
        self.topics = {n:(i,get_message(t)) for i,n,t in self.db.execute('select id,name,type from topics')}
        self.cache = {}
    def load(self, topic):
        if topic not in self.cache:
            i,c = self.topics[topic]
            self.cache[topic] = [(t/1e9,deserialize_message(b,c)) for t,b in self.db.execute(
                'select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
        return self.cache[topic]
    def at(self, topic, time):
        rows = self.load(topic)
        i = bisect.bisect_right([t for t,m in rows], time)-1
        assert i >= 0
        return rows[i]

def path_costs(grid, path):
    origin = path.poses[0].pose
    c,s = math.cos(yaw(origin.orientation)),math.sin(yaw(origin.orientation))
    result = {}
    for layer in ['collision_layer','clearance_cost']:
        a = grid.data[grid.layers.index(layer)]
        nx,ny = a.layout.dim[1].size,a.layout.dim[0].size
        matrix = np.asarray(a.data).reshape((nx,ny),order='F')
        values=[]
        for pp in path.poses:
            dx,dy=pp.pose.position.x-origin.position.x,pp.pose.position.y-origin.position.y
            lx,ly=c*dx+s*dy,-s*dx+c*dy
            ix=int(math.floor((grid.info.pose.position.x+grid.info.length_x/2-lx)/grid.info.resolution))
            iy=int(math.floor((grid.info.pose.position.y+grid.info.length_y/2-ly)/grid.info.resolution))
            values.append(float(matrix[ix,iy]) if 0<=ix<nx and 0<=iy<ny else None)
        result[layer]={'positive_states':sum(v is not None and v>0 for v in values),
                       'outside_states':sum(v is None for v in values), 'values':values}
    return result

bags = {k:Bag(v) for k,v in RUNS.items()}
result = {'alignment':'Latest preceding bag receipt samples; geometry from published path initial pose.',
          'forecast_warning':'Future GT follows replanned commands, not the frozen planned sequence; error is not isolated dynamics error.',
          'map_cost_warning':'Grid joined by receipt time, not the exact grid used inside solver.', 'cases':[]}
for failed,lap,axis,target in [('R24',2,'y',2.1535),('R26',4,'x',4.306)]:
    failbag=bags[failed]
    hit=next(t for t,m in failbag.load('/sim/collision') if m.data in ['wall','obstacle'])
    success=bags['R25']
    bounds={m.data:t for t,m in reversed(success.load('/sim/lap_count'))}
    candidates=[]
    for t,m in success.load('/car_state/odom_GT'):
        p=m.pose.pose.position
        if bounds[lap-1]<=t<=bounds[lap] and (
            (-5<p.x<-2 and 1.5<p.y<4) if failed=='R24' else (2<p.x<6 and -2<p.y<2)):
            candidates.append((t,m))
    anchor=min(candidates,key=lambda r:abs(getattr(r[1].pose.pose.position,axis)-target))[0]
    fig,axs=plt.subplots(1,3,figsize=(17,5))
    case={'failure':failed,'lap':lap,'success_reference':'R25','runs':[]}
    for label,center,color in [(failed,hit,'tab:red'),('R25',anchor,'tab:blue')]:
        bag=bags[label]
        od=[(t,m) for t,m in bag.load('/car_state/odom_GT') if center-2<=t<=center+(.7 if label=='R25' else 0)]
        axs[0].plot([m.pose.pose.position.x for t,m in od],[m.pose.pose.position.y for t,m in od],color=color,label=label)
        _,obs=bag.at('/sim/obstacles',center)
        for pp in obs.poses:
            angle=yaw(pp.orientation);rot=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
            corners=np.array([[-.15,-.15],[.15,-.15],[.15,.15],[-.15,.15]])@rot.T+[pp.position.x,pp.position.y]
            axs[0].add_patch(Polygon(corners,fill=False,edgecolor='orange'))
        entry={'run':label,'snapshots':[]}
        for dt in [-1,-.5,-.2,-.05]:
            t=center+dt
            ts,odom=bag.at('/car_state/odom_GT',t);tc,cmd=bag.at('/mppi_sim/drive',t)
            tr,risk=bag.at('/mppi/collision_rate',t);tp,path=bag.at('/mppi/best_path',t)
            tg,grid=bag.at('/mppi/local_costmap',tp)
            snap={'dt':dt,'xy':[odom.pose.pose.position.x,odom.pose.pose.position.y],
                  'yaw':yaw(odom.pose.pose.orientation),'steer':cmd.drive.steering_angle,
                  'speed':cmd.drive.speed,'candidate_collision_fraction':risk.data,
                  'joined_path_costs':path_costs(grid,path),'forecast_vs_replanned_GT':[]}
            for horizon in [.1,.25,.5]:
                if label!= 'R25' and tp+horizon>=hit: continue
                future_t,future=bag.at('/car_state/odom_GT',tp+horizon)
                pp=path.poses[round(horizon/.05)].pose
                actual=future.pose.pose
                snap['forecast_vs_replanned_GT'].append({'horizon_s':horizon,
                    'xy_error_m':math.hypot(pp.position.x-actual.position.x,pp.position.y-actual.position.y),
                    'yaw_error_rad':abs(wrap(yaw(pp.orientation)-yaw(actual.orientation))),
                    'actual_sample_age_s':tp+horizon-future_t})
            entry['snapshots'].append(snap)
            if dt==-.5:
                axs[0].plot([pp.pose.position.x for pp in path.poses],[pp.pose.position.y for pp in path.poses], '--',color=color,alpha=.6)
        for topic,ax in [('/mppi_sim/drive',axs[1]),('/mppi/collision_rate',axs[2])]:
            vals=[(t,m) for t,m in bag.load(topic) if center-2<=t<=center]
            ax.plot([t-center for t,m in vals],[m.drive.steering_angle if topic.endswith('drive') else m.data for t,m in vals],color=color,label=label)
        case['runs'].append(entry)
    xy=np.array([s['xy'] for r in case['runs'] for s in r['snapshots']])
    axs[0].set_xlim(xy[:,0].min()-1,xy[:,0].max()+1);axs[0].set_ylim(xy[:,1].min()-1,xy[:,1].max()+1)
    axs[0].set_aspect('equal');axs[0].set_title('GT and frozen prediction at -0.5 s')
    axs[1].set_title('Steering command [rad]');axs[2].set_title('Candidate collision fraction');axs[2].set_ylim(0,1.05)
    for ax in axs: ax.legend()
    for ax in axs[1:]:ax.set_xlabel('seconds relative to encounter anchor')
    fig.tight_layout();fig.savefig(OUT/f'{failed}_vs_R25.png',dpi=160);plt.close(fig)
    result['cases'].append(case)
(OUT/'comparison.json').write_text(json.dumps(result,indent=2))
for case in result['cases']:
    print(case['failure'],[(r['run'],[(s['dt'],s['joined_path_costs']['collision_layer']['positive_states'],
          [(x['horizon_s'],round(x['xy_error_m'],3)) for x in s['forecast_vs_replanned_GT']]) for s in r['snapshots']]) for r in case['runs']])
