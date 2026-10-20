"""Analyze localization and GT tracking in an explicit bag-time window."""
import argparse,json,math,sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path)
    parser.add_argument('--start',type=float,required=True)
    parser.add_argument('--end',type=float,required=True)
    args=parser.parse_args();root=args.run_dir
    db=sqlite3.connect(f'file:{root/"telemetry/telemetry_0.db3"}?mode=ro',uri=True)
    data={}
    for tid,n,typ in db.execute('SELECT id,name,type FROM topics'):
        if n not in ['/car_state/odom_GT','/pf/viz/inferred_pose','/global_waypoints','/drive']:continue
        cls=get_message(typ)
        data[n]=[(t/1e9,deserialize_message(b,cls)) for t,b in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(tid,))]
    gt=np.array([(t,m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation),m.twist.twist.linear.x) for t,m in data['/car_state/odom_GT']])
    errors=[]
    for t,m in data['/pf/viz/inferred_pose']:
        i=np.searchsorted(gt[:,0],t,side='right')-1
        if i<0 or t-gt[i,0]>.1:continue
        g=gt[i];p=m.pose
        h=yaw(p.orientation)-g[3]
        errors.append([t,np.hypot(p.position.x-g[1],p.position.y-g[2]),abs(math.degrees(math.atan2(math.sin(h),math.cos(h))))])
    errors=np.array(errors);window=errors[(errors[:,0]>=args.start)&(errors[:,0]<args.end)]
    gwin=gt[(gt[:,0]>=args.start)&(gt[:,0]<args.end)]
    w=data['/global_waypoints'][0][1].wpnts
    xy=np.array([(p.x_m,p.y_m) for p in w]);a=xy[:-1];v=xy[1:]-a
    v2=np.sum(v*v,axis=1);mask=v2>1e-12;a=a[mask];v=v[mask];v2=v2[mask]
    d=[]
    for g in gwin:
        offset=g[1:3]-a
        u=np.clip(np.sum(offset*v,axis=1)/v2,0,1)
        d.append(float(np.sqrt(np.min(np.sum((offset-u[:,None]*v)**2,axis=1)))))
    command_start=data['/drive'][0][0]
    before=errors[errors[:,0]<=command_start]
    result=dict(start=args.start,end=args.end,localization_samples=len(window),
        pf_position_error_mean_m=float(window[:,1].mean()),pf_position_error_max_m=float(window[:,1].max()),
        pf_yaw_abs_error_mean_deg=float(window[:,2].mean()),pf_yaw_abs_error_max_deg=float(window[:,2].max()),
        gt_tracking_abs_mean_m=float(np.mean(d)),gt_tracking_abs_max_m=float(max(d)),
        gt_tracking_method='distance to recorded global waypoint polyline segments',
        gt_vx_max_mps=float(gwin[:,4].max()),initial_gt_pose=gt[0,1:4].tolist(),initial_gt_vx=float(gt[0,4]),
        start_check_file_present=(root/'start_check.json').exists())
    if len(before):result['last_pre_command_localization_error']={'position_m':float(before[-1,1]),'heading_deg':float(before[-1,2])}
    np.savetxt(root/'localization_errors.csv',errors,delimiter=',',header='bag_time_s,position_error_m,absolute_yaw_error_deg',comments='')
    (root/'pf_tracking_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
