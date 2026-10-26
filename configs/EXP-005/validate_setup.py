"""Stationary harness validation in an isolated ROS domain; no controller/bridge."""
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseArray, PoseStamped
from f110_msgs.msg import WpntArray

root = Path(__file__).resolve().parents[2]
assert os.environ.get('ROS_DOMAIN_ID') == '87'
rclpy.init()
node = rclpy.create_node('fixed_layout_validation')
poses = []
diags = []
paths = []
node.create_subscription(PoseArray, '/sim/obstacles', lambda m: poses.append(
    [(p.position.x,p.position.y,p.orientation.z,p.orientation.w) for p in m.poses]), 10)
node.create_subscription(String, '/state_machine/diagnostics', lambda m: diags.append(json.loads(m.data)), 10)
node.create_subscription(WpntArray, '/state_machine/active_avoidance_wpnts', lambda m: paths.append(m), 10)
lap = node.create_publisher(Int32, '/sim/lap_count', 10)
procs = []
logs = []
try:
    log = open(root/'configs/EXP-005/spawner_validation.log','w'); logs.append(log)
    proc = subprocess.Popen(['ros2','launch','race_manager','race_manager_launch.xml',
        'num_obstacles:=2','seed:=2026','plot:=false',
        f'fixed_layout_file:={root}/configs/EXP-005/fixed_layout.json',
        'log_dir:=/tmp/exp005_stationary_validation'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    procs.append(proc)
    deadline=time.monotonic()+12
    while len(poses)<2 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    assert poses and len(poses[-1])==2, 'no fixed obstacle poses'
    original=poses[-1]
    for _ in range(25):
        lap.publish(Int32(data=3));rclpy.spin_once(node,timeout_sec=.1)
    assert all(p==original for p in poses), 'fixed layout changed after lap update'
    expected=json.loads((root/'configs/EXP-005/fixed_layout.json').read_text())['obstacles']
    assert [(p[0],p[1]) for p in original]==[(p['x'],p['y']) for p in expected]
    # Feed recorded inputs to state_machine alone: no simulator or drive publisher.
    db=sqlite3.connect(f'file:{root}/runs/EXP-004/R07/telemetry/telemetry_0.db3?mode=ro',uri=True)
    pubs=[]
    mapping={'/global_waypoints':'/global_waypoints','/global_waypoints_scaled':'/global_waypoints_scaled',
             '/car_state/frenet/odom':'/car_state/frenet/odom','/car_state/pose_GT':'/car_state/pose',
             '/perception/obstacles':'/perception/obstacles','/planner/avoidance/otwpnts':'/planner/avoidance/otwpnts'}
    for i,n,ty in db.execute('SELECT id,name,type FROM topics'):
        if n not in mapping:continue
        cls=get_message(ty)
        row=db.execute('SELECT data FROM messages WHERE topic_id=? AND timestamp<=? ORDER BY timestamp DESC LIMIT 1',(i,int(1792564037.4*1e9))).fetchone()
        assert row, n
        msg=deserialize_message(row[0],cls)
        pubs.append((node.create_publisher(cls,mapping[n],10),msg))
    log=open(root/'configs/EXP-005/state_machine_validation.log','w');logs.append(log)
    proc=subprocess.Popen(['ros2','launch','state_machine','state_machine_launch.xml','mode:=head_to_head'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    procs.append(proc)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline and len(diags)<20:
        for pub,msg in pubs:pub.publish(msg)
        rclpy.spin_once(node,timeout_sec=.04)
    assert len(diags)>=20 and paths, 'no diagnostic output'
    assert any(d['cached_path_points']>0 for d in diags), 'cached path was not exposed'
    result={'fixed_layout_unchanged_after_lap_update':True,'pose_messages':len(poses),
            'diagnostic_messages':len(diags),'active_path_messages':len(paths),
            'last_diagnostic':diags[-1],
            'scope':'isolated domain87, state machine with recorded static inputs; no controller, bridge, or driving'}
    (root/'configs/EXP-005/validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
finally:
    for p in procs:
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    for p in procs:
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    for log in logs:log.close()
    node.destroy_node();rclpy.shutdown()
