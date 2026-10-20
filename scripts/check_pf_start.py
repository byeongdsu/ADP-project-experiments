"""Read-only simulation check: PF agrees with stationary GT before PP starts."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('check_pf_start')
    latest = {}

    def receive(key, msg):
        latest[key] = (time.monotonic(), msg)

    subscriptions = [
        node.create_subscription(Odometry, '/car_state/odom_GT',
                                 lambda m: receive('gt', m), 10),
        node.create_subscription(PoseStamped, '/pf/viz/inferred_pose',
                                 lambda m: receive('pf', m), 10)]
    stable_since = None
    result = {'passed': False, 'position_limit_m': .30, 'heading_limit_deg': 15.,
              'stationary_speed_limit_mps': .05, 'stable_seconds_required': 2.,
              'scope': 'simulation initialization check only; no pose correction published'}
    print('Waiting for stationary PF/GT agreement for 2 seconds...', flush=True)
    try:
        deadline = time.monotonic()+args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if len(latest) < 2 or any(now-t > .5 for t,m in latest.values()):
                stable_since = None
                continue
            g = latest['gt'][1]
            p = latest['pf'][1].pose
            q = g.pose.pose
            pos = math.hypot(p.position.x-q.position.x, p.position.y-q.position.y)
            angle = yaw(p.orientation)-yaw(q.orientation)
            heading = abs(math.degrees(math.atan2(math.sin(angle), math.cos(angle))))
            speed = math.hypot(g.twist.twist.linear.x, g.twist.twist.linear.y)
            result.update(position_error_m=pos, heading_error_deg=heading, gt_speed_mps=speed)
            good = all(math.isfinite(v) for v in (pos,heading,speed)) and pos < .30 and heading < 15 and speed < .05
            if not good:
                stable_since = None
            elif stable_since is None:
                stable_since = now
            elif now-stable_since >= 2.:
                result['passed'] = True
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    print('PF READY: start PP now.' if result['passed'] else
          'PF NOT READY: keep PP stopped; check initialization and sensor topics.')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
