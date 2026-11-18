#!/usr/bin/env python3
"""Counts collisions and lap times for one run, and writes the log.

The collision verdict comes from the simulator's own check, so nothing the car
does can influence it. A collision stops the car for good; getting it going again
is the operator's job (/initialpose), and that move voids the lap in progress.
"""

import json
import math
import os
import signal
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32, String

from race_manager.track import Track, crossed_forward


def best_lap_time(laps):
    """The fastest lap that counts.

    A lap counts only when it was driven clean and timed over the whole lap.
    A collision costs time, and a lap whose clock started where the car was put
    back on track is shorter than a lap rather than faster, so neither belongs
    in a best time.
    """
    counted = [lap['time'] for lap in laps
               if not lap['total'] and not lap.get('partial')]
    return min(counted, default=None)


class Scorer(Node):
    def __init__(self):
        super().__init__('scorer')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('map_name', '')
        self.declare_parameter('log_dir', 'race_result')
        self.declare_parameter('seed', -1)
        self.declare_parameter('odom_topic', '/car_state/odom_GT')
        self.declare_parameter('teleport_m', 1.5)

        self.track = Track(self.get_parameter('waypoints_file').value)
        self.line_a, self.line_b, self.heading = self.track.finish_line()

        started = datetime.now()
        log_dir = os.path.abspath(self.get_parameter('log_dir').value)
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(
            log_dir, f'race_result_{started.strftime("%Y%m%d_%H%M%S")}.json')

        self.started_at = started.isoformat(timespec='seconds')
        self.t0 = None
        self.lap_t0 = None
        self.lap = 0
        self.laps = []
        self.collisions = {'obstacle': 0, 'wall': 0}
        # Hits since the last credited lap, so every hit lands on the lap that earns it
        self.pending = {'obstacle': 0, 'wall': 0}
        # The lap clock started somewhere other than the line, so it is timing
        # less than a lap
        self.partial = False
        self.events = []

        self.pos = None
        self.prev_pos = None
        self.prev_kind = 'none'

        self.teleport_m = self.get_parameter('teleport_m').value
        self.prev_s = None
        self.progress = 0.0
        self.teleports = []

        self.lap_pub = self.create_publisher(Int32, '/sim/lap_count', 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.odom_callback, 10)
        self.create_subscription(String, '/sim/collision', self.collision_callback, 10)

        self.write_log()
        self.get_logger().info(
            f'seed {self.get_parameter("seed").value} -> {self.log_path}')
        self.get_logger().warn(
            'a collision stops the car for good; reset it with /initialpose to keep going')

    # ---------- clock ----------

    def now(self):
        """Wall clock. The bridge publishes no /clock, so sim time would stall."""
        return time.monotonic()

    def elapsed(self):
        return 0.0 if self.t0 is None else self.now() - self.t0

    # ---------- callbacks ----------

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y)
        if self.prev_pos is None:
            self.prev_pos = self.pos
            return

        # At 100 Hz no car covers teleport_m in one sample, so that jump means someone moved it
        jumped = math.dist(self.prev_pos, self.pos) > self.teleport_m
        if jumped:
            self.on_teleport()
        elif crossed_forward(self.prev_pos, self.pos,
                             self.line_a, self.line_b, self.heading):
            self.on_finish_line()

        # Sum the signed arc travelled: raw s comparisons read a nudge across the seam as a lap
        s_now = self.track.nearest_s(*self.pos)
        if not jumped and self.prev_s is not None:
            self.progress += self.track.arc_delta(self.prev_s, s_now)
        self.prev_s = s_now
        self.prev_pos = self.pos

    def on_teleport(self):
        """Moving the car by hand voids the lap in progress, so a jump cannot earn one."""
        self.progress = 0.0
        self.prev_s = None
        if self.t0 is not None:
            self.lap_t0 = self.now()
        self.partial = True
        self.teleports.append({
            't': round(self.elapsed(), 3),
            'lap': self.lap,
            'x': round(self.pos[0], 3),
            'y': round(self.pos[1], 3),
        })
        self.get_logger().warn(
            f'car was moved to ({self.pos[0]:.2f}, {self.pos[1]:.2f}) — '
            f'this lap is void, timing restarts at the next crossing')

    def on_finish_line(self):
        now = self.now()
        if self.t0 is None:
            self.t0 = now
            self.lap_t0 = now
            self.progress = 0.0
            self.partial = False
            self.get_logger().info('crossed the line — timing started')
            self.lap_pub.publish(Int32(data=0))
            return

        if self.progress < self.track.length / 2.0:
            # Reached the line without covering half the track, so it is not a lap
            covered = self.progress
            self.lap_t0 = now
            self.progress = 0.0
            self.partial = False
            self.get_logger().warn(
                f'crossing ignored: only {covered:.1f} m of track covered — timing restarts')
            return

        self.progress = 0.0
        self.lap += 1
        partial = self.partial
        record = {
            'lap': self.lap,
            'time': round(now - self.lap_t0, 3),
            'obstacle': self.pending['obstacle'],
            'wall': self.pending['wall'],
            'total': sum(self.pending.values()),
            'partial': partial,
        }
        self.laps.append(record)
        self.pending = {'obstacle': 0, 'wall': 0}
        self.partial = False
        self.lap_t0 = now
        self.get_logger().info(
            f'lap {self.lap}  {record["time"]:.2f} s  '
            f'{record["total"]} hits (obstacle {record["obstacle"]}, wall {record["wall"]})'
            + ('  — timed from where the car was put back, so it is not a best lap'
               if partial else ''))
        self.lap_pub.publish(Int32(data=self.lap))
        self.write_log()

    def collision_callback(self, msg: String):
        kind = msg.data
        rising = kind in ('obstacle', 'wall') and self.prev_kind == 'none'
        self.prev_kind = kind
        if not rising or self.pos is None:
            return

        self.collisions[kind] += 1
        self.pending[kind] += 1
        self.events.append({
            't': round(self.elapsed(), 3),
            'lap': self.lap,
            'kind': kind,
            'x': round(self.pos[0], 3),
            'y': round(self.pos[1], 3),
        })
        self.get_logger().warn(
            f'{kind} collision #{self.collisions[kind]} at '
            f'({self.pos[0]:.2f}, {self.pos[1]:.2f}) on lap {self.lap}')

        self.write_log()

    # ---------- output ----------

    def best_lap(self):
        return best_lap_time(self.laps)

    def write_log(self):
        data = {
            'started_at': self.started_at,
            'map': self.get_parameter('map_name').value,
            'waypoints_file': self.track.file,
            'seed': self.get_parameter('seed').value,
            'laps_completed': self.lap,
            'best_lap': self.best_lap(),
            'laps': self.laps,
            'total_time': round(sum(l['time'] for l in self.laps), 3),
            'collisions': dict(self.collisions, total=sum(self.collisions.values())),
            'collision_events': self.events,
            'teleports': self.teleports,
        }
        tmp = self.log_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        os.replace(tmp, self.log_path)

    def print_summary(self):
        # print, not the logger: on Ctrl-C the /rosout publisher is already down
        best = self.best_lap()
        hits = self.collisions
        out = [
            '',
            f'--- seed {self.get_parameter("seed").value}'
            f'  ·  {self.get_parameter("map_name").value or "track"}'
            f'  ·  {self.lap} laps ---',
            f'  best lap    {best:.2f} s' if best is not None else '  best lap    -',
            f'  collisions  {sum(hits.values())}'
            f'   (obstacle {hits["obstacle"]}, wall {hits["wall"]})',
            '',
            '  lap      time     hits   obstacle   wall',
        ]
        for l in self.laps:
            mark = ' *' if best is not None and math.isclose(
                l['time'], best, abs_tol=0.0005) else '  '
            out.append(f'  {l["lap"]:>3}{mark}{l["time"]:>8.2f} s{l["total"]:>7}'
                       f'{l["obstacle"]:>11}{l["wall"]:>7}'
                       + ('   partial' if l.get('partial') else ''))
        if self.pending['obstacle'] or self.pending['wall']:
            out.append(f'    -      (lap not finished){sum(self.pending.values()):>5}'
                       f'{self.pending["obstacle"]:>11}{self.pending["wall"]:>7}')
        left_out = [l for l in self.laps if l['total'] or l.get('partial')]
        if left_out:
            out.append('')
            out.append(f'  best lap is the fastest clean lap driven line to line'
                       f'  ({len(left_out)} of {len(self.laps)} left out)')
        out += [
            '',
            f'  log:    {self.log_path}',
            f'  report: ros2 run race_manager report {self.log_path}',
            '',
        ]
        print('\n'.join(out), flush=True)


def main():
    rclpy.init()
    node = Scorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Ctrl-C hits the whole process group, so a second one can land mid-summary
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    node.write_log()
    node.print_summary()
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
