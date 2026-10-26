#!/usr/bin/env python3
"""
노드명: obstacle_spawner

역할: 이번 lap에서 사용할 장애물 위치 후보를 선택하고 /sim/obstacles로 토픽 발행한다.

The candidate pool is generated, not stored: freeze the inputs (waypoints md5 + parameters
+ seed, all logged at startup), not the list.
"""

import hashlib
import math
import os
import random
import signal

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

from race_manager.placement import draw, generate_candidates, parse_recipe
from race_manager.track import Track
from race_manager.fixed_layout import load_fixed_layout

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)

REPUBLISH_PERIOD = 1.0
# Without a lifetime RViz keeps drawing the cubes after this node dies.
MARKER_LIFETIME = Duration(sec=3)

GEN_PARAMS = ('obstacle_size', 'wall_margin', 'car_width', 'squeeze_clearance',
              'candidate_spacing', 'candidate_d_step', 'candidate_thetas',
              'exclusion_ahead', 'exclusion_behind', 'min_severity')


class ObstacleManager(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('num_obstacles', 4)
        self.declare_parameter('seed', -1)
        self.declare_parameter('fixed_layout_file', '')
        self.declare_parameter('obstacle_size', 0.30)
        self.declare_parameter('min_obstacle_gap', 0.50)
        self.declare_parameter('wall_margin', 0.15)
        self.declare_parameter('car_width', 0.31)
        self.declare_parameter('squeeze_clearance', 0.05)
        self.declare_parameter('exclusion_ahead', 5.0)
        self.declare_parameter('exclusion_behind', 1.0)
        self.declare_parameter('candidate_spacing', 0.5)
        self.declare_parameter('candidate_d_step', 0.10)
        self.declare_parameter('candidate_thetas',
                               [0.0, math.pi / 8, math.pi / 4, 3 * math.pi / 8])
        self.declare_parameter('min_severity', 0.0)
        self.declare_parameter('chicane_span', 2.5)
        self.declare_parameter('recipes', ['chicane:1:0.10 forced:1:0.20',
                                          'forced:2:0.20',
                                          'chicane:2:0.10',
                                          'forced:1:0.30 either:1:0.20'])

        self.num_obstacles = self.get_parameter('num_obstacles').value
        self.size = self.get_parameter('obstacle_size').value
        self.min_obstacle_gap = self.get_parameter('min_obstacle_gap').value
        self.chicane_span = self.get_parameter('chicane_span').value
        self.recipes = [(r, *parse_recipe(r)) for r in self.get_parameter('recipes').value]
        if not self.recipes:
            raise RuntimeError('no recipes configured')

        seed = self.get_parameter('seed').value
        if seed is not None and seed >= 0:
            self.seed = int(seed)
            self.reproducible = True
        else:
            self.seed = random.SystemRandom().randrange(2 ** 31)
            self.reproducible = False
        self.rng = random.Random(self.seed)

        self.track = Track(self.get_parameter('waypoints_file').value)
        layout_file = self.get_parameter('fixed_layout_file').value
        self.fixed_layout = load_fixed_layout(
            layout_file, map_name=os.path.basename(os.path.dirname(
                self.get_parameter('waypoints_file').value)),
            size=self.size, count=self.num_obstacles) if layout_file else None
        self.candidates = self._generate_candidates() if self.fixed_layout is None else []
        if self.fixed_layout is None and not self.candidates:
            raise RuntimeError('no obstacle candidates available')

        self.obstacles_pub = self.create_publisher(PoseArray, '/sim/obstacles', LATCHED)
        self.markers_pub = self.create_publisher(MarkerArray, '/sim/obstacles/markers', LATCHED)

        self.create_subscription(Int32, '/sim/lap_count', self.lap_callback, 10)

        self._log_fingerprint()

        self.lap = 0
        self.place()

        self.create_timer(REPUBLISH_PERIOD, self.republish)

    # ---------- candidates ----------

    def _generate_candidates(self):
        out = generate_candidates(
            self.track,
            size=self.size,
            wall_margin=self.get_parameter('wall_margin').value,
            car_width=self.get_parameter('car_width').value,
            squeeze_clearance=self.get_parameter('squeeze_clearance').value,
            spacing=self.get_parameter('candidate_spacing').value,
            d_step=self.get_parameter('candidate_d_step').value,
            thetas=list(self.get_parameter('candidate_thetas').value),
            exclusion_ahead=self.get_parameter('exclusion_ahead').value,
            exclusion_behind=self.get_parameter('exclusion_behind').value,
            min_severity=self.get_parameter('min_severity').value,
        )
        by_side = {k: sum(1 for c in out if c.side == k) for k in ('either', 'LEFT', 'RIGHT')}
        self.get_logger().info(
            f'generated {len(out)} candidates — either {by_side["either"]}, '
            f'LEFT {by_side["LEFT"]}, RIGHT {by_side["RIGHT"]}')
        return out

    def _log_fingerprint(self):
        """Everything needed to reproduce the layout, so no candidate file has to be kept."""
        wpnts = self.get_parameter('waypoints_file').value
        h = hashlib.md5()
        with open(wpnts, 'rb') as f:
            for chunk in iter(lambda: f.read(1 << 20), b''):
                h.update(chunk)
        settings = ', '.join(f'{k}={self.get_parameter(k).value}' for k in GEN_PARAMS)
        self.get_logger().info(
            f'seed {self.seed} '
            f'({"reproducible" if self.reproducible else "random — practice mode"}), '
            f'waypoints md5 {h.hexdigest()}, {settings}, '
            f'recipes {self.get_parameter("recipes").value}, '
            f'picking {self.num_obstacles} per lap')

    # ---------- placement ----------

    def pick(self, slots, spread):
        return draw(
            self.candidates, self.num_obstacles, self.rng, self.track,
            wall_margin=self.get_parameter('wall_margin').value,
            size=self.size,
            min_obstacle_gap=self.min_obstacle_gap,
            slots=slots,
            chicane_span=self.chicane_span,
            car_width=self.get_parameter('car_width').value,
            squeeze_clearance=self.get_parameter('squeeze_clearance').value,
            spread=spread,
        )

    def place(self):
        if self.fixed_layout is not None:
            self.current = self.fixed_layout
            self.republish(clear=True)
            self.get_logger().info(
                f'lap {self.lap}: FIXED layout {self.get_parameter("fixed_layout_file").value}; '
                f'poses {[(c.x, c.y, c.theta) for c in self.current]}')
            return
        recipe, spread, slots = self.rng.choice(self.recipes)
        self.current, relaxed = self.pick(slots, spread)
        if len(self.current) < self.num_obstacles:
            self.get_logger().warn(
                f'only {len(self.current)}/{self.num_obstacles} obstacles fit with '
                f'{self.min_obstacle_gap} m clearance — add candidates or lower num_obstacles')
        if relaxed:
            self.get_logger().warn(f'lap {self.lap}: recipe relaxed — {"; ".join(relaxed)}')
        self.republish(clear=True)
        spots = ', '.join(f'{c.id}[{c.side} sev {c.severity:.2f} vis {c.visibility:.1f}]'
                          for c in self.current)
        self.get_logger().info(
            f'lap {self.lap} [{recipe}]: {len(self.current)} obstacles — {spots}')

    def republish(self, clear=False):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for cand in self.current:
            pose = Pose()
            pose.position.x = cand.x
            pose.position.y = cand.y
            pose.orientation.z = math.sin(cand.theta / 2.0)
            pose.orientation.w = math.cos(cand.theta / 2.0)
            msg.poses.append(pose)
        self.obstacles_pub.publish(msg)
        self.markers_pub.publish(self._markers(clear))

    def _markers(self, clear):
        """Stable ids, re-sent with the same values every second so RViz picks
        them up whenever it connects. DELETEALL only on a new layout -- sending
        it on every republish makes the cubes flicker."""
        array = MarkerArray()
        if clear:
            wipe = Marker()
            wipe.action = Marker.DELETEALL
            array.markers.append(wipe)
        stamp = self.get_clock().now().to_msg()
        for i, cand in enumerate(self.current):
            box = Marker()
            box.header.frame_id = 'map'
            box.header.stamp = stamp
            box.ns = 'sim_obstacles'
            box.id = i
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = cand.x
            box.pose.position.y = cand.y
            box.pose.position.z = self.size / 2.0
            box.pose.orientation.z = math.sin(cand.theta / 2.0)
            box.pose.orientation.w = math.cos(cand.theta / 2.0)
            box.scale.x = box.scale.y = box.scale.z = self.size
            box.color.r, box.color.g, box.color.b, box.color.a = 0.76, 0.35, 0.11, 0.9
            box.lifetime = MARKER_LIFETIME
            array.markers.append(box)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = stamp
            label.ns = 'sim_obstacle_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = cand.x
            label.pose.position.y = cand.y
            label.pose.position.z = self.size + 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.28
            label.color.r, label.color.g, label.color.b, label.color.a = 0.76, 0.35, 0.11, 1.0
            label.lifetime = MARKER_LIFETIME
            label.text = str(i + 1)
            array.markers.append(label)
        return array

    def lap_callback(self, msg: Int32):
        if msg.data == self.lap:
            return
        self.lap = msg.data
        self.place()


def main():
    rclpy.init()
    node = ObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
