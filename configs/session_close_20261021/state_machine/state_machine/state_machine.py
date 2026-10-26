import os
import rclpy
import numpy as np
from builtin_interfaces.msg import Time
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from f110_msgs.msg import WpntArray, OTWpntArray, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray
try:
    from vesc_msgs.msg import VescStateStamped
except:
    pass

from state_machine.transitions import dummy_transition, timetrials_transition, head_to_head_transition
from state_machine.state_types import StateType
from state_machine.states import DefaultStateLogic
from state_machine.state_machine_params import StateMachineParams


def time_to_float(time_instant: Time):
    return time_instant.sec + time_instant.nanosec * 1e-9


class StateMachine(Node):

    def __init__(self):
        super().__init__(
            'state_machine',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # PARAMETER DECLARATION
        self.params = StateMachineParams(self)

        self.ftg_disabled = True  # TODO: fix with global params?

        # update on parameter changes for rate
        self.add_on_set_parameters_callback(self.params.parameters_callback)

        # SUBSCRIPTIONS
        if self.params.test_on_car:
            self.battery_sub = self.create_subscription(VescStateStamped, "/vesc/sensors/core", self.battery_cb, 10)
        else:
            self.battery_level = self.params.volt_threshold + 1

        self.max_speed = None
        self.create_subscription(WpntArray, "/global_waypoints", self.glb_wpnts_cb, 10)
        while self.max_speed is None:  # equivalent of wait for message
            self.get_logger().info("Waiting for global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        self.glb_wpnts = None
        self.num_glb_wpnts = 0
        self.track_length = 1
        self.gb_max_idx = 10
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.glb_wpnts_scaled_cb, 10)
        while self.glb_wpnts is None:  # equivalent of wait for message
            self.get_logger().info("Waiting for scaled global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        self.cur_s = None
        self.cur_d = None
        self.create_subscription(Odometry, '/car_state/frenet/odom', self.car_state_frenet_cb, 10)  # car frenet coordinates
        while self.cur_s is None:
            self.get_logger().info("Waiting for car state frenet message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        if self.params.mode == 'head_to_head':
            self.current_position = None
            self.create_subscription(PoseStamped, '/car_state/pose', self.car_state_cb, 10)
            while self.current_position is None:
                self.get_logger().info("Waiting for car state pose message", throttle_duration_sec=0.5)
                rclpy.spin_once(self)

            self.last_valid_avoidance_wpnts = None
            self.splini_ttl_counter = 0
            self.avoidance_wpnts = None
            self.create_subscription(OTWpntArray, '/planner/avoidance/otwpnts', self.avoidance_cb, 10)

            self.obstacles = []
            self.create_subscription(ObstacleArray, "/perception/obstacles", self.obstacles_cb, 10)

            self.ftg_counter = 0

        # INITIALIZATIONS
        self.waypoints_dist = 0.1
        self.state = StateType(self.params.initial_state)
        self.local_waypoints = WpntArray()
        self.first_visualization = True
        self.x_viz = 0
        self.y_viz = 0

        # choose how to transition between states
        if self.params.mode == 'dummy':
            self.state_transition = dummy_transition
        elif self.params.mode == 'timetrials':
            self.state_transition = timetrials_transition
        elif self.params.mode == 'head_to_head':
            self.state_transition = head_to_head_transition
        else:
            raise NotImplementedError(f"Mode {self.params.mode} not recognized")

        # choose what to do in the different states
        self.state_logic = DefaultStateLogic

        # PUBLICATIONS
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.state_marker_pub = self.create_publisher(Marker, 'state_marker', 10)
        self.loc_wpnt_pub = self.create_publisher(WpntArray, 'local_waypoints', 10)
        self.vis_loc_wpnt_pub = self.create_publisher(MarkerArray, 'local_waypoints/markers', 10)

        # main loop
        self.main_loop = self.create_timer(1 / self.params.rate_hz, self.main_loop_callback)

        # 카운터 초기화
        self.trailing_to_gbtrack_count = 0
        self.trailing_to_gbtrack_counting_threshold = self.params.Trailing_to_GBtrack_counting_threshold

    #############
    # CALLBACKS #
    #############
    def battery_cb(self, msg: VescStateStamped):
        self.battery_level = msg.state.voltage_input

    def glb_wpnts_scaled_cb(self, data: WpntArray):
        """
        Callback function of velocity interpolator subscriber.
        """
        self.glb_wpnts = data.wpnts[:-1]  # exclude last point (because last point == first point)
        self.num_glb_wpnts = len(self.glb_wpnts)
        self.track_length = data.wpnts[-1].s_m
        # Get spacing between wpnts for rough approximations
        self.wpnt_dist = data.wpnts[1].s_m - data.wpnts[0].s_m
        self.gb_max_idx = len(data.wpnts) - 1

        # 로그: 스케일드 웨이포인트 상태
        self.get_logger().info(f'[WAYPOINTS] track_length={self.track_length:.3f} '
                               f'wpnt_dist={self.wpnt_dist:.4f} gb_max_idx={self.gb_max_idx}')

    def glb_wpnts_cb(self, data: WpntArray):
        self.max_speed = max([wpnt.vx_mps for wpnt in data.wpnts])
        # 로그: 글로벌 웨이포인트 최고 속도
        self.get_logger().info(f'[WAYPOINTS] max_speed (from global) = {self.max_speed:.3f} m/s')

    def car_state_frenet_cb(self, msg: Odometry):
        self.cur_s = msg.pose.pose.position.x
        self.cur_d = msg.pose.pose.position.y

    def avoidance_cb(self, data: OTWpntArray):
        """Subscribes to spliner waypoints"""
        if len(data.wpnts) > 0:
            self.splini_ttl_counter = int(self.params.splini_ttl * self.params.rate_hz)
            self.avoidance_wpnts = data
        else:
            pass

    def obstacles_cb(self, data):
        if len(data.obstacles) != 0:
            self.obstacles = data.obstacles
        else:
            self.obstacles = []

    def car_state_cb(self, data: PoseStamped):
        x = data.pose.position.x
        y = data.pose.position.y
        rot = Rotation.from_quat([data.pose.orientation.x, data.pose.orientation.y,
                                  data.pose.orientation.z, data.pose.orientation.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]
        self.current_position = [x, y, theta]

    ### Properties to evaulate state transitions
    @property
    def _low_bat(self) -> bool:
        return self.battery_level < self.params.volt_threshold

    @property
    def _check_close_to_raceline(self) -> bool:
        return np.abs(self.cur_d) < self.params.gb_ego_width_m  # [m]

    @property
    def _check_ofree(self) -> bool:
        o_free = True
        if self.params.overtake_mode == "spliner":
            if self.last_valid_avoidance_wpnts is not None:
                horizon = self.params.overtaking_horizon_m  # Horizon in front of cur_s [m]
                for obs in self.obstacles:
                    obs_s = obs.s_center
                    dist_to_obj = (obs_s - self.cur_s) % self.track_length
                    if dist_to_obj < horizon and len(self.last_valid_avoidance_wpnts):
                        obs_d = obs.d_center
                        avoid_wpnt_idx = np.argmin(
                            np.array([abs(avoid_s.s_m - obs_s) for avoid_s in self.last_valid_avoidance_wpnts])
                        )
                        ot_d = self.last_valid_avoidance_wpnts[avoid_wpnt_idx].d_m
                        ot_obs_dist = ot_d - obs_d
                        if abs(ot_obs_dist) < self.params.lateral_width_ot_m:
                            o_free = False
                            self.get_logger().info(f"O_FREE False, obs dist to ot lane: {ot_obs_dist} m")
                            break
            else:
                o_free = True
            return o_free
        else:
            self.get_logger().error(f"Unknown overtake planner")
            raise NotImplementedError

    @property
    def _check_gbfree(self) -> bool:
        gb_free = True
        horizon = self.params.gb_horizon_m  # Horizon in front of cur_s [m]
        for obs in self.obstacles:
            obs_s = obs.s_center
            gap = (obs_s - self.cur_s) % self.track_length
            if gap < horizon:
                obs_d = obs.d_center
                if abs(obs_d) < self.params.lateral_width_gb_m:
                    gb_free = False
                    break
        return gb_free

    @property
    def _check_enemy_in_front(self) -> bool:
        horizon = self.params.gb_horizon_m  # Horizon in front of cur_s [m]
        for obs in self.obstacles:
            gap = (obs.s_start - self.cur_s) % self.track_length
            if gap < horizon:
                return True
        return False

    @property
    def _check_availability_splini_wpts(self) -> bool:
        if self.avoidance_wpnts is None:
            return False
        elif len(self.avoidance_wpnts.wpnts) == 0:
            return False
        elif (
            abs(time_to_float(self.avoidance_wpnts.header.stamp) - time_to_float(self.avoidance_wpnts.last_switch_time))
            < self.params.splini_hyst_timer_sec
        ):
            self.get_logger().debug(
                f"Still too fresh into the switch...{abs(time_to_float(self.avoidance_wpnts.last_switch_time) - time_to_float(self.get_clock().now().to_msg()))}")
            return False
        else:
            self.last_valid_avoidance_wpnts = self.avoidance_wpnts.wpnts.copy()
            return True

    @property
    def _check_ftg(self) -> bool:
        threshold = self.params.ftg_timer_sec * self.params.rate_hz
        if self.ftg_disabled:
            return False
        else:
            if self.cur_state == StateType.TRAILING and self.cur_vs < self.params.ftg_threshold_speed:
                self.ftg_counter += 1
                self.get_logger().warn(f"[{self.name}] FTG counter: {self.ftg_counter}/{threshold}")
            else:
                self.ftg_counter = 0
            return self.ftg_counter > threshold

    @property
    def _check_emergency_break(self) -> bool:
        emergency_break = False
        if self.obstacles != []:
            horizon = self.emergency_break_horizon
            for obs in self.obstacles:
                dist_to_obj = (obs.s_center - self.cur_s) % self.track_length
                if dist_to_obj < horizon:
                    local_wpnt_idx = np.argmin(
                        np.array([abs(avoid_s.s_m - obs.s_center) for avoid_s in self.local_waypoints.wpnts])
                    )
                    ot_d = self.local_waypoints.wpnts[local_wpnt_idx].d_m
                    ot_obs_dist = ot_d - obs.d_center
                    if abs(ot_obs_dist) < self.params.lateral_width_ot_m:
                        emergency_break = True
                        self.get_logger().info("emergency break")
        else:
            emergency_break = False
        return emergency_break

    ###########
    # HELPERS #
    ###########
    def get_splini_wpts(self) -> WpntArray:
        """Obtain the waypoints by fusing those obtained by spliner with the global ones."""
        splini_glob = self.glb_wpnts.copy()
        if self.last_valid_avoidance_wpnts is not None:
            if self.last_valid_avoidance_wpnts[-1].s_m > self.last_valid_avoidance_wpnts[0].s_m:
                splini_idxs = [
                    s
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int(self.last_valid_avoidance_wpnts[-1].s_m / self.waypoints_dist + 0.5),
                    )
                ]
            else:
                splini_idxs = [
                    int(s % (self.track_length / self.waypoints_dist) + 0.5)
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int((self.track_length + self.last_valid_avoidance_wpnts[-1].s_m) / self.waypoints_dist + 0.5),
                    )
                ]
            for i, s in enumerate(splini_idxs):
                splini_glob[s] = self.last_valid_avoidance_wpnts[min(i, len(self.last_valid_avoidance_wpnts) - 1)]
        else:
            self.get_logger().warn(f"No valid avoidance waypoints, passing global waypoints")
        return splini_glob

    ###############
    # PUB HELPERS #
    ###############
    def _pub_local_waypoints(self, wpts: WpntArray):
        loc_markers = MarkerArray()
        loc_wpnts = wpts
        loc_wpnts.header.stamp = self.get_clock().now().to_msg()
        loc_wpnts.header.frame_id = "map"

        for i, wpnt in enumerate(loc_wpnts.wpnts):
            mrk = Marker()
            mrk.header.frame_id = "map"
            mrk.type = mrk.SPHERE
            mrk.scale.x = 0.15
            mrk.scale.y = 0.15
            mrk.scale.z = 0.15
            mrk.color.a = 1.0
            mrk.color.g = 1.0

            mrk.id = i
            mrk.pose.position.x = wpnt.x_m
            mrk.pose.position.y = wpnt.y_m
            mrk.pose.position.z = wpnt.vx_mps / self.max_speed
            mrk.pose.orientation.w = 1.0
            loc_markers.markers.append(mrk)

        if len(loc_wpnts.wpnts) == 0:
            self.get_logger().warn("No local waypoints published...")
        else:
            self.loc_wpnt_pub.publish(loc_wpnts)

        self.vis_loc_wpnt_pub.publish(loc_markers)

    def visualize_state(self, state: StateType):
        if self.first_visualization:
            self.first_visualization = False
            x0 = self.glb_wpnts[0].x_m
            y0 = self.glb_wpnts[0].y_m
            x1 = self.glb_wpnts[1].x_m
            y1 = self.glb_wpnts[1].y_m
            xy_norm = (
                -np.array([y1 - y0, x0 - x1]) / np.linalg.norm([y1 - y0, x0 - x1]) * 1.25 * self.glb_wpnts[0].d_left
            )

            self.x_viz = x0 + xy_norm[0]
            self.y_viz = y0 + xy_norm[1]

        mrk = Marker()
        mrk.type = mrk.SPHERE
        mrk.id = int(1)
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.color.a = 1.0
        mrk.color.g = 1.0
        mrk.pose.position.x = float(self.x_viz)
        mrk.pose.position.y = float(self.y_viz)
        mrk.pose.position.z = 0.0
        mrk.pose.orientation.w = 1.0
        mrk.scale.x = 1.0
        mrk.scale.y = 1.0
        mrk.scale.z = 1.0

        if state == StateType.GB_TRACK:
            mrk.color.g = 1.0
        elif state == StateType.OVERTAKE:
            mrk.color.r = 1.0
            mrk.color.g = 1.0
            mrk.color.b = 1.0
        elif state == StateType.TRAILING:
            mrk.color.r = 0.0
            mrk.color.g = 0.0
            mrk.color.b = 1.0
        elif state == StateType.FTGONLY:
            mrk.color.r = 1.0
            mrk.color.g = 0.0
            mrk.color.b = 0.0
        self.state_marker_pub.publish(mrk)

    #############
    # MAIN LOOP #
    #############
    def main_loop_callback(self):
        self.get_logger().debug(f"Current state: {self.state}")
        if self.params.force_state:
            self.state = self.params.force_state_choice
        else:
            self.state = self.state_transition(self)

        msg = String()
        msg.data = str(self.state)
        self.state_pub.publish(msg)
        self.visualize_state(state=self.state)

        self.local_waypoints.wpnts = self.state_logic(self)
        self._pub_local_waypoints(self.local_waypoints)

        if self.params.mode == "head_to_head" and self.params.overtake_mode == "spliner":
            self.splini_ttl_counter -= 1
            if self.splini_ttl_counter <= 0:
                self.last_valid_avoidance_wpnts = None
                self.avoidance_wpnts = WpntArray()
                self.splini_ttl_counter = -1


# defined as entry point in setup.py:
def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachine()
    rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


