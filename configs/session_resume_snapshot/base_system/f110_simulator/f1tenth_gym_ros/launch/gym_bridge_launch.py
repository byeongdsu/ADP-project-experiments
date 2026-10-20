# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math

from f1tenth_gym_ros.map_selection import prefer_orig_map_yaml


def configure_simulator_map(context):
    """Select one map YAML for simulator physics and the map server."""
    configured = LaunchConfiguration('map_yaml_path').perform(context)
    resolved = prefer_orig_map_yaml(configured)
    print(f'[gym_bridge_launch] using simulator map {resolved}')
    actions = [SetLaunchConfiguration('resolved_map_yaml_path', resolved)]
    # Keep initial poses specific to the selected map, independent of GUI clicks
    # and the generic SIM/sim.yaml defaults.
    pose_file = os.path.join(os.path.dirname(configured), 'start_pose.yaml')
    if os.path.isfile(pose_file):
        with open(pose_file) as stream:
            pose = yaml.safe_load(stream)
        for key in ('sx', 'sy', 'stheta'):
            value = float(pose[key])
            if not math.isfinite(value):
                raise ValueError(f'{pose_file}: {key} must be finite')
            actions.append(SetLaunchConfiguration(f'map_start_{key}', str(value)))
        print(f'[gym_bridge_launch] fixed start pose from {pose_file}: {pose}')
    return actions


def generate_launch_description():
    ld = LaunchDescription()

    map_yaml_path = LaunchConfiguration('resolved_map_yaml_path')
    map_yaml_path_arg = DeclareLaunchArgument(
        'map_yaml_path', description="Path to map YAML file. Passed in via top-level launchfile.")
    resolve_map = OpaqueFunction(function=configure_simulator_map)

    # The localisation stack brings its own map_server under the same node name, so running
    # both leaves two lifecycle managers fighting over one node and the winner is a race.
    # base_system turns this one off in sim; standalone runs keep it.
    map_server_arg = DeclareLaunchArgument(
        'map_server', default_value='true',
        description="Serve /map from here. Turn off when localisation already serves it.")

    sim_setup_params = os.path.join(
        get_package_share_directory('stack_master'),
        'config',
        'SIM',
        'sim.yaml')

    config_dict = yaml.safe_load(open(sim_setup_params, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_setup_params,
                    {'map_path': map_yaml_path},
                    {key: ParameterValue(LaunchConfiguration(
                        f'map_start_{key}', default=str(config_dict['bridge']['ros__parameters'][key])),
                        value_type=float) for key in ('sx', 'sy', 'stheta')},
                    {'sim_params': os.path.join(get_package_share_directory('stack_master'), 'config', 'SIM', 'sim_params.yaml')}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=[
            # '-d', os.path.join(get_package_share_directory('stack_master'), 'config', 'SIM', 'sim.rviz')]
            '-d', "/home/misys/forza_ws/race_stack/forza_rviz.rviz"]
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        condition=IfCondition(LaunchConfiguration('map_server')),
        parameters=[{'yaml_filename': map_yaml_path},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=IfCondition(LaunchConfiguration('map_server')),
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(
            get_package_share_directory('f1tenth_gym_ros'), 'config', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(
            get_package_share_directory('f1tenth_gym_ros'), 'config', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')]
    )
    # TODO: add IMU

    # finalize
    ld.add_action(map_yaml_path_arg)
    ld.add_action(map_server_arg)
    ld.add_action(resolve_map)
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    if has_opp:
        ld.add_action(opp_robot_publisher)

    return ld
